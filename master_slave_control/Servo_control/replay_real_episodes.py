#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
达妙真机机械臂 + 夹爪：ALOHA 规范轨迹重播系统

功能特性：
1. 兼容 ALOHA 规范：直接读取 HDF5 中的 `/observations/qpos` 键值。
2. 7-DoF 同步控制：前 6 维使用 POS_VEL 控制机械臂，第 7 维使用 MIT 模式控制夹爪。
3. 安全预引导：重播前提供 4 秒的 S 型速度曲线引导，防止第一帧位置突变导致机械臂抽搐。
4. 物理防暴走：内置 SafetyGuard 进行步长削峰和真机跟踪误差监控。
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path

import h5py
import numpy as np

# 根据你的实际路径调整导入
CURRENT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(CURRENT_DIR.parents[0]))
sys.path.insert(0, str(CURRENT_DIR.parents[1]))

from reBotArm_control_py.actuator import RobotArm

# =============================================================================
# 全局标志与配置
# =============================================================================
_running = True

# 机械臂 6 轴安全限制
DEFAULT_CMD_VLIM = np.array([1.0, 1.0, 1.0, 2.0, 2.0, 2.0], dtype=np.float64)
DEFAULT_MAX_STEP = np.array([0.02, 0.02, 0.02, 0.03, 0.03, 0.03], dtype=np.float64)


def _sigint_handler(signum, frame) -> None:
    global _running
    print("\n[Replay] 收到退出信号，准备安全停机...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)


# =============================================================================
# 夹爪控制函数 (复用遥操作中的逻辑)
# =============================================================================
def _load_gripper_cfg_func():
    try:
        from reBotArm_control_py.actuator.gripper import load_cfg
        return load_cfg
    except ImportError:
        raise ImportError("无法加载 gripper.py 中的 load_cfg，请检查环境。")


def setup_damiao_gripper(arm, gripper_cfg_path: Path):
    if arm is None or not gripper_cfg_path.exists(): return None, None
    load_gripper_cfg = _load_gripper_cfg_func()
    g_cfg = load_gripper_cfg(str(gripper_cfg_path))["gripper"]

    shared_damiao_controller = arm._ctrl_map.get("damiao")
    if not shared_damiao_controller: raise RuntimeError("未找到 damiao 控制器")

    gripper_name = getattr(g_cfg, "name", "gripper")
    if gripper_name in arm._motor_map:
        g_mot = arm._motor_map[gripper_name]
    else:
        g_mot = shared_damiao_controller.add_damiao_motor(g_cfg.motor_id, g_cfg.feedback_id, g_cfg.model)
        arm._motor_map[gripper_name] = g_mot

    from motorbridge import Mode
    g_mot.ensure_mode(Mode.MIT, 1000)
    shared_damiao_controller.enable_all()
    time.sleep(0.2)
    return g_mot, shared_damiao_controller


def send_damiao_gripper_mit(g_mot, target_rad, kp=1.0, kd=0.05, tau=0.0) -> None:
    if g_mot:
        try:
            g_mot.send_mit(float(target_rad), 0.0, float(kp), float(kd), float(tau))
        except:
            pass


def get_gripper_feedback_pos(g_mot) -> float:
    if g_mot is None: return 0.0
    try:
        return float(g_mot.get_state().pos)
    except:
        return 0.0


# =============================================================================
# 核心防护网
# =============================================================================
def _unwrap_near(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """防止关节跨越 360 度边界时发生反转抽搐"""
    return np.asarray(values) + 2.0 * np.pi * np.round((np.asarray(reference) - np.asarray(values)) / (2.0 * np.pi))


def _clip_rate(target: np.ndarray, previous: np.ndarray, max_step: np.ndarray) -> np.ndarray:
    """指令平滑：强制限制单步最大变化量"""
    return previous + np.clip(target - previous, -max_step, max_step)


class SafetyGuard:
    def __init__(self, max_step, max_tracking_error=1.0, breach_samples=20):
        self.max_step = np.asarray(max_step, dtype=np.float64)
        self.max_tracking_error = max_tracking_error
        self.breach_samples = breach_samples
        self.command = None
        self._breach_count = 0

    def initialize(self, q_real_now: np.ndarray) -> np.ndarray:
        self.command = np.asarray(q_real_now, dtype=np.float64)[:6].copy()
        return self.command

    def next_command(self, q_target: np.ndarray, q_feedback: np.ndarray) -> np.ndarray:
        q_target = np.asarray(q_target, dtype=np.float64)[:6]
        q_feedback = np.asarray(q_feedback, dtype=np.float64)[:6]

        q_target_cmd = _unwrap_near(q_target, q_feedback)
        tracking_error = np.max(np.abs(q_target_cmd - q_feedback))

        if tracking_error > self.max_tracking_error:
            self._breach_count += 1
            if self._breach_count >= self.breach_samples:
                raise RuntimeError(f"⚠️ 真机跟踪误差过大 ({tracking_error:.2f} rad)，触发断电保护。")
        else:
            self._breach_count = 0

        self.command = _clip_rate(q_target_cmd, _unwrap_near(self.command, q_feedback), self.max_step)
        return self.command.copy()


def close_arm_fast(arm) -> None:
    if arm:
        try:
            arm.disable(retries=0); time.sleep(0.1)
        except:
            pass


# =============================================================================
# 主逻辑
# =============================================================================
def main():
    global _running

    parser = argparse.ArgumentParser(description="达妙真机 ALOHA 规范重播系统")
    parser.add_argument("--dataset", "-d", type=str, required=True, help="HDF5 文件路径")
    parser.add_argument("--cfg", type=Path, default=None, help="RobotArm 配置文件")
    parser.add_argument("--gripper-cfg", type=Path, default=Path("./config/gripper.yaml"))
    parser.add_argument("--speed-scale", type=float, default=1.0, help="重播速度倍率")
    parser.add_argument("--rate", type=float, default=50.0, help="控制频率")
    args = parser.parse_args()

    # 1. 📂 读取 ALOHA 规范数据
    if not Path(args.dataset).exists():
        print(f"❌ 找不到数据集: {args.dataset}")
        return

    print(f"\n📂 正在加载数据集: {args.dataset}")
    with h5py.File(args.dataset, 'r') as f:
        # 严格读取 /observations/qpos (N, 7)
        qpos_data = np.array(f['/observations/qpos'])
        record_rate = f.attrs.get('hz_rate', args.rate)

    total_frames = len(qpos_data)
    record_dt = 1.0 / record_rate
    print(f"✅ 数据加载完成: 共 {total_frames} 帧, 原始录制频率 {record_rate} Hz")

    # 2. 🤖 初始化硬件
    arm = RobotArm(cfg_path=str(args.cfg) if args.cfg else None)
    arm.connect()
    arm.enable()
    arm.mode_pos_vel(vlim=DEFAULT_CMD_VLIM)

    gripper_motor, _ = setup_damiao_gripper(arm, args.gripper_cfg)

    # 3. 🛡️ 状态初始化与安全网挂载
    q_feedback_raw = np.asarray(arm.get_positions(request=True)[:6])
    gripper_fb_start = get_gripper_feedback_pos(gripper_motor)

    guard = SafetyGuard(max_step=DEFAULT_MAX_STEP)
    q_cmd = guard.initialize(q_feedback_raw)
    gripper_cmd = gripper_fb_start

    # 4. ⏱️ 核心时钟配置
    PREPARE_DURATION = 4.0  # 4 秒平滑引导期
    REPLAY_DURATION = (total_frames * record_dt) / args.speed_scale

    print(f"\n🚀 [重播启动] 速度: {args.speed_scale}x | 引导: {PREPARE_DURATION}s | 重播: {REPLAY_DURATION:.2f}s")

    t_start = time.perf_counter()
    cmd_period = 1.0 / args.rate
    frame = 0

    try:
        while _running:
            loop_start = time.perf_counter()
            elapsed = loop_start - t_start

            # ---------------- A. 获取实时反馈 ----------------
            q_feedback = np.asarray(arm.get_positions(request=True)[:6])

            # ---------------- B. 计算轨迹插值 ----------------
            if elapsed < PREPARE_DURATION:
                # 引导阶段：利用余弦曲线，从当前实际位置平滑滑动到录制轨迹的第 0 帧
                progress = elapsed / PREPARE_DURATION
                smooth = (1.0 - math.cos(progress * math.pi)) / 2.0

                target_q_6dof = q_cmd + (_unwrap_near(qpos_data[0][:6], q_cmd) - q_cmd) * smooth
                target_gripper = gripper_cmd + (qpos_data[0][6] - gripper_cmd) * smooth
                stage = "预引导 (Prepare)"

            elif elapsed < PREPARE_DURATION + REPLAY_DURATION:
                # 重播阶段：根据经过的时间动态索引数据帧
                replay_time = elapsed - PREPARE_DURATION
                idx = min(int((replay_time * args.speed_scale) / record_dt), total_frames - 1)

                target_q_6dof = qpos_data[idx][:6]
                target_gripper = qpos_data[idx][6]
                stage = "重播中 (Replay)"

            else:
                print(f"\n✅ [重播完成] 轨迹执行完毕！耗时: {elapsed:.2f}s")
                break

            # ---------------- C. 步长削峰与指令下发 ----------------
            q_cmd = guard.next_command(target_q_6dof, q_feedback)
            arm.pos_vel(q_cmd, vlim=DEFAULT_CMD_VLIM)

            send_damiao_gripper_mit(gripper_motor, target_gripper)

            # ---------------- D. 终端监控打印 ----------------
            if frame % 25 == 0:
                err = np.max(np.abs(q_cmd - q_feedback))
                print(
                    f"[{stage}] t={elapsed:5.2f}s | J1指令: {q_cmd[0]:+.2f} | 夹爪指令: {target_gripper:+.2f} | 最大跟踪误差: {err:4.2f} rad",
                    end="\r")

            frame += 1

            # 精确时钟睡眠
            sleep_time = cmd_period - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"\n❌ [异常中止] {e}")
    finally:
        print("\n[退出流程] 切断硬件连接并卸载电机力矩...")
        close_arm_fast(arm)


if __name__ == "__main__":
    main()

# python replay_real_episodes.py --dataset ./data/rebot_test/episode_0.hdf5
