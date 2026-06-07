#!/usr/bin/env python3
"""reBotArm 纯物理位置重播系统 (带自动安全停机与监控)

功能特性：
1. 预引导阶段：从当前位置平滑引导至重播起点，消除启动突变。
2. 保护模式：重播结束后进入缓冲期，随后自动执行失能保护。
3. 纯物理闭环：基于 POS_VEL 模式，依靠 SafetyGuard 削峰与误差监控。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
import math
from pathlib import Path

import h5py
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RobotArm

# --------------------------------------------------------------------------- #
# 配置参数
# --------------------------------------------------------------------------- #
DEFAULT_CMD_VLIM = np.array([1.5, 1.5, 1.5, 3.0, 3.0, 3.0])
DEFAULT_MAX_STEP = np.array([0.05, 0.05, 0.05, 0.06, 0.06, 0.06])
DEFAULT_TRACKING_BREACH_SAMPLES = 20

_running = True


def _sigint_handler(signum, frame) -> None:
    global _running
    print("\n[Replay] 收到退出信号，准备安全关闭...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)


def _parse_vector(values: list[float] | None, default: np.ndarray, name: str) -> np.ndarray:
    arr = default.astype(np.float64) if values is None else np.asarray(values, dtype=np.float64)
    if arr.shape != default.shape:
        raise ValueError(f"{name} 必须提供 {default.size} 个数，当前为 {arr.size} 个")
    return arr


def _clip_rate(target: np.ndarray, previous: np.ndarray, max_step: np.ndarray) -> np.ndarray:
    return previous + np.clip(target - previous, -max_step, max_step)


def _unwrap_near(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    return values + 2.0 * np.pi * np.round((reference - values) / (2.0 * np.pi))


def close_arm_fast(arm) -> None:
    if arm is None: return
    try:
        print("[硬件保护] 执行失能指令...")
        arm.disable(retries=0)
        time.sleep(0.1)
    except Exception:
        pass
    getattr(arm, "_ctrl_map", {}).clear()
    getattr(arm, "_motor_map", {}).clear()


class SafetyGuard:
    def __init__(self, num_joints: int, max_step: np.ndarray, max_tracking_error: float, tracking_breach_samples: int):
        self.num_joints = num_joints
        self.max_step = max_step[:num_joints]
        self.max_tracking_error = max_tracking_error
        self.tracking_breach_samples = max(int(tracking_breach_samples), 1)
        self.command = None
        self._tracking_breach_count = 0

    def initialize(self, q_real_now: np.ndarray) -> np.ndarray:
        self.command = np.asarray(q_real_now, dtype=np.float64)[:self.num_joints].copy()
        return self.command.copy()

    def next_command(self, q_target: np.ndarray, q_feedback: np.ndarray) -> np.ndarray:
        q_target = np.asarray(q_target, dtype=np.float64)[:self.num_joints]
        q_feedback = np.asarray(q_feedback, dtype=np.float64)[:self.num_joints]
        q_target_cmd = _unwrap_near(q_target, q_feedback)
        previous_cmd = _unwrap_near(self.command, q_feedback)

        tracking_error = np.max(np.abs(q_target_cmd - q_feedback))
        if tracking_error > self.max_tracking_error:
            self._tracking_breach_count += 1
            if self._tracking_breach_count >= self.tracking_breach_samples:
                raise RuntimeError(f"真机跟踪误差过大: {tracking_error:.3f} rad，触发保护停机。")
        else:
            self._tracking_breach_count = 0

        self.command = _clip_rate(q_target_cmd, previous_cmd, self.max_step)
        return self.command.copy()


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="reBotArm 纯物理轨迹重播 (POS_VEL 模式)")
    parser.add_argument("--task_name", "-t", type=str, default="test_task")
    parser.add_argument("--dataset_dir", "-d", type=str, required=True)
    parser.add_argument("--episode_idx", "-idx", type=int, default=0)

    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--speed_scale", type=float, default=1.0)
    parser.add_argument("--max-tracking-error", type=float, default=1.0)
    parser.add_argument("--vlim", type=float, nargs=6, default=None)
    parser.add_argument("--max-step", type=float, nargs=6, default=None)
    parser.add_argument("--print-every", type=int, default=50)
    return parser


def main() -> None:
    global _running
    args = build_argparser().parse_args()
    vlim = _parse_vector(args.vlim, DEFAULT_CMD_VLIM, "--vlim")
    max_step = _parse_vector(args.max_step, DEFAULT_MAX_STEP, "--max-step")

    with h5py.File(args.dataset_dir, 'r') as f:
        qpos_data = np.array(f['qpos'])
        record_rate = f.attrs.get('hz_rate', args.rate)

    total_frames = len(qpos_data)
    record_dt = 1.0 / record_rate

    arm = RobotArm()
    arm.connect()
    arm.enable()
    arm.mode_pos_vel(vlim=vlim)
    time.sleep(0.5)

    q_raw_feedback = arm.get_positions(request=True)[:6]
    q_feedback = _unwrap_near(q_raw_feedback, qpos_data[0][:6])

    guard = SafetyGuard(6, max_step, args.max_tracking_error, DEFAULT_TRACKING_BREACH_SAMPLES)
    guard.initialize(q_feedback)

    PREPARE_DURATION = 4.0
    REPLAY_DURATION = (total_frames * record_dt) / args.speed_scale
    PROTECT_DURATION = 2.0

    t_start = time.perf_counter()
    frame = 0
    cmd_period = 1.0 / args.rate

    try:
        print(f"\n🚀 [重播启动] 任务: {args.task_name}, 速度: {args.speed_scale}x")
        print(f"   准备耗时: {PREPARE_DURATION}s, 重播预计耗时: {REPLAY_DURATION:.2f}s")

        while _running:
            t0 = time.perf_counter()
            elapsed = t0 - t_start
            q_feedback = arm.get_positions(request=True)[:6]

            # 计算速度（如果接口支持，否则置零）
            try:
                q_vel_feedback = arm.get_velocities(request=True)[:6]
            except AttributeError:
                q_vel_feedback = np.zeros(6)

            # 生成目标指令
            if elapsed < PREPARE_DURATION:
                progress = elapsed / PREPARE_DURATION
                smooth = (1.0 - math.cos(progress * math.pi)) / 2.0
                q_target = q_feedback + (qpos_data[0][:6] - q_feedback) * smooth
                stage = "准备"
            elif elapsed < PREPARE_DURATION + REPLAY_DURATION:
                replay_time = elapsed - PREPARE_DURATION
                idx = min(int((replay_time * args.speed_scale) / record_dt), total_frames - 1)
                q_target = qpos_data[idx][:6]
                stage = "重播"
            else:
                print(f"\n✅ [重播完成] 轨迹已结束 (elapsed={elapsed:.2f}s, REPLAY_DURATION={REPLAY_DURATION:.2f}s)")
                break

            # 下发指令
            q_cmd = guard.next_command(q_target, q_feedback)
            arm.pos_vel(q_cmd, vlim=vlim)

            # 增强打印监控
            if frame % args.print_every == 0:
                tracking_error = np.abs(q_cmd - q_feedback)
                info = []
                for i in range(6):
                    info.append(
                        f"J{i+1}: pos {q_feedback[i]*180/np.pi:.1f}° "
                        f"(cmd {q_cmd[i]*180/np.pi:.1f}°) "
                        f"vel {q_vel_feedback[i]:.2f} rad/s "
                        f"err {tracking_error[i]*180/np.pi:.1f}°"
                    )
                print(f"[t={elapsed:.2f}s][{stage}] " + " | ".join(info))

            frame += 1
            # 控制循环周期
            time.sleep(max(0, cmd_period - (time.perf_counter() - t0)))

    except Exception as exc:
        print(f"\n[运行异常] {exc}")
    finally:
        close_arm_fast(arm)
        print("\n[退出] 机械臂已失能。")


if __name__ == "__main__":
    main()

# rebot_position_replay_hjx.py --task_name test_task --dataset_dir ./collected_data/test_task/episode_0.hdf5 --episode_idx 0
