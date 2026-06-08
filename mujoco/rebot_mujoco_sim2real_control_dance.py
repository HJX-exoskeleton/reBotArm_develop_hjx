#!/usr/bin/env python3
"""reBotArm Sim2Real 舞蹈控制部署 (支持纯仿真预览与实机部署)

修复了启动时目标位置阶跃导致的安全停机问题。
状态机新增了“准备阶段”，让机械臂从零位平滑过渡到舞蹈起始姿态。
新增 `--sim-only` 参数，可在不连接真机的情况下纯看仿真效果。

运行示例 (纯仿真预览):
    uv run python mujoco/rebot_mujoco_sim2real_dance.py --sim-only

运行示例 (真机部署):
    uv run python mujoco/rebot_mujoco_sim2real_dance.py --start-from-keyframe --calibrate-current-as-keyframe
"""

from __future__ import annotations

import argparse
import importlib.util
import signal
import sys
import time
import math
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# --------------------------------------------------------------------------- #
# 配置参数
# --------------------------------------------------------------------------- #
ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_XML = ROOT_DIR / "mujoco" / "xml" / "rebot_fixend" / "reBot-DevArm_fixend.xml"
DEFAULT_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))

DEFAULT_CMD_VLIM = np.array([1.5, 1.5, 1.5, 3.0, 3.0, 3.0])
DEFAULT_MAX_STEP = np.array([0.05, 0.05, 0.05, 0.06, 0.06, 0.06])
DEFAULT_SOFT_MARGIN = 0.0
DEFAULT_SETTLE_SAMPLES = 30
DEFAULT_SETTLE_INTERVAL = 0.02
DEFAULT_TRACKING_BREACH_SAMPLES = 20
DEFAULT_VISUAL_MAX_STEP = np.array([0.050, 0.050, 0.050, 0.050, 0.050, 0.050])

_running = True


# --------------------------------------------------------------------------- #
# 安全退出与导入
# --------------------------------------------------------------------------- #
def _sigint_handler(signum, frame) -> None:
    global _running
    print("\n[sim2real] 收到退出信号，准备安全关闭...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)


def _load_robot_arm_class():
    try:
        from reBotArm_control_py.actuator import RobotArm
        return RobotArm
    except ImportError as exc:
        print(f"[导入] 常规导入 RobotArm 失败，改为直接加载 actuator/arm.py: {exc}")

    arm_py = ROOT_DIR / "reBotArm_control_py" / "actuator" / "arm.py"
    spec = importlib.util.spec_from_file_location("_rebotarm_actuator_arm", arm_py)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载 RobotArm: {arm_py}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.RobotArm


def _parse_vector(values: list[float] | None, default: np.ndarray, name: str) -> np.ndarray:
    arr = default.astype(np.float64) if values is None else np.asarray(values, dtype=np.float64)
    if arr.shape != default.shape:
        raise ValueError(f"{name} 必须提供 {default.size} 个数，当前为 {arr.size} 个")
    return arr


# --------------------------------------------------------------------------- #
# MuJoCo 映射与限位核心
# --------------------------------------------------------------------------- #
def _joint_id(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    return jid


def _actuator_id_for_joint(model: mujoco.MjModel, joint_id: int) -> int | None:
    for act_id in range(model.nu):
        if (int(model.actuator_trntype[act_id]) == int(mujoco.mjtTrn.mjTRN_JOINT)
                and int(model.actuator_trnid[act_id, 0]) == joint_id):
            return act_id
    return None


def _clip_rate(target: np.ndarray, previous: np.ndarray, max_step: np.ndarray) -> np.ndarray:
    return previous + np.clip(target - previous, -max_step, max_step)


def _unwrap_near(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    return values + 2.0 * np.pi * np.round((reference - values) / (2.0 * np.pi))


def _sim_to_real_unclipped(q_sim: np.ndarray, signs: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    return (np.asarray(q_sim, dtype=np.float64)[:6] - offsets) / signs


def read_stable_positions(arm, reference: np.ndarray, samples: int, interval: float) -> np.ndarray:
    values = []
    for _ in range(samples):
        q = arm.get_positions(request=True)[: arm.num_joints]
        values.append(_unwrap_near(q, reference[: arm.num_joints]))
        time.sleep(interval)
    return np.median(np.vstack(values), axis=0)


def close_arm_fast(arm) -> None:
    if arm is None:
        return
    try:
        arm.disable(retries=0)
        time.sleep(0.1)
    except Exception:
        pass
    for ctrl in list(getattr(arm, "_ctrl_map", {}).values()):
        try:
            ctrl.shutdown()
            time.sleep(0.02)
            ctrl.close()
        except Exception:
            pass
    getattr(arm, "_ctrl_map", {}).clear()
    getattr(arm, "_motor_map", {}).clear()


class SimToRealMapper:
    def __init__(self, model, joint_names, signs, offsets, soft_margin):
        self.model = model
        self.joint_names = joint_names
        self.signs = signs
        self.offsets = offsets
        self.joint_ids = np.array([_joint_id(model, name) for name in joint_names], dtype=np.int32)
        self.qpos_addrs = np.array([model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.qvel_addrs = np.array([model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.actuator_ids = [_actuator_id_for_joint(model, int(jid)) for jid in self.joint_ids]
        self.visual_q_sim = None

        sim_ranges = np.array([model.jnt_range[jid] for jid in self.joint_ids], dtype=np.float64)
        real_limits = (sim_ranges - offsets[:, None]) / signs[:, None]
        self.real_lower = np.minimum(real_limits[:, 0], real_limits[:, 1]) + soft_margin
        self.real_upper = np.maximum(real_limits[:, 0], real_limits[:, 1]) - soft_margin

    def sim_qpos(self, data: mujoco.MjData) -> np.ndarray:
        return data.qpos[self.qpos_addrs].copy()

    def sim_control_target(self, data: mujoco.MjData) -> np.ndarray:
        q_sim = self.sim_qpos(data)
        for i, act_id in enumerate(self.actuator_ids):
            if act_id is not None:
                q_sim[i] = float(data.ctrl[act_id])
        return q_sim

    def real_to_sim(self, q_real: np.ndarray) -> np.ndarray:
        q_sim = np.asarray(q_real, dtype=np.float64)[: len(self.joint_names)]
        return q_sim * self.signs + self.offsets

    def sim_to_real(self, q_sim: np.ndarray) -> np.ndarray:
        q_real = (np.asarray(q_sim, dtype=np.float64)[: len(self.joint_names)] - self.offsets) / self.signs
        return np.clip(q_real, self.real_lower, self.real_upper)

    def set_sim_pose(self, data: mujoco.MjData, q_sim: np.ndarray) -> None:
        q_sim = np.asarray(q_sim, dtype=np.float64)[: len(self.joint_names)]
        self.visual_q_sim = q_sim.copy()
        data.qpos[self.qpos_addrs] = q_sim
        data.qvel[self.qvel_addrs] = 0.0
        for i, act_id in enumerate(self.actuator_ids):
            if act_id is not None:
                data.ctrl[act_id] = float(q_sim[i])
        mujoco.mj_forward(self.model, data)

    def update_visual_pose(self, data: mujoco.MjData, q_target: np.ndarray, max_step: np.ndarray) -> np.ndarray:
        q_target = np.asarray(q_target, dtype=np.float64)[: len(self.joint_names)]
        if self.visual_q_sim is None:
            self.visual_q_sim = q_target.copy()
        else:
            self.visual_q_sim = _clip_rate(q_target, self.visual_q_sim, max_step)
        self.set_sim_pose(data, self.visual_q_sim)
        return self.visual_q_sim.copy()


class SafetyGuard:
    def __init__(self, mapper, max_step, max_start_error, max_tracking_error, tracking_breach_samples):
        self.mapper = mapper
        self.max_step = max_step
        self.max_start_error = max_start_error
        self.max_tracking_error = max_tracking_error
        self.tracking_breach_samples = max(int(tracking_breach_samples), 1)
        self.command = None
        self._tracking_breach_count = 0

    def initialize(self, q_real_now, q_target, allow_large_start):
        q_real_now = np.asarray(q_real_now, dtype=np.float64)[: len(self.mapper.joint_names)]
        q_target = np.asarray(q_target, dtype=np.float64)[: len(self.mapper.joint_names)]
        start_error = np.max(np.abs(q_target - q_real_now))
        if start_error > self.max_start_error and not allow_large_start:
            raise RuntimeError(f"启动目标与真机位置差距过大: max_error={start_error:.3f} rad。")
        self.command = q_real_now.copy()
        return self.command.copy()

    def next_command(self, q_target, q_feedback):
        q_feedback = np.asarray(q_feedback, dtype=np.float64)[: len(self.mapper.joint_names)]
        q_target = np.clip(q_target, self.mapper.real_lower, self.mapper.real_upper)
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


def reset_to_keyframe(model, data, key_name):
    if model.nkey == 0:
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        return None
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key_name) if key_name else 0
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    return key_id


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="reBotArm Sim2Real 舞蹈控制")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="MuJoCo XML 路径")
    parser.add_argument("--cfg", type=Path, default=None, help="真机 arm.yaml 路径")
    parser.add_argument("--rate", type=float, default=50.0, help="控制周期频率 Hz")
    parser.add_argument("--viewer-rate", type=float, default=60.0, help="Viewer 刷新频率 Hz")
    parser.add_argument("--keyframe", type=str, default=None, help="启动 keyframe")
    parser.add_argument("--start-from-keyframe", action="store_true")
    parser.add_argument("--calibrate-current-as-keyframe", action="store_true")
    parser.add_argument("--allow-large-start", action="store_true")
    parser.add_argument("--max-start-error", type=float, default=0.25)
    parser.add_argument("--max-tracking-error", type=float, default=1.0)
    parser.add_argument("--tracking-breach-samples", type=int, default=DEFAULT_TRACKING_BREACH_SAMPLES)
    parser.add_argument("--soft-margin", type=float, default=DEFAULT_SOFT_MARGIN)
    parser.add_argument("--settle-samples", type=int, default=DEFAULT_SETTLE_SAMPLES)
    parser.add_argument("--settle-interval", type=float, default=DEFAULT_SETTLE_INTERVAL)
    parser.add_argument("--visual-max-step", type=float, nargs=6, default=None)
    parser.add_argument("--print-every", type=int, default=50)
    parser.add_argument("--signs", type=float, nargs=6, default=None)
    parser.add_argument("--offsets", type=float, nargs=6, default=None)
    parser.add_argument("--vlim", type=float, nargs=6, default=None)
    parser.add_argument("--max-step", type=float, nargs=6, default=None)

    # 仿真预览开关
    parser.add_argument("--sim-only", action="store_true", help="仅运行纯仿真模式，不连接物理机械臂")
    return parser


def get_dance_pose(dance_params, num_joints, time_t):
    """计算某个时刻的数学舞蹈目标姿态"""
    pose = np.zeros(num_joints)
    for i in range(num_joints):
        p = dance_params[i]
        pose[i] = p["center"] + p["amp"] * math.sin(2 * math.pi * p["freq"] * time_t + p["phase"])
    return pose


def main() -> None:
    global _running
    args = build_argparser().parse_args()

    signs = _parse_vector(args.signs, np.ones(6), "--signs")
    offsets = _parse_vector(args.offsets, np.zeros(6), "--offsets")
    vlim = _parse_vector(args.vlim, DEFAULT_CMD_VLIM, "--vlim")
    max_step = _parse_vector(args.max_step, DEFAULT_MAX_STEP, "--max-step")
    visual_max_step = _parse_vector(args.visual_max_step, DEFAULT_VISUAL_MAX_STEP, "--visual-max-step")

    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    key_id = reset_to_keyframe(model, data, args.keyframe)

    arm = None
    if not args.sim_only:
        RobotArm = _load_robot_arm_class()
        arm = RobotArm(cfg_path=str(args.cfg) if args.cfg is not None else None)

    frame = 0
    cmd_period = 1.0 / args.rate
    viewer_period = 1.0 / args.viewer_rate
    next_viewer_sync = 0.0

    # 声明 viewer 变量以便在异常时安全关闭
    viewer_obj = None

    try:
        print("=" * 60)
        q_reference = _sim_to_real_unclipped(data.qpos[:6], signs, offsets)

        if not args.sim_only:
            print("  [硬件模式] 正在连接真机与初始化...")
            arm.connect()
            arm.enable()
            arm.mode_pos_vel(vlim=vlim)
            q_feedback = read_stable_positions(arm, q_reference, args.settle_samples, args.settle_interval)
            print(f"[启动] 真实关节反馈(rad): {np.round(q_feedback, 3).tolist()}")
        else:
            print("  [纯仿真模式] 开启 (--sim-only)，已跳过真机连接。")
            q_feedback = q_reference.copy()
            print(f"[启动] 虚拟关节反馈(rad): {np.round(q_feedback, 3).tolist()}")

        if args.calibrate_current_as_keyframe:
            offsets = data.qpos[:6].copy() - signs * q_feedback[:6]
            print(f"[标定] 自动计算 Offsets, 此时的起始姿态即为虚拟 0 位。")

        mapper = SimToRealMapper(model, DEFAULT_JOINT_NAMES, signs, offsets, args.soft_margin)
        guard = SafetyGuard(mapper, max_step, args.max_start_error, args.max_tracking_error,
                            args.tracking_breach_samples)

        if not args.start_from_keyframe:
            mapper.set_sim_pose(data, mapper.real_to_sim(q_feedback))

        q_target = mapper.sim_to_real(mapper.sim_control_target(data))
        q_cmd = guard.initialize(q_feedback, q_target, args.allow_large_start)

        if not args.sim_only:
            q_feedback = _unwrap_near(arm.get_positions(request=True)[: arm.num_joints], q_cmd)
            arm.pos_vel(q_cmd, vlim=vlim)

        # ---------------------------------------------------------
        # 舞蹈轨迹与状态机参数 (根据 max_step 安全限速进行降幅降频处理)
        # J5: Vmax 降至 1.57 rad/s (原为 3.51)
        # J6: Vmax 降至 2.51 rad/s (原为 12.56)
        # ---------------------------------------------------------
        dance_params = [
            {"amp": 1.2, "freq": 0.2, "phase": 0.0, "center": 0.0},
            {"amp": 0.15, "freq": 0.35, "phase": 0.0, "center": -0.56},
            {"amp": 0.15, "freq": 0.35, "phase": np.pi, "center": -0.68},
            {"amp": 0.15, "freq": 0.5, "phase": np.pi / 4, "center": 0.15},
            {"amp": 0.15, "freq": 0.5, "phase": np.pi / 2, "center": 0.0},  # 降频降幅
            {"amp": 0.15, "freq": 0.4, "phase": 0.0, "center": 0.0},  # 降频降幅
        ]
        PREPARE_DURATION = 4.0
        DANCE_DURATION = 15.0
        RETURN_DURATION = 4.0
        dance_end_qpos = None

        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer_obj = viewer
            print(f"\n[{'仿真' if args.sim_only else 'Sim2Real'}] 状态机已启动，开始全自动控制。")
            t_start = time.perf_counter()

            while _running and viewer.is_running():
                t0 = time.perf_counter()
                elapsed = t0 - t_start

                # 1. 获取反馈 (仿真模式下，假定完美跟踪上一周期的物理命令 q_cmd)
                if not args.sim_only:
                    q_feedback = arm.get_positions(request=True)[: arm.num_joints]
                else:
                    q_feedback = q_cmd[:6]

                # 2. 读取基础仿真目标阵列
                q_sim_target = mapper.sim_control_target(data)
                num_controlled_joints = min(6, model.nu)

                # 3. 状态机：注入自动化指令
                if elapsed < PREPARE_DURATION:
                    progress = elapsed / PREPARE_DURATION
                    smooth_progress = (1.0 - math.cos(progress * math.pi)) / 2.0
                    dance_first_pose = get_dance_pose(dance_params, num_controlled_joints, 0.0)
                    for i in range(num_controlled_joints):
                        q_sim_target[i] = 0.0 + (dance_first_pose[i] - 0.0) * smooth_progress

                elif elapsed < PREPARE_DURATION + DANCE_DURATION:
                    dance_time = elapsed - PREPARE_DURATION
                    dance_pose = get_dance_pose(dance_params, num_controlled_joints, dance_time)
                    for i in range(num_controlled_joints):
                        q_sim_target[i] = dance_pose[i]

                elif elapsed < PREPARE_DURATION + DANCE_DURATION + RETURN_DURATION:
                    if dance_end_qpos is None:
                        dance_end_qpos = np.copy(q_sim_target[:6])
                        print(f"\n[状态切换] 舞蹈结束，开始平滑回零...")
                    progress = (elapsed - (PREPARE_DURATION + DANCE_DURATION)) / RETURN_DURATION
                    smooth_progress = (1.0 - math.cos(progress * math.pi)) / 2.0
                    for i in range(num_controlled_joints):
                        q_sim_target[i] = dance_end_qpos[i] + (0.0 - dance_end_qpos[i]) * smooth_progress

                else:
                    if dance_end_qpos is not None:
                        print(f"[状态切换] 已安全归零，保持休息姿态。")
                        dance_end_qpos = None
                    for i in range(num_controlled_joints):
                        q_sim_target[i] = 0.0

                for i, act_id in enumerate(mapper.actuator_ids):
                    if act_id is not None:
                        data.ctrl[act_id] = q_sim_target[i]

                # 4. 经过核心映射与安全守卫
                q_target = mapper.sim_to_real(q_sim_target)
                q_cmd = guard.next_command(q_target, q_feedback)

                # 硬件下发
                if not args.sim_only:
                    arm.pos_vel(q_cmd, vlim=vlim)

                # 5. 更新视觉反馈画面
                q_visual = mapper.update_visual_pose(data, q_sim_target, visual_max_step)

                now = time.perf_counter()
                if now >= next_viewer_sync:
                    viewer.sync()
                    next_viewer_sync = now + viewer_period

                if args.print_every > 0 and frame % (args.print_every * 2) == 0:
                    cmd_str = " ".join(f"{v:+.2f}" for v in q_cmd)
                    fb_str = " ".join(f"{v:+.2f}" for v in q_feedback[:6])
                    print(f"[t={elapsed:05.2f}s] cmd=[{cmd_str}] fb=[{fb_str}]")

                frame += 1
                sleep_time = cmd_period - (time.perf_counter() - t0)
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        _running = False
    except Exception as exc:
        _running = False
        print(f"\n[保护停机] 触发异常: {exc}")
        # 安全捕获渲染器并关闭，避免引发底层的 Segmentation Fault
        if viewer_obj is not None and viewer_obj.is_running():
            viewer_obj.close()
    finally:
        if not args.sim_only:
            print("\n[退出流程] 正在停止真机并关闭通信...")
            close_arm_fast(arm)
        print("[退出流程] 进程已完全关闭。")


if __name__ == "__main__":
    main()