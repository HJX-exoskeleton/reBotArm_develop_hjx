#!/usr/bin/env python3
"""reBotArm MuJoCo sim2real 关节同步控制。

MuJoCo viewer 右侧 control 栏中的 joint1 ~ joint6 position actuator
会被映射为真机目标关节角，真机使用 POS_VEL 位置模式跟踪。脚本默认
启动时将仿真姿态对齐到真机当前姿态，避免从 XML 零位直接拉动真机。

映射关系:
    q_real_cmd = (q_sim - offsets) / signs

运行示例:
    uv run python mujoco/rebot_mujoco_sim2real.py
    uv run python mujoco/rebot_mujoco_sim2real.py --start-from-keyframe --calibrate-current-as-keyframe
    uv run python mujoco/rebot_mujoco_sim2real.py --signs 1 -1 1 1 1 1
"""

from __future__ import annotations

import argparse
import importlib.util
import signal
import sys
import time
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

# 保护参数偏保守；需要更快跟踪时优先调大命令行参数，而不是改硬编码。
# DEFAULT_CMD_VLIM = np.array([0.50, 0.50, 0.50, 0.35, 0.35, 0.35])
DEFAULT_CMD_VLIM = np.array([1.5, 1.5, 1.5, 3.0, 3.0, 3.0])
# DEFAULT_MAX_STEP = np.array([0.020, 0.020, 0.020, 0.015, 0.015, 0.015])
DEFAULT_MAX_STEP = np.array([0.05, 0.05, 0.05, 0.06, 0.06, 0.06])
DEFAULT_SOFT_MARGIN = 0.0
DEFAULT_SETTLE_SAMPLES = 30
DEFAULT_SETTLE_INTERVAL = 0.02
DEFAULT_TRACKING_BREACH_SAMPLES = 20
DEFAULT_VISUAL_MAX_STEP = np.array([0.080, 0.080, 0.080, 0.060, 0.060, 0.060])  # MuJoCo 画面每周期最大变化量 rad，用于增加视觉阻尼

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
    """优先常规导入；失败时绕过顶层包导入，仅加载 actuator/arm.py。"""
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
# MuJoCo 映射与限位
# --------------------------------------------------------------------------- #

def _joint_id(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise KeyError(f"MuJoCo XML 中找不到关节: {joint_name}")
    return jid


def _actuator_id_for_joint(model: mujoco.MjModel, joint_id: int) -> int | None:
    for act_id in range(model.nu):
        if (
            int(model.actuator_trntype[act_id]) == int(mujoco.mjtTrn.mjTRN_JOINT)
            and int(model.actuator_trnid[act_id, 0]) == joint_id
        ):
            return act_id
    return None


def _clip_rate(target: np.ndarray, previous: np.ndarray, max_step: np.ndarray) -> np.ndarray:
    return previous + np.clip(target - previous, -max_step, max_step)


def _unwrap_near(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """将角度按 2pi 等价展开到 reference 附近，兼容单圈/多圈反馈切换。"""
    values = np.asarray(values, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    return values + 2.0 * np.pi * np.round((reference - values) / (2.0 * np.pi))


def _sim_to_real_unclipped(q_sim: np.ndarray, signs: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    return (np.asarray(q_sim, dtype=np.float64)[:6] - offsets) / signs


def read_stable_positions(arm, reference: np.ndarray, samples: int, interval: float) -> np.ndarray:
    """多次读取并按 reference 展开取中位数，滤掉模式切换后的反馈跳变。"""
    values = []
    for _ in range(samples):
        q = arm.get_positions(request=True)[: arm.num_joints]
        values.append(_unwrap_near(q, reference[: arm.num_joints]))
        time.sleep(interval)
    return np.median(np.vstack(values), axis=0)


def close_arm_fast(arm) -> None:
    """快速失能并关闭通信，避免 IDE 停止进程时长时间卡在 disable 轮询。"""
    try:
        arm.disable(retries=0)
        time.sleep(0.1)
    except Exception as exc:
        print(f"[退出流程] 电机失能异常: {exc}")

    for ctrl in list(getattr(arm, "_ctrl_map", {}).values()):
        try:
            ctrl.shutdown()
            time.sleep(0.02)
            ctrl.close()
        except Exception as exc:
            print(f"[退出流程] 控制器关闭异常: {exc}")
    getattr(arm, "_ctrl_map", {}).clear()
    getattr(arm, "_motor_map", {}).clear()


class SimToRealMapper:
    """读取 MuJoCo control/qpos，并转换为真机位置命令。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        joint_names: tuple[str, ...],
        signs: np.ndarray,
        offsets: np.ndarray,
        soft_margin: float,
    ) -> None:
        if np.any(signs == 0.0):
            raise ValueError("--signs 中不能包含 0")

        self.model = model
        self.joint_names = joint_names
        self.signs = signs
        self.offsets = offsets
        self.joint_ids = np.array([_joint_id(model, name) for name in joint_names], dtype=np.int32)
        self.qpos_addrs = np.array([model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.qvel_addrs = np.array([model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.actuator_ids = [_actuator_id_for_joint(model, int(jid)) for jid in self.joint_ids]
        self.visual_q_sim: np.ndarray | None = None

        sim_ranges = np.array([model.jnt_range[jid] for jid in self.joint_ids], dtype=np.float64)
        real_limits = (sim_ranges - offsets[:, None]) / signs[:, None]
        self.real_lower = np.minimum(real_limits[:, 0], real_limits[:, 1]) + soft_margin
        self.real_upper = np.maximum(real_limits[:, 0], real_limits[:, 1]) - soft_margin
        if np.any(self.real_lower >= self.real_upper):
            raise ValueError("软限位 margin 过大，导致可用关节范围为空")

    def sim_qpos(self, data: mujoco.MjData) -> np.ndarray:
        return data.qpos[self.qpos_addrs].copy()

    def sim_control_target(self, data: mujoco.MjData) -> np.ndarray:
        """读取 viewer 右侧 control 栏的目标；无 actuator 时退回 qpos。"""
        q_sim = self.sim_qpos(data)
        for i, act_id in enumerate(self.actuator_ids):
            if act_id is None:
                continue
            ctrl = float(data.ctrl[act_id])
            if self.model.actuator_ctrllimited[act_id]:
                lo, hi = self.model.actuator_ctrlrange[act_id]
                ctrl = float(np.clip(ctrl, lo, hi))
                data.ctrl[act_id] = ctrl
            q_sim[i] = ctrl
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
            if act_id is None:
                continue
            ctrl = float(q_sim[i])
            if self.model.actuator_ctrllimited[act_id]:
                lo, hi = self.model.actuator_ctrlrange[act_id]
                ctrl = float(np.clip(ctrl, lo, hi))
            data.ctrl[act_id] = ctrl
        mujoco.mj_forward(self.model, data)

    def update_visual_pose(self, data: mujoco.MjData, q_target: np.ndarray, max_step: np.ndarray) -> np.ndarray:
        """对显示用关节角做限速平滑，避免画面比真机更“硬”。"""
        q_target = np.asarray(q_target, dtype=np.float64)[: len(self.joint_names)]
        if self.visual_q_sim is None:
            self.visual_q_sim = q_target.copy()
        else:
            self.visual_q_sim = _clip_rate(q_target, self.visual_q_sim, max_step)
        self.set_sim_pose(data, self.visual_q_sim)
        return self.visual_q_sim.copy()


def reset_to_keyframe(model: mujoco.MjModel, data: mujoco.MjData, key_name: str | None) -> int | None:
    if model.nkey == 0:
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        return None

    key_id = 0
    if key_name:
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key_name)
        if key_id < 0:
            raise KeyError(f"MuJoCo XML 中找不到 keyframe: {key_name}")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    return key_id


# --------------------------------------------------------------------------- #
# 保护状态机
# --------------------------------------------------------------------------- #

class SafetyGuard:
    """对 sim2real 命令做限位、步进和跟踪误差保护。

    mapper 输出的是带 offset 的虚拟实机坐标；实际电机反馈可能在任意 2pi
    等价分支上。保护器会先用虚拟坐标检查 XML 限位，再把下发命令展开到
    当前反馈附近，避免 POS_VEL 被多圈角度拉到错误分支。
    """

    def __init__(
        self,
        mapper: SimToRealMapper,
        max_step: np.ndarray,
        max_start_error: float,
        max_tracking_error: float,
        tracking_breach_samples: int,
    ) -> None:
        self.mapper = mapper
        self.max_step = max_step
        self.max_start_error = max_start_error
        self.max_tracking_error = max_tracking_error
        self.tracking_breach_samples = max(int(tracking_breach_samples), 1)
        self.command: np.ndarray | None = None
        self._tracking_breach_count = 0

    def _error_report(self, q_target: np.ndarray, q_feedback: np.ndarray) -> str:
        lines = []
        for name, target, feedback in zip(self.mapper.joint_names, q_target, q_feedback):
            err = target - feedback
            lines.append(f"  {name}: target={target:+.3f}, feedback={feedback:+.3f}, error={err:+.3f}")
        return "\n".join(lines)

    def initialize(self, q_real_now: np.ndarray, q_target: np.ndarray, allow_large_start: bool) -> np.ndarray:
        q_real_now = np.asarray(q_real_now, dtype=np.float64)[: len(self.mapper.joint_names)]
        q_target = np.asarray(q_target, dtype=np.float64)[: len(self.mapper.joint_names)]
        start_error = np.max(np.abs(q_target - q_real_now))
        if start_error > self.max_start_error and not allow_large_start:
            raise RuntimeError(
                f"启动目标与真机当前位置差距过大: max_error={start_error:.3f} rad。"
                "默认已阻止运动。\n"
                f"{self._error_report(q_target, q_real_now)}\n"
                "如果当前真机姿态就是仿真 keyframe 姿态，请使用 --calibrate-current-as-keyframe；"
                "如果确认要让真机主动运动到 keyframe，再加 --allow-large-start。"
            )
        self.command = q_real_now.copy()
        return self.command.copy()

    def next_command(self, q_target: np.ndarray, q_feedback: np.ndarray) -> np.ndarray:
        if self.command is None:
            raise RuntimeError("SafetyGuard 尚未 initialize")

        q_feedback = np.asarray(q_feedback, dtype=np.float64)[: len(self.mapper.joint_names)]
        q_target = np.clip(q_target, self.mapper.real_lower, self.mapper.real_upper)
        q_target_cmd = _unwrap_near(q_target, q_feedback)
        previous_cmd = _unwrap_near(self.command, q_feedback)

        tracking_error = np.max(np.abs(q_target_cmd - q_feedback))
        if tracking_error > self.max_tracking_error:
            self._tracking_breach_count += 1
            if self._tracking_breach_count >= self.tracking_breach_samples:
                error_report = self._error_report(q_target_cmd, q_feedback)
                raise RuntimeError(
                    f"真机跟踪误差过大: max_error={tracking_error:.3f} rad, "
                    f"连续超限={self._tracking_breach_count} 次，已触发保护停机。\n"
                    f"{error_report}"
                )
        else:
            self._tracking_breach_count = 0

        self.command = _clip_rate(q_target_cmd, previous_cmd, self.max_step)
        return self.command.copy()


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #

def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="reBotArm MuJoCo sim2real 关节同步控制")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="MuJoCo XML 路径")
    parser.add_argument("--cfg", type=Path, default=None, help="真机 arm.yaml 路径，默认使用库内 config/arm.yaml")
    parser.add_argument("--rate", type=float, default=50.0, help="真机命令发送频率 Hz")
    parser.add_argument("--viewer-rate", type=float, default=60.0, help="MuJoCo viewer 刷新频率 Hz")
    parser.add_argument("--keyframe", type=str, default=None, help="启动时使用的 MuJoCo keyframe 名称，默认使用第 0 个 key")
    parser.add_argument("--start-from-keyframe", action="store_true", help="从 XML keyframe 目标开始控制真机；默认仿真先对齐真机当前位置")
    parser.add_argument("--calibrate-current-as-keyframe", action="store_true", help="将当前真机反馈标定为当前 MuJoCo qpos/keyframe，不改电机零点，仅自动计算本次 offsets")
    parser.add_argument("--allow-large-start", action="store_true", help="允许启动时目标与真机当前位置差距超过保护阈值")
    parser.add_argument("--max-start-error", type=float, default=0.25, help="启动最大允许关节误差 rad")
    parser.add_argument("--max-tracking-error", type=float, default=0.8, help="运行中最大允许跟踪误差 rad")
    parser.add_argument("--tracking-breach-samples", type=int, default=DEFAULT_TRACKING_BREACH_SAMPLES, help="跟踪误差连续超限多少次后停机")
    parser.add_argument("--soft-margin", type=float, default=DEFAULT_SOFT_MARGIN, help="相对 XML 限位收缩的安全边界 rad")
    parser.add_argument("--settle-samples", type=int, default=DEFAULT_SETTLE_SAMPLES, help="启动标定前反馈稳定采样次数")
    parser.add_argument("--settle-interval", type=float, default=DEFAULT_SETTLE_INTERVAL, help="启动标定前每次采样间隔 s")
    parser.add_argument("--visual-max-step", type=float, nargs=6, default=None, help="MuJoCo 画面每周期最大变化量 rad，用于增加视觉阻尼")
    parser.add_argument("--print-every", type=int, default=50, help="每 N 个控制周期打印一次状态，0 表示不打印")
    parser.add_argument("--signs", type=float, nargs=6, default=None, help="6 个关节方向系数")
    parser.add_argument("--offsets", type=float, nargs=6, default=None, help="6 个关节零点偏置，单位 rad")
    parser.add_argument("--vlim", type=float, nargs=6, default=None, help="POS_VEL 每轴速度限制 rad/s")
    parser.add_argument("--max-step", type=float, nargs=6, default=None, help="每个控制周期最大命令变化 rad")
    return parser


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

    RobotArm = _load_robot_arm_class()
    arm = RobotArm(cfg_path=str(args.cfg) if args.cfg is not None else None)

    print("=" * 60)
    print("  reBotArm Sim2Real: MuJoCo qpos -> 真机 POS_VEL")
    print("=" * 60)
    print(f"[MuJoCo] XML: {args.xml}, keyframe id={key_id}")

    frame = 0
    cmd_period = 1.0 / args.rate
    viewer_period = 1.0 / args.viewer_rate
    next_viewer_sync = 0.0

    try:
        print("[真机] 初始化控制器...")
        arm.connect()
        arm.enable()
        arm.mode_pos_vel(vlim=vlim)

        q_reference = _sim_to_real_unclipped(data.qpos[:6], signs, offsets)
        print("[启动] 正在采集稳定反馈用于标定/初始化...")
        q_feedback = read_stable_positions(
            arm,
            reference=q_reference,
            samples=max(args.settle_samples, 1),
            interval=max(args.settle_interval, 0.0),
        )
        print(f"[启动] 稳定反馈(rad)={q_feedback.tolist()}")

        if args.calibrate_current_as_keyframe:
            offsets = data.qpos[:6].copy() - signs * q_feedback[:6]
            print("[标定] 已将当前真机姿态作为当前 MuJoCo qpos/keyframe。")
            print(f"[标定] 本次自动 offsets(rad)={offsets.tolist()}")

        mapper = SimToRealMapper(model, DEFAULT_JOINT_NAMES, signs, offsets, args.soft_margin)
        guard = SafetyGuard(
            mapper,
            max_step,
            args.max_start_error,
            args.max_tracking_error,
            args.tracking_breach_samples,
        )
        print(f"[映射] signs={signs.tolist()}, offsets(rad)={offsets.tolist()}")
        print(f"[保护] real_lower={mapper.real_lower.round(3).tolist()}")
        print(f"[保护] real_upper={mapper.real_upper.round(3).tolist()}")
        print(
            f"[保护] vlim={vlim.tolist()}, max_step={max_step.tolist()}, "
            f"tracking_breach_samples={args.tracking_breach_samples}"
        )
        print(f"[视觉] visual_max_step={visual_max_step.tolist()}")

        if not args.start_from_keyframe:
            mapper.set_sim_pose(data, mapper.real_to_sim(q_feedback))
            print("[启动] 已将 MuJoCo 姿态对齐到真机当前位置。")

        q_target = mapper.sim_to_real(mapper.sim_control_target(data))
        q_cmd = guard.initialize(q_feedback, q_target, args.allow_large_start)
        q_feedback = _unwrap_near(
            arm.get_positions(request=True)[: arm.num_joints],
            q_cmd,
        )
        q_cmd = guard.next_command(q_target, q_feedback)
        arm.pos_vel(q_cmd, vlim=vlim)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("[MuJoCo] viewer 已启动。拖动右侧 control 栏，真机会按保护限制跟踪。")
            while _running and viewer.is_running():
                t0 = time.perf_counter()

                q_feedback = arm.get_positions(request=True)[: arm.num_joints]
                q_sim_target = mapper.sim_control_target(data)
                q_target = mapper.sim_to_real(q_sim_target)
                q_cmd = guard.next_command(q_target, q_feedback)
                arm.pos_vel(q_cmd, vlim=vlim)

                # control 栏修改的是 data.ctrl；这里让画面以限速方式跟随，增加阻尼感。
                q_visual = mapper.update_visual_pose(data, q_sim_target, visual_max_step)

                now = time.perf_counter()
                if now >= next_viewer_sync:
                    viewer.sync()
                    next_viewer_sync = now + viewer_period

                if args.print_every > 0 and frame % args.print_every == 0:
                    target_str = " ".join(f"{v:+.3f}" for v in q_target)
                    cmd_str = " ".join(f"{v:+.3f}" for v in q_cmd)
                    fb_str = " ".join(f"{v:+.3f}" for v in q_feedback[:6])
                    sim_str = " ".join(f"{v:+.3f}" for v in q_visual)
                    print(f"[{frame:06d}] target=[{target_str}] cmd=[{cmd_str}] fb=[{fb_str}] sim=[{sim_str}]")

                frame += 1
                sleep_time = cmd_period - (time.perf_counter() - t0)
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        _running = False
    except Exception as exc:
        _running = False
        print(f"\n[保护停机] {exc}")
    finally:
        print("\n[退出流程] 正在停止真机并关闭通信...")
        close_arm_fast(arm)
        print("[退出流程] sim2real 已关闭。")


if __name__ == "__main__":
    main()

# uv run python rebot_mujoco_sim2real_position_setting.py --start-from-keyframe --calibrate-current-as-keyframe
