#!/usr/bin/env python3
"""reBotArm 重力补偿 + MuJoCo real2sim 数字孪生。

真机保持与 /gravity_compensation/rebot_gravity_compensation_control.py 相同的 MIT 重力补偿模式；
MuJoCo viewer 读取真机反馈关节角并实时同步显示。

映射关系:
    q_sim = clip(q_real * signs + offsets, mujoco_joint_range)

运行示例:
    uv run python mujoco/rebot_gravity_compensation_mujoco_real2sim.py
    uv run python mujoco/rebot_gravity_compensation_mujoco_real2sim.py --signs 1 -1 1 1 1 1
"""

from __future__ import annotations

import argparse
import importlib.util
import signal
import sys
import threading
import time
from pathlib import Path

import mujoco   # pip install mujoco==3.3.0
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


# --------------------------------------------------------------------------- #
# 配置参数
# --------------------------------------------------------------------------- #

ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_XML = ROOT_DIR / "mujoco" / "xml" / "rebot_fixend" / "reBot-DevArm_fixend.xml"
DEFAULT_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))

# 与 9_gravity_compensation_hjx.py 保持一致的重力补偿参数。
TORQUE_LIMITS = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
KD_CONFIG = np.array([1.0, 2.0, 1.5, 1.0, 0.8, 0.6])
GRAVITY_SCALES = np.array([1.50, 0.75, 0.75, 0.75, 1.0, 1.0])

_running = True


# --------------------------------------------------------------------------- #
# 安全退出与延迟导入
# --------------------------------------------------------------------------- #

def _sigint_handler(signum, frame) -> None:
    global _running
    print("\n[real2sim] 收到退出信号，准备安全关闭...")
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


def _load_gravity_functions():
    """延迟加载动力学函数，避免 --help 阶段提前加载 Pinocchio。"""
    from reBotArm_control_py.dynamics import (
        load_dynamics_model,
        compute_generalized_gravity,
    )

    return load_dynamics_model, compute_generalized_gravity


def _parse_vector(values: list[float] | None, default: np.ndarray, name: str) -> np.ndarray:
    arr = default.astype(np.float64) if values is None else np.asarray(values, dtype=np.float64)
    if arr.shape != default.shape:
        raise ValueError(f"{name} 必须提供 {default.size} 个数，当前为 {arr.size} 个")
    return arr


# --------------------------------------------------------------------------- #
# MuJoCo 关节映射
# --------------------------------------------------------------------------- #

def _joint_id(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise KeyError(f"MuJoCo XML 中找不到关节: {joint_name}")
    return jid


def _actuator_id_for_joint(model: mujoco.MjModel, joint_id: int) -> int | None:
    """查找绑定到指定 joint 的 position actuator。"""
    for act_id in range(model.nu):
        if (
            int(model.actuator_trntype[act_id]) == int(mujoco.mjtTrn.mjTRN_JOINT)
            and int(model.actuator_trnid[act_id, 0]) == joint_id
        ):
            return act_id
    return None


class RealToSimMapper:
    """将真机关节反馈写入 MuJoCo 的 qpos 和 position actuator ctrl。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        joint_names: tuple[str, ...],
        signs: np.ndarray,
        offsets: np.ndarray,
        clamp: bool = True,
    ) -> None:
        self.model = model
        self.joint_names = joint_names
        self.signs = signs
        self.offsets = offsets
        self.clamp = clamp

        self.joint_ids = np.array(
            [_joint_id(model, name) for name in joint_names],
            dtype=np.int32,
        )
        self.qpos_addrs = np.array([model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.qvel_addrs = np.array([model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.ranges = np.array([model.jnt_range[jid] for jid in self.joint_ids], dtype=np.float64)
        self.actuator_ids = [_actuator_id_for_joint(model, int(jid)) for jid in self.joint_ids]

    def real_to_sim(self, q_real: np.ndarray) -> np.ndarray:
        q_sim = np.asarray(q_real, dtype=np.float64)[: len(self.joint_names)]
        q_sim = q_sim * self.signs + self.offsets
        if self.clamp:
            q_sim = np.clip(q_sim, self.ranges[:, 0], self.ranges[:, 1])
        return q_sim

    def apply(self, data: mujoco.MjData, q_real: np.ndarray, dq_real: np.ndarray | None = None) -> np.ndarray:
        q_sim = self.real_to_sim(q_real)
        data.qpos[self.qpos_addrs] = q_sim

        if dq_real is not None:
            dq_sim = np.asarray(dq_real, dtype=np.float64)[: len(self.joint_names)] * self.signs
            data.qvel[self.qvel_addrs] = dq_sim
        else:
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
        return q_sim


# --------------------------------------------------------------------------- #
# 真机重力补偿控制
# --------------------------------------------------------------------------- #

class GravityCompensationState:
    """真机控制线程与 MuJoCo viewer 线程之间的反馈缓存。"""

    def __init__(self, num_joints: int) -> None:
        self._lock = threading.Lock()
        self.q = np.zeros(num_joints, dtype=np.float64)
        self.dq = np.zeros(num_joints, dtype=np.float64)
        self.tau_g = np.zeros(num_joints, dtype=np.float64)
        self.counter = 0
        self.has_feedback = False

    def update(self, q: np.ndarray, dq: np.ndarray, tau_g: np.ndarray) -> None:
        with self._lock:
            self.q = np.asarray(q, dtype=np.float64).copy()
            self.dq = np.asarray(dq, dtype=np.float64).copy()
            self.tau_g = np.asarray(tau_g, dtype=np.float64).copy()
            self.counter += 1
            self.has_feedback = True

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, bool]:
        with self._lock:
            return (
                self.q.copy(),
                self.dq.copy(),
                self.tau_g.copy(),
                self.counter,
                self.has_feedback,
            )


def make_gravity_compensation_controller(state: GravityCompensationState, compute_generalized_gravity):
    """生成 RobotArm.start_control_loop 使用的重力补偿回调。"""
    def controller(arm, dt: float) -> None:
        if not _running:
            return

        q, dq, _ = arm.get_state()
        q_arm = q[: arm.num_joints]
        dq_arm = dq[: arm.num_joints]

        tau_g_raw = compute_generalized_gravity(q=q_arm[:6])
        tau_g = tau_g_raw * GRAVITY_SCALES[: arm.num_joints]
        tau_g_safe = np.clip(
            tau_g,
            -TORQUE_LIMITS[: arm.num_joints],
            TORQUE_LIMITS[: arm.num_joints],
        )

        arm.mit(
            pos=q_arm,
            vel=np.zeros(arm.num_joints),
            kp=np.zeros(arm.num_joints),
            kd=KD_CONFIG[: arm.num_joints],
            tau=tau_g_safe,
            request_feedback=True,
        )
        state.update(q_arm, dq_arm, tau_g_safe)

    return controller


# --------------------------------------------------------------------------- #
# MuJoCo 初始化
# --------------------------------------------------------------------------- #

def reset_to_keyframe(model: mujoco.MjModel, data: mujoco.MjData, key_name: str | None) -> int | None:
    """重置到 XML keyframe，并同步 position actuator 的 ctrl 初值。"""
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
    for act_id in range(model.nu):
        joint_id = int(model.actuator_trnid[act_id, 0])
        if int(model.actuator_trntype[act_id]) != int(mujoco.mjtTrn.mjTRN_JOINT) or joint_id < 0:
            continue

        qpos_addr = int(model.jnt_qposadr[joint_id])
        ctrl = float(data.qpos[qpos_addr])
        if model.actuator_ctrllimited[act_id]:
            lo, hi = model.actuator_ctrlrange[act_id]
            ctrl = float(np.clip(ctrl, lo, hi))
        data.ctrl[act_id] = ctrl
    mujoco.mj_forward(model, data)
    return key_id


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="reBotArm 真机到 MuJoCo 数字孪生映射")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="MuJoCo XML 路径")
    parser.add_argument("--cfg", type=Path, default=None, help="真机 arm.yaml 路径，默认使用库内 config/arm.yaml")
    parser.add_argument("--rate", type=float, default=50.0, help="MuJoCo viewer 刷新频率 Hz")
    parser.add_argument("--control-rate", type=float, default=None, help="真机重力补偿控制频率 Hz，默认使用 arm.yaml rate")
    parser.add_argument("--print-every", type=int, default=60, help="每 N 帧打印一次关节状态，0 表示不打印")
    parser.add_argument("--no-clamp", action="store_true", help="不按 MuJoCo 关节范围限幅")
    parser.add_argument("--keyframe", type=str, default=None, help="启动时使用的 MuJoCo keyframe 名称，默认使用第 0 个 key")
    parser.add_argument("--signs", type=float, nargs=6, default=None, help="6 个关节方向系数")
    parser.add_argument("--offsets", type=float, nargs=6, default=None, help="6 个关节零点偏置，单位 rad")
    return parser


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #

def main() -> None:
    global _running

    args = build_argparser().parse_args()
    joint_names = DEFAULT_JOINT_NAMES
    signs = _parse_vector(args.signs, np.ones(6), "--signs")
    offsets = _parse_vector(args.offsets, np.zeros(6), "--offsets")

    print("=" * 60)
    print("  reBotArm Real2Sim: 真机关节 -> MuJoCo 数字孪生")
    print("=" * 60)
    print(f"[MuJoCo] XML: {args.xml}")
    print(f"[映射] joints={joint_names}")
    print(f"[映射] signs={signs.tolist()}, offsets(rad)={offsets.tolist()}")

    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    key_id = reset_to_keyframe(model, data, args.keyframe)
    if key_id is None:
        print("[MuJoCo] XML 未定义 keyframe，已使用默认 data reset。")
    else:
        print(f"[MuJoCo] 已应用 keyframe id={key_id}, qpos={data.qpos[:6].tolist()}")

    mapper = RealToSimMapper(
        model=model,
        joint_names=joint_names,
        signs=signs,
        offsets=offsets,
        clamp=not args.no_clamp,
    )

    RobotArm = _load_robot_arm_class()
    load_dynamics_model, compute_generalized_gravity = _load_gravity_functions()
    load_dynamics_model()

    arm = RobotArm(cfg_path=str(args.cfg) if args.cfg is not None else None)
    state = GravityCompensationState(arm.num_joints)
    frame = 0
    period = 1.0 / args.rate

    try:
        print("[真机] 初始化控制器...")
        arm.connect()
        arm.enable()
        arm.mode_mit(kp=np.zeros(arm.num_joints), kd=KD_CONFIG[: arm.num_joints])
        arm.start_control_loop(
            make_gravity_compensation_controller(state, compute_generalized_gravity),
            rate=args.control_rate if args.control_rate is not None else arm._rate,
        )
        print("[真机] 已启动 MIT 重力补偿模式；拖拽真机时，MuJoCo 将同步显示反馈关节角。")

        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("[MuJoCo] viewer 已启动，关闭窗口或 Ctrl+C 退出。")
            while _running and viewer.is_running():
                t0 = time.perf_counter()

                q_real, dq_real, tau_g, counter, has_feedback = state.snapshot()
                if has_feedback:
                    q_sim = mapper.apply(data, q_real, dq_real)
                else:
                    q_sim = data.qpos[mapper.qpos_addrs].copy()

                viewer.sync()

                if has_feedback and args.print_every > 0 and frame % args.print_every == 0:
                    q_real_str = " ".join(f"{v:+.3f}" for v in q_real[:6])
                    q_sim_str = " ".join(f"{v:+.3f}" for v in q_sim)
                    tau_str = " ".join(f"{v:+.2f}" for v in tau_g[:6])
                    print(
                        f"[{frame:06d}/{counter:06d}] "
                        f"q_real=[{q_real_str}]  q_sim=[{q_sim_str}]  tau_g=[{tau_str}]"
                    )

                frame += 1
                sleep_time = period - (time.perf_counter() - t0)
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        _running = False
    finally:
        print("\n[退出流程] 正在停止真机并关闭通信...")
        try:
            arm.stop_control_loop()
            arm.disable()
            time.sleep(0.2)
        except Exception as exc:
            print(f"[退出流程] 电机失能异常: {exc}")
        try:
            arm.disconnect()
        except Exception as exc:
            print(f"[退出流程] 通信断开异常: {exc}")
        print("[退出流程] real2sim 已关闭。")


if __name__ == "__main__":
    main()
