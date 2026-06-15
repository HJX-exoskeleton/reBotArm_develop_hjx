#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机主手 -> 达妙真机机械臂 + 达妙真机夹爪 遥操作程序

控制链路：
    ID1~ID6 ST/SMS_STS 舵机主手
        -> 舵机角度 deg
        -> MuJoCo 标准关节空间 q_sim(rad)
        -> 达妙真机 6 轴 q_real(rad)
        -> SafetyGuard 限位/限速/跟踪误差保护
        -> RobotArm.pos_vel(q_cmd, vlim)

    ID7 ST/SMS_STS 舵机夹爪
        -> 舵机角度 deg
        -> gripper_norm 0~1
        -> 达妙夹爪真实目标角度 rad
        -> gripper_motor.send_mit(target_pos, 0, kp, kd, tau)

关键修复：
    真机夹爪不是普通 RobotArm.set_gripper() 控制。
    必须参考 real2sim_gravity_compensation_grasp.py：
        1. load_gripper_cfg(gripper.yaml)
        2. shared_damiao_controller.add_damiao_motor(...)
        3. arm._motor_map[g_cfg.name] = g_mot
        4. g_mot.ensure_mode(Mode.MIT, 1000)
        5. shared_damiao_controller.enable_all()
        6. g_mot.send_mit(...) 控制夹爪

推荐运行：
    python servo_arm_teleoperation_real_with_gripper.py \
      --xml /media/hjx/PSSD/hjx_ws/rebot_arm_servo_7dof/Python/Servo_control/xml/rebot_gripper/sim_reBot_grasp.xml \
      --port /dev/ttyUSB0 \
      --baudrate 115200 \
      --rate 50 \
      --read-rate 60 \
      --calibrate-current-as-master
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import signal
import sys
import threading
import time
from pathlib import Path

import mujoco
import numpy as np


# =============================================================================
# 0. 全局运行标志
# =============================================================================

_running = True


def _sigint_handler(signum, frame) -> None:
    global _running
    print("\n[teleop] 收到退出信号，准备安全关闭...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)


# =============================================================================
# 1. 路径发现与动态导入
# =============================================================================

CURRENT_DIR = Path(__file__).resolve().parent


def _candidate_roots() -> list[Path]:
    roots: list[Path] = []

    for p in [CURRENT_DIR, *CURRENT_DIR.parents]:
        roots.append(p)

    cwd = Path.cwd().resolve()

    for p in [cwd, *cwd.parents]:
        roots.append(p)

    unique_roots = []
    seen = set()

    for r in roots:
        if r not in seen:
            unique_roots.append(r)
            seen.add(r)

    return unique_roots


def _find_first_existing(relative_paths: list[str]) -> Path | None:
    for root in _candidate_roots():
        for rel in relative_paths:
            p = root / rel
            if p.exists():
                return p
    return None


def _inject_paths() -> tuple[Path | None, Path | None]:
    sdk_dir = _find_first_existing(["STservo_sdk", "Python/STservo_sdk"])
    robot_pkg_dir = _find_first_existing(["reBotArm_control_py", "Python/reBotArm_control_py"])

    add_paths: list[Path] = []

    if sdk_dir is not None:
        add_paths.append(sdk_dir)
        add_paths.append(sdk_dir.parent)

    if robot_pkg_dir is not None:
        add_paths.append(robot_pkg_dir.parent)

    add_paths.extend(_candidate_roots()[:5])

    for p in add_paths:
        sp = str(p)
        if sp not in sys.path:
            sys.path.insert(0, sp)

    return sdk_dir, robot_pkg_dir


SDK_DIR, ROBOT_PKG_DIR = _inject_paths()

try:
    from STservo_sdk import *  # noqa: F401,F403
except Exception as exc:
    print("❌ 无法导入 STservo_sdk。")
    print(f"当前脚本目录: {CURRENT_DIR}")
    print(f"自动发现 SDK_DIR: {SDK_DIR}")
    raise exc


def _load_robot_arm_class():
    try:
        from reBotArm_control_py.actuator import RobotArm
        return RobotArm
    except ImportError as exc:
        print(f"[导入] 常规导入 RobotArm 失败，尝试直接加载 arm.py: {exc}")

    possible_arm_py: list[Path] = []

    if ROBOT_PKG_DIR is not None:
        possible_arm_py.append(ROBOT_PKG_DIR / "actuator" / "arm.py")

    for root in _candidate_roots():
        possible_arm_py.append(root / "reBotArm_control_py" / "actuator" / "arm.py")
        possible_arm_py.append(root / "Python" / "reBotArm_control_py" / "actuator" / "arm.py")

    for arm_py in possible_arm_py:
        if arm_py.exists():
            spec = importlib.util.spec_from_file_location("_rebotarm_actuator_arm", arm_py)

            if spec is None or spec.loader is None:
                continue

            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            return module.RobotArm

    raise ImportError("无法加载 RobotArm，请检查 reBotArm_control_py/actuator/arm.py 是否存在。")


def _load_gripper_cfg_func():
    try:
        from reBotArm_control_py.actuator.gripper import load_cfg
        return load_cfg
    except ImportError:
        pass

    possible_gripper_py: list[Path] = []

    if ROBOT_PKG_DIR is not None:
        possible_gripper_py.append(ROBOT_PKG_DIR / "actuator" / "gripper.py")

    for root in _candidate_roots():
        possible_gripper_py.append(root / "reBotArm_control_py" / "actuator" / "gripper.py")
        possible_gripper_py.append(root / "Python" / "reBotArm_control_py" / "actuator" / "gripper.py")

    for gripper_py in possible_gripper_py:
        if gripper_py.exists():
            spec = importlib.util.spec_from_file_location("_rebotarm_actuator_gripper", gripper_py)

            if spec is None or spec.loader is None:
                continue

            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            return module.load_cfg

    raise ImportError("无法加载 gripper.py 中的 load_cfg，请检查 reBotArm_control_py/actuator/gripper.py。")


# =============================================================================
# 2. 默认路径与参数
# =============================================================================

def _default_xml_path() -> Path:
    candidates = [
        "mujoco/xml/rebot_gripper/sim_reBot_grasp.xml",
        "Python/Servo_control/xml/rebot_gripper/sim_reBot_grasp.xml",
        "Servo_control/xml/rebot_gripper/sim_reBot_grasp.xml",
        "mujoco/xml/rebot_fixend/reBot-DevArm_fixend.xml",
        "Python/mujoco/xml/rebot_fixend/reBot-DevArm_fixend.xml",
    ]

    p = _find_first_existing(candidates)

    if p is not None:
        return p

    return CURRENT_DIR / "xml" / "rebot_gripper" / "sim_reBot_grasp.xml"


def _default_gripper_cfg_path() -> Path:
    candidates = [
        "config/gripper.yaml",
        "Python/config/gripper.yaml",
    ]

    p = _find_first_existing(candidates)

    if p is not None:
        return p

    return CURRENT_DIR / "config" / "gripper.yaml"


DEFAULT_XML = _default_xml_path()
DEFAULT_GRIPPER_CFG = _default_gripper_cfg_path()

if os.name == "nt":
    DEFAULT_SERVO_PORT = "COM6"
else:
    DEFAULT_SERVO_PORT = "/dev/ttyUSB0"


# =============================================================================
# 3. 舵机主手与夹爪参数
# =============================================================================

ARM_SERVO_IDS = [1, 2, 3, 4, 5, 6]
GRIPPER_SERVO_ID = 7
ARM_DOF = len(ARM_SERVO_IDS)

SERVO_DIGITAL_RANGE = 4095.0
SERVO_ANGLE_RANGE = 360.0

JOINT_LIMITS_DEG = {
    1: {"min_deg": 50.0, "max_deg": 300.0, "home_deg": 180.0},
    2: {"min_deg": 10.0, "max_deg": 180.0, "home_deg": 180.0},
    3: {"min_deg": 22.0, "max_deg": 180.0, "home_deg": 180.0},
    4: {"min_deg": 100.0, "max_deg": 270.0, "home_deg": 180.0},
    5: {"min_deg": 90.0, "max_deg": 270.0, "home_deg": 180.0},
    6: {"min_deg": 90.0, "max_deg": 270.0, "home_deg": 180.0},

    # ID7 夹爪主手舵机
    7: {"min_deg": 90.0, "max_deg": 180.0, "home_deg": 180.0},
}

SAFETY_MARGIN_DEG = 0.0

DEFAULT_SERVO_TO_SIM_SIGN = np.array(
    [-1.0, 1.0, 1.0, -1.0, -1.0, -1.0],
    dtype=np.float64,
)

DEFAULT_SIM_HOME_RAD = np.zeros(6, dtype=np.float64)

# ID7 主手角度标定
GRIPPER_SERVO_CLOSED_DEG = 90.0
GRIPPER_SERVO_OPEN_DEG = 180.0

# 达妙真机夹爪角度标定，来自参考代码：
# 0.2 rad 闭合，-5.8 rad 张开
DEFAULT_GRIPPER_REAL_CLOSED_RAD = 0.2
DEFAULT_GRIPPER_REAL_OPEN_RAD = -5.8


# =============================================================================
# 4. 基础工具函数
# =============================================================================

def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(float(min_val), min(float(val), float(max_val)))


def servo_pos_to_deg(pos: int | float) -> float:
    return float(pos) / SERVO_DIGITAL_RANGE * SERVO_ANGLE_RANGE


def deg_to_rad(deg: float) -> float:
    return float(deg) * np.pi / 180.0


def limit_servo_deg(servo_id: int, angle_deg: float) -> float:
    cfg = JOINT_LIMITS_DEG[servo_id]
    min_deg = cfg["min_deg"] + SAFETY_MARGIN_DEG
    max_deg = cfg["max_deg"] - SAFETY_MARGIN_DEG
    return clamp(float(angle_deg), min_deg, max_deg)


def servo_deg_to_sim_rad(
    servo_id: int,
    angle_deg: float,
    arm_index: int,
    servo_to_sim_sign: np.ndarray,
    sim_home_rad: np.ndarray,
) -> float:
    cfg = JOINT_LIMITS_DEG[servo_id]
    home_deg = cfg["home_deg"]

    safe_deg = limit_servo_deg(servo_id, angle_deg)
    delta_deg = safe_deg - home_deg

    q_sim = sim_home_rad[arm_index] + servo_to_sim_sign[arm_index] * deg_to_rad(delta_deg)

    return float(q_sim)


def servo_deg_array_to_sim_rad(
    arm_deg_array: np.ndarray,
    servo_to_sim_sign: np.ndarray,
    sim_home_rad: np.ndarray,
) -> np.ndarray:
    q_sim = np.zeros(ARM_DOF, dtype=np.float64)

    for i, servo_id in enumerate(ARM_SERVO_IDS):
        q_sim[i] = servo_deg_to_sim_rad(
            servo_id=servo_id,
            angle_deg=float(arm_deg_array[i]),
            arm_index=i,
            servo_to_sim_sign=servo_to_sim_sign,
            sim_home_rad=sim_home_rad,
        )

    return q_sim


def gripper_servo_deg_to_norm(angle_deg: float, invert_gripper: bool) -> float:
    """
    ID7 舵机角度 -> 夹爪归一化开合量

    默认：
        90°  -> 0.0 闭合
        180° -> 1.0 张开
    """
    angle_deg = limit_servo_deg(GRIPPER_SERVO_ID, angle_deg)

    denom = GRIPPER_SERVO_OPEN_DEG - GRIPPER_SERVO_CLOSED_DEG

    if abs(denom) < 1e-9:
        norm = 0.0
    else:
        norm = (angle_deg - GRIPPER_SERVO_CLOSED_DEG) / denom

    norm = clamp(norm, 0.0, 1.0)

    if invert_gripper:
        norm = 1.0 - norm

    return float(norm)


def gripper_norm_to_real_rad(
    norm: float,
    closed_rad: float,
    open_rad: float,
) -> float:
    """
    归一化开合量 -> 达妙真机夹爪目标角度 rad

    默认：
        norm=0.0 -> 0.0 rad 闭合
        norm=1.0 -> -5.8 rad 张开
    """
    norm = clamp(norm, 0.0, 1.0)
    target = float(closed_rad) + norm * (float(open_rad) - float(closed_rad))

    lower = min(float(closed_rad), float(open_rad))
    upper = max(float(closed_rad), float(open_rad))

    return clamp(target, lower, upper)


def smooth_update(prev: np.ndarray, target: np.ndarray, alpha: float) -> np.ndarray:
    alpha = clamp(float(alpha), 0.0, 1.0)
    return alpha * target + (1.0 - alpha) * prev


def smooth_update_scalar(prev: float, target: float, alpha: float) -> float:
    alpha = clamp(float(alpha), 0.0, 1.0)
    return float(alpha * target + (1.0 - alpha) * prev)


def read_servo_angle(scs, servo_id: int, last_angle: float) -> tuple[float, bool]:
    try:
        pos, speed, result, error = scs.ReadPosSpeed(servo_id)

        if result == COMM_SUCCESS:
            angle_deg = servo_pos_to_deg(pos)
            angle_deg = limit_servo_deg(servo_id, angle_deg)
            return float(angle_deg), True

        return float(last_angle), False

    except Exception:
        return float(last_angle), False


def release_servo_torque(scs, servo_ids: list[int]) -> None:
    print("\n🔓 正在释放舵机主手力矩，用于手动拖动遥操作...")

    for servo_id in servo_ids:
        try:
            result, error = scs.write1ByteTxRx(
                servo_id,
                STS_TORQUE_ENABLE,
                0,
            )

            if result == COMM_SUCCESS:
                print(f"✅ 主手 ID={servo_id} 力矩已释放")
            else:
                print(f"⚠️ 主手 ID={servo_id} 力矩释放失败: {scs.getTxRxResult(result)}")

        except Exception as e:
            print(f"⚠️ 主手 ID={servo_id} 力矩释放异常: {e}")

        time.sleep(0.02)


# =============================================================================
# 5. MuJoCo 标准空间 -> 达妙真机空间
# =============================================================================

DEFAULT_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))

DEFAULT_CMD_VLIM = np.array([0.8, 0.8, 0.8, 1.2, 1.2, 1.2], dtype=np.float64)
DEFAULT_MAX_STEP = np.array([0.015, 0.015, 0.015, 0.020, 0.020, 0.020], dtype=np.float64)

DEFAULT_SOFT_MARGIN = 0.0
DEFAULT_SETTLE_SAMPLES = 30
DEFAULT_SETTLE_INTERVAL = 0.02
DEFAULT_TRACKING_BREACH_SAMPLES = 20


def _parse_vector(values: list[float] | None, default: np.ndarray, name: str) -> np.ndarray:
    arr = default.astype(np.float64) if values is None else np.asarray(values, dtype=np.float64)

    if arr.shape != default.shape:
        raise ValueError(f"{name} 必须提供 {default.size} 个数，当前为 {arr.size} 个。")

    return arr


def _joint_id(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

    if jid < 0:
        raise RuntimeError(f"XML 中找不到 joint: {joint_name}")

    return int(jid)


def _clip_rate(target: np.ndarray, previous: np.ndarray, max_step: np.ndarray) -> np.ndarray:
    return previous + np.clip(target - previous, -max_step, max_step)


def _unwrap_near(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)

    return values + 2.0 * np.pi * np.round((reference - values) / (2.0 * np.pi))


def _sim_to_real_unclipped(q_sim: np.ndarray, signs: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    return (np.asarray(q_sim, dtype=np.float64)[:6] - offsets) / signs


def read_stable_positions(
    arm,
    reference: np.ndarray,
    samples: int,
    interval: float,
) -> np.ndarray:
    values = []

    for _ in range(max(int(samples), 1)):
        q = np.asarray(arm.get_positions(request=True)[:6], dtype=np.float64)
        values.append(_unwrap_near(q, reference[:6]))
        time.sleep(max(float(interval), 0.0))

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

    try:
        getattr(arm, "_ctrl_map", {}).clear()
        getattr(arm, "_motor_map", {}).clear()
    except Exception:
        pass


class SimToRealMapper:
    def __init__(
        self,
        model: mujoco.MjModel,
        joint_names: tuple[str, ...],
        signs: np.ndarray,
        offsets: np.ndarray,
        soft_margin: float,
    ):
        self.model = model
        self.joint_names = tuple(joint_names)
        self.signs = np.asarray(signs, dtype=np.float64)
        self.offsets = np.asarray(offsets, dtype=np.float64)

        if np.any(np.abs(self.signs) < 1e-9):
            raise ValueError("sim_to_real signs 中不能出现 0。")

        self.joint_ids = np.array(
            [_joint_id(model, name) for name in self.joint_names],
            dtype=np.int32,
        )

        sim_ranges = []

        for jid in self.joint_ids:
            if int(model.jnt_limited[jid]) == 1:
                sim_ranges.append(model.jnt_range[jid].copy())
            else:
                sim_ranges.append(np.array([-np.inf, np.inf], dtype=np.float64))

        sim_ranges = np.asarray(sim_ranges, dtype=np.float64)

        real_limits = (sim_ranges - self.offsets[:, None]) / self.signs[:, None]

        self.real_lower = np.minimum(real_limits[:, 0], real_limits[:, 1]) + soft_margin
        self.real_upper = np.maximum(real_limits[:, 0], real_limits[:, 1]) - soft_margin

    def real_to_sim(self, q_real: np.ndarray) -> np.ndarray:
        q_real = np.asarray(q_real, dtype=np.float64)[: len(self.joint_names)]
        return q_real * self.signs + self.offsets

    def sim_to_real(self, q_sim: np.ndarray) -> np.ndarray:
        q_sim = np.asarray(q_sim, dtype=np.float64)[: len(self.joint_names)]
        q_real = (q_sim - self.offsets) / self.signs

        return np.clip(q_real, self.real_lower, self.real_upper)

    def print_info(self) -> None:
        print("\n" + "=" * 90)
        print("MuJoCo 标准关节空间 -> 达妙真机空间映射")
        print("=" * 90)

        for i, name in enumerate(self.joint_names):
            lower = self.real_lower[i]
            upper = self.real_upper[i]

            lower_str = f"{lower:+.3f}" if np.isfinite(lower) else "-inf"
            upper_str = f"{upper:+.3f}" if np.isfinite(upper) else "+inf"

            print(
                f"{name}: "
                f"q_sim = {self.signs[i]:+.0f} * q_real + {self.offsets[i]:+.3f}, "
                f"real_limit=[{lower_str}, {upper_str}] rad"
            )

        print("=" * 90 + "\n")


class SafetyGuard:
    def __init__(
        self,
        mapper: SimToRealMapper,
        max_step: np.ndarray,
        max_start_error: float,
        max_tracking_error: float,
        tracking_breach_samples: int,
    ):
        self.mapper = mapper
        self.max_step = np.asarray(max_step, dtype=np.float64)
        self.max_start_error = float(max_start_error)
        self.max_tracking_error = float(max_tracking_error)
        self.tracking_breach_samples = max(int(tracking_breach_samples), 1)
        self.command: np.ndarray | None = None
        self._tracking_breach_count = 0

    def initialize(
        self,
        q_real_now: np.ndarray,
        q_target: np.ndarray,
        allow_large_start: bool,
    ) -> np.ndarray:
        q_real_now = np.asarray(q_real_now, dtype=np.float64)[:6]
        q_target = np.asarray(q_target, dtype=np.float64)[:6]

        start_error = np.max(np.abs(q_target - q_real_now))

        if start_error > self.max_start_error and not allow_large_start:
            raise RuntimeError(
                f"启动目标与达妙真机当前位置差距过大: "
                f"max_error={start_error:.3f} rad。"
                f"请先让主手和从手机械臂姿态接近，或者使用 --calibrate-current-as-master。"
            )

        self.command = q_real_now.copy()

        return self.command.copy()

    def next_command(
        self,
        q_target: np.ndarray,
        q_feedback: np.ndarray,
    ) -> np.ndarray:
        if self.command is None:
            raise RuntimeError("SafetyGuard 尚未 initialize。")

        q_feedback = np.asarray(q_feedback, dtype=np.float64)[:6]
        q_target = np.asarray(q_target, dtype=np.float64)[:6]

        q_target = np.clip(q_target, self.mapper.real_lower, self.mapper.real_upper)

        q_target_cmd = _unwrap_near(q_target, q_feedback)
        previous_cmd = _unwrap_near(self.command, q_feedback)

        tracking_error = np.max(np.abs(q_target_cmd - q_feedback))

        if tracking_error > self.max_tracking_error:
            self._tracking_breach_count += 1

            if self._tracking_breach_count >= self.tracking_breach_samples:
                raise RuntimeError(
                    f"达妙真机跟踪误差过大: {tracking_error:.3f} rad，触发保护停机。"
                )
        else:
            self._tracking_breach_count = 0

        self.command = _clip_rate(q_target_cmd, previous_cmd, self.max_step)

        return self.command.copy()


# =============================================================================
# 6. 达妙夹爪使能与 MIT 控制
# =============================================================================

def setup_damiao_gripper(
    arm,
    gripper_cfg_path: Path,
    gripper_name_fallback: str = "gripper",
):
    """
    参考 real2sim_gravity_compensation_grasp.py 中的逻辑：
        g_cfg = load_gripper_cfg(... )["gripper"]
        shared_damiao_controller = arm._ctrl_map["damiao"]
        g_mot = shared_damiao_controller.add_damiao_motor(...)
        arm._motor_map[g_cfg.name] = g_mot
        g_mot.ensure_mode(Mode.MIT, 1000)
        shared_damiao_controller.enable_all()
    """
    if arm is None:
        return None, None, None

    if not gripper_cfg_path.exists():
        raise FileNotFoundError(f"夹爪配置文件不存在: {gripper_cfg_path}")

    load_gripper_cfg = _load_gripper_cfg_func()
    g_cfg = load_gripper_cfg(str(gripper_cfg_path))["gripper"]

    if "damiao" not in arm._ctrl_map:
        raise RuntimeError(
            "arm._ctrl_map 中没有 'damiao' 控制器，无法添加达妙夹爪电机。"
        )

    shared_damiao_controller = arm._ctrl_map["damiao"]

    gripper_name = getattr(g_cfg, "name", gripper_name_fallback)

    if gripper_name in arm._motor_map:
        g_mot = arm._motor_map[gripper_name]
        print(f"✅ 夹爪电机已存在于 arm._motor_map: {gripper_name}")
    else:
        g_mot = shared_damiao_controller.add_damiao_motor(
            g_cfg.motor_id,
            g_cfg.feedback_id,
            g_cfg.model,
        )
        arm._motor_map[gripper_name] = g_mot
        print(
            f"✅ 已添加达妙夹爪电机: "
            f"name={gripper_name}, motor_id={g_cfg.motor_id}, "
            f"feedback_id={g_cfg.feedback_id}, model={g_cfg.model}"
        )

    try:
        from motorbridge import Mode

        g_mot.ensure_mode(Mode.MIT, 1000)
        shared_damiao_controller.enable_all()
        time.sleep(0.2)

        print("✅ 夹爪已切入 MIT 模式并完成使能")

        st = g_mot.get_state()

        if st is not None:
            print(f"✅ 夹爪初始位置: {st.pos:.3f} rad")
        else:
            print("⚠️ 夹爪初始状态读取为空，但电机已尝试使能。")

    except Exception as e:
        raise RuntimeError(f"夹爪 MIT 模式配置或使能失败: {e}") from e

    return g_mot, shared_damiao_controller, gripper_name


def send_damiao_gripper_mit(
    g_mot,
    controller,
    target_rad: float,
    kp: float,
    kd: float,
    tau: float,
    request_feedback: bool = True,
) -> bool:
    if g_mot is None:
        return False

    try:
        g_mot.send_mit(
            float(target_rad),
            0.0,
            float(kp),
            float(kd),
            float(tau),
        )

        if request_feedback:
            try:
                g_mot.request_feedback()
            except Exception:
                pass

            if controller is not None:
                try:
                    controller.poll_feedback_once()
                except Exception:
                    pass

        return True

    except Exception as e:
        print(f"⚠️ 夹爪 MIT 命令发送失败: {e}")
        return False


def get_gripper_feedback_pos(g_mot) -> float | None:
    if g_mot is None:
        return None

    try:
        st = g_mot.get_state()

        if st is None:
            return None

        return float(st.pos)

    except Exception:
        return None


# =============================================================================
# 7. 舵机读取线程
# =============================================================================

def servo_reader_worker(
    scs,
    state_lock: threading.Lock,
    shared_state: dict,
    read_rate: float,
    servo_to_sim_sign: np.ndarray,
    sim_home_rad: np.ndarray,
    enable_gripper: bool,
    invert_gripper: bool,
    gripper_real_closed_rad: float,
    gripper_real_open_rad: float,
) -> None:
    global _running

    read_period = 1.0 / max(float(read_rate), 1e-6)

    last_arm_deg = np.array(
        [JOINT_LIMITS_DEG[servo_id]["home_deg"] for servo_id in ARM_SERVO_IDS],
        dtype=np.float64,
    )

    last_gripper_deg = JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"]

    print(f"📡 舵机主手读取线程已启动，目标读取频率: {read_rate:.1f} Hz")

    while _running:
        loop_start = time.perf_counter()

        arm_deg = np.array(last_arm_deg, dtype=np.float64)
        success_count = 0
        failed_ids: list[int] = []

        for i, servo_id in enumerate(ARM_SERVO_IDS):
            angle_deg, ok = read_servo_angle(scs, servo_id, arm_deg[i])
            arm_deg[i] = angle_deg

            if ok:
                success_count += 1
            else:
                failed_ids.append(servo_id)

        last_arm_deg = arm_deg.copy()

        q_sim = servo_deg_array_to_sim_rad(
            arm_deg_array=arm_deg,
            servo_to_sim_sign=servo_to_sim_sign,
            sim_home_rad=sim_home_rad,
        )

        if enable_gripper:
            gripper_deg, gripper_ok = read_servo_angle(
                scs,
                GRIPPER_SERVO_ID,
                last_gripper_deg,
            )

            if gripper_ok:
                success_count += 1
            else:
                failed_ids.append(GRIPPER_SERVO_ID)

            last_gripper_deg = float(gripper_deg)

            gripper_norm = gripper_servo_deg_to_norm(
                gripper_deg,
                invert_gripper=invert_gripper,
            )

            gripper_target_rad = gripper_norm_to_real_rad(
                gripper_norm,
                closed_rad=gripper_real_closed_rad,
                open_rad=gripper_real_open_rad,
            )
        else:
            gripper_deg = last_gripper_deg
            gripper_norm = 1.0
            gripper_target_rad = gripper_real_open_rad

        now = time.perf_counter()

        with state_lock:
            shared_state["arm_deg"] = arm_deg.copy()
            shared_state["target_q_sim"] = q_sim.copy()

            shared_state["gripper_deg"] = float(gripper_deg)
            shared_state["gripper_norm"] = float(gripper_norm)
            shared_state["gripper_target_rad"] = float(gripper_target_rad)

            shared_state["success_count"] = int(success_count)
            shared_state["failed_ids"] = list(failed_ids)
            shared_state["timestamp"] = now
            shared_state["read_frame"] += 1

        elapsed = time.perf_counter() - loop_start
        sleep_time = read_period - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)


def wait_for_servo_ready(
    state_lock: threading.Lock,
    shared_state: dict,
    min_success_count: int,
    timeout: float = 5.0,
) -> bool:
    t0 = time.perf_counter()

    while time.perf_counter() - t0 < timeout:
        with state_lock:
            read_frame = int(shared_state.get("read_frame", 0))
            success_count = int(shared_state.get("success_count", 0))

        if read_frame > 0 and success_count >= min_success_count:
            return True

        time.sleep(0.02)

    return False


# =============================================================================
# 8. 命令行参数
# =============================================================================

def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ST/SMS_STS 舵机主手 -> 达妙真机机械臂 + 达妙夹爪遥操作"
    )

    parser.add_argument("--xml", type=Path, default=DEFAULT_XML)
    parser.add_argument("--joint-names", type=str, default="joint1,joint2,joint3,joint4,joint5,joint6")

    parser.add_argument("--port", type=str, default=DEFAULT_SERVO_PORT)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--read-rate", type=float, default=60.0)
    parser.add_argument("--keep-servo-torque", action="store_true")

    parser.add_argument("--cfg", type=Path, default=None)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--vlim", type=float, nargs=6, default=None)
    parser.add_argument("--max-step", type=float, nargs=6, default=None)
    parser.add_argument("--alpha-master", type=float, default=0.85)

    parser.add_argument("--servo-to-sim-signs", type=float, nargs=6, default=None)
    parser.add_argument("--sim-home", type=float, nargs=6, default=None)

    parser.add_argument("--signs", type=float, nargs=6, default=None)
    parser.add_argument("--offsets", type=float, nargs=6, default=None)
    parser.add_argument("--calibrate-current-as-master", action="store_true")
    parser.add_argument("--soft-margin", type=float, default=DEFAULT_SOFT_MARGIN)

    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--gripper-cfg", type=Path, default=DEFAULT_GRIPPER_CFG)
    parser.add_argument("--invert-gripper", action="store_true")
    parser.add_argument("--alpha-gripper", type=float, default=0.85)

    parser.add_argument("--gripper-real-closed-rad", type=float, default=DEFAULT_GRIPPER_REAL_CLOSED_RAD)
    parser.add_argument("--gripper-real-open-rad", type=float, default=DEFAULT_GRIPPER_REAL_OPEN_RAD)

    parser.add_argument("--gripper-kp", type=float, default=1.0)
    parser.add_argument("--gripper-kd", type=float, default=0.05)
    parser.add_argument("--gripper-tau", type=float, default=0.0)
    parser.add_argument("--gripper-send-every", type=int, default=1)
    parser.add_argument("--gripper-delta-threshold", type=float, default=0.003)

    parser.add_argument("--allow-large-start", action="store_true")
    parser.add_argument("--max-start-error", type=float, default=0.25)
    parser.add_argument("--max-tracking-error", type=float, default=1.50)
    parser.add_argument("--tracking-breach-samples", type=int, default=DEFAULT_TRACKING_BREACH_SAMPLES)
    parser.add_argument("--settle-samples", type=int, default=DEFAULT_SETTLE_SAMPLES)
    parser.add_argument("--settle-interval", type=float, default=DEFAULT_SETTLE_INTERVAL)
    parser.add_argument("--max-servo-age", type=float, default=0.5)
    parser.add_argument("--print-every", type=int, default=25)

    return parser


# =============================================================================
# 9. 打印映射信息
# =============================================================================

def print_mapping_info(
    servo_to_sim_sign: np.ndarray,
    sim_home_rad: np.ndarray,
    enable_gripper: bool,
    args,
) -> None:
    print("\n" + "=" * 90)
    print("舵机主手 -> MuJoCo 标准关节空间映射")
    print("=" * 90)

    for i, servo_id in enumerate(ARM_SERVO_IDS):
        cfg = JOINT_LIMITS_DEG[servo_id]
        print(
            f"ID{servo_id}: "
            f"real=[{cfg['min_deg']:.1f}, {cfg['max_deg']:.1f}] deg, "
            f"home={cfg['home_deg']:.1f} deg "
            f"-> joint{i + 1}, "
            f"sign={servo_to_sim_sign[i]:+.0f}, "
            f"sim_home={sim_home_rad[i]:+.3f} rad"
        )

    print("-" * 90)

    if enable_gripper:
        print(
            f"ID7 gripper: "
            f"{GRIPPER_SERVO_CLOSED_DEG:.1f}deg -> norm=0.0 -> "
            f"{args.gripper_real_closed_rad:.3f} rad closed"
        )
        print(
            f"ID7 gripper: "
            f"{GRIPPER_SERVO_OPEN_DEG:.1f}deg -> norm=1.0 -> "
            f"{args.gripper_real_open_rad:.3f} rad open"
        )
        print(
            f"gripper MIT: kp={args.gripper_kp:.3f}, "
            f"kd={args.gripper_kd:.3f}, tau={args.gripper_tau:.3f}, "
            f"invert={args.invert_gripper}"
        )
    else:
        print("ID7 gripper: disabled")

    print("=" * 90 + "\n")


# =============================================================================
# 10. 主程序
# =============================================================================

def main() -> None:
    global _running

    args = build_argparser().parse_args()

    enable_gripper = not args.no_gripper

    servo_to_sim_sign = _parse_vector(
        args.servo_to_sim_signs,
        DEFAULT_SERVO_TO_SIM_SIGN,
        "--servo-to-sim-signs",
    )

    sim_home_rad = _parse_vector(
        args.sim_home,
        DEFAULT_SIM_HOME_RAD,
        "--sim-home",
    )

    signs = _parse_vector(
        args.signs,
        np.ones(6, dtype=np.float64),
        "--signs",
    )

    offsets = _parse_vector(
        args.offsets,
        np.zeros(6, dtype=np.float64),
        "--offsets",
    )

    vlim = _parse_vector(
        args.vlim,
        DEFAULT_CMD_VLIM,
        "--vlim",
    )

    max_step = _parse_vector(
        args.max_step,
        DEFAULT_MAX_STEP,
        "--max-step",
    )

    joint_names = tuple(x.strip() for x in args.joint_names.split(",") if x.strip())

    if len(joint_names) != 6:
        print(f"❌ joint-names 必须是 6 个，当前是 {len(joint_names)}: {joint_names}")
        return

    if not args.xml.exists():
        print(f"❌ MuJoCo XML 不存在: {args.xml}")
        return

    if enable_gripper and not args.gripper_cfg.exists():
        print(f"❌ 夹爪配置文件不存在: {args.gripper_cfg}")
        return

    print("\n" + "=" * 90)
    print("舵机主手遥控达妙真机机械臂 + 达妙夹爪")
    print("=" * 90)
    print(f"MuJoCo XML: {args.xml}")
    print(f"舵机串口: {args.port}, baudrate={args.baudrate}")
    print(f"舵机读取频率: {args.read_rate:.1f} Hz")
    print(f"达妙控制频率: {args.rate:.1f} Hz")
    print(f"RobotArm cfg: {args.cfg}")
    print(f"夹爪启用: {enable_gripper}")
    print(f"夹爪 cfg: {args.gripper_cfg}")
    print("=" * 90)

    print_mapping_info(
        servo_to_sim_sign=servo_to_sim_sign,
        sim_home_rad=sim_home_rad,
        enable_gripper=enable_gripper,
        args=args,
    )

    # ---------- 加载 MuJoCo XML ----------
    print(f"\n📂 正在加载 MuJoCo XML: {args.xml}")

    try:
        model = mujoco.MjModel.from_xml_path(str(args.xml))
    except Exception as e:
        print(f"❌ MuJoCo XML 加载失败: {e}")
        return

    # ---------- 打开舵机串口 ----------
    print(f"\n🔌 正在打开舵机主手串口: {args.port}, baudrate={args.baudrate}")

    portHandler = PortHandler(args.port)
    scs = sts(portHandler)

    try:
        if not portHandler.openPort():
            print(f"❌ 舵机串口打开失败: {args.port}")
            return

        if not portHandler.setBaudRate(args.baudrate):
            print(f"❌ 舵机串口波特率设置失败: {args.baudrate}")
            portHandler.closePort()
            return

    except Exception as e:
        print(f"❌ 舵机串口打开异常: {e}")
        return

    print("✅ 舵机主手串口已打开")

    print("等待 ESP32 进入透传状态...")
    time.sleep(2.5)

    try:
        portHandler.ser.reset_input_buffer()
        portHandler.ser.reset_output_buffer()
        print("✅ 已清空舵机串口缓冲区")
    except Exception as e:
        print(f"⚠️ 清空串口缓冲区失败，可忽略: {e}")

    if not args.keep_servo_torque:
        release_ids = ARM_SERVO_IDS.copy()

        if enable_gripper:
            release_ids.append(GRIPPER_SERVO_ID)

        release_servo_torque(scs, release_ids)
    else:
        print("⚠️ keep-servo-torque 已启用，不释放舵机主手力矩。")

    # ---------- 初始化共享状态 ----------
    home_arm_deg = np.array(
        [JOINT_LIMITS_DEG[servo_id]["home_deg"] for servo_id in ARM_SERVO_IDS],
        dtype=np.float64,
    )

    home_q_sim = servo_deg_array_to_sim_rad(
        home_arm_deg,
        servo_to_sim_sign=servo_to_sim_sign,
        sim_home_rad=sim_home_rad,
    )

    home_gripper_deg = JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"]

    home_gripper_norm = gripper_servo_deg_to_norm(
        home_gripper_deg,
        invert_gripper=args.invert_gripper,
    )

    home_gripper_target_rad = gripper_norm_to_real_rad(
        home_gripper_norm,
        closed_rad=args.gripper_real_closed_rad,
        open_rad=args.gripper_real_open_rad,
    )

    state_lock = threading.Lock()

    shared_state = {
        "arm_deg": home_arm_deg.copy(),
        "target_q_sim": home_q_sim.copy(),

        "gripper_deg": float(home_gripper_deg),
        "gripper_norm": float(home_gripper_norm),
        "gripper_target_rad": float(home_gripper_target_rad),

        "success_count": 0,
        "failed_ids": [],
        "timestamp": time.perf_counter(),
        "read_frame": 0,
    }

    reader_thread = threading.Thread(
        target=servo_reader_worker,
        args=(
            scs,
            state_lock,
            shared_state,
            args.read_rate,
            servo_to_sim_sign,
            sim_home_rad,
            enable_gripper,
            args.invert_gripper,
            args.gripper_real_closed_rad,
            args.gripper_real_open_rad,
        ),
        daemon=True,
    )

    reader_thread.start()

    min_success_count = ARM_DOF + (1 if enable_gripper else 0)

    if not wait_for_servo_ready(
        state_lock=state_lock,
        shared_state=shared_state,
        min_success_count=min_success_count,
        timeout=5.0,
    ):
        print("❌ 等待舵机主手读取超时。")
        _running = False

        try:
            portHandler.closePort()
        except Exception:
            pass

        return

    with state_lock:
        initial_arm_deg = shared_state["arm_deg"].copy()
        initial_q_sim = shared_state["target_q_sim"].copy()
        initial_gripper_deg = float(shared_state["gripper_deg"])
        initial_gripper_norm = float(shared_state["gripper_norm"])
        initial_gripper_target_rad = float(shared_state["gripper_target_rad"])
        initial_success_count = int(shared_state["success_count"])
        initial_failed_ids = list(shared_state["failed_ids"])

    print(f"\n[主手启动] arm_deg = {np.round(initial_arm_deg, 2).tolist()}")
    print(f"[主手启动] q_sim   = {np.round(initial_q_sim, 3).tolist()}")

    if enable_gripper:
        print(
            f"[主手启动] ID7={initial_gripper_deg:.1f}deg, "
            f"norm={initial_gripper_norm:.3f}, "
            f"gripper_target={initial_gripper_target_rad:.3f} rad"
        )

    print(f"[主手启动] success_count={initial_success_count}, failed={initial_failed_ids}")

    # ---------- 初始化达妙真机 ----------
    arm = None
    gripper_motor = None
    gripper_controller = None
    gripper_name = None

    try:
        print("\n🤖 正在初始化达妙真机 RobotArm...")

        RobotArm = _load_robot_arm_class()
        arm = RobotArm(cfg_path=str(args.cfg) if args.cfg is not None else None)

        arm.connect()
        arm.enable()
        arm.mode_pos_vel(vlim=vlim)

        if enable_gripper:
            print("\n🦾 正在配置并使能达妙夹爪电机...")

            gripper_motor, gripper_controller, gripper_name = setup_damiao_gripper(
                arm=arm,
                gripper_cfg_path=args.gripper_cfg,
            )

            print(f"✅ 达妙夹爪已加入 motor_map: {gripper_name}")

        q_reference = _sim_to_real_unclipped(initial_q_sim, signs, offsets)

        q_feedback = read_stable_positions(
            arm=arm,
            reference=q_reference,
            samples=args.settle_samples,
            interval=args.settle_interval,
        )

        print(f"[达妙启动] 当前真机反馈 q_feedback(rad): {np.round(q_feedback, 3).tolist()}")

        if args.calibrate_current_as_master:
            offsets = initial_q_sim.copy() - signs * q_feedback[:6]
            print("\n[标定] 已启用 --calibrate-current-as-master")
            print("[标定] 当前舵机主手姿态将对应当前达妙真机姿态。")
            print(f"[标定] 自动计算 offsets = {np.round(offsets, 4).tolist()}")

        mapper = SimToRealMapper(
            model=model,
            joint_names=joint_names,
            signs=signs,
            offsets=offsets,
            soft_margin=args.soft_margin,
        )

        mapper.print_info()

        guard = SafetyGuard(
            mapper=mapper,
            max_step=max_step,
            max_start_error=args.max_start_error,
            max_tracking_error=args.max_tracking_error,
            tracking_breach_samples=args.tracking_breach_samples,
        )

        q_target = mapper.sim_to_real(initial_q_sim)

        q_cmd = guard.initialize(
            q_real_now=q_feedback,
            q_target=q_target,
            allow_large_start=args.allow_large_start,
        )

        print(f"[初始化] 初始目标 q_target(rad): {np.round(q_target, 3).tolist()}")
        print(f"[初始化] 初始命令 q_cmd(rad):    {np.round(q_cmd, 3).tolist()}")

        arm.pos_vel(q_cmd, vlim=vlim)

        filtered_gripper_target_rad = initial_gripper_target_rad
        last_sent_gripper_target_rad = None

        if enable_gripper:
            ok = send_damiao_gripper_mit(
                g_mot=gripper_motor,
                controller=gripper_controller,
                target_rad=filtered_gripper_target_rad,
                kp=args.gripper_kp,
                kd=args.gripper_kd,
                tau=args.gripper_tau,
                request_feedback=True,
            )

            if ok:
                print(
                    f"✅ 初始夹爪 MIT 命令已发送: "
                    f"{filtered_gripper_target_rad:.3f} rad"
                )
            else:
                print("⚠️ 初始夹爪 MIT 命令发送失败。")

        # ---------- 主控制循环 ----------
        cmd_period = 1.0 / max(float(args.rate), 1e-6)
        frame = 0
        missing_count = 0
        last_print_time = 0.0

        filtered_q_sim = initial_q_sim.copy()

        print("\n✅ 遥操作主循环已启动")
        print("拖动 ID1~ID6，达妙机械臂将跟随运动。")
        if enable_gripper:
            print("拖动 ID7，达妙真机夹爪将跟随开合。")
        print("按 Ctrl+C 退出。\n")

        while _running:
            loop_start = time.perf_counter()
            now = loop_start

            with state_lock:
                arm_deg = shared_state["arm_deg"].copy()
                target_q_sim_raw = shared_state["target_q_sim"].copy()

                gripper_deg = float(shared_state["gripper_deg"])
                gripper_norm = float(shared_state["gripper_norm"])
                gripper_target_rad_raw = float(shared_state["gripper_target_rad"])

                success_count = int(shared_state["success_count"])
                failed_ids = list(shared_state["failed_ids"])
                servo_timestamp = float(shared_state["timestamp"])

            servo_age = now - servo_timestamp

            if servo_age > args.max_servo_age:
                raise RuntimeError(
                    f"舵机主手数据超时: age={servo_age * 1000:.1f} ms，"
                    f"超过阈值 {args.max_servo_age * 1000:.1f} ms，触发保护停机。"
                )

            expected_count = ARM_DOF + (1 if enable_gripper else 0)

            if success_count < expected_count:
                missing_count += 1

            # 1. 机械臂 6 轴
            filtered_q_sim = smooth_update(
                filtered_q_sim,
                target_q_sim_raw,
                args.alpha_master,
            )

            q_target_real = mapper.sim_to_real(filtered_q_sim)

            q_feedback = np.asarray(
                arm.get_positions(request=True)[:6],
                dtype=np.float64,
            )

            q_feedback = _unwrap_near(q_feedback, q_cmd)

            q_cmd = guard.next_command(q_target_real, q_feedback)

            arm.pos_vel(q_cmd, vlim=vlim)

            # 2. 达妙夹爪 MIT 控制
            gripper_feedback_pos = None

            if enable_gripper:
                filtered_gripper_target_rad = smooth_update_scalar(
                    filtered_gripper_target_rad,
                    gripper_target_rad_raw,
                    args.alpha_gripper,
                )

                should_send_gripper = False

                if frame % max(int(args.gripper_send_every), 1) == 0:
                    if last_sent_gripper_target_rad is None:
                        should_send_gripper = True
                    else:
                        delta_g = abs(filtered_gripper_target_rad - last_sent_gripper_target_rad)

                        if delta_g >= args.gripper_delta_threshold:
                            should_send_gripper = True

                if should_send_gripper:
                    ok = send_damiao_gripper_mit(
                        g_mot=gripper_motor,
                        controller=gripper_controller,
                        target_rad=filtered_gripper_target_rad,
                        kp=args.gripper_kp,
                        kd=args.gripper_kd,
                        tau=args.gripper_tau,
                        request_feedback=True,
                    )

                    if ok:
                        last_sent_gripper_target_rad = filtered_gripper_target_rad

                gripper_feedback_pos = get_gripper_feedback_pos(gripper_motor)

            # 3. 状态打印
            if args.print_every > 0 and frame % args.print_every == 0:
                t_print = time.perf_counter()

                if t_print - last_print_time >= 0.2:
                    deg_str = " ".join(f"{v:6.1f}" for v in arm_deg)
                    qsim_str = " ".join(f"{v:+6.2f}" for v in filtered_q_sim)
                    cmd_str = " ".join(f"{v:+6.2f}" for v in q_cmd)
                    fb_str = " ".join(f"{v:+6.2f}" for v in q_feedback)

                    if enable_gripper:
                        gfb_str = "None" if gripper_feedback_pos is None else f"{gripper_feedback_pos:+.3f}"

                        print(
                            f"[{frame:06d}] "
                            f"servo_deg=[{deg_str}] | "
                            f"q_sim=[{qsim_str}] | "
                            f"cmd=[{cmd_str}] | "
                            f"fb=[{fb_str}] | "
                            f"ID7={gripper_deg:6.1f}deg -> "
                            f"norm={gripper_norm:.2f}, "
                            f"gtarget={filtered_gripper_target_rad:+.3f}rad, "
                            f"gfb={gfb_str} | "
                            f"age={servo_age * 1000:.1f}ms | "
                            f"miss={missing_count} | "
                            f"failed={failed_ids}"
                        )
                    else:
                        print(
                            f"[{frame:06d}] "
                            f"servo_deg=[{deg_str}] | "
                            f"q_sim=[{qsim_str}] | "
                            f"cmd=[{cmd_str}] | "
                            f"fb=[{fb_str}] | "
                            f"age={servo_age * 1000:.1f}ms | "
                            f"miss={missing_count} | "
                            f"failed={failed_ids}"
                        )

                    last_print_time = t_print

            frame += 1

            elapsed = time.perf_counter() - loop_start
            sleep_time = cmd_period - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)
            elif elapsed > cmd_period + 0.05:
                print(
                    f"⚠️ 控制循环严重超时: "
                    f"{elapsed * 1000:.1f} ms, "
                    f"目标周期 {cmd_period * 1000:.1f} ms"
                )

    except KeyboardInterrupt:
        _running = False

    except Exception as exc:
        _running = False
        print(f"\n[保护停机] 触发异常: {exc}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n[退出流程] 正在关闭舵机主手与达妙真机通信...")
        _running = False

        try:
            reader_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            close_arm_fast(arm)
            print("[退出流程] 达妙真机通信已关闭")
        except Exception as e:
            print(f"[退出流程] 关闭达妙真机通信异常: {e}")

        try:
            portHandler.closePort()
            print("[退出流程] 舵机主手串口已关闭")
        except Exception as e:
            print(f"[退出流程] 关闭舵机串口异常: {e}")

        print("[退出流程] 安全退出。")


if __name__ == "__main__":
    main()

# python servo_arm_teleoperation_real.py --xml /home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/master_slave_control/Servo_control/xml/rebot_gripper/sim_reBot_grasp.xml --port /dev/ttyUSB0 --baudrate 115200 --rate 50 --read-rate 60 --calibrate-current-as-master
