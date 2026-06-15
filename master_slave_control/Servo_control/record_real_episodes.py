#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机主手 -> 达妙真机机械臂 + 达妙真机夹爪 遥操作程序 (集成多线程真实RGB流)

控制链路：
    ID1~ID6 ST/SMS_STS 舵机主手 -> q_sim(rad) -> 达妙真机 6 轴 q_real(rad) -> SafetyGuard -> pos_vel
    ID7 ST/SMS_STS 舵机夹爪 -> gripper_norm -> 达妙夹爪目标角度 -> send_mit

数据采集特性：
    1. 按下【Enter】键触发录制，支持 --time 或 --episode_len。
    2. 自动保存为标准 ALOHA 规范 HDF5 (time, qpos, qvel, action, images/*)。
    3. 后台多线程读取 MJPG 视频流并实时转换为 RGB，彻底解耦，绝不阻塞 50Hz 控制主循环。
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

import cv2  # 🌟 新增：OpenCV 用于读取真实相机
import h5py
from tqdm import tqdm

import mujoco
import numpy as np

# =============================================================================
# 0. 全局运行标志与数据采集配置
# =============================================================================

_running = True
_is_recording = False

# 基础数据缓存
recorded_timestamps = []
recorded_qpos = []
recorded_qvel = []
recorded_action = []

# 相机图像流缓存字典
recorded_images = {}

_config = {
    "task_name": "teleop_task",
    "base_save_dir": Path("./collected_data"),
    "episode_len": 500,
    "dt": 0.02,
    "rate": 50,
    "episode_idx": None,
    # 🌟 修改：目前先挂载全局高空相机 cam_high
    "camera_names": ["cam_high"]
}


def _sigint_handler(signum, frame) -> None:
    global _running
    print("\n[teleop] 收到退出信号，准备安全关闭...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)


# =============================================================================
# 0.5 🌟 新增：独立的多线程相机读取类
# =============================================================================
class ThreadedCamera:
    """独立的后台相机读取线程，避免 OpenCV I/O 阻塞 50Hz 的遥操作主循环"""

    def __init__(self, src=2, width=640, height=480, name="camera"):
        self.name = name
        self.capture = cv2.VideoCapture(src)

        # 强制使用 MJPG 编码压缩 USB 带宽
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.ret, self.frame = self.capture.read()
        self.valid = self.ret

        if not self.valid:
            print(f"⚠️ [警告] 相机 {self.name} (src={src}) 初始化失败，将输出全黑图像以维持时序对齐。")
            self.frame = np.zeros((height, width, 3), dtype=np.uint8)
        else:
            print(f"📷 [成功] 相机 {self.name} (src={src}) 后台读取线程已启动。")

        self.running = True
        self.lock = threading.Lock()

        if self.valid:
            self.thread = threading.Thread(target=self._update, daemon=True)
            self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.capture.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def read_rgb(self):
        """主线程调用的极速读取接口，直接返回内存最新帧的 RGB 格式"""
        with self.lock:
            current_frame = self.frame.copy()
        if self.valid:
            return cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        return current_frame

    def release(self):
        self.running = False
        if self.capture.isOpened():
            self.capture.release()


# =============================================================================
# 1. 数据保存与终端交互逻辑 (ALOHA 规范 + 相机流)
# =============================================================================

def save_to_hdf5():
    """标准 ALOHA 规范 .hdf5 数据集安全持久化落盘（含影像流）"""
    global recorded_timestamps, recorded_qpos, recorded_qvel, recorded_action, recorded_images

    if not recorded_timestamps:
        print("\n[💾 导出失败] 未采集到有效数据。")
        return

    task_name = _config["task_name"]
    task_sub_dir = _config["base_save_dir"] / task_name
    task_sub_dir.mkdir(parents=True, exist_ok=True)

    if _config["episode_idx"] is not None:
        final_episode_idx = _config["episode_idx"]
    else:
        existing_episodes = []
        for p in task_sub_dir.glob("episode_*.hdf5"):
            try:
                idx = int(p.stem.split("_")[1])
                existing_episodes.append(idx)
            except (IndexError, ValueError):
                continue
        final_episode_idx = max(existing_episodes) + 1 if existing_episodes else 0

    file_name = f"episode_{final_episode_idx}.hdf5"
    file_path = task_sub_dir / file_name
    total_frames = len(recorded_timestamps)

    print(f"\n\n[💾 存储线程] 正在向硬盘写入 {file_name} ({total_frames} 帧 ALOHA 规范数据)...")
    t0 = time.time()

    num_datasets = 4 + len(_config["camera_names"])

    try:
        with h5py.File(file_path, 'w') as f:
            with tqdm(total=num_datasets, desc="📝 HDF5数据集落盘", bar_format="{l_bar}{bar:30}{r_bar}") as pbar:
                f.create_dataset('/time', data=np.array(recorded_timestamps, dtype=np.float32), compression="gzip")
                pbar.update(1)

                f.create_dataset('/observations/qpos', data=np.array(recorded_qpos, dtype=np.float32),
                                 compression="gzip")
                pbar.update(1)

                f.create_dataset('/observations/qvel', data=np.array(recorded_qvel, dtype=np.float32),
                                 compression="gzip")
                pbar.update(1)

                f.create_dataset('/action/target_pos', data=np.array(recorded_action, dtype=np.float32),
                                 compression="gzip")
                pbar.update(1)

                for cam_name in _config["camera_names"]:
                    img_array = np.array(recorded_images[cam_name], dtype=np.uint8)
                    f.create_dataset(f'/observations/images/{cam_name}',
                                     data=img_array,
                                     compression="gzip",
                                     chunks=(1, img_array.shape[1], img_array.shape[2], img_array.shape[3]))
                    pbar.update(1)

            f.attrs['task_name'] = task_name
            f.attrs['episode_idx'] = final_episode_idx
            f.attrs['episode_len'] = _config["episode_len"]
            f.attrs['total_frames'] = total_frames
            f.attrs['hz_rate'] = _config["rate"]
            f.attrs['duration_seconds'] = total_frames * _config["dt"]
            f.attrs['robot_name'] = 'wheeled_dual_arm_robot'

        print(f"🎉 [💾 导出成功] 数据集固化完成！耗时: {time.time() - t0:.2f}s")
        print(f"📄 数据文件路径: {file_path.resolve()}\n")

    except Exception as e:
        print(f"❌ [💾 导出异常] 写入 HDF5 失败: {e}\n")

    recorded_timestamps.clear()
    recorded_qpos.clear()
    recorded_qvel.clear()
    recorded_action.clear()
    for cam_name in _config["camera_names"]:
        recorded_images[cam_name].clear()


def terminal_keyboard_listener():
    global _is_recording, _running
    while _running:
        try:
            input()
        except (KeyboardInterrupt, EOFError):
            break

        if not _running: break

        if not _is_recording:
            total_seconds = _config["episode_len"] * _config["dt"]
            print(
                f"\n🚀 [采集触发] 开始录制！频率: {_config['rate']}Hz, 目标长度: {_config['episode_len']} 步 ({total_seconds:.2f} 秒)")
            _is_recording = True

            def progress_bar_runner():
                total_steps = _config["episode_len"]
                with tqdm(total=total_steps, desc=f"🔴 [{_config['task_name']}] 运动轨迹录制中",
                          bar_format="{l_bar}{bar:40}{r_bar} [{elapsed}<{remaining}]") as pbar:

                    last_count = 0
                    while _is_recording and _running:
                        time.sleep(0.05)
                        current_count = len(recorded_timestamps)
                        pbar.update(current_count - last_count)
                        last_count = current_count

                    if last_count < total_steps:
                        pbar.update(total_steps - last_count)

                save_to_hdf5()
                print("💡 [提示] 随时再次按下【Enter (回车键)】可录制下一段数据。")

            threading.Thread(target=progress_bar_runner, daemon=True).start()
        else:
            print("\n⚠️ [警告] 系统当前正处于录制中，请勿重复操作。")


# =============================================================================
# 1.5 ~ 8. 路径导入、工具函数、命令行参数 (与原版保持一致)
# =============================================================================
CURRENT_DIR = Path(__file__).resolve().parent


def _candidate_roots() -> list[Path]:
    roots: list[Path] = []
    for p in [CURRENT_DIR, *CURRENT_DIR.parents]: roots.append(p)
    cwd = Path.cwd().resolve()
    for p in [cwd, *cwd.parents]: roots.append(p)
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
            if p.exists(): return p
    return None


def _inject_paths() -> tuple[Path | None, Path | None]:
    sdk_dir = _find_first_existing(["STservo_sdk", "Python/STservo_sdk"])
    robot_pkg_dir = _find_first_existing(["reBotArm_control_py", "Python/reBotArm_control_py"])
    add_paths: list[Path] = []
    if sdk_dir is not None:
        add_paths.append(sdk_dir);
        add_paths.append(sdk_dir.parent)
    if robot_pkg_dir is not None: add_paths.append(robot_pkg_dir.parent)
    add_paths.extend(_candidate_roots()[:5])
    for p in add_paths:
        sp = str(p)
        if sp not in sys.path: sys.path.insert(0, sp)
    return sdk_dir, robot_pkg_dir


SDK_DIR, ROBOT_PKG_DIR = _inject_paths()

try:
    from STservo_sdk import *  # noqa: F401,F403
except Exception as exc:
    print("❌ 无法导入 STservo_sdk。")
    raise exc


def _load_robot_arm_class():
    try:
        from reBotArm_control_py.actuator import RobotArm
        return RobotArm
    except ImportError:
        pass
    possible_arm_py: list[Path] = []
    if ROBOT_PKG_DIR is not None: possible_arm_py.append(ROBOT_PKG_DIR / "actuator" / "arm.py")
    for root in _candidate_roots():
        possible_arm_py.append(root / "reBotArm_control_py" / "actuator" / "arm.py")
        possible_arm_py.append(root / "Python" / "reBotArm_control_py" / "actuator" / "arm.py")
    for arm_py in possible_arm_py:
        if arm_py.exists():
            spec = importlib.util.spec_from_file_location("_rebotarm_actuator_arm", arm_py)
            if spec and spec.loader:
                module = importlib.util.module_from_spec(spec)
                sys.modules[spec.name] = module
                spec.loader.exec_module(module)
                return module.RobotArm
    raise ImportError("无法加载 RobotArm。")


def _load_gripper_cfg_func():
    try:
        from reBotArm_control_py.actuator.gripper import load_cfg
        return load_cfg
    except ImportError:
        pass
    possible_gripper_py: list[Path] = []
    if ROBOT_PKG_DIR is not None: possible_gripper_py.append(ROBOT_PKG_DIR / "actuator" / "gripper.py")
    for root in _candidate_roots():
        possible_gripper_py.append(root / "reBotArm_control_py" / "actuator" / "gripper.py")
        possible_gripper_py.append(root / "Python" / "reBotArm_control_py" / "actuator" / "gripper.py")
    for gripper_py in possible_gripper_py:
        if gripper_py.exists():
            spec = importlib.util.spec_from_file_location("_rebotarm_actuator_gripper", gripper_py)
            if spec and spec.loader:
                module = importlib.util.module_from_spec(spec)
                sys.modules[spec.name] = module
                spec.loader.exec_module(module)
                return module.load_cfg
    raise ImportError("无法加载 gripper.py 中的 load_cfg。")


def _default_xml_path() -> Path:
    candidates = ["mujoco/xml/rebot_gripper/sim_reBot_grasp.xml",
                  "Python/Servo_control/xml/rebot_gripper/sim_reBot_grasp.xml"]
    p = _find_first_existing(candidates)
    return p if p else CURRENT_DIR / "xml" / "rebot_gripper" / "sim_reBot_grasp.xml"


def _default_gripper_cfg_path() -> Path:
    candidates = ["config/gripper.yaml", "Python/config/gripper.yaml"]
    p = _find_first_existing(candidates)
    return p if p else CURRENT_DIR / "config" / "gripper.yaml"


DEFAULT_XML = _default_xml_path()
DEFAULT_GRIPPER_CFG = _default_gripper_cfg_path()
DEFAULT_SERVO_PORT = "COM6" if os.name == "nt" else "/dev/ttyUSB0"

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
    7: {"min_deg": 90.0, "max_deg": 180.0, "home_deg": 180.0},
}
SAFETY_MARGIN_DEG = 0.0
DEFAULT_SERVO_TO_SIM_SIGN = np.array([-1.0, 1.0, 1.0, -1.0, -1.0, -1.0], dtype=np.float64)
DEFAULT_SIM_HOME_RAD = np.zeros(6, dtype=np.float64)
GRIPPER_SERVO_CLOSED_DEG = 90.0
GRIPPER_SERVO_OPEN_DEG = 180.0
DEFAULT_GRIPPER_REAL_CLOSED_RAD = 0.2
DEFAULT_GRIPPER_REAL_OPEN_RAD = -5.8


def clamp(val: float, min_val: float, max_val: float) -> float: return max(float(min_val),
                                                                           min(float(val), float(max_val)))


def servo_pos_to_deg(pos: int | float) -> float: return float(pos) / SERVO_DIGITAL_RANGE * SERVO_ANGLE_RANGE


def deg_to_rad(deg: float) -> float: return float(deg) * np.pi / 180.0


def limit_servo_deg(servo_id: int, angle_deg: float) -> float:
    cfg = JOINT_LIMITS_DEG[servo_id]
    return clamp(float(angle_deg), cfg["min_deg"] + SAFETY_MARGIN_DEG, cfg["max_deg"] - SAFETY_MARGIN_DEG)


def servo_deg_to_sim_rad(servo_id: int, angle_deg: float, arm_index: int, servo_to_sim_sign: np.ndarray,
                         sim_home_rad: np.ndarray) -> float:
    delta_deg = limit_servo_deg(servo_id, angle_deg) - JOINT_LIMITS_DEG[servo_id]["home_deg"]
    return float(sim_home_rad[arm_index] + servo_to_sim_sign[arm_index] * deg_to_rad(delta_deg))


def servo_deg_array_to_sim_rad(arm_deg_array: np.ndarray, servo_to_sim_sign: np.ndarray,
                               sim_home_rad: np.ndarray) -> np.ndarray:
    q_sim = np.zeros(ARM_DOF, dtype=np.float64)
    for i, servo_id in enumerate(ARM_SERVO_IDS):
        q_sim[i] = servo_deg_to_sim_rad(servo_id, float(arm_deg_array[i]), i, servo_to_sim_sign, sim_home_rad)
    return q_sim


def gripper_servo_deg_to_norm(angle_deg: float, invert_gripper: bool) -> float:
    angle_deg = limit_servo_deg(GRIPPER_SERVO_ID, angle_deg)
    denom = GRIPPER_SERVO_OPEN_DEG - GRIPPER_SERVO_CLOSED_DEG
    norm = 0.0 if abs(denom) < 1e-9 else clamp((angle_deg - GRIPPER_SERVO_CLOSED_DEG) / denom, 0.0, 1.0)
    return float(1.0 - norm) if invert_gripper else float(norm)


def gripper_norm_to_real_rad(norm: float, closed_rad: float, open_rad: float) -> float:
    target = float(closed_rad) + clamp(norm, 0.0, 1.0) * (float(open_rad) - float(closed_rad))
    return clamp(target, min(float(closed_rad), float(open_rad)), max(float(closed_rad), float(open_rad)))


def smooth_update(prev: np.ndarray, target: np.ndarray, alpha: float) -> np.ndarray:
    return clamp(float(alpha), 0.0, 1.0) * target + (1.0 - clamp(float(alpha), 0.0, 1.0)) * prev


def smooth_update_scalar(prev: float, target: float, alpha: float) -> float:
    return float(clamp(float(alpha), 0.0, 1.0) * target + (1.0 - clamp(float(alpha), 0.0, 1.0)) * prev)


def read_servo_angle(scs, servo_id: int, last_angle: float) -> tuple[float, bool]:
    try:
        pos, speed, result, error = scs.ReadPosSpeed(servo_id)
        if result == COMM_SUCCESS: return float(limit_servo_deg(servo_id, servo_pos_to_deg(pos))), True
        return float(last_angle), False
    except:
        return float(last_angle), False


def release_servo_torque(scs, servo_ids: list[int]) -> None:
    print("\n🔓 正在释放舵机主手力矩，用于手动拖动遥操作...")
    for servo_id in servo_ids:
        try:
            scs.write1ByteTxRx(servo_id, STS_TORQUE_ENABLE, 0)
        except:
            pass
        time.sleep(0.02)


DEFAULT_CMD_VLIM = np.array([0.8, 0.8, 0.8, 1.2, 1.2, 1.2], dtype=np.float64)
DEFAULT_MAX_STEP = np.array([0.015, 0.015, 0.015, 0.020, 0.020, 0.020], dtype=np.float64)
DEFAULT_SOFT_MARGIN = 0.0
DEFAULT_SETTLE_SAMPLES = 30
DEFAULT_SETTLE_INTERVAL = 0.02
DEFAULT_TRACKING_BREACH_SAMPLES = 20


def _parse_vector(values: list[float] | None, default: np.ndarray, name: str) -> np.ndarray:
    arr = default.astype(np.float64) if values is None else np.asarray(values, dtype=np.float64)
    if arr.shape != default.shape: raise ValueError(f"{name} 长度不匹配")
    return arr


def _joint_id(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0: raise RuntimeError(f"XML 找不到: {joint_name}")
    return int(jid)


def _clip_rate(target: np.ndarray, previous: np.ndarray, max_step: np.ndarray) -> np.ndarray:
    return previous + np.clip(target - previous, -max_step, max_step)


def _unwrap_near(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return np.asarray(values, dtype=np.float64) + 2.0 * np.pi * np.round(
        (np.asarray(reference, dtype=np.float64) - np.asarray(values, dtype=np.float64)) / (2.0 * np.pi))


def _sim_to_real_unclipped(q_sim: np.ndarray, signs: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    return (np.asarray(q_sim, dtype=np.float64)[:6] - offsets) / signs


def read_stable_positions(arm, reference: np.ndarray, samples: int, interval: float) -> np.ndarray:
    values = []
    for _ in range(max(int(samples), 1)):
        values.append(_unwrap_near(np.asarray(arm.get_positions(request=True)[:6], dtype=np.float64), reference[:6]))
        time.sleep(max(float(interval), 0.0))
    return np.median(np.vstack(values), axis=0)


def close_arm_fast(arm) -> None:
    if arm is None: return
    try:
        arm.disable(retries=0); time.sleep(0.1)
    except:
        pass
    for ctrl in list(getattr(arm, "_ctrl_map", {}).values()):
        try:
            ctrl.shutdown(); time.sleep(0.02); ctrl.close()
        except:
            pass


class SimToRealMapper:
    def __init__(self, model, joint_names, signs, offsets, soft_margin):
        self.joint_names = tuple(joint_names)
        self.signs = np.asarray(signs, dtype=np.float64)
        self.offsets = np.asarray(offsets, dtype=np.float64)
        self.joint_ids = np.array([_joint_id(model, name) for name in self.joint_names], dtype=np.int32)
        sim_ranges = []
        for jid in self.joint_ids:
            sim_ranges.append(
                model.jnt_range[jid].copy() if int(model.jnt_limited[jid]) == 1 else np.array([-np.inf, np.inf],
                                                                                              dtype=np.float64))
        real_limits = (np.asarray(sim_ranges, dtype=np.float64) - self.offsets[:, None]) / self.signs[:, None]
        self.real_lower = np.minimum(real_limits[:, 0], real_limits[:, 1]) + soft_margin
        self.real_upper = np.maximum(real_limits[:, 0], real_limits[:, 1]) - soft_margin

    def real_to_sim(self, q_real: np.ndarray) -> np.ndarray: return np.asarray(q_real, dtype=np.float64)[
                                                                    :len(self.joint_names)] * self.signs + self.offsets

    def sim_to_real(self, q_sim: np.ndarray) -> np.ndarray: return np.clip(
        (np.asarray(q_sim, dtype=np.float64)[:len(self.joint_names)] - self.offsets) / self.signs, self.real_lower,
        self.real_upper)


class SafetyGuard:
    def __init__(self, mapper, max_step, max_start_error, max_tracking_error, tracking_breach_samples):
        self.mapper = mapper;
        self.max_step = np.asarray(max_step, dtype=np.float64)
        self.max_start_error = float(max_start_error);
        self.max_tracking_error = float(max_tracking_error)
        self.tracking_breach_samples = max(int(tracking_breach_samples), 1)
        self.command: np.ndarray | None = None;
        self._tracking_breach_count = 0

    def initialize(self, q_real_now, q_target, allow_large_start) -> np.ndarray:
        self.command = np.asarray(q_real_now, dtype=np.float64)[:6].copy();
        return self.command.copy()

    def next_command(self, q_target, q_feedback) -> np.ndarray:
        q_target = np.clip(np.asarray(q_target, dtype=np.float64)[:6], self.mapper.real_lower, self.mapper.real_upper)
        q_target_cmd = _unwrap_near(q_target, q_feedback)
        tracking_error = np.max(np.abs(q_target_cmd - q_feedback))
        if tracking_error > self.max_tracking_error:
            self._tracking_breach_count += 1
            if self._tracking_breach_count >= self.tracking_breach_samples: raise RuntimeError("达妙真机跟踪误差过大。")
        else:
            self._tracking_breach_count = 0
        self.command = _clip_rate(q_target_cmd, _unwrap_near(self.command, q_feedback), self.max_step)
        return self.command.copy()


def setup_damiao_gripper(arm, gripper_cfg_path, gripper_name_fallback="gripper"):
    if arm is None: return None, None, None
    load_gripper_cfg = _load_gripper_cfg_func()
    g_cfg = load_gripper_cfg(str(gripper_cfg_path))["gripper"]
    shared_damiao_controller = arm._ctrl_map["damiao"]
    gripper_name = getattr(g_cfg, "name", gripper_name_fallback)
    if gripper_name in arm._motor_map:
        g_mot = arm._motor_map[gripper_name]
    else:
        g_mot = shared_damiao_controller.add_damiao_motor(g_cfg.motor_id, g_cfg.feedback_id, g_cfg.model)
        arm._motor_map[gripper_name] = g_mot
    from motorbridge import Mode
    g_mot.ensure_mode(Mode.MIT, 1000)
    shared_damiao_controller.enable_all()
    time.sleep(0.2)
    return g_mot, shared_damiao_controller, gripper_name


def send_damiao_gripper_mit(g_mot, controller, target_rad, kp, kd, tau, request_feedback=True) -> bool:
    if g_mot is None: return False
    try:
        g_mot.send_mit(float(target_rad), 0.0, float(kp), float(kd), float(tau))
        if request_feedback:
            try:
                g_mot.request_feedback()
            except:
                pass
            if controller:
                try:
                    controller.poll_feedback_once()
                except:
                    pass
        return True
    except:
        return False


def get_gripper_feedback_pos(g_mot) -> float | None:
    if g_mot is None: return None
    try:
        return float(g_mot.get_state().pos)
    except:
        return None


def get_gripper_feedback_vel(g_mot) -> float | None:
    if g_mot is None: return None
    try:
        return float(g_mot.get_state().vel)
    except:
        return 0.0


def servo_reader_worker(scs, state_lock, shared_state, read_rate, servo_to_sim_sign, sim_home_rad, enable_gripper,
                        invert_gripper, closed_rad, open_rad):
    global _running
    read_period = 1.0 / max(float(read_rate), 1e-6)
    last_arm_deg = np.array([JOINT_LIMITS_DEG[i]["home_deg"] for i in ARM_SERVO_IDS], dtype=np.float64)
    last_gripper_deg = JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"]

    while _running:
        loop_start = time.perf_counter()
        arm_deg = np.array(last_arm_deg, dtype=np.float64)
        success_count = 0
        failed_ids = []
        for i, servo_id in enumerate(ARM_SERVO_IDS):
            angle_deg, ok = read_servo_angle(scs, servo_id, arm_deg[i])
            arm_deg[i] = angle_deg
            if ok:
                success_count += 1
            else:
                failed_ids.append(servo_id)
        last_arm_deg = arm_deg.copy()
        q_sim = servo_deg_array_to_sim_rad(arm_deg, servo_to_sim_sign, sim_home_rad)

        if enable_gripper:
            gripper_deg, gripper_ok = read_servo_angle(scs, GRIPPER_SERVO_ID, last_gripper_deg)
            if gripper_ok:
                success_count += 1
            else:
                failed_ids.append(GRIPPER_SERVO_ID)
            last_gripper_deg = float(gripper_deg)
            gripper_norm = gripper_servo_deg_to_norm(gripper_deg, invert_gripper)
            gripper_target_rad = gripper_norm_to_real_rad(gripper_norm, closed_rad, open_rad)
        else:
            gripper_deg, gripper_norm, gripper_target_rad = last_gripper_deg, 1.0, open_rad

        with state_lock:
            shared_state.update(
                {"arm_deg": arm_deg.copy(), "target_q_sim": q_sim.copy(), "gripper_deg": float(gripper_deg),
                 "gripper_norm": float(gripper_norm), "gripper_target_rad": float(gripper_target_rad),
                 "success_count": int(success_count), "failed_ids": list(failed_ids),
                 "timestamp": time.perf_counter(), "read_frame": shared_state["read_frame"] + 1})
        sleep_time = read_period - (time.perf_counter() - loop_start)
        if sleep_time > 0: time.sleep(sleep_time)


def wait_for_servo_ready(state_lock, shared_state, min_success_count, timeout=5.0) -> bool:
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        with state_lock:
            if shared_state.get("read_frame", 0) > 0 and shared_state.get("success_count",
                                                                          0) >= min_success_count: return True
        time.sleep(0.02)
    return False


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="舵机主手 -> 达妙真机遥操作 (带多相机真实数据采集)")
    parser.add_argument("--task_name", "-t", type=str, default="teleop_task", help="采集任务名称")
    parser.add_argument("--save_dir", "-d", type=str, default="./data", help="数据保存根目录")
    parser.add_argument("--episode_len", "-l", type=int, default=500, help="设定单次录制的时间步数")
    parser.add_argument("--time", "-sec", type=float, default=None, help="以秒为单位设定录制时长")
    parser.add_argument("--episode_idx", "-idx", type=int, default=None, help="显式指定录制索引号")
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
    parser.add_argument("--print-every", type=int, default=50)
    return parser


# =============================================================================
# 10. 主程序
# =============================================================================
def main() -> None:
    global _running, _is_recording, recorded_images

    args = build_argparser().parse_args()

    dt_actual = 1.0 / args.rate
    if args.time is not None:
        final_episode_len = int(args.time * args.rate)
    else:
        final_episode_len = args.episode_len

    _config["task_name"] = args.task_name
    _config["base_save_dir"] = Path(args.save_dir)
    _config["episode_len"] = final_episode_len
    _config["dt"] = dt_actual
    _config["rate"] = args.rate
    _config["episode_idx"] = args.episode_idx

    # 🌟 1. 初始化相机流全局缓存槽
    for cam_name in _config["camera_names"]:
        recorded_images[cam_name] = []

    # 🌟 2. 构建真实的相机读取对象字典（未来扩展无缝对接）
    # 如果相机打不开或不存在，字典里依然会有对象，但它的 read_rgb 会返回安全的黑屏图
    cameras = {}
    if "cam_high" in _config["camera_names"]:
        cameras["cam_high"] = ThreadedCamera(src=2, name="cam_high")
    if "cam_wrist" in _config["camera_names"]:
        # 预留了你以后的相机接口，指定不同 src 即可
        cameras["cam_wrist"] = ThreadedCamera(src=4, name="cam_wrist")

    enable_gripper = not args.no_gripper
    servo_to_sim_sign = _parse_vector(args.servo_to_sim_signs, DEFAULT_SERVO_TO_SIM_SIGN, "signs")
    sim_home_rad = _parse_vector(args.sim_home, DEFAULT_SIM_HOME_RAD, "home")
    signs = _parse_vector(args.signs, np.ones(6, dtype=np.float64), "signs")
    offsets = _parse_vector(args.offsets, np.zeros(6, dtype=np.float64), "offsets")
    vlim = _parse_vector(args.vlim, DEFAULT_CMD_VLIM, "vlim")
    max_step = _parse_vector(args.max_step, DEFAULT_MAX_STEP, "max-step")
    joint_names = tuple(x.strip() for x in args.joint_names.split(",") if x.strip())

    print("\n" + "=" * 60)
    print("  🚀 主从遥操作 + ALOHA 多相机规范数据采集系统就绪")
    print(f"  📝 目标任务名称: {args.task_name}")
    print(f"  ⚡ 控制与录制频率: {args.rate} Hz")
    print(f"  ⏱️ 单次录制规模: {final_episode_len} 步 ({final_episode_len * dt_actual:.2f} 秒)")
    print(f"  📷 已挂载真实相机: {_config['camera_names']}")
    print("=" * 60)

    model = mujoco.MjModel.from_xml_path(str(args.xml))

    portHandler = PortHandler(args.port)
    scs = sts(portHandler)
    if not portHandler.openPort() or not portHandler.setBaudRate(args.baudrate): return

    time.sleep(2.5)
    if not args.keep_servo_torque:
        release_servo_torque(scs, ARM_SERVO_IDS + ([GRIPPER_SERVO_ID] if enable_gripper else []))

    home_arm_deg = np.array([JOINT_LIMITS_DEG[i]["home_deg"] for i in ARM_SERVO_IDS], dtype=np.float64)
    home_q_sim = servo_deg_array_to_sim_rad(home_arm_deg, servo_to_sim_sign, sim_home_rad)
    home_gripper_target_rad = gripper_norm_to_real_rad(
        gripper_servo_deg_to_norm(JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"], args.invert_gripper),
        args.gripper_real_closed_rad, args.gripper_real_open_rad)

    state_lock = threading.Lock()
    shared_state = {
        "arm_deg": home_arm_deg.copy(), "target_q_sim": home_q_sim.copy(),
        "gripper_deg": float(JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"]),
        "gripper_norm": float(
            gripper_servo_deg_to_norm(JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"], args.invert_gripper)),
        "gripper_target_rad": float(home_gripper_target_rad),
        "success_count": 0, "failed_ids": [], "timestamp": time.perf_counter(), "read_frame": 0,
    }

    reader_thread = threading.Thread(target=servo_reader_worker, args=(
        scs, state_lock, shared_state, args.read_rate, servo_to_sim_sign, sim_home_rad, enable_gripper,
        args.invert_gripper,
        args.gripper_real_closed_rad, args.gripper_real_open_rad), daemon=True)
    reader_thread.start()

    if not wait_for_servo_ready(state_lock, shared_state, ARM_DOF + (1 if enable_gripper else 0)): return

    with state_lock:
        initial_q_sim = shared_state["target_q_sim"].copy()
        initial_gripper_target_rad = float(shared_state["gripper_target_rad"])

    RobotArm = _load_robot_arm_class()
    arm = RobotArm(cfg_path=str(args.cfg) if args.cfg else None)
    arm.connect()
    arm.enable()
    arm.mode_pos_vel(vlim=vlim)

    gripper_motor, gripper_controller = None, None
    if enable_gripper:
        gripper_motor, gripper_controller, _ = setup_damiao_gripper(arm, args.gripper_cfg)

    q_feedback = read_stable_positions(arm, _sim_to_real_unclipped(initial_q_sim, signs, offsets), args.settle_samples,
                                       args.settle_interval)
    if args.calibrate_current_as_master: offsets = initial_q_sim.copy() - signs * q_feedback[:6]

    mapper = SimToRealMapper(model, joint_names, signs, offsets, args.soft_margin)
    guard = SafetyGuard(mapper, max_step, args.max_start_error, args.max_tracking_error, args.tracking_breach_samples)
    q_cmd = guard.initialize(q_feedback, mapper.sim_to_real(initial_q_sim), args.allow_large_start)
    arm.pos_vel(q_cmd, vlim=vlim)

    filtered_gripper_target_rad = initial_gripper_target_rad
    if enable_gripper: send_damiao_gripper_mit(gripper_motor, gripper_controller, filtered_gripper_target_rad,
                                               args.gripper_kp, args.gripper_kd, args.gripper_tau)

    listener_thread = threading.Thread(target=terminal_keyboard_listener, daemon=True)
    listener_thread.start()
    print("\n📌 [录制就绪] 控制台按下【回车键】开始采集当前 Episode 数据...")

    cmd_period = 1.0 / args.rate
    filtered_q_sim = initial_q_sim.copy()
    frame = 0

    try:
        while _running:
            loop_start = time.perf_counter()

            with state_lock:
                target_q_sim_raw = shared_state["target_q_sim"].copy()
                gripper_target_rad_raw = float(shared_state["gripper_target_rad"])
                servo_age = loop_start - float(shared_state["timestamp"])

            if servo_age > args.max_servo_age: raise RuntimeError("舵机主手数据超时")

            filtered_q_sim = smooth_update(filtered_q_sim, target_q_sim_raw, args.alpha_master)
            q_target_real = mapper.sim_to_real(filtered_q_sim)

            q_feedback = np.asarray(arm.get_positions(request=True)[:6], dtype=np.float64)
            try:
                v_feedback = np.asarray(arm.get_velocities(request=True)[:6], dtype=np.float64)
            except:
                v_feedback = np.zeros(6, dtype=np.float64)

            q_feedback_unwrapped = _unwrap_near(q_feedback, q_cmd)
            q_cmd = guard.next_command(q_target_real, q_feedback_unwrapped)
            arm.pos_vel(q_cmd, vlim=vlim)

            gripper_fb_pos, gripper_fb_vel = 0.0, 0.0
            if enable_gripper:
                filtered_gripper_target_rad = smooth_update_scalar(filtered_gripper_target_rad, gripper_target_rad_raw,
                                                                   args.alpha_gripper)
                if frame % max(int(args.gripper_send_every), 1) == 0:
                    send_damiao_gripper_mit(gripper_motor, gripper_controller, filtered_gripper_target_rad,
                                            args.gripper_kp, args.gripper_kd, args.gripper_tau, request_feedback=True)

                g_pos = get_gripper_feedback_pos(gripper_motor)
                if g_pos is not None: gripper_fb_pos = g_pos
                g_vel = get_gripper_feedback_vel(gripper_motor)
                if g_vel is not None: gripper_fb_vel = g_vel

            # 🌟 3. 核心截断式采集层（读取真实的 RGB 相机流）
            if _is_recording:
                current_frame_count = len(recorded_timestamps)
                if current_frame_count < _config["episode_len"]:
                    rec_time = current_frame_count * _config["dt"]
                    recorded_timestamps.append(rec_time)

                    recorded_qpos.append(np.concatenate([q_feedback, [gripper_fb_pos]]))
                    recorded_qvel.append(np.concatenate([v_feedback, [gripper_fb_vel]]))
                    recorded_action.append(np.concatenate([q_cmd, [filtered_gripper_target_rad]]))

                    # 🌟 动态匹配已挂载的真实相机并获取 RGB 数据
                    for cam_name in _config["camera_names"]:
                        if cam_name in cameras:
                            recorded_images[cam_name].append(cameras[cam_name].read_rgb())
                        else:
                            # 降级容错机制：防止写错配置导致程序中断
                            recorded_images[cam_name].append(np.zeros((480, 640, 3), dtype=np.uint8))
                else:
                    _is_recording = False

            if frame % args.print_every == 0 and not _is_recording:
                print(
                    f"[{frame:06d}] fb_J1={q_feedback[0]:+.2f} | cmd_J1={q_cmd[0]:+.2f} | gripper_cmd={filtered_gripper_target_rad:+.2f}",
                    end="\r")

            frame += 1
            sleep_time = cmd_period - (time.perf_counter() - loop_start)
            if sleep_time > 0: time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"\n[停机] {exc}")
    finally:
        _running = False
        close_arm_fast(arm)
        portHandler.closePort()

        # 🌟 4. 安全退出：释放所有硬件相机占用
        for cam in cameras.values():
            cam.release()
        print("\n🧹 所有硬件资源已安全释放。")


if __name__ == "__main__":
    main()

# python record_real_episodes.py --xml /home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/master_slave_control/Servo_control/xml/rebot_gripper/sim_reBot_grasp.xml --port /dev/ttyUSB0 --baudrate 115200 --rate 50 --read-rate 60 --calibrate-current-as-master --task_name rebot_test --save_dir ./data --episode_len 1000 --episode_idx 0


