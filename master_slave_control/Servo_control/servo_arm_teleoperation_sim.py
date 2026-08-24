#!/usr/bin/env python3
"""
ST/SMS_STS 舵机机械臂 -> MuJoCo Real2Sim 低延迟遥操作程序

重要：
- 保留 MuJoCo data.ctrl 控制逻辑；
- 不直接写 data.qpos；
- 因此 gripper actuator、接触和夹取物体逻辑不会被破坏。

适配当前 XML：
- MuJoCo 机械臂本体：joint1 ~ joint6
- MuJoCo 夹爪：gripper
- 真实舵机：ID1 ~ ID6 映射 joint1 ~ joint6
- 真实舵机：ID7 映射 gripper 开合
"""

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


# ================= 动态环境变量注入 =================
current_dir = Path(__file__).resolve().parent
python_root = current_dir.parent
sdk_path = python_root / "STservo_sdk"

print(f"[路径检查] 当前脚本目录: {current_dir}")
print(f"[路径检查] Python根目录: {python_root}")
print(f"[路径检查] SDK目录: {sdk_path}")

if not sdk_path.exists():
    print(f"❌ 未找到 STservo_sdk 目录: {sdk_path}")
    print("请确认你的目录结构是否类似：")
    print(r"H:\hjx_ws\rebot_arm_servo_7dof\Python\STservo_sdk")
    sys.exit(1)

for p in [str(python_root), str(sdk_path)]:
    if p not in sys.path:
        sys.path.append(p)

from STservo_sdk import *


# ================= 1. 全局运行标志 =================
_running = True


def _sigint_handler(signum, frame):
    global _running
    print("\n[real2sim] 收到退出信号，准备安全关闭...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)


# ================= 2. 默认路径与串口 =================
# DEFAULT_XML = current_dir / "xml" / "rebot_gripper" / "sim_reBot_grasp.xml"
DEFAULT_XML = "/home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/mujoco/xml/rebotarm_b601_colored/sim_rebotarm_colored_grasp.xml"

if os.name == "nt":
    DEFAULT_PORT = "COM6"
else:
    DEFAULT_PORT = "/dev/ttyUSB0"


# ================= 3. 舵机与 MuJoCo 映射参数 =================
ARM_SERVO_IDS = [1, 2, 3, 4, 5, 6]
GRIPPER_SERVO_ID = 7

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_ACTUATOR_NAME = "gripper_position"  # old: gripper

ARM_DOF = len(ARM_SERVO_IDS)

SERVO_DIGITAL_RANGE = 4095.0
SERVO_ANGLE_RANGE = 360.0


# ================= 4. 真实机械臂软件限位 =================
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


# ================= 5. Real -> Sim 方向映射 =================
# 如果某个 MuJoCo 关节方向反了，改对应正负号。
REAL_TO_SIM_SIGN = np.array(
    [
        -1.0,  # ID1 -> joint1
        1.0,   # ID2 -> joint2
        1.0,   # ID3 -> joint3
        -1.0,  # ID4 -> joint4
        -1.0,  # ID5 -> joint5
        -1.0,  # ID6 -> joint6
    ],
    dtype=np.float32,
)

# MuJoCo home 偏置：默认真实 180° -> MuJoCo 0 rad
SIM_HOME_RAD = np.array(
    [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)


# ================= 6. ID7 -> gripper 映射 =================
GRIPPER_SIM_CLOSED_METER = 0.001
GRIPPER_SIM_OPEN_METER = 0.05

# 根据你的 ID7 标定：
# ID7 初始 180°，极限 90°
# 默认：
# 180° -> gripper open
# 90°  -> gripper closed
GRIPPER_REAL_OPEN_DEG = 180.0
GRIPPER_REAL_CLOSED_DEG = 90.0

# 如果夹爪开合反了，改成 True
INVERT_GRIPPER = False


# ================= 7. 工具函数 =================
def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))


def servo_pos_to_deg(pos):
    return (float(pos) / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE


def deg_to_rad(deg):
    return float(deg) * np.pi / 180.0


def limit_real_deg(servo_id, angle_deg):
    cfg = JOINT_LIMITS_DEG[servo_id]
    min_deg = cfg["min_deg"] + SAFETY_MARGIN_DEG
    max_deg = cfg["max_deg"] - SAFETY_MARGIN_DEG
    return clamp(float(angle_deg), min_deg, max_deg)


def real_arm_deg_to_sim_rad(servo_id, angle_deg, arm_index):
    cfg = JOINT_LIMITS_DEG[servo_id]
    home_deg = cfg["home_deg"]

    safe_deg = limit_real_deg(servo_id, angle_deg)
    delta_deg = safe_deg - home_deg

    sim_rad = SIM_HOME_RAD[arm_index] + REAL_TO_SIM_SIGN[arm_index] * deg_to_rad(delta_deg)
    return float(sim_rad)


def real_gripper_deg_to_sim_meter(
    angle_deg,
    ctrl_min=GRIPPER_SIM_CLOSED_METER,
    ctrl_max=GRIPPER_SIM_OPEN_METER,
):
    angle_deg = limit_real_deg(GRIPPER_SERVO_ID, angle_deg)

    denom = GRIPPER_REAL_OPEN_DEG - GRIPPER_REAL_CLOSED_DEG

    if abs(denom) < 1e-6:
        normalized = 0.0
    else:
        normalized = (angle_deg - GRIPPER_REAL_CLOSED_DEG) / denom

    normalized = clamp(normalized, 0.0, 1.0)

    if INVERT_GRIPPER:
        normalized = 1.0 - normalized

    sim_meter = ctrl_min + normalized * (ctrl_max - ctrl_min)
    sim_meter = clamp(sim_meter, ctrl_min, ctrl_max)

    return float(sim_meter), float(normalized)


def smooth_update(prev, target, alpha):
    """
    一阶低通滤波。
    alpha 越大越跟手。
    alpha=1.0 表示不滤波。
    """
    alpha = clamp(float(alpha), 0.0, 1.0)
    return alpha * target + (1.0 - alpha) * prev


# ================= 8. 舵机读取 =================
def read_servo_angle(scs, servo_id, last_angle):
    try:
        pos, speed, result, error = scs.ReadPosSpeed(servo_id)

        if result == COMM_SUCCESS:
            angle_deg = servo_pos_to_deg(pos)
            angle_deg = limit_real_deg(servo_id, angle_deg)
            return angle_deg, True

        return last_angle, False

    except Exception:
        return last_angle, False


def arm_deg_array_to_sim_rad(arm_deg_array):
    sim_q = np.zeros(ARM_DOF, dtype=np.float32)

    for i, servo_id in enumerate(ARM_SERVO_IDS):
        sim_q[i] = real_arm_deg_to_sim_rad(
            servo_id=servo_id,
            angle_deg=arm_deg_array[i],
            arm_index=i,
        )

    return sim_q


def release_servo_torque(scs, servo_ids):
    print("\n🔓 正在释放真实舵机力矩，用于手动拖动示教...")

    for servo_id in servo_ids:
        try:
            result, error = scs.write1ByteTxRx(
                servo_id,
                STS_TORQUE_ENABLE,
                0,
            )

            if result == COMM_SUCCESS:
                print(f"✅ ID={servo_id} 力矩已释放")
            else:
                print(f"⚠️ ID={servo_id} 力矩释放失败: {scs.getTxRxResult(result)}")

        except Exception as e:
            print(f"⚠️ ID={servo_id} 力矩释放异常: {e}")

        time.sleep(0.02)


# ================= 9. MuJoCo 执行器查找 =================
def find_joint_actuators(model, joint_names):
    """
    根据 joint name 查找绑定 actuator。
    你的 XML 前 6 个 position actuator 没有 name，
    所以这里通过 actuator_trnid 反查 joint。
    """
    actuator_ids = []

    print("\n🔍 MuJoCo 执行器列表:")
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            i,
        )
        trn_type = int(model.actuator_trntype[i])
        trn_id = int(model.actuator_trnid[i, 0])
        print(f"  actuator[{i}] name={act_name}, trntype={trn_type}, trnid={trn_id}")

    print("\n🔍 按 joint 名称匹配 actuator:")

    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name,
        )

        if joint_id < 0:
            actuator_ids.append(-1)
            print(f"❌ XML 中未找到 joint: {joint_name}")
            continue

        found = False

        for i in range(model.nu):
            if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
                trn_id = model.actuator_trnid[i, 0]

                if trn_id == joint_id:
                    act_name = mujoco.mj_id2name(
                        model,
                        mujoco.mjtObj.mjOBJ_ACTUATOR,
                        i,
                    )
                    actuator_ids.append(i)
                    print(f"✅ {joint_name} -> actuator[{i}] name={act_name}")
                    found = True
                    break

        if not found:
            actuator_ids.append(-1)
            print(f"❌ 未找到 joint {joint_name} 对应的 actuator")

    return actuator_ids


def find_gripper_actuator(model):
    act_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        GRIPPER_ACTUATOR_NAME,
    )

    if act_id >= 0:
        print(f"✅ 找到 gripper actuator: {GRIPPER_ACTUATOR_NAME}, id={act_id}")
        return act_id

    print(f"⚠️ 未找到名为 {GRIPPER_ACTUATOR_NAME} 的 gripper actuator")
    return -1


def clamp_ctrl_to_actuator_range(model, act_id, value):
    if act_id < 0:
        return value

    if model.actuator_ctrllimited[act_id]:
        ctrl_min = float(model.actuator_ctrlrange[act_id, 0])
        ctrl_max = float(model.actuator_ctrlrange[act_id, 1])
        return clamp(float(value), ctrl_min, ctrl_max)

    return float(value)


def set_arm_ctrl(model, data, actuator_ids, sim_q):
    """
    机械臂本体控制：
    只写 data.ctrl，不写 data.qpos。
    """
    for i, act_id in enumerate(actuator_ids):
        if act_id < 0:
            continue

        cmd = clamp_ctrl_to_actuator_range(model, act_id, float(sim_q[i]))
        data.ctrl[act_id] = cmd


def set_gripper_ctrl(model, data, gripper_act_id, gripper_cmd):
    """
    夹爪控制：
    只写 data.ctrl，保留 gripper actuator 和接触动力学。
    """
    if gripper_act_id < 0:
        return

    cmd = clamp_ctrl_to_actuator_range(model, gripper_act_id, float(gripper_cmd))
    data.ctrl[gripper_act_id] = cmd


def print_mapping_info(joint_names):
    print("\n" + "=" * 90)
    print("Real2Sim 映射关系")
    print("=" * 90)

    for i, servo_id in enumerate(ARM_SERVO_IDS):
        cfg = JOINT_LIMITS_DEG[servo_id]
        print(
            f"ID{servo_id} real [{cfg['min_deg']:.1f}, {cfg['max_deg']:.1f}] deg, "
            f"home={cfg['home_deg']:.1f} deg "
            f"-> {joint_names[i]} sim, sign={REAL_TO_SIM_SIGN[i]:+.0f}, "
            f"sim_home={SIM_HOME_RAD[i]:+.3f} rad"
        )

    print(
        f"ID{GRIPPER_SERVO_ID} real "
        f"[{GRIPPER_REAL_CLOSED_DEG:.1f}, {GRIPPER_REAL_OPEN_DEG:.1f}] deg "
        f"-> gripper sim "
        f"[{GRIPPER_SIM_CLOSED_METER:.3f}, {GRIPPER_SIM_OPEN_METER:.3f}] m"
    )

    print("=" * 90 + "\n")


# ================= 10. 舵机读取线程 =================
def servo_reader_worker(
    scs,
    state_lock,
    shared_state,
    read_rate,
    no_gripper,
    gripper_ctrl_min,
    gripper_ctrl_max,
):
    """
    独立线程读取真实舵机。
    这样串口读取不会阻塞 MuJoCo 的 data.ctrl 控制循环。
    """
    global _running

    read_period = 1.0 / max(read_rate, 1e-6)

    last_arm_deg = np.array(
        [JOINT_LIMITS_DEG[servo_id]["home_deg"] for servo_id in ARM_SERVO_IDS],
        dtype=np.float32,
    )

    last_gripper_deg = JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"]

    print(f"📡 舵机读取线程已启动，目标读取频率: {read_rate:.1f} Hz")

    while _running:
        loop_start = time.perf_counter()

        arm_deg = np.array(last_arm_deg, dtype=np.float32)

        success_count = 0
        failed_ids = []

        for i, servo_id in enumerate(ARM_SERVO_IDS):
            angle, ok = read_servo_angle(scs, servo_id, arm_deg[i])
            arm_deg[i] = angle

            if ok:
                success_count += 1
            else:
                failed_ids.append(servo_id)

        if not no_gripper:
            gripper_deg, ok = read_servo_angle(
                scs,
                GRIPPER_SERVO_ID,
                last_gripper_deg,
            )

            if ok:
                success_count += 1
            else:
                failed_ids.append(GRIPPER_SERVO_ID)

        else:
            gripper_deg = last_gripper_deg

        last_arm_deg = arm_deg
        last_gripper_deg = float(gripper_deg)

        target_arm_q = arm_deg_array_to_sim_rad(arm_deg)

        target_gripper_cmd, gripper_norm = real_gripper_deg_to_sim_meter(
            gripper_deg,
            ctrl_min=gripper_ctrl_min,
            ctrl_max=gripper_ctrl_max,
        )

        now = time.perf_counter()

        with state_lock:
            shared_state["arm_deg"] = arm_deg.copy()
            shared_state["gripper_deg"] = float(gripper_deg)
            shared_state["target_arm_q"] = target_arm_q.copy()
            shared_state["target_gripper_cmd"] = float(target_gripper_cmd)
            shared_state["gripper_norm"] = float(gripper_norm)
            shared_state["success_count"] = int(success_count)
            shared_state["failed_ids"] = list(failed_ids)
            shared_state["timestamp"] = now
            shared_state["read_frame"] += 1

        elapsed = time.perf_counter() - loop_start
        sleep_time = read_period - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)


# ================= 11. 主程序 =================
def main():
    global _running

    parser = argparse.ArgumentParser(
        description="Low-latency ST/SMS_STS Servo Arm -> MuJoCo Real2Sim using data.ctrl"
    )

    parser.add_argument(
        "--xml",
        type=Path,
        default=DEFAULT_XML,
        help="MuJoCo XML 文件路径",
    )

    parser.add_argument(
        "--port",
        type=str,
        default=DEFAULT_PORT,
        help="舵机串口，例如 Windows: COM6, Linux: /dev/ttyUSB0",
    )

    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="电脑到 ESP32 的串口波特率，必须和 ESP32 Serial.begin 一致",
    )

    parser.add_argument(
        "--read-rate",
        type=float,
        default=60.0,
        help="真实舵机读取线程频率，默认 60Hz",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=400.0,
        help="MuJoCo data.ctrl 控制循环频率。你的 XML timestep=0.0025 时建议 400Hz",
    )

    parser.add_argument(
        "--viewer-hz",
        type=float,
        default=60.0,
        help="viewer 刷新频率，默认 60Hz",
    )

    parser.add_argument(
        "--alpha-arm",
        type=float,
        default=0.90,
        help="机械臂滤波系数，越大越跟手。1.0 表示不滤波",
    )

    parser.add_argument(
        "--alpha-gripper",
        type=float,
        default=0.90,
        help="夹爪滤波系数，越大越跟手。1.0 表示不滤波",
    )

    parser.add_argument(
        "--joint-names",
        type=str,
        default="joint1,joint2,joint3,joint4,joint5,joint6",
        help="MuJoCo 中机械臂 joint 名称，用逗号分隔",
    )

    parser.add_argument(
        "--keep-torque",
        action="store_true",
        help="如果设置该参数，则不释放真实舵机力矩",
    )

    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="如果设置该参数，则不读取 ID7，不控制 gripper",
    )

    args = parser.parse_args()

    joint_names = [x.strip() for x in args.joint_names.split(",") if x.strip()]

    if len(joint_names) != ARM_DOF:
        print(f"❌ joint-names 数量必须是 {ARM_DOF}，当前是 {len(joint_names)}")
        print(f"当前 joint_names = {joint_names}")
        return

    if not args.xml.exists():
        print(f"❌ XML 文件不存在: {args.xml}")
        return

    # ================= A. 加载 MuJoCo =================
    print(f"\n📂 正在加载 MuJoCo XML: {args.xml}")

    try:
        model = mujoco.MjModel.from_xml_path(str(args.xml))
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"❌ MuJoCo XML 加载失败: {e}")
        return

    arm_actuator_ids = find_joint_actuators(model, joint_names)
    gripper_act_id = -1 if args.no_gripper else find_gripper_actuator(model)

    if all(act_id < 0 for act_id in arm_actuator_ids):
        print("❌ 没有找到任何机械臂 actuator，请检查 XML 中 joint 名称。")
        return

    if gripper_act_id >= 0:
        gripper_ctrl_min = float(model.actuator_ctrlrange[gripper_act_id, 0])
        gripper_ctrl_max = float(model.actuator_ctrlrange[gripper_act_id, 1])
    else:
        gripper_ctrl_min = GRIPPER_SIM_CLOSED_METER
        gripper_ctrl_max = GRIPPER_SIM_OPEN_METER

    print_mapping_info(joint_names)

    print(f"MuJoCo timestep = {model.opt.timestep:.6f} s")
    print("当前模式：data.ctrl actuator 控制模式")
    print("说明：不会直接写 data.qpos，夹取物体逻辑保留。")

    # ================= B. 初始化舵机串口 =================
    print(f"\n🔌 正在打开舵机串口: {args.port}, baudrate={args.baudrate}")

    portHandler = PortHandler(args.port)
    scs = sts(portHandler)

    try:
        if not portHandler.openPort():
            print(f"❌ 串口打开失败: {args.port}")
            return

        if not portHandler.setBaudRate(args.baudrate):
            print(f"❌ 串口波特率设置失败: {args.baudrate}")
            portHandler.closePort()
            return

    except Exception as e:
        print(f"❌ 串口打开异常: {e}")
        print("请检查：")
        print("1. Windows 下是否使用 COM6 / COM3，而不是 /dev/ttyUSB0")
        print("2. 设备管理器里的实际 COM 号是否正确")
        print("3. 串口是否被 Arduino IDE / 串口助手 / 其他 Python 程序占用")
        return

    print("✅ 舵机串口已打开")

    print("等待 ESP32 进入透传状态...")
    time.sleep(2.5)

    try:
        portHandler.ser.reset_input_buffer()
        portHandler.ser.reset_output_buffer()
        print("✅ 已清空串口缓冲区")
    except Exception as e:
        print(f"⚠️ 清空串口缓冲区失败，可忽略: {e}")

    if not args.keep_torque:
        servo_ids_to_release = ARM_SERVO_IDS.copy()

        if not args.no_gripper:
            servo_ids_to_release.append(GRIPPER_SERVO_ID)

        release_servo_torque(scs, servo_ids_to_release)

    else:
        print("⚠️ keep-torque 已启用，不释放真实舵机力矩。")

    # ================= C. 初始化共享状态 =================
    home_arm_deg = np.array(
        [JOINT_LIMITS_DEG[servo_id]["home_deg"] for servo_id in ARM_SERVO_IDS],
        dtype=np.float32,
    )

    home_arm_q = arm_deg_array_to_sim_rad(home_arm_deg)

    gripper_cmd_init, _ = real_gripper_deg_to_sim_meter(
        JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"],
        ctrl_min=gripper_ctrl_min,
        ctrl_max=gripper_ctrl_max,
    )

    state_lock = threading.Lock()

    shared_state = {
        "arm_deg": home_arm_deg.copy(),
        "gripper_deg": JOINT_LIMITS_DEG[GRIPPER_SERVO_ID]["home_deg"],
        "target_arm_q": home_arm_q.copy(),
        "target_gripper_cmd": float(gripper_cmd_init),
        "gripper_norm": 1.0,
        "success_count": 0,
        "failed_ids": [],
        "timestamp": time.perf_counter(),
        "read_frame": 0,
    }

    filtered_arm_q = home_arm_q.copy()
    filtered_gripper_cmd = float(gripper_cmd_init)

    # 初始 data.ctrl
    set_arm_ctrl(
        model=model,
        data=data,
        actuator_ids=arm_actuator_ids,
        sim_q=filtered_arm_q,
    )

    if not args.no_gripper and gripper_act_id >= 0:
        set_gripper_ctrl(
            model=model,
            data=data,
            gripper_act_id=gripper_act_id,
            gripper_cmd=filtered_gripper_cmd,
        )

    # ================= D. 启动舵机读取线程 =================
    reader_thread = threading.Thread(
        target=servo_reader_worker,
        args=(
            scs,
            state_lock,
            shared_state,
            args.read_rate,
            args.no_gripper,
            gripper_ctrl_min,
            gripper_ctrl_max,
        ),
        daemon=True,
    )

    reader_thread.start()

    # ================= E. MuJoCo Viewer 主循环 =================
    sim_period = 1.0 / max(args.rate, 1e-6)
    viewer_period = 1.0 / max(args.viewer_hz, 1e-6)

    last_viewer_sync = 0.0
    last_print_time = 0.0
    frame = 0
    missing_count = 0

    # 如果 XML timestep=0.0025，rate=400Hz 时每轮 step 一次就是实时。
    # 如果用户设置较低 rate，则每轮补多个 mj_step，避免仿真物理时间变慢。
    timestep = max(float(model.opt.timestep), 1e-9)
    steps_per_loop = max(1, int(round(sim_period / timestep)))

    print(f"data.ctrl 主循环频率: {args.rate:.1f} Hz")
    print(f"每轮 mj_step 次数: {steps_per_loop}")
    print(f"viewer 刷新频率: {args.viewer_hz:.1f} Hz")
    print(f"arm alpha: {args.alpha_arm:.2f}, gripper alpha: {args.alpha_gripper:.2f}")

    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            print("\n👀 MuJoCo viewer 已启动")
            print("💡 当前保持 data.ctrl 控制逻辑，可正常夹取仿真物体")
            print("💡 拖动真实舵机机械臂，MuJoCo 会同步跟随")
            print("💡 按 Ctrl+C 或关闭 viewer 退出")

            while _running and viewer.is_running():
                loop_start = time.perf_counter()
                now = time.perf_counter()

                # 1. 读取最新舵机目标
                with state_lock:
                    target_arm_q = shared_state["target_arm_q"].copy()
                    target_gripper_cmd = float(shared_state["target_gripper_cmd"])
                    arm_deg = shared_state["arm_deg"].copy()
                    gripper_deg = float(shared_state["gripper_deg"])
                    success_count = int(shared_state["success_count"])
                    failed_ids = list(shared_state["failed_ids"])
                    data_age = now - float(shared_state["timestamp"])

                expected_count = ARM_DOF + (0 if args.no_gripper else 1)

                if success_count < expected_count:
                    missing_count += 1

                # 2. 低通滤波，alpha 越大延迟越小
                filtered_arm_q = smooth_update(
                    filtered_arm_q,
                    target_arm_q,
                    args.alpha_arm,
                )

                filtered_gripper_cmd = smooth_update(
                    filtered_gripper_cmd,
                    target_gripper_cmd,
                    args.alpha_gripper,
                )

                # 3. 写入 data.ctrl
                set_arm_ctrl(
                    model=model,
                    data=data,
                    actuator_ids=arm_actuator_ids,
                    sim_q=filtered_arm_q,
                )

                if not args.no_gripper and gripper_act_id >= 0:
                    set_gripper_ctrl(
                        model=model,
                        data=data,
                        gripper_act_id=gripper_act_id,
                        gripper_cmd=filtered_gripper_cmd,
                    )

                # 4. 推进 MuJoCo
                for _ in range(steps_per_loop):
                    mujoco.mj_step(model, data)

                # 5. viewer 降频刷新，避免渲染拖慢控制
                now_after_step = time.perf_counter()

                if now_after_step - last_viewer_sync >= viewer_period:
                    viewer.sync()
                    last_viewer_sync = now_after_step

                # 6. 状态打印，1Hz
                if now_after_step - last_print_time >= 1.0:
                    deg_str = " ".join([f"{x:6.1f}" for x in arm_deg])
                    rad_str = " ".join([f"{x:+6.2f}" for x in filtered_arm_q])

                    if not args.no_gripper:
                        print(
                            f"[{frame:06d}] "
                            f"arm_deg=[{deg_str}] | "
                            f"sim_rad=[{rad_str}] | "
                            f"ID7={gripper_deg:6.1f}deg -> "
                            f"gripper={filtered_gripper_cmd:.4f}m | "
                            f"age={data_age * 1000:.1f}ms | "
                            f"miss={missing_count} | failed={failed_ids}"
                        )
                    else:
                        print(
                            f"[{frame:06d}] "
                            f"arm_deg=[{deg_str}] | "
                            f"sim_rad=[{rad_str}] | "
                            f"age={data_age * 1000:.1f}ms | "
                            f"miss={missing_count} | failed={failed_ids}"
                        )

                    last_print_time = now_after_step

                frame += 1

                # 7. 控制循环锁频
                elapsed = time.perf_counter() - loop_start
                sleep_time = sim_period - elapsed

                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif elapsed > sim_period + 0.05:
                    print(
                        f"⚠️ real2sim 主循环严重超时: "
                        f"{elapsed * 1000:.1f} ms, "
                        f"目标周期 {sim_period * 1000:.1f} ms"
                    )

    except KeyboardInterrupt:
        _running = False

    finally:
        print("\n[退出流程] 正在关闭 real2sim...")
        _running = False

        try:
            reader_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            portHandler.closePort()
            print("[退出流程] 舵机串口已关闭")
        except Exception:
            pass

        print("[退出流程] 安全退出。")


if __name__ == "__main__":
    main()

# 滤波关掉的指令
# python servo_arm_teleoperation_sim.py --xml /home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/master_slave_control/Servo_control/xml/rebot_gripper/sim_reBot_grasp.xml --port /dev/ttyUSB0 --baudrate 115200 --read-rate 60 --rate 400 --viewer-hz 60 --alpha-arm 1.0 --alpha-gripper 1.0

