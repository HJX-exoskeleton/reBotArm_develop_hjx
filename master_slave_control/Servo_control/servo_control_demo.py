import time
import math
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import sys
import os

# ================= 动态环境变量注入 =================
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sdk_path = os.path.join(project_root, 'STservo_sdk')

if project_root not in sys.path:
    sys.path.append(project_root)

if sdk_path not in sys.path:
    sys.path.append(sdk_path)

from STservo_sdk import *  # 微雪舵机库


# ================= 1. 硬件参数 =================

# 如果 ESP32 透传代码是 Serial.begin(115200)，这里必须是 115200
# 如果 ESP32 透传代码是 Serial.begin(1000000)，这里必须改成 1000000
BAUDRATE = 115200

# Windows
# DEVICENAME = 'COM6'
# Linux
DEVICENAME = '/dev/ttyUSB0'

SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]
DOF = len(SERVO_IDS)

SERVO_DIGITAL_RANGE = 4095.0
SERVO_ANGLE_RANGE = 360.0


def angle_to_pos(angle):
    return int((angle / SERVO_ANGLE_RANGE) * SERVO_DIGITAL_RANGE)


def pos_to_angle(pos):
    return (pos / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE


def clamp_value(val, min_val, max_val):
    return max(min_val, min(val, max_val))


# ================= 2. 7 自由度软件限位 =================
# 你标定的机械极限位置
# 注意：这里按 min_deg ~ max_deg 作为软件限位范围
JOINT_LIMITS_DEG = {
    1: {
        "name": "ID1",
        "desc_min": "左偏",
        "desc_max": "右偏",
        "min_deg": 50.0,
        "max_deg": 300.0,
        "home_deg": 180.0,
    },
    2: {
        "name": "ID2",
        "desc_min": "前倾",
        "desc_max": "初始",
        "min_deg": 10.0,
        "max_deg": 180.0,
        "home_deg": 180.0,
    },
    3: {
        "name": "ID3",
        "desc_min": "抬头极限",
        "desc_max": "初始",
        "min_deg": 22.0,
        "max_deg": 180.0,
        "home_deg": 180.0,
    },
    4: {
        "name": "ID4",
        "desc_min": "低头",
        "desc_max": "抬头",
        "min_deg": 100.0,
        "max_deg": 270.0,
        "home_deg": 180.0,
    },
    5: {
        "name": "ID5",
        "desc_min": "右偏",
        "desc_max": "左偏",
        "min_deg": 90.0,
        "max_deg": 270.0,
        "home_deg": 180.0,
    },
    6: {
        "name": "ID6",
        "desc_min": "逆转",
        "desc_max": "顺转",
        "min_deg": 90.0,
        "max_deg": 270.0,
        "home_deg": 180.0,
    },
    7: {
        "name": "ID7",
        "desc_min": "极限位置",
        "desc_max": "初始",
        "min_deg": 90.0,
        "max_deg": 180.0,
        "home_deg": 180.0,
    },
}

# 软件限位安全余量，建议先留 3~5 度，避免贴边撞限位
SAFETY_MARGIN_DEG = 3.0
# 是否启用安全余量
USE_SAFETY_MARGIN = False  # True


def get_safe_min_deg(servo_id):
    min_deg = JOINT_LIMITS_DEG[servo_id]["min_deg"]
    max_deg = JOINT_LIMITS_DEG[servo_id]["max_deg"]

    if USE_SAFETY_MARGIN:
        return min_deg + SAFETY_MARGIN_DEG

    return min_deg


def get_safe_max_deg(servo_id):
    min_deg = JOINT_LIMITS_DEG[servo_id]["min_deg"]
    max_deg = JOINT_LIMITS_DEG[servo_id]["max_deg"]

    if USE_SAFETY_MARGIN:
        return max_deg - SAFETY_MARGIN_DEG

    return max_deg


def limit_angle(servo_id, angle):
    """
    对角度进行软件限位。
    所有目标角度都必须先经过这个函数。
    """
    safe_min = get_safe_min_deg(servo_id)
    safe_max = get_safe_max_deg(servo_id)

    return clamp_value(angle, safe_min, safe_max)


def limit_pos(servo_id, pos):
    """
    对原始位置值进行软件限位。
    """
    angle = pos_to_angle(pos)
    safe_angle = limit_angle(servo_id, angle)
    return angle_to_pos(safe_angle)


def safe_angle_to_pos(servo_id, angle):
    """
    角度 -> 软件限位 -> 原始位置值。
    """
    safe_angle = limit_angle(servo_id, angle)
    return angle_to_pos(safe_angle)


def print_joint_limits():
    print("\n" + "=" * 80)
    print("当前 7 自由度软件限位参数")
    print("=" * 80)

    for servo_id in SERVO_IDS:
        cfg = JOINT_LIMITS_DEG[servo_id]
        safe_min = get_safe_min_deg(servo_id)
        safe_max = get_safe_max_deg(servo_id)

        print(
            f"ID={servo_id} | "
            f"机械范围: {cfg['min_deg']:.1f}° ~ {cfg['max_deg']:.1f}° | "
            f"软件安全范围: {safe_min:.1f}° ~ {safe_max:.1f}° | "
            f"Home: {cfg['home_deg']:.1f}°"
        )

    print("=" * 80 + "\n")


# ================= 3. 7 自由度 demo 轨迹参数 =================
# Home 角度全部从软件限位表中读取
HOME_DEG = [
    JOINT_LIMITS_DEG[servo_id]["home_deg"]
    for servo_id in SERVO_IDS
]

HOME_POS = [
    safe_angle_to_pos(servo_id, JOINT_LIMITS_DEG[servo_id]["home_deg"])
    for servo_id in SERVO_IDS
]

# 每个关节的 demo 运动幅度。
# 这里不是最终强制值，最终仍然会经过软件限位。
# 对于 ID2、ID3、ID7 这种 home 在最大边界附近的关节，代码会自动采用单侧运动。
DEMO_AMPLITUDE_DEG = {
    1: 40.0,
    2: 10.0,
    3: 45.0,
    4: 10.0,
    5: 15.0,
    6: 25.0,
    7: 45.0,
}

PHASE_RAD = {
    1: 0.0,
    2: math.pi / 8,
    3: 2 * math.pi / 5,
    4: 3 * math.pi / 5,
    5: 4 * math.pi / 5,
    6: math.pi,
    7: 6 * math.pi / 5,
}

FREQ_SCALE = {
    1: 1.0,
    2: 0.8,
    3: 0.8,
    4: 1.5,
    5: 1.1,
    6: 0.7,
    7: 1.3,
}

# 整体时间缩放，越大运动越快
TIME_SCALE = 1.0


def compute_single_joint_demo_angle(servo_id, t):
    """
    根据每个关节的限位自动生成安全 demo 角度。

    逻辑：
    1. 如果 home 在范围中间，做双向正弦运动；
    2. 如果 home 接近最大边界，例如 ID2/ID3/ID7，则只向 min 方向单侧运动；
    3. 如果 home 接近最小边界，则只向 max 方向单侧运动；
    4. 最后仍然经过 limit_angle 二次保护。
    """
    safe_min = get_safe_min_deg(servo_id)
    safe_max = get_safe_max_deg(servo_id)
    home = limit_angle(servo_id, JOINT_LIMITS_DEG[servo_id]["home_deg"])

    amp_cmd = DEMO_AMPLITUDE_DEG[servo_id]
    phase = PHASE_RAD[servo_id]
    freq = FREQ_SCALE[servo_id]

    s = math.sin(freq * t + phase)

    dist_to_min = home - safe_min
    dist_to_max = safe_max - home

    eps = 1e-6

    # home 靠近最大边界，只能向 min 方向运动
    if dist_to_max <= 1.0:
        amp = min(amp_cmd, dist_to_min)
        # wave: 0~1
        wave = 0.5 * (1.0 + s)
        angle = home - amp * wave

    # home 靠近最小边界，只能向 max 方向运动
    elif dist_to_min <= 1.0:
        amp = min(amp_cmd, dist_to_max)
        wave = 0.5 * (1.0 + s)
        angle = home + amp * wave

    # home 在范围中间，可以双向摆动
    else:
        amp = min(amp_cmd, dist_to_min, dist_to_max)
        angle = home + amp * s

    return limit_angle(servo_id, angle)


def compute_demo_targets(t):
    """
    计算 7 自由度 demo 的目标角度和目标位置。
    """
    target_deg_list = []
    target_pos_list = []

    for servo_id in SERVO_IDS:
        safe_angle = compute_single_joint_demo_angle(servo_id, t)
        safe_pos = safe_angle_to_pos(servo_id, safe_angle)

        target_deg_list.append(safe_angle)
        target_pos_list.append(safe_pos)

    return target_deg_list, target_pos_list


# ================= 4. 控制与绘图参数 =================
CONTROL_HZ = 50.0
CONTROL_DT = 1.0 / CONTROL_HZ

# 7 个舵机全部读取可能拖慢。如果通信不稳，可以改成 2 或 3。
READ_EVERY_N_FRAMES = 2

MAX_POINTS = 150

is_running = True
data_lock = threading.Lock()

t_data = deque(maxlen=MAX_POINTS)

target_data = {
    servo_id: deque(maxlen=MAX_POINTS)
    for servo_id in SERVO_IDS
}

real_data = {
    servo_id: deque(maxlen=MAX_POINTS)
    for servo_id in SERVO_IDS
}


# ================= 5. 硬件初始化 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)

if not portHandler.openPort():
    print(f"❌ 串口打开失败: {DEVICENAME}")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print(f"❌ 波特率设置失败: {BAUDRATE}")
    portHandler.closePort()
    quit()

print(f"✅ 串口已打开: {DEVICENAME}")
print(f"✅ 波特率已设置: {BAUDRATE}")

print("等待 ESP32 进入透传状态...")
time.sleep(2.5)

try:
    portHandler.ser.reset_input_buffer()
    portHandler.ser.reset_output_buffer()
    print("✅ 已清空串口缓冲区")
except Exception as e:
    print(f"⚠️ 清空串口缓冲区失败，可忽略: {e}")

time.sleep(0.2)
print_joint_limits()


# ================= 6. 舵机控制函数 =================
def enable_all_torque():
    print("\n正在开启 ID=1~7 舵机力矩...")

    for servo_id in SERVO_IDS:
        result, error = scs.write1ByteTxRx(
            servo_id,
            STS_TORQUE_ENABLE,
            1
        )

        if result == COMM_SUCCESS:
            print(f"✅ ID={servo_id} 力矩已开启")
        else:
            print(f"⚠️ ID={servo_id} 力矩开启失败: {scs.getTxRxResult(result)}")

        time.sleep(0.03)


def disable_all_torque():
    print("\n正在释放 ID=1~7 舵机力矩...")

    for servo_id in SERVO_IDS:
        result, error = scs.write1ByteTxRx(
            servo_id,
            STS_TORQUE_ENABLE,
            0
        )

        if result == COMM_SUCCESS:
            print(f"✅ ID={servo_id} 力矩已释放")
        else:
            print(f"⚠️ ID={servo_id} 力矩释放失败: {scs.getTxRxResult(result)}")

        time.sleep(0.03)


def sync_write_positions(target_positions, speed=0, acc=0):
    """
    同步下发 7 个舵机位置。

    重要：
    即使外部传入了非法 target_positions，
    这里也会再次执行软件限位，防止越界。
    """
    if len(target_positions) != DOF:
        raise ValueError(f"target_positions 长度错误，应为 {DOF}，实际为 {len(target_positions)}")

    scs.groupSyncWrite.clearParam()

    safe_positions = []

    for servo_id, target_pos in zip(SERVO_IDS, target_positions):
        safe_pos = limit_pos(servo_id, int(target_pos))
        safe_positions.append(safe_pos)

        scs.SyncWritePosEx(
            servo_id,
            int(safe_pos),
            speed,
            acc
        )

    result = scs.groupSyncWrite.txPacket()
    return result, safe_positions


def move_to_home():
    print("\n正在复位到 7 自由度安全 Home 位...")

    result, safe_positions = sync_write_positions(
        target_positions=HOME_POS,
        speed=2000,
        acc=50
    )

    if result == COMM_SUCCESS:
        print(f"✅ 复位指令已下发: {[int(x) for x in safe_positions]}")
        print(f"✅ Home角度: {[round(pos_to_angle(x), 2) for x in safe_positions]}")
    else:
        print(f"⚠️ 复位指令下发异常: {scs.getTxRxResult(result)}")

    time.sleep(1.5)


def read_real_angles(last_real_angles):
    """
    读取 7 个舵机真实角度。
    如果某个舵机读取失败，则沿用上一帧角度。
    """
    real_angles = list(last_real_angles)

    for i, servo_id in enumerate(SERVO_IDS):
        pos, speed, result, error = scs.ReadPosSpeed(servo_id)

        if result == COMM_SUCCESS:
            raw_angle = pos_to_angle(pos)
            # 真实读数也做显示限位，避免异常包导致曲线跳飞
            real_angles[i] = limit_angle(servo_id, raw_angle)

    return real_angles


# ================= 7. 后台控制与读取线程 =================
def robot_control_thread():
    global is_running

    start_time = time.time()
    frame_count = 0

    last_real_angles = list(HOME_DEG)

    print("🤖 7 自由度 demo 控制线程已启动，已启用软件限位...")

    while is_running:
        loop_start = time.time()

        t = (time.time() - start_time) * TIME_SCALE
        frame_count += 1

        # 1. 计算 7 个舵机安全目标
        target_deg_list, target_pos_list = compute_demo_targets(t)

        # 2. 同步下发 7 个舵机目标位置
        result, safe_positions = sync_write_positions(
            target_positions=target_pos_list,
            speed=0,
            acc=0
        )

        # 3. 读取真实位置
        if frame_count % READ_EVERY_N_FRAMES == 0:
            last_real_angles = read_real_angles(last_real_angles)

        # 4. 数据压栈，供前台画图
        with data_lock:
            t_data.append(frame_count)

            for i, servo_id in enumerate(SERVO_IDS):
                target_data[servo_id].append(target_deg_list[i])
                real_data[servo_id].append(last_real_angles[i])

        # 5. 锁频
        time_spent = time.time() - loop_start
        time_left = CONTROL_DT - time_spent

        if time_left > 0:
            time.sleep(time_left)


# ================= 8. 启动硬件 =================
enable_all_torque()
move_to_home()


# ================= 9. 启动后台线程 =================
thread = threading.Thread(target=robot_control_thread)
thread.daemon = True
thread.start()


# ================= 10. 前台 Matplotlib 绘图 =================
plt.style.use('fast')

fig, axes = plt.subplots(7, 1, figsize=(12, 12), sharex=True)
fig.canvas.manager.set_window_title('7自由度舵机 Demo 轨迹追踪分析 - 软件限位版')

target_lines = {}
real_lines = {}

for i, servo_id in enumerate(SERVO_IDS):
    ax = axes[i]

    line_tgt, = ax.plot(
        [],
        [],
        linestyle='--',
        linewidth=1.2,
        label=f'Target ID:{servo_id}'
    )

    line_real, = ax.plot(
        [],
        [],
        linestyle='-',
        linewidth=1.6,
        label=f'Real ID:{servo_id}'
    )

    target_lines[servo_id] = line_tgt
    real_lines[servo_id] = line_real

    safe_min = get_safe_min_deg(servo_id)
    safe_max = get_safe_max_deg(servo_id)

    ax.set_ylim(safe_min - 10, safe_max + 10)
    ax.set_ylabel(f'ID{servo_id}\nAngle(°)')
    ax.grid(True, linestyle=':')
    ax.legend(loc='upper right', fontsize=8)

    # 画出软件限位线
    ax.axhline(safe_min, linestyle=':', linewidth=1)
    ax.axhline(safe_max, linestyle=':', linewidth=1)

axes[-1].set_xlabel('Time (frames)')


def update_plot(frame):
    with data_lock:
        x = list(t_data)

        for servo_id in SERVO_IDS:
            target_lines[servo_id].set_data(
                x,
                list(target_data[servo_id])
            )

            real_lines[servo_id].set_data(
                x,
                list(real_data[servo_id])
            )

        current_frame = x[-1] if len(x) > 0 else 0

    if current_frame > MAX_POINTS:
        for ax in axes:
            ax.set_xlim(current_frame - MAX_POINTS, current_frame)
    else:
        for ax in axes:
            ax.set_xlim(0, MAX_POINTS)

    all_lines = []
    for servo_id in SERVO_IDS:
        all_lines.append(target_lines[servo_id])
        all_lines.append(real_lines[servo_id])

    return all_lines


# ================= 11. 启动与清理 =================
try:
    print("📈 绘图界面已加载，关闭绘图窗口即可退出程序。")

    ani = FuncAnimation(
        fig,
        update_plot,
        interval=50,
        blit=False,
        cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()

except KeyboardInterrupt:
    pass

finally:
    print("\n🛑 正在执行安全退出程序...")

    is_running = False
    thread.join(timeout=1.0)

    try:
        print("正在回到安全 Home 位...")
        result, safe_positions = sync_write_positions(
            target_positions=HOME_POS,
            speed=1500,
            acc=40
        )
        time.sleep(1.0)
    except Exception as e:
        print(f"回 Home 异常: {e}")

    disable_all_torque()
    portHandler.closePort()
    print("程序已结束，串口已关闭。")

