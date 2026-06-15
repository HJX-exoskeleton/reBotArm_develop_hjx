import time
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


# ================= 1. 基础硬件参数配置 =================
# 必须与 ESP32 透传代码中的 Serial.begin(115200) 保持一致
BAUDRATE = 115200

# Windows
# DEVICENAME = 'COM6'
# Linux
DEVICENAME = '/dev/ttyUSB0'

# 舵机 ID 范围：1 ~ 7
SERVO_ID_START = 1
SERVO_ID_END = 7
SERVO_IDS = list(range(SERVO_ID_START, SERVO_ID_END + 1))

SERVO_DIGITAL_RANGE = 4095.0
SERVO_ANGLE_RANGE = 360.0

# 图上最多保留多少个点
MAX_POINTS = 100

# 动画刷新间隔，单位 ms
# 7 个舵机通过 115200 透传读取，建议 80~100ms 更稳
UPDATE_INTERVAL_MS = 100


# ================= 2. 图表数据缓存 =================
t_data = deque(maxlen=MAX_POINTS)

# 每个舵机一个角度缓存和速度缓存
angle_data = {
    servo_id: deque(maxlen=MAX_POINTS)
    for servo_id in SERVO_IDS
}

speed_data = {
    servo_id: deque(maxlen=MAX_POINTS)
    for servo_id in SERVO_IDS
}


# ================= 3. 硬件初始化 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)

if not portHandler.openPort():
    print(f"串口打开失败: {DEVICENAME}，请检查端口号或是否被占用！")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print(f"波特率设置失败: {BAUDRATE}")
    portHandler.closePort()
    quit()

print(f"成功打开串口: {DEVICENAME}")
print(f"成功设置波特率: {BAUDRATE}")

print("等待 ESP32 开机进入透传状态...")
time.sleep(2.5)


# ================= 4. 工具函数 =================
def digital_to_angle(pos):
    """
    将 0~4095 的舵机位置值转换为 0~360°。
    """
    return (pos / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE


def release_servo_torque(servo_id):
    """
    释放单个舵机力矩，方便手动转动测试。
    """
    try:
        scs.unLockEprom(servo_id)
        result, error = scs.write1ByteTxRx(
            servo_id,
            STS_TORQUE_ENABLE,
            0
        )

        if result == COMM_SUCCESS:
            print(f"ID={servo_id} 力矩已释放")
        else:
            print(f"ID={servo_id} 力矩释放失败: {scs.getTxRxResult(result)}")

    except Exception as e:
        print(f"ID={servo_id} 力矩释放异常: {e}")


def release_all_torque():
    """
    释放 ID=1~7 所有舵机力矩。
    """
    print("\n正在释放 ID=1~7 所有舵机力矩...")

    for servo_id in SERVO_IDS:
        release_servo_torque(servo_id)
        time.sleep(0.05)

    print("力矩释放完成。现在可以用手转动舵机观察曲线。\n")


def read_servo_angle_speed(servo_id):
    """
    读取单个舵机的位置和速度。
    如果读取失败，则保持上一个值。
    """
    pos, speed, result, error = scs.ReadPosSpeed(servo_id)

    if result == COMM_SUCCESS:
        angle = digital_to_angle(pos)
        spd = speed
        return angle, spd, True

    else:
        # 通信失败时，使用上一帧数据，避免曲线突然掉到 0
        last_angle = angle_data[servo_id][-1] if len(angle_data[servo_id]) > 0 else 0
        last_speed = speed_data[servo_id][-1] if len(speed_data[servo_id]) > 0 else 0

        print(f"ID={servo_id} 读取失败: {scs.getTxRxResult(result)}")
        return last_angle, last_speed, False


# ================= 5. 上电安全机制：释放所有舵机力矩 =================
release_all_torque()


# ================= 6. 初始化 Matplotlib 图表 =================
plt.style.use('fast')

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
fig.canvas.manager.set_window_title('微雪总线舵机实时遥测系统 ID=1~7')

# 颜色表，7 个舵机够用
colors = plt.cm.tab10(range(len(SERVO_IDS)))

# 保存每个舵机对应的曲线对象
angle_lines = {}
speed_lines = {}

# --- 图表 1：角度 ---
for index, servo_id in enumerate(SERVO_IDS):
    line, = ax1.plot(
        [],
        [],
        label=f'Angle ID:{servo_id}',
        linewidth=2,
        color=colors[index]
    )
    angle_lines[servo_id] = line

ax1.set_ylim(0, 360)
ax1.set_ylabel('Angle (Degree)')
ax1.set_title('Real-time Servo Angle, ID=1~7')
ax1.legend(loc='upper right', ncol=2)
ax1.grid(True, linestyle='--', alpha=0.6)


# --- 图表 2：速度 ---
for index, servo_id in enumerate(SERVO_IDS):
    line, = ax2.plot(
        [],
        [],
        label=f'Speed ID:{servo_id}',
        linewidth=2,
        linestyle='--',
        color=colors[index]
    )
    speed_lines[servo_id] = line

ax2.set_ylim(-4000, 4000)
ax2.set_xlabel('Time (frames)')
ax2.set_ylabel('Raw Speed')
ax2.set_title('Real-time Servo Speed, ID=1~7')
ax2.legend(loc='upper right', ncol=2)
ax2.grid(True, linestyle='--', alpha=0.6)

# 速度 0 参考线
ax2.axhline(0, color='black', linewidth=1, alpha=0.3)


# ================= 7. 核心更新函数 =================
frame_count = 0


def update_plot(frame):
    global frame_count

    frame_count += 1
    t_data.append(frame_count)

    all_lines = []

    for servo_id in SERVO_IDS:
        angle, speed, ok = read_servo_angle_speed(servo_id)

        angle_data[servo_id].append(angle)
        speed_data[servo_id].append(speed)

        angle_lines[servo_id].set_data(t_data, angle_data[servo_id])
        speed_lines[servo_id].set_data(t_data, speed_data[servo_id])

        all_lines.append(angle_lines[servo_id])
        all_lines.append(speed_lines[servo_id])

        # 7 个舵机连续读取，稍微留一点间隔，避免透传拥堵
        time.sleep(0.003)

    # 动态滑动 X 轴
    if frame_count > MAX_POINTS:
        ax1.set_xlim(frame_count - MAX_POINTS, frame_count)
        ax2.set_xlim(frame_count - MAX_POINTS, frame_count)
    else:
        ax1.set_xlim(0, MAX_POINTS)
        ax2.set_xlim(0, MAX_POINTS)

    return all_lines


# ================= 8. 启动动画 =================
try:
    print("=" * 70)
    print("开始实时绘制 ID=1~7 舵机角度和速度曲线")
    print("关闭绘图窗口即可退出程序")
    print("=" * 70)

    ani = FuncAnimation(
        fig,
        update_plot,
        interval=UPDATE_INTERVAL_MS,
        blit=False,
        cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()

except KeyboardInterrupt:
    print("\n检测到 Ctrl+C，正在退出...")

except Exception as e:
    print(f"出现异常: {e}")

finally:
    portHandler.closePort()
    print("绘图窗口已关闭，串口已安全释放。")
