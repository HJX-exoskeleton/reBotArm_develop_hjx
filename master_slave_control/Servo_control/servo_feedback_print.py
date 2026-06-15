import time
import sys
import os

# ================= 动态环境变量注入 =================
# 获取当前脚本所在目录 (.../Python/Servo_control)
current_dir = os.path.dirname(os.path.abspath(__file__))

# 获取项目根目录 (.../Python)
project_root = os.path.dirname(current_dir)

# 精准定位 SDK 所在的绝对路径 (.../Python/STservo_sdk)
sdk_path = os.path.join(project_root, 'STservo_sdk')

# 将【项目根目录】和【SDK 内部目录】都加入环境变量
if project_root not in sys.path:
    sys.path.append(project_root)

if sdk_path not in sys.path:
    sys.path.append(sdk_path)

from STservo_sdk import *  # 微雪舵机库


# ================= 1. 基础参数设置 =================

# 必须与 ESP32 透传代码中的 Serial.begin(1000000) 保持一致
BAUDRATE = 115200  # 115200

# Windows 示例
# DEVICENAME = 'COM6'
# Linux 示例
DEVICENAME = '/dev/ttyUSB0'

# 舵机 ID 范围：1 ~ 7
SERVO_ID_START = 1
SERVO_ID_END = 7
SERVO_IDS = list(range(SERVO_ID_START, SERVO_ID_END + 1))

# ST3215 / SMS_STS 角度转换参数
SERVO_DIGITAL_RANGE = 4095.0
SERVO_ANGLE_RANGE = 360.0

# 读取间隔
READ_INTERVAL = 0.1  # 100 ms


# ================= 2. 初始化硬件 =================

portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)

if portHandler.openPort():
    print(f"成功打开串口: {DEVICENAME}")
else:
    print(f"无法打开串口 {DEVICENAME}，请检查端口号！")
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print(f"成功设置波特率: {BAUDRATE}")
else:
    print("无法设置波特率!")
    portHandler.closePort()
    quit()


# ================= 3. 等待 ESP32 透传程序就绪 =================

print("正在等待 ESP32 开机重启完毕，请稍候...")
time.sleep(2.5)
print("ESP32 就绪，开始发送指令！")


# ================= 4. 工具函数 =================

def digital_to_angle(pos):
    """
    将 0~4095 的位置值转换为 0~360°。
    """
    return (pos / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE


def read_voltage_temp(servo_id):
    """
    读取单个舵机的电压和温度。
    返回:
        voltage_value, temperature_value, voltage_result, temperature_result
    """
    volt, volt_res, volt_err = scs.read1ByteTxRx(servo_id, STS_PRESENT_VOLTAGE)
    temp, temp_res, temp_err = scs.read1ByteTxRx(servo_id, STS_PRESENT_TEMPERATURE)

    return volt, temp, volt_res, temp_res


def release_servo_torque(servo_id):
    """
    释放单个舵机力矩。
    """
    try:
        # 有些指令需要先解锁，这里保留你的原始逻辑
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

    print("力矩释放流程完成。你现在可以用手转动舵机。\n")


def read_servo_status(servo_id):
    """
    读取单个舵机状态。
    返回一个字典。
    """
    pos, speed, result, error = scs.ReadPosSpeed(servo_id)

    if result != COMM_SUCCESS:
        return {
            "id": servo_id,
            "success": False,
            "message": scs.getTxRxResult(result),
            "pos": None,
            "angle": None,
            "speed": None,
            "voltage": None,
            "temperature": None,
        }

    volt, temp, volt_res, temp_res = read_voltage_temp(servo_id)

    if volt_res == COMM_SUCCESS:
        real_volt = volt / 10.0
    else:
        real_volt = None

    if temp_res == COMM_SUCCESS:
        real_temp = temp
    else:
        real_temp = None

    angle = digital_to_angle(pos)

    return {
        "id": servo_id,
        "success": True,
        "message": "OK",
        "pos": pos,
        "angle": angle,
        "speed": speed,
        "voltage": real_volt,
        "temperature": real_temp,
    }


def print_header():
    """
    打印表头。
    """
    print(
        "Time(ms) | ID | Pos  | Angle(deg) | Speed | Voltage(V) | Temp(C) | Status"
    )
    print("-" * 85)


def print_servo_status(current_time_ms, status):
    """
    格式化打印单个舵机状态。
    """
    servo_id = status["id"]

    if status["success"]:
        pos = status["pos"]
        angle = status["angle"]
        speed = status["speed"]
        voltage = status["voltage"]
        temperature = status["temperature"]

        voltage_str = f"{voltage:4.1f}" if voltage is not None else "FAIL"
        temp_str = f"{temperature:2d}" if temperature is not None else "FAIL"

        print(
            f"{current_time_ms:08d} | "
            f"{servo_id:2d} | "
            f"{pos:4d} | "
            f"{angle:10.2f} | "
            f"{speed:5d} | "
            f"{voltage_str:>10} | "
            f"{temp_str:>7} | "
            f"OK"
        )

    else:
        print(
            f"{current_time_ms:08d} | "
            f"{servo_id:2d} | "
            f"---- | "
            f"---------- | "
            f"----- | "
            f"---------- | "
            f"------- | "
            f"{status['message']}"
        )


# ================= 5. 上电安全机制：释放 ID=1~7 力矩 =================

release_all_torque()


print("\n" + "=" * 85)
print("开始读取 ID=1~7 舵机状态")
print("你可以用手转动舵机观察 Pos / Angle 数据变化。按 Ctrl+C 退出。")
print("=" * 85 + "\n")


# ================= 6. 主循环读取 =================

try:
    start_time = time.time()

    while True:
        current_time_ms = int((time.time() - start_time) * 1000)

        print_header()

        for servo_id in SERVO_IDS:
            status = read_servo_status(servo_id)
            print_servo_status(current_time_ms, status)

            # 给透传和总线一点缓冲时间，避免 7 个舵机连续读取过快
            time.sleep(0.01)

        print("-" * 85)
        print()

        time.sleep(READ_INTERVAL)

except KeyboardInterrupt:
    print("\n检测到 Ctrl+C，正在安全退出...")

finally:
    portHandler.closePort()
    print("串口已关闭。")
