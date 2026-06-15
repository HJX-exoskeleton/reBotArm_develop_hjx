#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import os
import platform


# ================= 动态环境变量注入 =================
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sdk_path = os.path.join(project_root, "STservo_sdk")

if project_root not in sys.path:
    sys.path.append(project_root)

if sdk_path not in sys.path:
    sys.path.append(sdk_path)

from STservo_sdk import *  # 微雪舵机库


# ================= 1. 参数配置 =================
BAUDRATE = 115200

# 根据系统自动选择默认串口
if platform.system().lower().startswith("win"):
    DEVICENAME = "COM6"
else:
    DEVICENAME = "/dev/ttyUSB0"

# 7 自由度舵机 ID
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]

# ============================================================
# 关键修改：
#   ID1~ID6 初始化到 180°
#   ID7 初始化到 90°，对应夹爪闭合
# ============================================================
HOME_DEG = {
    1: 180.0,
    2: 180.0,
    3: 180.0,
    4: 180.0,
    5: 180.0,
    6: 180.0,
    7: 90.0,     # 夹爪主手初始闭合, 180 表示张开
}

# 复位速度和加速度
HOME_SPEED = 1500
HOME_ACC = 40

# 到位判断阈值，单位是舵机原始位置值
POSITION_THRESHOLD = 10

# ESP32 透传启动等待时间
ESP32_BOOT_WAIT = 0.5

# 复位超时时间，避免某个舵机异常导致死循环
ARRIVAL_TIMEOUT = 20.0

# 每次打印状态的间隔
PRINT_INTERVAL = 0.2


def angle_to_pos(angle):
    """
    将角度 0~360° 转换为 ST3215 / SMS_STS 的位置值 0~4095。
    """
    angle = max(0.0, min(float(angle), 360.0))
    return int((angle / 360.0) * 4095.0)


def pos_to_angle(pos):
    """
    将位置值 0~4095 转换为角度。
    """
    return (float(pos) / 4095.0) * 360.0


def build_home_pos_map():
    """
    生成每个舵机自己的目标位置。
    ID1~ID6 -> 180°
    ID7     -> 90°
    """
    home_pos = {}

    for servo_id in SERVO_IDS:
        target_deg = HOME_DEG[servo_id]
        home_pos[servo_id] = angle_to_pos(target_deg)

    return home_pos


# ================= 2. 初始化硬件 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)


def open_serial():
    if not portHandler.openPort():
        print(f"❌ 串口打开失败: {DEVICENAME}")
        return False

    if not portHandler.setBaudRate(BAUDRATE):
        print(f"❌ 波特率设置失败: {BAUDRATE}")
        portHandler.closePort()
        return False

    print(f"✅ 串口已打开: {DEVICENAME}")
    print(f"✅ 波特率已设置: {BAUDRATE}")

    print("正在等待 ESP32 进入透传状态...")
    time.sleep(ESP32_BOOT_WAIT)

    try:
        portHandler.ser.reset_input_buffer()
        portHandler.ser.reset_output_buffer()
        print("✅ 已清空串口缓冲区")
    except Exception as e:
        print(f"⚠️ 清空串口缓冲区失败，可忽略: {e}")

    return True


# ================= 3. 舵机控制函数 =================
def enable_torque_all():
    """
    开启 ID=1~7 所有舵机力矩。
    """
    print("\n正在开启 7 个舵机力矩...")

    for servo_id in SERVO_IDS:
        try:
            try:
                scs.unLockEprom(servo_id)
            except Exception:
                pass

            result, error = scs.write1ByteTxRx(
                servo_id,
                STS_TORQUE_ENABLE,
                1
            )

            if result == COMM_SUCCESS:
                print(f"✅ ID={servo_id} 力矩已开启")
            else:
                print(f"⚠️ ID={servo_id} 力矩开启失败: {scs.getTxRxResult(result)}")

        except Exception as e:
            print(f"⚠️ ID={servo_id} 力矩开启异常: {e}")

        time.sleep(0.05)


def disable_torque_all():
    """
    释放 ID=1~7 所有舵机力矩。
    """
    print("\n正在释放 7 个舵机力矩...")

    for servo_id in SERVO_IDS:
        try:
            result, error = scs.write1ByteTxRx(
                servo_id,
                STS_TORQUE_ENABLE,
                0
            )

            if result == COMM_SUCCESS:
                print(f"✅ ID={servo_id} 力矩已释放")
            else:
                print(f"⚠️ ID={servo_id} 力矩释放失败: {scs.getTxRxResult(result)}")

        except Exception as e:
            print(f"⚠️ ID={servo_id} 力矩释放异常: {e}")

        time.sleep(0.05)


def sync_move_all_to_home():
    """
    同步发送 7 个舵机回初始化位置指令。

    ID1~ID6 -> 180°
    ID7     -> 90°
    """
    home_pos = build_home_pos_map()

    scs.groupSyncWrite.clearParam()

    print("\n准备同步下发初始化目标：")

    for servo_id in SERVO_IDS:
        target_deg = HOME_DEG[servo_id]
        target_pos = home_pos[servo_id]

        scs.SyncWritePosEx(
            servo_id,
            target_pos,
            HOME_SPEED,
            HOME_ACC
        )

        if servo_id == 7:
            print(f"  ID{servo_id}: {target_deg:.1f}°，Pos={target_pos}  ← 夹爪闭合位")
        else:
            print(f"  ID{servo_id}: {target_deg:.1f}°，Pos={target_pos}")

    result = scs.groupSyncWrite.txPacket()

    if result == COMM_SUCCESS:
        print("\n✅ 初始化指令已同步下发")
    else:
        print(f"\n⚠️ 同步初始化指令发送异常: {scs.getTxRxResult(result)}")

    return home_pos


def wait_until_all_arrived(home_pos):
    """
    轮询检查 7 个舵机是否都到达各自目标位置附近。
    注意：ID7 的目标位置不是 180°，而是 90°。
    """
    print("\n正在检查初始化进度...")

    start_time = time.time()
    last_print_time = 0.0

    while True:
        now = time.time()

        all_arrived = True
        status_text = []

        for servo_id in SERVO_IDS:
            target_pos = home_pos[servo_id]

            try:
                pos, speed, result, error = scs.ReadPosSpeed(servo_id)

                if result == COMM_SUCCESS:
                    diff = abs(int(pos) - int(target_pos))
                    angle = pos_to_angle(pos)
                    target_angle = HOME_DEG[servo_id]

                    if servo_id == 7:
                        status_text.append(
                            f"ID{servo_id}: {angle:6.2f}° -> {target_angle:6.2f}°, diff={diff:4d} [gripper]"
                        )
                    else:
                        status_text.append(
                            f"ID{servo_id}: {angle:6.2f}° -> {target_angle:6.2f}°, diff={diff:4d}"
                        )

                    if diff >= POSITION_THRESHOLD:
                        all_arrived = False

                else:
                    status_text.append(f"ID{servo_id}: READ_FAIL")
                    all_arrived = False

            except Exception as e:
                status_text.append(f"ID{servo_id}: EXCEPTION={e}")
                all_arrived = False

            time.sleep(0.005)

        if now - last_print_time >= PRINT_INTERVAL:
            print(" | ".join(status_text), end="\r")
            last_print_time = now

        if all_arrived:
            print()
            print("✅ 7 个舵机均已到达初始化位置。")
            print("✅ ID1~ID6 = 180°，ID7 = 90°，夹爪主手初始化为闭合状态。")
            break

        if now - start_time > ARRIVAL_TIMEOUT:
            print()
            print(f"⚠️ 初始化等待超时 {ARRIVAL_TIMEOUT:.1f}s，部分舵机可能未到位。")
            print("最后一次状态：")
            print(" | ".join(status_text))
            break

        time.sleep(0.05)


# ================= 4. 执行复位逻辑 =================
def home_servos_7dof():
    print("\n" + "=" * 80)
    print("🏠 正在执行 7 自由度舵机初始化程序")
    print("目标角度：")
    for servo_id in SERVO_IDS:
        if servo_id == 7:
            print(f"  ID{servo_id}: {HOME_DEG[servo_id]:.1f}°  ← 夹爪主手闭合位")
        else:
            print(f"  ID{servo_id}: {HOME_DEG[servo_id]:.1f}°")
    print(f"舵机 ID: {SERVO_IDS}")
    print("=" * 80)

    enable_torque_all()

    home_pos = sync_move_all_to_home()

    wait_until_all_arrived(home_pos)

    print("\n进入待机状态：释放所有舵机力矩。")
    disable_torque_all()


# ================= 5. 运行 =================
if __name__ == "__main__":
    try:
        if open_serial():
            home_servos_7dof()

    except KeyboardInterrupt:
        print("\n\n🛑 检测到 Ctrl+C，中断初始化程序。")
        disable_torque_all()

    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        disable_torque_all()

    finally:
        try:
            portHandler.closePort()
            print("程序结束，串口已关闭。")
        except Exception:
            pass
