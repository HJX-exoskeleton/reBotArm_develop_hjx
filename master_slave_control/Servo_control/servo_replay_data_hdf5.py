import time
import h5py
import os
import argparse
from tqdm import tqdm

import sys

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


# ================= 工具函数 =================
def enable_servo_torque(scs, servo_id):
    """
    开启单个舵机力矩。
    """
    try:
        scs.unLockEprom(servo_id)
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


def disable_servo_torque(scs, servo_id):
    """
    释放单个舵机力矩。
    """
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


def enable_all_torque(scs, servo_ids):
    """
    开启所有舵机力矩。
    """
    print("\n🔒 正在开启 ID=1~7 所有舵机力矩...")

    for servo_id in servo_ids:
        enable_servo_torque(scs, servo_id)
        time.sleep(0.05)

    print("✅ 所有舵机力矩开启流程完成。\n")


def disable_all_torque(scs, servo_ids):
    """
    释放所有舵机力矩。
    """
    print("\n🔓 正在释放 ID=1~7 所有舵机力矩...")

    for servo_id in servo_ids:
        disable_servo_torque(scs, servo_id)
        time.sleep(0.05)

    print("✅ 所有舵机力矩释放流程完成。\n")


def sync_write_positions(scs, servo_ids, target_positions, speed=0, acc=0):
    """
    同步下发多个舵机目标位置。

    参数:
        scs: 舵机控制对象
        servo_ids: 舵机 ID 列表
        target_positions: 目标位置列表，长度应等于 servo_ids
        speed: 速度参数
        acc: 加速度参数
    """
    scs.groupSyncWrite.clearParam()

    for servo_id, pos in zip(servo_ids, target_positions):
        pos_int = int(pos)
        scs.SyncWritePosEx(
            servo_id,
            pos_int,
            speed,
            acc
        )

    result = scs.groupSyncWrite.txPacket()

    return result


def smooth_move_to_start(scs, servo_ids, start_positions, homing_speed=1500, homing_acc=40, wait_time=2.0):
    """
    平滑移动到录像第一帧位置。
    """
    print("\n" + "=" * 70)
    print("🛡️ 正在缓慢移动至录像起始位置...")
    print(f"起始位置: {[int(x) for x in start_positions]}")

    result = sync_write_positions(
        scs=scs,
        servo_ids=servo_ids,
        target_positions=start_positions,
        speed=homing_speed,
        acc=homing_acc
    )

    if result == COMM_SUCCESS:
        print("✅ 起始位置指令已发送")
    else:
        print(f"⚠️ 起始位置指令发送异常: {scs.getTxRxResult(result)}")

    print(f"等待 {wait_time:.1f} 秒让机械结构缓慢就位...")
    time.sleep(wait_time)

    print("=" * 70)


# ================= 主程序 =================
def main():
    # --- A. 设置 argparse ---
    parser = argparse.ArgumentParser(description='7自由度 HDF5 舵机轨迹回放系统')

    parser.add_argument('--task_name', type=str, default='control_test',
                        help='任务名称')

    parser.add_argument('--dataset_dir', type=str, default='./data/',
                        help='数据集目录')

    parser.add_argument('--episode_idx', type=int, default=0,
                        help='要回放的回合编号')

    parser.add_argument('--port', type=str, default='COM6',
                        help='串口号，例如 Windows: COM6, Linux: /dev/ttyUSB0')

    parser.add_argument('--hz', type=float, default=50.0,
                        help='回放频率，默认 50Hz')

    parser.add_argument('--servo_start', type=int, default=1,
                        help='起始舵机 ID，默认 1')

    parser.add_argument('--servo_end', type=int, default=7,
                        help='结束舵机 ID，默认 7')

    parser.add_argument('--homing_speed', type=int, default=1500,
                        help='回放前移动到起始位的速度')

    parser.add_argument('--homing_acc', type=int, default=40,
                        help='回放前移动到起始位的加速度')

    parser.add_argument('--play_speed', type=int, default=0,
                        help='正式回放时的速度参数，默认 0，跟随录制轨迹')

    parser.add_argument('--play_acc', type=int, default=0,
                        help='正式回放时的加速度参数，默认 0，跟随录制轨迹')

    args = parser.parse_args()

    # --- B. 舵机 ID 设置 ---
    servo_ids = list(range(args.servo_start, args.servo_end + 1))
    dof = len(servo_ids)

    BAUDRATE = 115200  # 必须与 ESP32 透传代码中的 Serial.begin(115200) 一致
    target_dt = 1.0 / args.hz

    # --- C. 加载 HDF5 数据 ---
    file_path = os.path.join(
        args.dataset_dir,
        args.task_name,
        f'episode_{args.episode_idx}.hdf5'
    )

    if not os.path.exists(file_path):
        print(f"❌ 找不到数据文件: {file_path}")
        return

    print(f"📂 正在加载数据: {file_path}")

    with h5py.File(file_path, 'r') as root:
        if '/action/target_pos' not in root:
            print("❌ HDF5 文件中没有找到 /action/target_pos")
            return

        action_data = root['/action/target_pos'][:]

        if 'servo_ids' in root.attrs:
            saved_servo_ids = list(root.attrs['servo_ids'])
            print(f"📌 文件内记录的 servo_ids: {saved_servo_ids}")

    num_frames = len(action_data)

    if action_data.ndim != 2:
        print(f"❌ action_data 维度错误，当前 shape={action_data.shape}")
        print("正确格式应该是 [num_frames, 7]")
        return

    if action_data.shape[1] != dof:
        print(f"❌ 数据自由度不匹配！")
        print(f"当前 action_data shape = {action_data.shape}")
        print(f"当前设置 servo_ids = {servo_ids}, dof = {dof}")
        print("请确认录制数据是否为 7 自由度，或者调整 --servo_start / --servo_end。")
        return

    print("\n" + "=" * 70)
    print("✅ 数据加载成功")
    print(f"文件路径: {file_path}")
    print(f"总帧数: {num_frames}")
    print(f"动作维度: {action_data.shape[1]}")
    print(f"预计时长: {num_frames / args.hz:.2f} 秒")
    print(f"回放频率: {args.hz:.1f} Hz")
    print(f"舵机 ID: {servo_ids}")
    print("=" * 70)

    # --- D. 初始化硬件 ---
    portHandler = PortHandler(args.port)
    scs = sts(portHandler)

    if not portHandler.openPort():
        print(f"❌ 串口 {args.port} 打开失败，请检查端口号或是否被占用！")
        return

    if not portHandler.setBaudRate(BAUDRATE):
        print(f"❌ 波特率设置失败: {BAUDRATE}")
        portHandler.closePort()
        return

    print(f"✅ 成功打开串口: {args.port}")
    print(f"✅ 成功设置波特率: {BAUDRATE}")

    print("🔌 正在等待 ESP32 开机进入透传状态...")
    time.sleep(2.5)

    try:
        # --- E. 开启 7 个舵机力矩 ---
        enable_all_torque(scs, servo_ids)

        # --- F. 平滑移动到起始帧 ---
        start_positions = action_data[0]
        smooth_move_to_start(
            scs=scs,
            servo_ids=servo_ids,
            start_positions=start_positions,
            homing_speed=args.homing_speed,
            homing_acc=args.homing_acc,
            wait_time=2.0
        )

        # --- G. 倒计时准备 ---
        print("⚠️ 请让开机械臂 / 云台 / 舵机活动范围！")
        for i in range(3, 0, -1):
            print(f"回放将在 {i} 秒后开始...")
            time.sleep(1)

        print("▶️ 开始 7 自由度轨迹回放...")

        # --- H. 核心回放循环 ---
        overrun_count = 0

        for frame_idx in tqdm(
            range(num_frames),
            desc="数据回放进度",
            unit="帧",
            ncols=90,
            colour='cyan'
        ):
            loop_start = time.time()

            target_positions = action_data[frame_idx]

            result = sync_write_positions(
                scs=scs,
                servo_ids=servo_ids,
                target_positions=target_positions,
                speed=args.play_speed,
                acc=args.play_acc
            )

            if result != COMM_SUCCESS:
                print(f"\n⚠️ 第 {frame_idx} 帧同步写入异常: {scs.getTxRxResult(result)}")

            # 精准锁频
            time_spent = time.time() - loop_start
            time_left = target_dt - time_spent

            if time_left > 0:
                time.sleep(time_left)
            else:
                overrun_count += 1

        print("\n✅ 轨迹回放完成")
        print(f"⚠️ 回放周期超时帧数: {overrun_count}")

    except KeyboardInterrupt:
        print("\n\n🛑 检测到人为中断，回放提前停止！")

    finally:
        # --- I. 落地为安：释放所有舵机力矩 ---
        disable_all_torque(scs, servo_ids)
        portHandler.closePort()
        print("🎉 串口已关闭，回放结束。")


if __name__ == '__main__':
    main()

# python servo_replay_data_hdf5.py  --task_name "control_test" --dataset_dir "/home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/master_slave_control/Servo_control/data" --port /dev/ttyUSB0 --episode_idx 0

