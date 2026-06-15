import time
import h5py
import numpy as np
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


# ================= 1. ALOHA 规范化数据收集器 =================
class AlohaDataCollector:
    def __init__(self, camera_names=None):
        if camera_names is None:
            camera_names = []

        self.camera_names = camera_names

        self.data_dict = {
            '/time': [],
            '/observations/qpos': [],      # 当前真实关节位置，shape = [T, 7]
            '/observations/qvel': [],      # 当前真实关节速度，shape = [T, 7]
            '/action/target_pos': [],      # 示教动作，shape = [T, 7]
        }

        for cam_name in self.camera_names:
            self.data_dict[f'/observations/images/{cam_name}'] = []

    def append_step_data(self, current_time, qpos, qvel, action):
        self.data_dict['/time'].append(current_time)
        self.data_dict['/observations/qpos'].append(qpos)
        self.data_dict['/observations/qvel'].append(qvel)
        self.data_dict['/action/target_pos'].append(action)

    def save_to_hdf5(self, dataset_dir, task_name, episode_idx, servo_ids):
        task_dir = os.path.join(dataset_dir, task_name)
        os.makedirs(task_dir, exist_ok=True)

        file_path = os.path.join(task_dir, f'episode_{episode_idx}.hdf5')

        print(f"\n🔄 正在将数据写入: {file_path}")
        t0 = time.time()

        with h5py.File(file_path, 'w') as root:
            # 保存基础数据
            for key, value_list in self.data_dict.items():
                if len(value_list) == 0:
                    continue

                data_np = np.array(value_list, dtype=np.float32)
                root.create_dataset(key, data=data_np, compression='gzip')

            # 保存一些元信息，后续训练/检查数据时很有用
            root.attrs['servo_ids'] = np.array(servo_ids, dtype=np.int32)
            root.attrs['dof'] = len(servo_ids)
            root.attrs['data_format'] = 'ALOHA-style servo demonstration'
            root.attrs['qpos_unit'] = 'raw_position_0_4095'
            root.attrs['qvel_unit'] = 'raw_speed'
            root.attrs['action_meaning'] = 'target_pos equals demonstrated qpos'

        total_frames = len(self.data_dict['/time'])
        print(f"✅ 保存成功！耗时: {time.time() - t0:.2f}s | 总帧数: {total_frames}")
        print(f"✅ qpos shape: ({total_frames}, {len(servo_ids)})")
        print(f"✅ qvel shape: ({total_frames}, {len(servo_ids)})")
        print(f"✅ action shape: ({total_frames}, {len(servo_ids)})")


# ================= 2. 舵机工具函数 =================
def release_servo_torque(scs, servo_id):
    """
    释放单个舵机力矩，让舵机可以手动示教。
    """
    try:
        scs.unLockEprom(servo_id)
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


def release_all_torque(scs, servo_ids):
    """
    释放所有舵机力矩。
    """
    print("\n🔓 正在释放所有舵机力矩...")

    for servo_id in servo_ids:
        release_servo_torque(scs, servo_id)
        time.sleep(0.05)

    print("✅ 力矩释放流程完成，可以开始手动示教。\n")


def read_all_servos(scs, servo_ids, last_qpos, last_qvel):
    """
    读取 ID=1~7 所有舵机的位置和速度。

    采用逐舵机容错：
    - 某个 ID 读取成功：更新该 ID 的位置和速度；
    - 某个 ID 读取失败：只保留该 ID 上一帧数据，不影响其他 ID。
    """
    qpos = list(last_qpos)
    qvel = list(last_qvel)

    success_count = 0
    failed_ids = []

    for i, servo_id in enumerate(servo_ids):
        pos, spd, result, error = scs.ReadPosSpeed(servo_id)

        if result == COMM_SUCCESS:
            qpos[i] = pos
            qvel[i] = spd
            success_count += 1
        else:
            failed_ids.append(servo_id)

        # 通过 ESP32 透传 + 115200 波特率时，连续读 7 个舵机需要稍微缓冲
        # time.sleep(0.001)

    return qpos, qvel, success_count, failed_ids


# ================= 3. 主程序 =================
def main():
    parser = argparse.ArgumentParser(description='7自由度舵机手动示教数据录制系统 ALOHA-HDF5')

    parser.add_argument('--task_name', type=str, default='control_test',
                        help='任务名称，将作为子文件夹名')

    parser.add_argument('--dataset_dir', type=str, default='./data/',
                        help='数据集保存根目录')

    parser.add_argument('--episode_len', type=int, default=800,
                        help='最大录制步数，默认800步')

    parser.add_argument('--episode_idx', type=int, default=0,
                        help='当前录制回合编号，如 episode_0.hdf5')

    parser.add_argument('--port', type=str, default='COM6',
                        help='串口号，例如 Windows: COM6, Linux: /dev/ttyUSB0')

    parser.add_argument('--hz', type=float, default=50.0,
                        help='采集频率，默认50Hz')

    parser.add_argument('--servo_start', type=int, default=1,
                        help='起始舵机ID，默认1')

    parser.add_argument('--servo_end', type=int, default=7,
                        help='结束舵机ID，默认7')

    args = parser.parse_args()

    # ================= A. 基础硬件参数 =================
    BAUDRATE = 115200  # 必须与 ESP32 透传程序 Serial.begin(115200) 一致

    servo_ids = list(range(args.servo_start, args.servo_end + 1))
    dof = len(servo_ids)

    target_dt = 1.0 / args.hz

    print("\n" + "=" * 70)
    print("7自由度舵机示教数据采集程序")
    print(f"串口: {args.port}")
    print(f"波特率: {BAUDRATE}")
    print(f"舵机ID: {servo_ids}")
    print(f"自由度数量: {dof}")
    print(f"目标采集频率: {args.hz:.1f} Hz")
    print(f"单帧目标周期: {target_dt * 1000:.2f} ms")
    print("=" * 70)

    # ================= B. 初始化硬件 =================
    portHandler = PortHandler(args.port)
    scs = sts(portHandler)

    if not portHandler.openPort():
        print(f"❌ 串口 {args.port} 打开失败，请检查端口号或是否被占用！")
        return

    if not portHandler.setBaudRate(BAUDRATE):
        print(f"❌ 串口波特率设置失败: {BAUDRATE}")
        portHandler.closePort()
        return

    print(f"✅ 成功打开串口: {args.port}")
    print(f"✅ 成功设置波特率: {BAUDRATE}")

    print("🔌 正在等待 ESP32 开机进入透传状态...")
    time.sleep(2.5)

    # ================= C. 释放所有舵机力矩 =================
    release_all_torque(scs, servo_ids)

    # ================= D. 录制准备与倒计时 =================
    print("\n" + "=" * 70)
    print(f"🎥 录制任务: {args.task_name}")
    print(f"📦 回合编号: episode_{args.episode_idx}.hdf5")
    print(f"📏 计划步数: {args.episode_len}")
    print(f"🦾 数据维度: qpos/qvel/action = {dof}维")
    print("=" * 70)
    print("⚠️ 请准备手动转动 7 个舵机进行示教。")

    for i in range(3, 0, -1):
        print(f"录制将在 {i} 秒后开始...")
        time.sleep(1)

    print("🔴 正在录制，请开始动作！")

    # ================= E. 核心录制循环 =================
    collector = AlohaDataCollector()
    start_time = time.time()

    # 初始容错缓存：默认所有舵机在中位 2048，速度为 0
    last_qpos = [2048 for _ in servo_ids]
    last_qvel = [0 for _ in servo_ids]

    dropped_frame_count = 0
    overrun_count = 0

    try:
        for step in tqdm(
            range(args.episode_len),
            desc="数据采集进度",
            unit="帧",
            ncols=90,
            colour='green',
            mininterval=0.5
        ):
            loop_start = time.time()
            current_time = loop_start - start_time

            # 读取 7 个舵机状态
            qpos, qvel, success_count, failed_ids = read_all_servos(
                scs=scs,
                servo_ids=servo_ids,
                last_qpos=last_qpos,
                last_qvel=last_qvel
            )

            # 如果存在掉线舵机，当前帧仍保存，但对应 ID 使用上一帧数据
            if success_count < dof:
                dropped_frame_count += 1

            last_qpos = qpos
            last_qvel = qvel

            # 在手动示教模式中，当前真实位置就是 target action
            action = list(qpos)

            # 写入内存缓存
            collector.append_step_data(
                current_time=current_time,
                qpos=qpos,
                qvel=qvel,
                action=action
            )

            # 锁频
            time_spent = time.time() - loop_start
            time_left = target_dt - time_spent

            if time_left > 0:
                time.sleep(time_left)
            else:
                overrun_count += 1

    except KeyboardInterrupt:
        print("\n\n🛑 检测到 Ctrl+C，录制提前结束，将保存已录制数据。")

    finally:
        # ================= F. 保存 HDF5 =================
        collector.save_to_hdf5(
            dataset_dir=args.dataset_dir,
            task_name=args.task_name,
            episode_idx=args.episode_idx,
            servo_ids=servo_ids
        )

        portHandler.closePort()

        print("\n" + "=" * 70)
        print("🎉 采集任务结束")
        print(f"掉帧/通信不完整帧数: {dropped_frame_count}")
        print(f"采集周期超时帧数: {overrun_count}")
        print("串口已关闭")
        print("=" * 70)


if __name__ == '__main__':
    main()

# python servo_record_data_hdf5.py --task_name "control_test" --dataset_dir "/home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/master_slave_control/Servo_control/data" --port /dev/ttyUSB0 --episode_len 800 --episode_idx 0

