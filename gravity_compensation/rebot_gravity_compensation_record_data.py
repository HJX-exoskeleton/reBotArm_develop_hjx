#!/usr/bin/env python3
"""重力补偿控制演示（高性能纯终端参数化采集 - 全功能闭环版）。

功能特性：
1. 显式索引指定：新增 --episode_idx 参数，支持显式命名，完美对齐重播端的调试习惯。
2. 自动兼容自增：未指定索引时自动检测历史文件自增命名，兼顾灵活性与严谨性。
3. 全参数联动：支持命令行指定 --rate (频率), --episode_len (步数), --time (秒数)。
4. 安全停机：无论程序是正常结束还是异常中断，退出时都会自动对全轴电机执行【失能 (Disable)】。
"""
import argparse
import os
import signal
import sys
import time
from pathlib import Path
import threading

import numpy as np
import h5py
from tqdm import tqdm

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RobotArm
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
    compute_generalized_gravity,
)

# --------------------------------------------------------------------------- #
# 全局控制标志与高级调参配置
# --------------------------------------------------------------------------- #
_running = True

# [安全保险] 限制各关节最大前馈力矩
TORQUE_LIMITS = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

# 各轴特异化阻尼系数 (Kd) # 各轴重力前馈缩放系数
# KD_CONFIG = np.array([1.0, 2.0, 1.5, 1.0, 0.8, 0.6])
# GRAVITY_SCALES = np.array([1.50, 0.7, 0.7, 0.75, 1.0, 1.0])

# 各轴特异化阻尼系数 (Kd) # 各轴重力前馈缩放系数
KD_CONFIG = np.array([1.0, 1.0, 1.0, 1.0, 0.8, 0.8])
GRAVITY_SCALES = np.array([1.50, 0.85, 0.85, 0.82, 1.0, 1.0])

# --------------------------------------------------------------------------- #
# 📊 采集状态控制变量与临时缓存
# --------------------------------------------------------------------------- #
_is_recording = False

recorded_timestamps = []
recorded_qpos = []
recorded_qvel = []
recorded_tau = []

# 全局配置字典
_config = {
    "task_name": "test_task",
    "base_save_dir": Path("./collected_data"),
    "episode_len": 5000,
    "dt": 0.02,
    "rate": 50,
    "episode_idx": None  # 💡 预留显式索引位置
}


def _sigint_handler(signum, frame):
    global _running
    print("\n[gravity_comp] 收到 Ctrl+C 中断信号，正在紧急安全退出...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


# --------------------------------------------------------------------------- #
# 控制回调（高频实时循环驱动：硬实时核心）
# --------------------------------------------------------------------------- #

def gravity_compensation_controller(arm: RobotArm, dt: float) -> None:
    """重力补偿控制回调。专注于硬实时控制。"""
    global _is_recording

    if not hasattr(gravity_compensation_controller, "_counter"):
        gravity_compensation_controller._counter = 0

    q = arm.get_positions()
    v = arm.get_velocities() if hasattr(arm, 'get_velocities') else np.zeros_like(q)

    # Pinocchio 计算并修正广义重力向量
    tau_g_raw = compute_generalized_gravity(q=q[:6])
    tau_g = tau_g_raw * GRAVITY_SCALES[:arm.num_joints]
    tau_g_safe = np.clip(tau_g, -TORQUE_LIMITS[:arm.num_joints], TORQUE_LIMITS[:arm.num_joints])

    # 严格基于计算出的最终 episode_len 帧数进行截断
    if _is_recording:
        current_frame_count = len(recorded_timestamps)
        if current_frame_count < _config["episode_len"]:
            rec_time = current_frame_count * _config["dt"]
            recorded_timestamps.append(rec_time)
            recorded_qpos.append(q[:6])
            recorded_qvel.append(v[:6])
            recorded_tau.append(tau_g_safe[:6])
        else:
            _is_recording = False

    # MIT 前馈安全帧发送
    arm.mit(
        pos=q,
        vel=np.zeros(arm.num_joints),
        kp=np.zeros(arm.num_joints),
        kd=KD_CONFIG[:arm.num_joints],
        tau=tau_g_safe,
        request_feedback=True,
    )

    # 降频终端状态打印（非录制状态下每 100 周期打印一次，录制时完全静音）
    gravity_compensation_controller._counter += 1
    if gravity_compensation_controller._counter % 100 == 0 and not _is_recording:
        print(
            f"[{gravity_compensation_controller._counter:5d}] [🟢 重力补偿运行中] "
            f"tau_g_2轴/3轴 = {tau_g_safe[1]:+.2f}/{tau_g_safe[2]:+.2f} N·m",
            end="\r"
        )


# --------------------------------------------------------------------------- #
# 💾 HDF5 持久化落地函数
# --------------------------------------------------------------------------- #
def save_to_hdf5():
    """标准 .hdf5 数据集安全持久化落盘，支持自增和显式指定命名。"""
    if not recorded_timestamps:
        print("\n[💾 导出失败] 未采集到有效数据。")
        return

    task_name = _config["task_name"]
    task_sub_dir = _config["base_save_dir"] / task_name
    task_sub_dir.mkdir(parents=True, exist_ok=True)

    # 💡 核心策略：如果命令行显式传了 episode_idx，就直接用它；否则自动扫描并自增
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
    print(f"\n\n[💾 存储线程] 正在向硬盘写入 {file_name} ({total_frames} 帧示教数据)...")

    try:
        with h5py.File(file_path, 'w') as f:
            with tqdm(total=4, desc="📝 HDF5数据集落盘", bar_format="{l_bar}{bar:30}{r_bar}") as pbar:
                f.create_dataset('timestamp', data=np.array(recorded_timestamps), compression="gzip")
                pbar.update(1)
                f.create_dataset('qpos', data=np.array(recorded_qpos), compression="gzip")
                pbar.update(1)
                f.create_dataset('qvel', data=np.array(recorded_qvel), compression="gzip")
                pbar.update(1)
                f.create_dataset('effort', data=np.array(recorded_tau), compression="gzip")
                pbar.update(1)

            # 写入元数据标签（包含显式索引值）
            f.attrs['task_name'] = task_name
            f.attrs['episode_idx'] = final_episode_idx
            f.attrs['episode_len'] = _config["episode_len"]
            f.attrs['total_frames'] = total_frames
            f.attrs['hz_rate'] = _config["rate"]
            f.attrs['duration_seconds'] = total_frames * _config["dt"]
            f.attrs['robot_name'] = 'reBotArm_6DOF'

        print(f"🎉 [💾 导出成功] 数据集固化完成！")
        print(f"📄 数据文件路径: {file_path.resolve()}\n")
    except Exception as e:
        print(f"❌ [💾 导出异常] 写入 HDF5 失败: {e}\n")

    # 彻底释放内存缓存
    recorded_timestamps.clear()
    recorded_qpos.clear()
    recorded_qvel.clear()
    recorded_tau.clear()


# --------------------------------------------------------------------------- #
# 后台标准终端键盘事件监听
# --------------------------------------------------------------------------- #
def terminal_keyboard_listener():
    """纯终端下的回车事件监听器"""
    global _is_recording, _running
    while _running:
        try:
            input()
        except (KeyboardInterrupt, EOFError):
            break

        if not _running:
            break

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
            print("\n⚠️ [警告] 系统当前正处于高频录制中，请勿重复操作。")


# --------------------------------------------------------------------------- #
# 主程序
# --------------------------------------------------------------------------- #

def main() -> None:
    global _running

    parser = argparse.ArgumentParser(
        description="reBotArm 高性能数据采集系统 - 全参数联动终极终端",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--task_name", "-t",
        type=str,
        default="test_task",
        help="设定当前采集的具身智能示教任务名称"
    )
    parser.add_argument(
        "--save_dir", "-d",
        type=str,
        default="./collected_data",
        help="设定数据落盘保存的基础根目录路径"
    )
    parser.add_argument(
        "--rate", "-r",
        type=int,
        default=50,
        help="控制和采样频率 (Hz)。系统会据此动态更新底层的物理周期时间步长 dt"
    )
    parser.add_argument(
        "--episode_len", "-l",
        type=int,
        default=5000,
        help="设定单次录制的时间步数/帧数。注：如果显式传入了 --time，则此参数将被覆盖"
    )
    parser.add_argument(
        "--time", "-sec",
        type=float,
        default=None,
        help="直接以秒（Seconds）为单位设定录制时长。程序会自动结合传入的 --rate 将其转换为精准步数"
    )
    # 💡 核心新增：--episode_idx 参数，支持外部直接指定索引编号
    parser.add_argument(
        "--episode_idx", "-idx",
        type=int,
        default=None,
        help="显式指定当前录制片段的 Episode 索引号。如果不传，则系统自动检索历史文件并自增"
    )
    args = parser.parse_args()

    # 动态解算物理步长 dt
    dt_actual = 1.0 / args.rate

    # 核心联动判断：若显式指定了秒数，结合当前自定义的频率动态计算总步数
    if args.time is not None:
        calculated_len = int(args.time * args.rate)
        print(
            f"💡 [全参数联动] 设定时间: {args.time}s, 基于自定义控制频率: {args.rate}Hz, 自动对齐解算总步数: {calculated_len} 步。")
        final_episode_len = calculated_len
    else:
        final_episode_len = args.episode_len

    # 将所有对齐完的配置参数推入全局字典
    _config["task_name"] = args.task_name
    _config["base_save_dir"] = Path(args.save_dir)
    _config["episode_len"] = final_episode_len
    _config["dt"] = dt_actual
    _config["rate"] = args.rate
    _config["episode_idx"] = args.episode_idx  # 💡 缓存显式索引到全局配置

    print("=" * 60)
    print("  reBotArm 高性能重力补偿控制 + ALOHA 规范数据采集系统 (全参数动态配置)")
    print(f"  📝 目标任务名称: {args.task_name}")
    print(f"  ⚡ 实时控制频率: {args.rate} Hz (采样周期 dt: {dt_actual:.4f} 秒)")
    print(f"  ⏱️ 单次录制规模: {final_episode_len} 步 (真实总物理时长: {final_episode_len * dt_actual:.2f} 秒)")
    if args.episode_idx is not None:
        print(f"  🔢 显式命名指派: episode_{args.episode_idx}.hdf5")
    else:
        print(f"  🔢 命名模式指派: 自动化历史增量自增模式")
    print(f"  📂 自动化子文件夹归类位置: {(Path(args.save_dir) / args.task_name).resolve()}")
    print("=" * 60)

    # 初始化底层硬件并建立通信
    model = load_dynamics_model()
    arm = RobotArm()
    arm.connect()
    arm.enable()
    arm.mode_mit(kp=np.zeros(arm.num_joints), kd=KD_CONFIG[:arm.num_joints])

    # 挂载控制循环
    arm.start_control_loop(gravity_compensation_controller, rate=args.rate)
    print(f"[控制循环] 底层硬实时循环已挂载 @ {args.rate} Hz")

    listener_thread = threading.Thread(target=terminal_keyboard_listener, daemon=True)
    listener_thread.start()
    print("📌 [系统就绪] 终端回车监听器就绪。请随时按下回车开始采集数据...")
    print("-" * 60)

    try:
        while _running:
            time.sleep(0.1)
    except Exception as e:
        print(f"\n[主线程异常] {e}")
    finally:
        print("\n" + "=" * 50)
        print("[退出流程] 启动安全卸载与停机保护程序...")
        _running = False

        try:
            print("[退出流程] 正在向全轴电机发送【失能 (Disable)】指令...")
            arm.disable()
            time.sleep(0.2)
        except Exception as e:
            print(f"[⚠️警告] 电机安全失能失败: {e}")

        arm.disconnect()
        print("[退出流程] 机械臂连接已安全切断，硬件已彻底失能保护。")
        print("=" * 50 + "\n")


if __name__ == "__main__":
    main()

# python rebot_gravity_compensation_record_data.py --task_name test_task --save_dir ./collected_data --rate 50 --time 10 --episode_idx 0


