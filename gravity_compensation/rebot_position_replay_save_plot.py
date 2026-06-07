#!/usr/bin/env python3
"""reBotArm 纯物理位置重播系统 (带自动安全停机、监控与后台可视化)

功能特性：
1. 预引导阶段：从当前位置平滑引导至重播起点，消除启动突变。
2. 保护模式：重播结束后进入缓冲期，随后自动执行失能保护。
3. 纯物理闭环：基于 POS_VEL 模式，依靠 SafetyGuard 削峰与误差监控。
4. 后台可视化：后台绘制完整轨迹图，结束时保存，避免GUI阻塞控制循环。
5. 单图显示：所有关节qpos显示在同一张图上，便于比较。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
import math
import multiprocessing as mp
from pathlib import Path
from collections import deque
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from datetime import datetime

import h5py
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RobotArm

# --------------------------------------------------------------------------- #
# 配置参数
# --------------------------------------------------------------------------- #
DEFAULT_CMD_VLIM = np.array([1.5, 1.5, 1.5, 3.0, 3.0, 3.0])
DEFAULT_MAX_STEP = np.array([0.05, 0.05, 0.05, 0.06, 0.06, 0.06])
DEFAULT_TRACKING_BREACH_SAMPLES = 20
DEFAULT_VIS_WINDOW_SIZE = 200  # 可视化窗口数据点数
DEFAULT_VIS_UPDATE_INTERVAL = 5  # 控制循环到可视化进程的数据发送间隔

_running = True


def _sigint_handler(signum, frame) -> None:
    global _running
    print("\n[Replay] 收到退出信号，准备安全关闭...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)


@dataclass
class VisualizationData:
    """可视化数据结构"""
    time_data: deque
    joint_positions: List[deque]  # 6个关节
    joint_velocities: List[deque]  # 6个关节
    joint_commands: List[deque]  # 6个关节
    tracking_errors: List[deque]  # 6个关节
    recording_started: bool = False
    start_time_offset: float = 0.0  # 记录开始时间偏移

    def __init__(self, window_size: int = DEFAULT_VIS_WINDOW_SIZE):
        self.time_data = deque(maxlen=window_size)
        self.joint_positions = [deque(maxlen=window_size) for _ in range(6)]
        self.joint_velocities = [deque(maxlen=window_size) for _ in range(6)]
        self.joint_commands = [deque(maxlen=window_size) for _ in range(6)]
        self.tracking_errors = [deque(maxlen=window_size) for _ in range(6)]
        self.recording_started = False
        self.start_time_offset = 0.0

    def start_recording(self, start_time: float) -> None:
        """开始记录数据"""
        if not self.recording_started:
            self.recording_started = True
            self.start_time_offset = start_time
            print(f"[Recording] 开始记录数据，时间偏移: {start_time:.2f}s")

    def add_data(self, t: float, q_feedback: np.ndarray, q_vel_feedback: np.ndarray,
                 q_cmd: np.ndarray, q_target: np.ndarray) -> None:
        """添加新数据点（只有在开始记录后才保存）"""
        if not self.recording_started:
            return

        # 计算相对于记录开始的时间
        t_relative = t - self.start_time_offset

        self.time_data.append(t_relative)

        for i in range(6):
            self.joint_positions[i].append(q_feedback[i])
            self.joint_velocities[i].append(q_vel_feedback[i])
            self.joint_commands[i].append(q_cmd[i])
            self.tracking_errors[i].append(abs(q_cmd[i] - q_feedback[i]))

    def get_full_data(self) -> Dict[str, Any]:
        """获取完整数据用于绘图"""
        return {
            'time_data': list(self.time_data),
            'joint_positions': [list(pos) for pos in self.joint_positions],
            'joint_commands': [list(cmd) for cmd in self.joint_commands],
            'start_time_offset': self.start_time_offset
        }


def save_final_plot(task_name: str, speed_scale: float,
                    data: Dict[str, Any], output_dir: str = ".") -> str:
    """保存最终轨迹图"""
    import matplotlib
    matplotlib.use('Agg')  # 使用非交互式后端
    import matplotlib.pyplot as plt

    # 创建输出目录
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True)

    # 创建图形 - 所有关节在一个图上
    fig, ax = plt.subplots(figsize=(16, 8))

    # 关节颜色
    joint_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1',
                    '#96CEB4', '#FFEAA7', '#DDA0DD']

    # 关节标签
    joint_labels = [f'Joint {i + 1}' for i in range(6)]

    time_data = data.get('time_data', [])
    joint_positions = data.get('joint_positions', [])
    joint_commands = data.get('joint_commands', [])
    start_time_offset = data.get('start_time_offset', 0.0)

    if not time_data:
        plt.close(fig)
        return ""

    # 绘制所有关节
    for i in range(6):
        if i < len(joint_positions) and joint_positions[i]:
            # 实际位置线条（实线）
            ax.plot(time_data, joint_positions[i],
                    color=joint_colors[i],
                    linewidth=2,
                    label=f'{joint_labels[i]} (actual)',
                    alpha=0.8)

            # 指令位置线条（虚线）
            if i < len(joint_commands) and joint_commands[i]:
                ax.plot(time_data, joint_commands[i],
                        color=joint_colors[i],
                        linewidth=1.5,
                        linestyle='--',
                        label=f'{joint_labels[i]} (command)',
                        alpha=0.6)

    # 设置图形属性
    ax.set_xlabel('Time (s) [from replay start]', fontsize=12)
    ax.set_ylabel('Joint Position (rad)', fontsize=12)

    # 添加子标题显示开始记录的时间
    if start_time_offset > 0:
        title = f'Robot Arm Replay - {task_name} (Speed: {speed_scale}x)\nRecording started at t={start_time_offset:.1f}s'
    else:
        title = f'Robot Arm Replay - {task_name} (Speed: {speed_scale}x)'

    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=9, ncol=2)

    # 自动调整坐标轴范围
    if time_data and len(time_data) > 1:
        x_min, x_max = min(time_data), max(time_data)
        ax.set_xlim(x_min, x_max)

        # 找到所有数据的Y轴范围
        y_min, y_max = float('inf'), float('-inf')
        for i in range(6):
            if i < len(joint_positions) and joint_positions[i]:
                y_min = min(y_min, min(joint_positions[i]))
                y_max = max(y_max, max(joint_positions[i]))
            if i < len(joint_commands) and joint_commands[i]:
                y_min = min(y_min, min(joint_commands[i]))
                y_max = max(y_max, max(joint_commands[i]))

        if y_max > y_min:
            margin = (y_max - y_min) * 0.1
            ax.set_ylim(y_min - margin, y_max + margin)

    fig.tight_layout()

    # 保存图片
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = output_path / f"replay_final_{task_name}_{timestamp}.png"

    try:
        fig.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig)
        return str(filename)
    except Exception as e:
        plt.close(fig)
        raise e


def _parse_vector(values: list[float] | None, default: np.ndarray, name: str) -> np.ndarray:
    arr = default.astype(np.float64) if values is None else np.asarray(values, dtype=np.float64)
    if arr.shape != default.shape:
        raise ValueError(f"{name} 必须提供 {default.size} 个数，当前为 {arr.size} 个")
    return arr


def _clip_rate(target: np.ndarray, previous: np.ndarray, max_step: np.ndarray) -> np.ndarray:
    return previous + np.clip(target - previous, -max_step, max_step)


def _unwrap_near(values: np.ndarray, reference: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    return values + 2.0 * np.pi * np.round((reference - values) / (2.0 * np.pi))


def close_arm_fast(arm) -> None:
    if arm is None: return
    try:
        print("[硬件保护] 执行失能指令...")
        arm.disable(retries=0)
        time.sleep(0.1)
    except Exception:
        pass
    getattr(arm, "_ctrl_map", {}).clear()
    getattr(arm, "_motor_map", {}).clear()


class SafetyGuard:
    def __init__(self, num_joints: int, max_step: np.ndarray, max_tracking_error: float, tracking_breach_samples: int):
        self.num_joints = num_joints
        self.max_step = max_step[:num_joints]
        self.max_tracking_error = max_tracking_error
        self.tracking_breach_samples = max(int(tracking_breach_samples), 1)
        self.command = None
        self._tracking_breach_count = 0
        self.max_error_history = deque(maxlen=100)  # 记录最大误差历史

    def initialize(self, q_real_now: np.ndarray) -> np.ndarray:
        self.command = np.asarray(q_real_now, dtype=np.float64)[:self.num_joints].copy()
        return self.command.copy()

    def next_command(self, q_target: np.ndarray, q_feedback: np.ndarray) -> np.ndarray:
        q_target = np.asarray(q_target, dtype=np.float64)[:self.num_joints]
        q_feedback = np.asarray(q_feedback, dtype=np.float64)[:self.num_joints]
        q_target_cmd = _unwrap_near(q_target, q_feedback)
        previous_cmd = _unwrap_near(self.command, q_feedback)

        tracking_error = np.max(np.abs(q_target_cmd - q_feedback))
        self.max_error_history.append(tracking_error)

        if tracking_error > self.max_tracking_error:
            self._tracking_breach_count += 1
            if self._tracking_breach_count >= self.tracking_breach_samples:
                avg_max_error = np.mean(self.max_error_history) if self.max_error_history else 0
                raise RuntimeError(
                    f"真机跟踪误差过大: 当前 {tracking_error:.3f} rad, "
                    f"历史平均最大 {avg_max_error:.3f} rad, 触发保护停机。"
                )
        else:
            self._tracking_breach_count = 0

        self.command = _clip_rate(q_target_cmd, previous_cmd, self.max_step)
        return self.command.copy()


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="reBotArm 纯物理轨迹重播 (POS_VEL 模式)")
    parser.add_argument("--task_name", "-t", type=str, default="test_task")
    parser.add_argument("--dataset_dir", "-d", type=str, required=True)
    parser.add_argument("--episode_idx", "-idx", type=int, default=0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--speed_scale", type=float, default=1.0)
    parser.add_argument("--max-tracking-error", type=float, default=1.0)
    parser.add_argument("--vlim", type=float, nargs=6, default=None)
    parser.add_argument("--max-step", type=float, nargs=6, default=None)
    parser.add_argument("--print-every", type=int, default=50)
    parser.add_argument("--vis-update-interval", type=int, default=DEFAULT_VIS_UPDATE_INTERVAL,
                        help="控制循环到可视化进程的数据发送间隔")
    parser.add_argument("--no-visualization", action="store_true",
                        help="禁用可视化")
    parser.add_argument("--save-visualization", action="store_true",
                        help="结束时保存可视化图形")
    parser.add_argument("--vis-window-size", type=int, default=0,
                        help="可视化窗口大小（0表示保存完整轨迹）")
    parser.add_argument("--vis-output-dir", type=str, default="./replay_visualizations",
                        help="可视化图片保存目录")
    parser.add_argument("--record-start-time", type=float, default=4.0,
                        help="开始记录数据的时间（秒）")
    return parser


def main() -> None:
    global _running
    args = build_argparser().parse_args()
    vlim = _parse_vector(args.vlim, DEFAULT_CMD_VLIM, "--vlim")
    max_step = _parse_vector(args.max_step, DEFAULT_MAX_STEP, "--max-step")

    with h5py.File(args.dataset_dir, 'r') as f:
        qpos_data = np.array(f['qpos'])
        record_rate = f.attrs.get('hz_rate', args.rate)

    total_frames = len(qpos_data)
    record_dt = 1.0 / record_rate

    # 设置可视化窗口大小（0表示保存完整轨迹）
    vis_window_size = args.vis_window_size if args.vis_window_size > 0 else None

    # 本地数据存储用于可视化
    vis_data = VisualizationData(window_size=vis_window_size)

    arm = RobotArm()
    arm.connect()
    arm.enable()
    arm.mode_pos_vel(vlim=vlim)
    time.sleep(0.5)

    q_raw_feedback = arm.get_positions(request=True)[:6]
    q_feedback = _unwrap_near(q_raw_feedback, qpos_data[0][:6])

    guard = SafetyGuard(6, max_step, args.max_tracking_error, DEFAULT_TRACKING_BREACH_SAMPLES)
    guard.initialize(q_feedback)

    PREPARE_DURATION = 4.0
    REPLAY_DURATION = (total_frames * record_dt) / args.speed_scale

    t_start = time.perf_counter()
    frame = 0
    cmd_period = 1.0 / args.rate

    # 记录重播统计
    stats = {
        'total_frames': total_frames,
        'replayed_frames': 0,
        'max_tracking_error': 0.0,
        'avg_tracking_error': 0.0,
        'error_sum': 0.0
    }

    # 记录控制循环时间
    control_times = deque(maxlen=100)
    recording_started = False

    try:
        print(f"\n🚀 [Replay Start] Task: {args.task_name}, Speed: {args.speed_scale}x")
        print(f"   Prepare: {PREPARE_DURATION}s, Replay: {REPLAY_DURATION:.2f}s")
        print(f"   Total frames: {total_frames}, Rate: {args.rate}Hz")
        print(f"   Recording start time: {args.record_start_time}s")
        if not args.no_visualization:
            if vis_window_size:
                print(f"   Visualization: Enabled (window: {vis_window_size} points)")
            else:
                print(f"   Visualization: Enabled (full trajectory)")
        else:
            print(f"   Visualization: Disabled")

        while _running:
            loop_start = time.perf_counter()
            elapsed = loop_start - t_start
            q_feedback = arm.get_positions(request=True)[:6]

            # 计算速度
            try:
                q_vel_feedback = arm.get_velocities(request=True)[:6]
            except AttributeError:
                q_vel_feedback = np.zeros(6)

            # 生成目标指令
            if elapsed < PREPARE_DURATION:
                progress = elapsed / PREPARE_DURATION
                smooth = (1.0 - math.cos(progress * math.pi)) / 2.0
                q_target = q_feedback + (qpos_data[0][:6] - q_feedback) * smooth
                stage = "Prepare"
            elif elapsed < PREPARE_DURATION + REPLAY_DURATION:
                replay_time = elapsed - PREPARE_DURATION
                idx = min(int((replay_time * args.speed_scale) / record_dt), total_frames - 1)
                q_target = qpos_data[idx][:6]
                stage = "Replay"
                stats['replayed_frames'] = idx
            else:
                print(f"\n✅ [Replay Complete] Trajectory finished (elapsed={elapsed:.2f}s)")
                break

            # 下发指令
            q_cmd = guard.next_command(q_target, q_feedback)
            arm.pos_vel(q_cmd, vlim=vlim)

            # 更新统计
            tracking_error = np.abs(q_cmd - q_feedback)
            current_max_error = np.max(tracking_error)
            stats['max_tracking_error'] = max(stats['max_tracking_error'], current_max_error)
            stats['error_sum'] += np.sum(tracking_error)
            stats['avg_tracking_error'] = stats['error_sum'] / (frame + 1) / 6

            # 记录控制周期时间
            control_end = time.perf_counter()
            control_times.append(control_end - loop_start)

            # 检查是否应该开始记录数据
            if not recording_started and elapsed >= args.record_start_time:
                recording_started = True
                vis_data.start_recording(elapsed)
                print(f"[Recording] 开始记录数据用于可视化 (t={elapsed:.2f}s)")

            # 更新本地可视化数据（只有开始记录后才保存）
            if recording_started:
                vis_data.add_data(elapsed, q_feedback, q_vel_feedback, q_cmd, q_target)

            # 增强打印监控
            if frame % args.print_every == 0:
                info = []
                for i in range(6):
                    info.append(
                        f"J{i + 1}: {q_feedback[i] * 180 / np.pi:6.1f}° "
                        f"(cmd: {q_cmd[i] * 180 / np.pi:6.1f}°) "
                        f"err: {tracking_error[i] * 180 / np.pi:4.1f}°"
                    )
                progress_percent = (stats['replayed_frames'] / stats['total_frames'] * 100) if stage == "Replay" else 0
                control_freq = 1.0 / np.mean(control_times) if control_times else 0

                recording_indicator = "[R]" if recording_started else "[ ]"

                if stage == "Prepare":
                    print(f"[t={elapsed:5.2f}s]{recording_indicator}[{stage:7s}] " + " | ".join(
                        info[:2]) + f" [freq={control_freq:5.1f}Hz]")
                else:
                    print(
                        f"[t={elapsed:5.2f}s]{recording_indicator}[{stage:7s} {progress_percent:5.1f}%] " + " | ".join(
                            info[:2]) + f" [freq={control_freq:5.1f}Hz]")

            frame += 1

            # 精确控制循环周期
            sleep_time = cmd_period - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 控制循环超时
                if frame % 100 == 0 and -sleep_time > 0.001:
                    print(f"[Warning] Control loop timeout {-sleep_time * 1000:.1f}ms")

    except Exception as exc:
        print(f"\n[Error] {exc}")
    finally:
        # 显示最终统计信息
        if stats['replayed_frames'] > 0:
            control_freq = 1.0 / np.mean(control_times) if control_times else 0
            print(f"\n📈 [Replay Statistics]")
            print(f"   Progress: {stats['replayed_frames']}/{stats['total_frames']} frames")
            print(
                f"   Max tracking error: {stats['max_tracking_error']:.4f} rad ({stats['max_tracking_error'] * 180 / np.pi:.2f}°)")
            print(
                f"   Avg tracking error: {stats['avg_tracking_error']:.4f} rad ({stats['avg_tracking_error'] * 180 / np.pi:.2f}°)")
            print(f"   Avg control frequency: {control_freq:.1f} Hz")
            print(f"   Completion: {(stats['replayed_frames'] / stats['total_frames'] * 100):.1f}%")
            print(f"   Recording duration: {vis_data.time_data[-1] if vis_data.time_data else 0:.2f}s")
            print(f"   Data points recorded: {len(vis_data.time_data)}")

        # 保存最终轨迹图
        if not args.no_visualization and vis_data.time_data:
            try:
                print("\n📊 [Visualization] Saving final trajectory plot...")
                data_for_plot = vis_data.get_full_data()
                if data_for_plot['time_data']:
                    filename = save_final_plot(
                        args.task_name,
                        args.speed_scale,
                        data_for_plot,
                        args.vis_output_dir
                    )
                    if filename:
                        print(f"📊 [Visualization] Final plot saved: {filename}")
            except Exception as e:
                print(f"❌ [Visualization] Failed to save plot: {e}")

        # 保存额外的数据
        if args.save_visualization and vis_data.time_data:
            try:
                # 保存数据到文件
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                data_filename = Path(args.vis_output_dir) / f"replay_data_{args.task_name}_{timestamp}.npz"

                np.savez_compressed(
                    data_filename,
                    time_data=np.array(vis_data.time_data),
                    joint_positions=np.array([list(pos) for pos in vis_data.joint_positions]),
                    joint_velocities=np.array([list(vel) for vel in vis_data.joint_velocities]),
                    joint_commands=np.array([list(cmd) for cmd in vis_data.joint_commands]),
                    tracking_errors=np.array([list(err) for err in vis_data.tracking_errors]),
                    task_name=args.task_name,
                    speed_scale=args.speed_scale,
                    control_frequency=control_freq,
                    start_time_offset=vis_data.start_time_offset
                )
                print(f"📊 [Data] Motion data saved: {data_filename}")
            except Exception as e:
                print(f"❌ [Data] Failed to save motion data: {e}")

        close_arm_fast(arm)
        print("\n[Exit] Robot arm disabled.")


if __name__ == "__main__":
    main()

"""

# 基本使用 - 保存完整轨迹
python rebot_position_replay_save_plot.py --task_name test_task --dataset_dir ./collected_data/test_task/episode_0.hdf5

# 只保存最近200个数据点
python rebot_position_replay_save_plot.py --task_name test_task --dataset_dir ./collected_data/test_task/episode_0.hdf5 --vis-window-size 200

# 不保存图片
python rebot_position_replay_save_plot.py --task_name test_task --dataset_dir ./collected_data/test_task/episode_0.hdf5 --no-visualization

"""
