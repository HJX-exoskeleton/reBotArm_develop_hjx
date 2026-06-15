#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
具身智能 HDF5 数据集全面分析与可视化工具
功能:
1. 提取并渲染 RGB 相机视频 (MP4)
2. 绘制 6轴机械臂关节 目标指令(Action) vs 实际反馈(qpos) 追踪曲线
3. 绘制 夹爪 (Gripper) 追踪曲线
"""

import h5py
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
from tqdm import tqdm

# =============================================================================
# 0. 配置路径
# =============================================================================
path = '/home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/master_slave_control/Servo_control/data/rebot_test/episode_0.hdf5'

base_dir = os.path.dirname(path)
file_name = os.path.basename(path).split('.')[0]  # 获取 'episode_0'

video_save_path = os.path.join(base_dir, f"{file_name}_cam_high.mp4")
plot_arm_save_path = os.path.join(base_dir, f"{file_name}_arm_tracking.png")
plot_gripper_save_path = os.path.join(base_dir, f"{file_name}_gripper_tracking.png")

print(f"📂 正在分析数据集: {path}")

# =============================================================================
# 1. 安全读取数据
# =============================================================================
with h5py.File(path, mode='r') as obj:
    # 提取时间戳
    time_steps = obj['time'][:]
    fps = obj.attrs.get('hz_rate', 50)

    # 提取本体感受数据 (N, 7) -> 前6个是机械臂，第7个是夹爪
    action_data = obj['action']['target_pos'][:]
    qpos_data = obj['observations']['qpos'][:]
    qvel_data = obj['observations']['qvel'][:]

    print("\n=== 动作与状态维度 ===")
    print(f"  Time Steps:    {time_steps.shape}")
    print(f"  Target Action: {action_data.shape}")
    print(f"  QPos (状态):   {qpos_data.shape}")

    # =============================================================================
    # 2. 绘制并保存关节曲线 (Action vs Qpos)
    # =============================================================================
    print("\n📈 正在生成关节跟踪曲线图...")

    # --- A. 6轴机械臂曲线 ---
    fig, axs = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle(f'Arm Joints Tracking: Command vs Feedback ({file_name})', fontsize=16)

    for i in range(6):
        row, col = i // 2, i % 2
        ax = axs[row, col]
        # 绘制目标指令 (虚线)
        ax.plot(time_steps, action_data[:, i], label='Command (Action)', linestyle='--', color='red', linewidth=2)
        # 绘制真实反馈 (实线)
        ax.plot(time_steps, qpos_data[:, i], label='Real (qpos)', color='blue', alpha=0.7, linewidth=2)

        ax.set_title(f'Joint {i + 1}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (rad)')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend()

    plt.tight_layout()
    plt.savefig(plot_arm_save_path, dpi=300)  # 保存高清图
    plt.close(fig)  # 释放内存
    print(f"✅ 机械臂曲线已保存至: {plot_arm_save_path}")

    # --- B. 夹爪曲线 ---
    fig_grp, ax_grp = plt.subplots(figsize=(8, 4))
    fig_grp.suptitle(f'Gripper Tracking ({file_name})', fontsize=14)

    # 夹爪是第 7 个元素 (索引为 6)
    ax_grp.plot(time_steps, action_data[:, 6], label='Command (Action)', linestyle='--', color='red', linewidth=2)
    ax_grp.plot(time_steps, qpos_data[:, 6], label='Real (qpos)', color='green', alpha=0.8, linewidth=2)

    ax_grp.set_xlabel('Time (s)')
    ax_grp.set_ylabel('Position')
    ax_grp.grid(True, linestyle=':', alpha=0.6)
    ax_grp.legend()

    plt.tight_layout()
    plt.savefig(plot_gripper_save_path, dpi=300)
    plt.close(fig_grp)
    print(f"✅ 夹爪曲线已保存至: {plot_gripper_save_path}")

    # =============================================================================
    # 3. 渲染相机视频
    # =============================================================================
    if 'images' in obj['observations'] and 'cam_high' in obj['observations']['images']:
        print("\n=== 开始导出相机视频 ===")
        cam_data = obj['observations']['images']['cam_high']
        num_frames, height, width, channels = cam_data.shape

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_save_path, fourcc, fps, (width, height))

        for i in tqdm(range(num_frames), desc="🎞️ 视频生成中", unit="帧"):
            frame_rgb = cam_data[i]
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            video_writer.write(frame_bgr)

        video_writer.release()
        print(f"🎉 视频导出完成！已保存至: {video_save_path}")
    else:
        print("\n⚠️ 未在数据集中找到 'cam_high' 图像数据，跳过视频渲染。")

print("\n✨ 所有分析任务完成！可以去文件夹查看图片和视频了。")


