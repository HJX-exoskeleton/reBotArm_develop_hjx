#!/usr/bin/env python3
"""reBotArm / ALOHA Dataset Offline Advanced Visualization System.

Reads a specified episode_x.hdf5 file, extracts qpos, effort, and timestamps,
and renders an elegant, high-contrast, bold trajectory analysis plot in pure English
to prevent any font encoding or missing character issues (such as blocks/tofu characters).
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import h5py
import matplotlib.pyplot as plt


def main() -> None:
    # 1. CLI Argument Parser Initialization
    parser = argparse.ArgumentParser(
        description="reBotArm Dataset Offline Advanced Visualization Terminal",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--dataset_dir", "-d",
        type=str,
        required=True,
        help="The target task directory containing episode_x.hdf5 files"
    )
    parser.add_argument(
        "--episode_idx", "-idx",
        type=int,
        default=0,
        help="The specific episode index to visualize"
    )
    args = parser.parse_args()

    # 2. Path Validation
    base_dir = Path(args.dataset_dir)
    target_file = base_dir / f"episode_{args.episode_idx}.hdf5"

    print("=" * 60)
    print("  reBotArm / ALOHA Dataset Offline Visualization Pipeline")
    print(f"  📂 Directory: {base_dir.resolve()}")
    print(f"  📄 Parsing file: {target_file.name}")
    print("=" * 60)

    if not target_file.exists():
        print(f"❌ [Error] Target HDF5 file does not exist: {target_file.resolve()}")
        sys.exit(1)

    # 3. Parse HDF5 Dataset
    try:
        with h5py.File(target_file, 'r') as f:
            qpos = np.array(f['qpos'])  # shape=(N, 6)
            effort = np.array(f['effort'])  # shape=(N, 6)
            timestamps = np.array(f['timestamp']) if 'timestamp' in f else None

            # Extract attributes
            task_name = f.attrs.get('task_name', base_dir.name)
            hz_rate = f.attrs.get('hz_rate', 'Unknown')
            total_frames = f.attrs.get('total_frames', len(qpos))
            duration = f.attrs.get('duration_seconds', 'Unknown')
    except Exception as e:
        print(f"❌ [Error] Failed to read HDF5 dataset: {e}")
        sys.exit(1)

    # Reconstruct timestamps if missing
    if timestamps is None:
        dt = 0.02 if hz_rate == 50 else 0.002
        timestamps = np.arange(total_frames) * dt

    if duration == 'Unknown':
        duration = f"{timestamps[-1]:.2f}"

    print(f"📊 [Dataset Metadata Summary]")
    print(f"   🔹 Task Name:     {task_name}")
    print(f"   🔹 Sampling Rate: {hz_rate} Hz")
    print(f"   🔹 Total Frames:  {total_frames} frames")
    print(f"   🔹 Duration:      {duration} seconds")
    print("-" * 60)

    # ----------------------------------------------------------------------- #
    # 🌟 4. Academic Plotting Attributes Customization (Pure English)
    # ----------------------------------------------------------------------- #
    # Force use of clean, standard sans-serif fonts natively supported by Linux/X11
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans']
    plt.rcParams['axes.unicode_minus'] = False

    # Initialize a 2x2 multi-plot canvas
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.canvas.manager.set_window_title(f"Dataset Analysis - {task_name} (Episode {args.episode_idx})")
    fig.patch.set_facecolor('#F8F9FA')  # Light gray tech background

    # Standard high-contrast academic colors
    COLORS = ['#1F77B4', '#D62728', '#2CA02C', '#9467BD', '#FF7F0E', '#17BECF']

    # --- [Plot 1: Full Joint Configurations (Top Left)] ---
    ax_pos_all = axes[0, 0]
    ax_pos_all.set_facecolor('#FFFFFF')
    for i in range(6):
        ax_pos_all.plot(timestamps, qpos[:, i], color=COLORS[i], lw=2.0, label=f"Joint {i + 1}", alpha=0.85)
    ax_pos_all.set_title("All Joint Configurations (qpos)", fontsize=13, fontweight='bold', color='#212529', pad=10)
    ax_pos_all.set_ylabel("Joint Position (rad)", fontsize=11, fontweight='bold')
    ax_pos_all.grid(True, linestyle='--', color='#E9ECEF', linewidth=0.8)
    ax_pos_all.legend(loc="upper right", frameon=True, facecolor='#FFFFFF', edgecolor='#CED4DA', ncol=2)

    # --- [Plot 2: Full Joint Efforts (Bottom Left)] ---
    ax_eff_all = axes[1, 0]
    ax_eff_all.set_facecolor('#FFFFFF')
    for i in range(6):
        ax_eff_all.plot(timestamps, effort[:, i], color=COLORS[i], lw=2.0, label=f"Joint {i + 1}", alpha=0.85)
    ax_eff_all.set_title("All Feedforward Joint Torques (Effort)", fontsize=13, fontweight='bold', color='#212529',
                         pad=10)
    ax_eff_all.set_xlabel("Time Elapsed (s)", fontsize=11, fontweight='bold')
    ax_eff_all.set_ylabel("Torque Output (N·m)", fontsize=11, fontweight='bold')
    ax_eff_all.grid(True, linestyle='--', color='#E9ECEF', linewidth=0.8)
    ax_eff_all.legend(loc="upper right", frameon=True, facecolor='#FFFFFF', edgecolor='#CED4DA', ncol=2)

    # --- [Plot 3: 🌟 Joint 2 Position Close-up (Top Right)] ---
    ax_j2_pos = axes[0, 1]
    ax_j2_pos.set_facecolor('#FFFFFF')
    # Bold line with lw=4.0 for emphasis
    ax_j2_pos.plot(timestamps, qpos[:, 1], color=COLORS[1], lw=4.0, label="Joint 2 (Shoulder Pitch)")
    ax_j2_pos.set_title("Joint 2 Position Characteristic Trajectory", fontsize=13, fontweight='bold', color='#D62728',
                        pad=10)
    ax_j2_pos.set_ylabel("Joint Position (rad)", fontsize=11, fontweight='bold')
    ax_j2_pos.grid(True, linestyle='--', color='#EDE0E0', linewidth=1.0)
    ax_j2_pos.legend(loc="upper right", frameon=True, facecolor='#FFF5F5', edgecolor='#D62728')

    # --- [Plot 4: 🌟 Joint 2 Effort Close-up (Bottom Right)] ---
    ax_j2_eff = axes[1, 1]
    ax_j2_eff.set_facecolor('#FFFFFF')
    ax_j2_eff.plot(timestamps, effort[:, 1], color=COLORS[1], lw=4.0, label="Joint 2 Gravity Comp Torque")
    ax_j2_eff.set_title("Joint 2 Gravity Compensation Torque Characteristic", fontsize=13, fontweight='bold',
                        color='#D62728', pad=10)
    ax_j2_eff.set_xlabel("Time Elapsed (s)", fontsize=11, fontweight='bold')
    ax_j2_eff.set_ylabel("Torque Output (N·m)", fontsize=11, fontweight='bold')
    ax_j2_eff.grid(True, linestyle='--', color='#EDE0E0', linewidth=1.0)
    ax_j2_eff.legend(loc="upper right", frameon=True, facecolor='#FFF5F5', edgecolor='#D62728')

    # Standardize border styles and tick marks
    for row in axes:
        for ax in row:
            for spine in ax.spines.values():
                spine.set_linewidth(1.2)
                spine.set_color('#6C757D')
            ax.tick_params(axis='both', labelsize=10, colors='#495057')

    plt.tight_layout(pad=3.0)

    # Auto-save chart as a high-resolution lossless PNG file for your documentation or paper figures
    output_img_path = base_dir / f"{task_name}_episode_{args.episode_idx}_analysis.png"
    plt.savefig(output_img_path, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none')
    print(f"📸 [Success] 300 DPI Academic chart exported successfully:")
    print(f"   📁 Path: {output_img_path.resolve()}\n")

    # Display window smoothly
    plt.show()


if __name__ == "__main__":
    main()

# python visualize_data.py --dataset_dir /home/hjx/hjx_file/rebot_devarm_ws/reBotArm_develop_hjx/gravity_compensation/collected_data/test_task --episode_idx 0

