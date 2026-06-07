#!/usr/bin/env python3
"""重力补偿控制演示（安全闭环优化版）。

主要修正：在流程彻底退出、异常触发或手动点击关闭 GUI 窗口时，
均能百分之百确保向全轴达妙电机发送【失能 (Disable)】指令，释放全部残余力矩。
"""
import signal
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RobotArm
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
    compute_generalized_gravity,
    get_default_gravity,
)

# --------------------------------------------------------------------------- #
# 全局控制标志与高级调参配置
# --------------------------------------------------------------------------- #

_running = True

# [安全保险] 限制各关节最大前馈力矩
TORQUE_LIMITS = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

# 各轴特异化阻尼系数 (Kd)  # 各轴重力前馈缩放系数
KD_CONFIG = np.array([1.0, 2.0, 1.5, 1.0, 0.8, 0.6])
GRAVITY_SCALES = np.array([1.50, 0.75, 0.75, 0.75, 1.0, 1.0])

# 各轴特异化阻尼系数 (Kd)  # 各轴重力前馈缩放系数
# KD_CONFIG = np.array([1.0, 1.0, 1.0, 1.0, 0.8, 0.8])
# GRAVITY_SCALES = np.array([1.50, 0.85, 0.85, 0.82, 1.0, 1.0])

# --------------------------------------------------------------------------- #
# 可视化全局缓存定义
# --------------------------------------------------------------------------- #
WINDOW_SIZE = 500
data_time_buffer = deque(maxlen=WINDOW_SIZE)
data_qpos_buffer = deque(maxlen=WINDOW_SIZE)
start_time = time.time()


def _sigint_handler(signum, frame):
    global _running
    print("\n[gravity_comp] 收到 Ctrl+C，触发停机保护...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


# --------------------------------------------------------------------------- #
# 🌟 [新增] 专门拦截鼠标点击关闭窗口事件的监听器
# --------------------------------------------------------------------------- #
def _on_window_close(event):
    global _running
    print("\n[gravity_comp] 检测到图形窗口被关闭，触发安全停机...')")
    _running = False


# --------------------------------------------------------------------------- #
# 控制回调（高频实时循环驱动）
# --------------------------------------------------------------------------- #

def gravity_compensation_controller(arm: RobotArm, dt: float) -> None:
    """重力补偿控制回调。"""
    if not hasattr(gravity_compensation_controller, "_counter"):
        gravity_compensation_controller._counter = 0

    if not _running:
        return

    q = arm.get_positions()

    current_timestamp = time.time() - start_time
    data_time_buffer.append(current_timestamp)
    data_qpos_buffer.append(q[:6])

    tau_g_raw = compute_generalized_gravity(q=q[:6])
    tau_g = tau_g_raw * GRAVITY_SCALES[:arm.num_joints]
    tau_g_safe = np.clip(tau_g, -TORQUE_LIMITS[:arm.num_joints], TORQUE_LIMITS[:arm.num_joints])

    arm.mit(
        pos=q,
        vel=np.zeros(arm.num_joints),
        kp=np.zeros(arm.num_joints),
        kd=KD_CONFIG[:arm.num_joints],
        tau=tau_g_safe,
        request_feedback=True,
    )

    gravity_compensation_controller._counter += 1
    if gravity_compensation_controller._counter % 50 == 0:
        print(
            f"[{gravity_compensation_controller._counter:5d}] "
            f"tau_g(计算/限幅) = " +
            " | ".join(f"{r:+.2f}/{s:+.2f}" for r, s in zip(tau_g_raw, tau_g_safe)) +
            " N·m"
        )


# --------------------------------------------------------------------------- #
# 主程序
# --------------------------------------------------------------------------- #

def main() -> None:
    global _running
    print("=" * 60)
    print("  reBotArm 重力补偿安全优化版 + 顶会级学术可视化 (安全全闭环)")
    print("=" * 60)

    model = load_dynamics_model()
    arm = RobotArm()
    arm.connect()
    arm.enable()
    arm.mode_mit(kp=np.zeros(arm.num_joints), kd=KD_CONFIG[:arm.num_joints])

    # ----------------------------------------------------------------------- #
    # 霸气学术风绘图属性高级定制
    # ----------------------------------------------------------------------- #
    plt.ion()
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'SimHei']
    plt.rcParams['axes.unicode_minus'] = False

    fig, axes = plt.subplots(2, 3, figsize=(15, 9), sharex=True)
    fig.canvas.manager.set_window_title("reBotArm 机械臂关节运动轨迹 (全闭环安全保护)")
    fig.patch.set_facecolor('#F8F9FA')
    axes = axes.flatten()

    # 🌟【核心安全改进】：绑定窗口关闭事件，点红叉时立刻让 _running 为 False
    fig.canvas.mpl_connect('close_event', _on_window_close)

    ACADEMIC_COLORS = ['#1F77B4', '#D62728', '#2CA02C', '#9467BD', '#FF7F0E', '#17BECF']

    lines = []
    for i in range(6):
        ax = axes[i]
        ax.set_facecolor('#FFFFFF')
        line, = ax.plot([], [], label=r'$\mathbf{Joint\ ' + str(i + 1) + r'}$',
                        color=ACADEMIC_COLORS[i], lw=3.0, alpha=0.95, antialiased=True)
        lines.append(line)
        ax.set_title(f"Joint {i + 1} Trajectory", fontsize=12, fontweight='bold', color='#212529', pad=10)
        ax.set_ylabel("Position (rad)", fontsize=10, fontweight='bold', color='#495057')
        ax.grid(True, which='both', linestyle='--', color='#E9ECEF', linewidth=0.8)
        for spine in ax.spines.values():
            spine.set_linewidth(1.2)
            spine.set_color('#6C757D')
        ax.legend(loc="upper right", frameon=True, facecolor='#FFFFFF', edgecolor='#CED4DA', fontsize=10)

    for j in [3, 4, 5]:
        axes[j].set_xlabel("Time Elapsed (s)", fontsize=11, fontweight='bold', color='#212529', labelpad=10)

    plt.tight_layout(pad=3.0)

    # 启动高频控制
    arm.start_control_loop(gravity_compensation_controller, rate=arm._rate)
    print(f"[控制循环] 启动 @ {arm._rate} Hz")
    print("-" * 60)

    # 主线程高画质刷新渲染
    try:
        while _running:
            if plt.fignum_exists(fig.number) and len(data_time_buffer) > 1:
                t_data = list(data_time_buffer)
                q_data = np.array(list(data_qpos_buffer))

                for i in range(6):
                    lines[i].set_data(t_data, q_data[:, i])
                    axes[i].set_xlim(t_data[0], t_data[-1] + 0.05)
                    y_min, y_max = q_data[:, i].min(), q_data[:, i].max()
                    y_range = y_max - y_min

                    if y_range < 0.01:
                        axes[i].set_ylim(y_min - 0.2, y_min + 0.2)
                    else:
                        axes[i].set_ylim(y_min - y_range * 0.15, y_max + y_range * 0.15)

                fig.canvas.draw()
                fig.canvas.flush_events()

            time.sleep(0.03)
    except Exception as e:
        print(f"\n[可视化异常] {e}")
    finally:
        print("\n" + "=" * 50)
        print("[退出流程] 正在紧急向全轴电机下发停机安全指令...")
        _running = False

        # 1. 注销并彻底关闭 Matplotlib 绘图区
        try:
            plt.ioff()
            plt.close('all')
        except:
            pass

        # 2. 🌟【最核心修改】：断开前，必须显式调用底层驱动的失能函数释放力矩
        try:
            print("[退出流程] 正在下发【失能 (Disable)】帧（全轴去电释放力矩）...")
            arm.disable()
            time.sleep(0.2)  # 给通信链路留出 200 毫秒的物理发送缓冲时间
        except Exception as ex:
            print(f"[⚠️警告] 底层电机失能失败: {ex}")

        # 3. 安全切断套接字/串口通道
        arm.disconnect()
        print("[退出流程] 机械臂已安全失能断开。")
        print("=" * 50 + "\n")


if __name__ == "__main__":
    main()
