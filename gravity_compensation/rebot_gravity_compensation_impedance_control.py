#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reBotArm 重力补偿 + 拖拽示教 + 关节空间阻抗控制安全版

功能：
1. 启动后进入 FREE_DRAG 自由拖拽模式：
   - kp = 0
   - 仅重力补偿 + 小阻尼
   - 可以手动拖拽机械臂

2. 检测机械臂静止一段时间后，自动记录当前关节角为 q_ref：
   - 自动切换到 IMPEDANCE_HOLD 模式

3. IMPEDANCE_HOLD 模式：
   - 持续做重力补偿
   - 通过关节空间 Kp/Kd 形成阻抗效果
   - 末端受到外力时，机械臂柔顺偏移
   - 松手后回到 q_ref 附近

4. 安全保护：
   - Ctrl+C 停机
   - GUI 关闭停机
   - q 数据异常停机
   - 力矩限幅
   - 力矩变化率限制
   - 退出时多次发送 Disable
"""

import signal
import sys
import time
from collections import deque
from pathlib import Path
from threading import Lock

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RobotArm
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
    compute_generalized_gravity,
)


# =============================================================================
# 全局运行标志
# =============================================================================

_running = True


def _request_stop(reason: str = ""):
    global _running
    if reason:
        print(f"\n[gravity_impedance] {reason}")
    _running = False


def _sigint_handler(signum, frame):
    _request_stop("收到 Ctrl+C，触发安全停机。")


signal.signal(signal.SIGINT, _sigint_handler)


def _on_window_close(event):
    _request_stop("检测到图形窗口关闭，触发安全停机。")


# =============================================================================
# 控制参数配置
# =============================================================================

# 动力学模型目前按前 6 个关节计算
DYN_DOF = 6

# ------------------------ 重力补偿参数 ------------------------
# 每轴最大前馈力矩限幅，单位 N·m
TORQUE_LIMITS_6 = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

# 每轴重力补偿缩放系数
# 如果某个关节会下坠：增大对应 scale
# 如果某个关节会自己上抬：减小对应 scale
GRAVITY_SCALES_6 = np.array([1.50, 1.00, 0.85, 0.85, 1.00, 1.00])

# 启动时重力补偿渐入时间，防止突然发力
GRAVITY_RAMP_TIME = 2.0

# 力矩变化率限制，单位 N·m/s
# 越小越平滑，但响应越慢
TAU_RATE_LIMIT_6 = np.array([25.0, 25.0, 25.0, 12.0, 12.0, 12.0])


# ------------------------ 自由拖拽模式参数 ------------------------
# 自由拖拽时 kp=0，只保留较小阻尼
# 如果拖起来发粘，降低 KD_DRAG
# 如果拖起来容易晃，略微增加 KD_DRAG
KD_DRAG_6 = np.array([0.6, 0.8, 0.8, 0.6, 0.3, 0.25])


# ------------------------ 阻抗保持模式参数 ------------------------
# 关节空间阻抗刚度，单位大致可理解为 N·m/rad
# 越大：越像“固定在原位”
# 越小：越柔顺，末端更容易被推开
KP_HOLD_6 = np.array([8.0, 10.0, 8.0, 4.0, 2.0, 1.5])
# KP_HOLD_6 = np.array([2.0, 3.0, 2.5, 1.2, 0.6, 0.4])

# 关节空间阻抗阻尼
# 越大：回弹更稳但更粘
# 越小：更轻快但可能振荡
KD_HOLD_6 = np.array([1.0, 1.3, 1.2, 0.8, 0.4, 0.3])
# KD_HOLD_6 = np.array([0.35, 0.45, 0.40, 0.25, 0.12, 0.10])


# ------------------------ 自动锁定判据 ------------------------
# 在自由拖拽模式下，如果所有关节速度都低于该阈值，则认为接近静止
AUTO_LOCK_VEL_THRESH = 0.018  # rad/s

# 连续静止多久后自动锁定当前位置
AUTO_LOCK_STILL_TIME = 1.0  # s

# 启动后至少等多久才允许自动锁定，避免刚开机立即误锁
AUTO_LOCK_MIN_START_TIME = 1.5  # s


# ------------------------ 速度估计滤波参数 ------------------------
# qdot 低通滤波系数，越大越跟随当前速度，越小越平滑
QD_FILTER_ALPHA = 0.25


# =============================================================================
# 可视化缓存
# =============================================================================

WINDOW_SIZE = 500
data_lock = Lock()

data_time_buffer = deque(maxlen=WINDOW_SIZE)
data_qpos_buffer = deque(maxlen=WINDOW_SIZE)
data_qref_buffer = deque(maxlen=WINDOW_SIZE)
data_mode_buffer = deque(maxlen=WINDOW_SIZE)


# =============================================================================
# 工具函数
# =============================================================================

def fit_to_n(arr, n, fill=0.0):
    """
    将 6 轴参数自动适配到 arm.num_joints。
    如果机械臂是 6 轴，原样返回。
    如果机械臂是 7 轴，前 6 轴使用配置，第 7 轴用 fill。
    """
    arr = np.asarray(arr, dtype=float).reshape(-1)
    out = np.full(n, fill, dtype=float)
    m = min(len(arr), n)
    out[:m] = arr[:m]
    return out


def safe_clip_tau(tau, limits):
    return np.clip(tau, -limits, limits)


# =============================================================================
# 重力补偿 + 阻抗控制器
# =============================================================================

class GravityImpedanceController:
    def __init__(self):
        self.mode = "FREE_DRAG"

        self.q_ref = None
        self.q_prev = None
        self.t_prev = None
        self.qd_filt = None

        self.tau_prev = None
        self.still_since = None

        self.start_time = time.time()
        self.counter = 0

        self.force_lock_request = False
        self.force_free_request = False

    def request_lock_current(self):
        self.force_lock_request = True

    def request_free_drag(self):
        self.force_free_request = True

    def _estimate_velocity(self, q, now):
        """
        用关节位置差分估计速度，并做一阶低通滤波。
        注意：这个速度主要用于自动锁定判断和可视化，不直接作为电机反馈。
        """
        if self.q_prev is None or self.t_prev is None:
            qd = np.zeros_like(q)
        else:
            dt = max(now - self.t_prev, 1e-4)
            qd = (q - self.q_prev) / dt

        if self.qd_filt is None:
            self.qd_filt = qd
        else:
            self.qd_filt = (
                QD_FILTER_ALPHA * qd
                + (1.0 - QD_FILTER_ALPHA) * self.qd_filt
            )

        self.q_prev = q.copy()
        self.t_prev = now
        return self.qd_filt.copy()

    def _compute_safe_gravity_tau(self, q, n, now):
        """
        计算重力补偿力矩，并加入：
        1. 每轴 scale
        2. 启动渐入
        3. 力矩限幅
        4. 力矩变化率限制
        """
        if len(q) < DYN_DOF:
            raise RuntimeError(f"当前关节数不足 {DYN_DOF}，无法进行 6 轴动力学重力补偿。")

        tau_g_raw_6 = compute_generalized_gravity(q=q[:DYN_DOF])
        tau_g_raw_6 = np.asarray(tau_g_raw_6, dtype=float).reshape(-1)

        if len(tau_g_raw_6) != DYN_DOF:
            raise RuntimeError(
                f"compute_generalized_gravity 返回维度异常，期望 {DYN_DOF}，实际 {len(tau_g_raw_6)}。"
            )

        # 每轴补偿比例
        tau_g_6 = tau_g_raw_6 * GRAVITY_SCALES_6

        # 启动渐入
        elapsed = now - self.start_time
        ramp = min(1.0, max(0.0, elapsed / GRAVITY_RAMP_TIME))
        tau_g_6 = ramp * tau_g_6

        # 幅值限幅
        tau_g_6 = safe_clip_tau(tau_g_6, TORQUE_LIMITS_6)

        # 适配到机械臂实际关节数
        tau_g = fit_to_n(tau_g_6, n, fill=0.0)
        torque_limits = fit_to_n(TORQUE_LIMITS_6, n, fill=2.0)
        tau_rate_limits = fit_to_n(TAU_RATE_LIMIT_6, n, fill=8.0)

        # 力矩变化率限制
        if self.tau_prev is not None and self.t_prev is not None:
            dt = max(now - self.t_prev, 1e-4)
            max_delta = tau_rate_limits * dt
            tau_g = self.tau_prev + np.clip(tau_g - self.tau_prev, -max_delta, max_delta)

        tau_g = safe_clip_tau(tau_g, torque_limits)
        self.tau_prev = tau_g.copy()

        return tau_g, tau_g_raw_6

    def _lock_current_position(self, q):
        self.q_ref = q.copy()
        self.mode = "IMPEDANCE_HOLD"
        self.still_since = None
        print("\n[模式切换] 已锁定当前位置 q_ref，进入 IMPEDANCE_HOLD 阻抗保持模式。")
        print("[提示] 现在末端受到外力时会柔顺偏移，松手后回到当前参考位姿附近。")

    def _switch_to_free_drag(self, q):
        self.mode = "FREE_DRAG"
        self.q_ref = q.copy()
        self.still_since = None
        print("\n[模式切换] 进入 FREE_DRAG 自由拖拽模式。")
        print("[提示] 拖拽机械臂到目标位置，静止约 1 秒后会自动锁定。")

    def __call__(self, arm: RobotArm, dt: float) -> None:
        global _running

        if not _running:
            return

        now = time.time()
        n = arm.num_joints

        try:
            q = np.asarray(arm.get_positions(), dtype=float).reshape(-1)

            if len(q) < n:
                raise RuntimeError(f"读取到的关节角数量 {len(q)} 小于 arm.num_joints={n}")

            q = q[:n]

            if not np.all(np.isfinite(q)):
                raise RuntimeError("关节角数据包含 NaN 或 Inf。")

            qd_est = self._estimate_velocity(q, now)
            tau_g, tau_g_raw_6 = self._compute_safe_gravity_tau(q, n, now)

            if self.q_ref is None:
                self.q_ref = q.copy()

            # 手动模式请求
            if self.force_free_request:
                self.force_free_request = False
                self._switch_to_free_drag(q)

            if self.force_lock_request:
                self.force_lock_request = False
                self._lock_current_position(q)

            # 自动锁定逻辑：只在 FREE_DRAG 模式下生效
            if self.mode == "FREE_DRAG":
                elapsed = now - self.start_time
                max_abs_qd = np.max(np.abs(qd_est[:DYN_DOF]))

                if elapsed > AUTO_LOCK_MIN_START_TIME and max_abs_qd < AUTO_LOCK_VEL_THRESH:
                    if self.still_since is None:
                        self.still_since = now
                    elif now - self.still_since >= AUTO_LOCK_STILL_TIME:
                        self._lock_current_position(q)
                else:
                    self.still_since = None

            # ================================================================
            # 模式 1：自由拖拽
            # ================================================================
            if self.mode == "FREE_DRAG":
                pos_cmd = q.copy()
                vel_cmd = np.zeros(n)

                kp_cmd = np.zeros(n)
                kd_cmd = fit_to_n(KD_DRAG_6, n, fill=0.2)

            # ================================================================
            # 模式 2：阻抗保持
            # ================================================================
            elif self.mode == "IMPEDANCE_HOLD":
                pos_cmd = self.q_ref.copy()
                vel_cmd = np.zeros(n)

                kp_cmd = fit_to_n(KP_HOLD_6, n, fill=0.0)
                kd_cmd = fit_to_n(KD_HOLD_6, n, fill=0.2)

            else:
                raise RuntimeError(f"未知控制模式: {self.mode}")

            # 下发 MIT 控制
            # 实际效果：
            # FREE_DRAG:
            #   tau_cmd = tau_g + Kd_drag * (0 - qdot)
            #
            # IMPEDANCE_HOLD:
            #   tau_cmd = tau_g + Kp_hold * (q_ref - q) + Kd_hold * (0 - qdot)
            arm.mit(
                pos=pos_cmd,
                vel=vel_cmd,
                kp=kp_cmd,
                kd=kd_cmd,
                tau=tau_g,
                request_feedback=True,
            )

            self.counter += 1

            # 可视化缓存
            plot_dof = min(DYN_DOF, n)
            with data_lock:
                data_time_buffer.append(now - self.start_time)
                data_qpos_buffer.append(q[:plot_dof].copy())

                if self.mode == "IMPEDANCE_HOLD" and self.q_ref is not None:
                    data_qref_buffer.append(self.q_ref[:plot_dof].copy())
                else:
                    data_qref_buffer.append(np.full(plot_dof, np.nan))

                data_mode_buffer.append(self.mode)

            if self.counter % 50 == 0:
                tau_safe_6 = tau_g[:DYN_DOF]
                print(
                    f"[{self.counter:5d}] mode={self.mode:<15s} | "
                    f"tau_g(raw/safe) = "
                    + " | ".join(
                        f"{r:+.2f}/{s:+.2f}"
                        for r, s in zip(tau_g_raw_6, tau_safe_6)
                    )
                    + " N·m"
                )

        except Exception as e:
            print(f"\n[控制异常] {e}")
            _running = False


# =============================================================================
# Matplotlib 键盘控制
# =============================================================================

def make_key_handler(controller: GravityImpedanceController):
    def _on_key_press(event):
        global _running

        if event.key == "h":
            print("\n[键盘] h：手动锁定当前位置。")
            controller.request_lock_current()

        elif event.key == "f":
            print("\n[键盘] f：切换到自由拖拽模式。")
            controller.request_free_drag()

        elif event.key == "q":
            print("\n[键盘] q：退出程序。")
            _running = False

    return _on_key_press


# =============================================================================
# 安全关闭
# =============================================================================

def safe_shutdown(arm: RobotArm):
    print("\n" + "=" * 60)
    print("[退出流程] 正在执行机械臂安全停机流程...")

    try:
        plt.ioff()
        plt.close("all")
    except Exception:
        pass

    # 如果 RobotArm 库有 stop_control_loop，优先停止控制线程
    try:
        if hasattr(arm, "stop_control_loop"):
            print("[退出流程] 正在停止控制循环...")
            arm.stop_control_loop()
            time.sleep(0.1)
    except Exception as ex:
        print(f"[警告] 停止控制循环失败: {ex}")

    # 多次发送 disable，降低通信丢帧风险
    try:
        print("[退出流程] 正在下发 Disable 指令，全轴释放力矩...")
        for _ in range(3):
            arm.disable()
            time.sleep(0.08)
    except Exception as ex:
        print(f"[警告] 电机失能失败: {ex}")

    try:
        arm.disconnect()
        print("[退出流程] 机械臂已断开连接。")
    except Exception as ex:
        print(f"[警告] 断开连接失败: {ex}")

    print("[退出流程] 安全停机完成。")
    print("=" * 60 + "\n")


# =============================================================================
# 主程序
# =============================================================================

def main() -> None:
    global _running

    print("=" * 70)
    print("  reBotArm 重力补偿 + 拖拽示教 + 关节空间阻抗控制安全版")
    print("=" * 70)
    print("[控制说明]")
    print("  启动后：FREE_DRAG 自由拖拽模式")
    print("  静止后：自动锁定当前位置，进入 IMPEDANCE_HOLD 阻抗保持模式")
    print("  按 h ：手动锁定当前位置")
    print("  按 f ：重新进入自由拖拽模式")
    print("  按 q ：安全退出")
    print("=" * 70)

    arm = None

    try:
        # 加载动力学模型
        # 如果 load_dynamics_model 内部设置了全局模型，这里必须保留
        _ = load_dynamics_model()

        # 连接机械臂
        arm = RobotArm()
        arm.connect()
        arm.enable()

        n = arm.num_joints
        print(f"[机械臂] 已连接，关节数: {n}")

        # 初始进入 MIT 模式：kp=0，自由拖拽阻尼
        arm.mode_mit(
            kp=np.zeros(n),
            kd=fit_to_n(KD_DRAG_6, n, fill=0.2),
        )

        controller = GravityImpedanceController()

        # ---------------------------------------------------------------------
        # 可视化初始化
        # ---------------------------------------------------------------------
        plt.ion()
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Arial", "SimHei"]
        plt.rcParams["axes.unicode_minus"] = False

        plot_dof = min(DYN_DOF, n)
        fig, axes = plt.subplots(2, 3, figsize=(15, 9), sharex=True)
        fig.canvas.manager.set_window_title(
            "reBotArm Gravity Compensation + Impedance Control"
        )
        fig.patch.set_facecolor("#F8F9FA")
        axes = axes.flatten()

        fig.canvas.mpl_connect("close_event", _on_window_close)
        fig.canvas.mpl_connect("key_press_event", make_key_handler(controller))

        colors = ["#1F77B4", "#D62728", "#2CA02C", "#9467BD", "#FF7F0E", "#17BECF"]

        q_lines = []
        ref_lines = []

        for i in range(6):
            ax = axes[i]
            ax.set_facecolor("#FFFFFF")

            if i < plot_dof:
                q_line, = ax.plot(
                    [],
                    [],
                    color=colors[i],
                    lw=2.8,
                    alpha=0.95,
                    label=f"Joint {i + 1} q",
                )

                ref_line, = ax.plot(
                    [],
                    [],
                    color="#343A40",
                    lw=1.8,
                    ls="--",
                    alpha=0.85,
                    label=f"Joint {i + 1} q_ref",
                )
            else:
                q_line, = ax.plot([], [])
                ref_line, = ax.plot([], [])

            q_lines.append(q_line)
            ref_lines.append(ref_line)

            ax.set_title(
                f"Joint {i + 1} Trajectory",
                fontsize=12,
                fontweight="bold",
                color="#212529",
                pad=10,
            )
            ax.set_ylabel(
                "Position (rad)",
                fontsize=10,
                fontweight="bold",
                color="#495057",
            )
            ax.grid(
                True,
                which="both",
                linestyle="--",
                color="#E9ECEF",
                linewidth=0.8,
            )

            for spine in ax.spines.values():
                spine.set_linewidth(1.2)
                spine.set_color("#6C757D")

            ax.legend(
                loc="upper right",
                frameon=True,
                facecolor="#FFFFFF",
                edgecolor="#CED4DA",
                fontsize=9,
            )

        for j in [3, 4, 5]:
            axes[j].set_xlabel(
                "Time Elapsed (s)",
                fontsize=11,
                fontweight="bold",
                color="#212529",
                labelpad=10,
            )

        fig.suptitle(
            "Mode: FREE_DRAG | Drag the arm, keep still to auto-lock. "
            "Keys: h=hold, f=free, q=quit",
            fontsize=13,
            fontweight="bold",
            color="#212529",
        )

        plt.tight_layout(pad=3.0)

        # ---------------------------------------------------------------------
        # 启动控制循环
        # ---------------------------------------------------------------------
        arm.start_control_loop(controller, rate=arm._rate)

        print(f"[控制循环] 启动 @ {arm._rate} Hz")
        print("[提示] 当前为 FREE_DRAG 模式，请拖拽机械臂到目标位置。")
        print("[提示] 静止约 1 秒后，程序会自动进入阻抗保持模式。")
        print("-" * 70)

        # ---------------------------------------------------------------------
        # 主线程刷新图像
        # ---------------------------------------------------------------------
        while _running:
            if plt.fignum_exists(fig.number):
                with data_lock:
                    if len(data_time_buffer) > 1:
                        t_data = list(data_time_buffer)
                        q_data = np.array(list(data_qpos_buffer))
                        qref_data = np.array(list(data_qref_buffer))
                        mode_now = data_mode_buffer[-1] if len(data_mode_buffer) > 0 else controller.mode
                    else:
                        t_data = None
                        q_data = None
                        qref_data = None
                        mode_now = controller.mode

                if t_data is not None:
                    for i in range(plot_dof):
                        q_lines[i].set_data(t_data, q_data[:, i])
                        ref_lines[i].set_data(t_data, qref_data[:, i])

                        axes[i].set_xlim(t_data[0], t_data[-1] + 0.05)

                        y_all = q_data[:, i]
                        if np.any(np.isfinite(qref_data[:, i])):
                            y_all = np.concatenate(
                                [y_all, qref_data[np.isfinite(qref_data[:, i]), i]]
                            )

                        y_min = np.nanmin(y_all)
                        y_max = np.nanmax(y_all)
                        y_range = y_max - y_min

                        if y_range < 0.01:
                            axes[i].set_ylim(y_min - 0.2, y_min + 0.2)
                        else:
                            axes[i].set_ylim(
                                y_min - y_range * 0.15,
                                y_max + y_range * 0.15,
                            )

                    fig.suptitle(
                        f"Mode: {mode_now} | "
                        "h=hold current, f=free drag, q=quit",
                        fontsize=13,
                        fontweight="bold",
                        color="#212529",
                    )

                    fig.canvas.draw()
                    fig.canvas.flush_events()

            time.sleep(0.03)

    except Exception as e:
        print(f"\n[主程序异常] {e}")

    finally:
        _running = False
        if arm is not None:
            safe_shutdown(arm)


if __name__ == "__main__":
    main()
