#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reBotArm 重力补偿 + 手动拖拽锁定 + 关节空间阻抗保持控制

本代码严格保留原始重力补偿代码中的核心参数：
1. TORQUE_LIMITS
2. KD_CONFIG
3. GRAVITY_SCALES

控制逻辑：
1. FREE_DRAG 模式：
   - 与原始重力补偿代码一致
   - kp = 0
   - kd = KD_CONFIG
   - tau = tau_g_safe
   - 可以手动拖拽机械臂

2. IMPEDANCE_HOLD 模式：
   - 保持原有重力补偿 tau_g_safe
   - 目标位置 pos = q_ref
   - 增加较小的关节空间 Kp，实现柔顺阻抗保持
   - kd 仍然使用原始 KD_CONFIG
   - 末端受力时可以偏移，松手后回到 q_ref 附近

键盘：
h：锁定当前位置，进入阻抗保持
f：回到自由拖拽 / 纯重力补偿
q：安全退出
[：降低阻抗刚度
]：增大阻抗刚度
s：打印当前状态
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
    get_default_gravity,
)


# --------------------------------------------------------------------------- #
# 全局控制标志与原始重力补偿参数
# --------------------------------------------------------------------------- #

_running = True

# =========================================================================== #
# 下面三组参数严格沿用你原始重力补偿代码，不修改
# =========================================================================== #

# [安全保险] 限制各关节最大前馈力矩
TORQUE_LIMITS = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

# 各轴特异化阻尼系数 Kd
KD_CONFIG = np.array([1.0, 2.5, 1.5, 1.5, 0.8, 0.6])

# 各轴重力前馈缩放系数
# GRAVITY_SCALES = np.array([1.50, 1.0, 0.85, 0.85, 1.0, 1.0])
GRAVITY_SCALES = np.array([1.50, 1.0, 0.95, 0.85, 1.0, 1.0])


# --------------------------------------------------------------------------- #
# 新增：阻抗保持参数
# --------------------------------------------------------------------------- #

# 阻抗保持刚度，初始设置得比较柔顺，避免 J2/J3 过硬。
# 注意：这里不是重力补偿参数，不改变原始 tau_g。
# 如果保持不住，可以按 ] 在线增加整体阻抗刚度。
KP_HOLD_BASE = np.array([3.5, 2.5, 3.5, 1.5, 1.0, 1.0])

# 阻抗刚度整体缩放系数，可通过 [ / ] 在线调节
KP_HOLD_SCALE = 3.0

# 是否开启自动锁定
# 真机调试建议 False，只用 h 手动锁定。
ENABLE_AUTO_LOCK = False

# 如果后续要打开自动锁定，可用下面参数
AUTO_LOCK_VEL_THRESH = 0.018
AUTO_LOCK_STILL_TIME = 1.0
AUTO_LOCK_MIN_START_TIME = 1.5

# 速度估计滤波，仅用于自动锁定判断和打印，不参与重力补偿
QD_FILTER_ALPHA = 0.25


# --------------------------------------------------------------------------- #
# 可视化全局缓存定义
# --------------------------------------------------------------------------- #

WINDOW_SIZE = 500
data_lock = Lock()

data_time_buffer = deque(maxlen=WINDOW_SIZE)
data_qpos_buffer = deque(maxlen=WINDOW_SIZE)
data_qref_buffer = deque(maxlen=WINDOW_SIZE)
data_mode_buffer = deque(maxlen=WINDOW_SIZE)
data_tau_buffer = deque(maxlen=WINDOW_SIZE)

start_time = time.time()


# --------------------------------------------------------------------------- #
# 工具函数
# --------------------------------------------------------------------------- #

def fit_to_n(arr, n, fill=0.0):
    arr = np.asarray(arr, dtype=float).reshape(-1)
    out = np.full(n, fill, dtype=float)
    m = min(len(arr), n)
    out[:m] = arr[:m]
    return out


def print_status(controller=None):
    print("\n" + "=" * 70)
    print("[当前控制状态]")
    if controller is not None:
        print(f"mode          = {controller.mode}")
    print(f"TORQUE_LIMITS = {TORQUE_LIMITS}")
    print(f"KD_CONFIG     = {KD_CONFIG}")
    print(f"GRAVITY_SCALES= {GRAVITY_SCALES}")
    print(f"KP_HOLD_BASE  = {KP_HOLD_BASE}")
    print(f"KP_HOLD_SCALE = {KP_HOLD_SCALE:.3f}")
    print(f"KP_HOLD_NOW   = {KP_HOLD_BASE * KP_HOLD_SCALE}")
    print("=" * 70 + "\n")


# --------------------------------------------------------------------------- #
# 信号与窗口关闭
# --------------------------------------------------------------------------- #

def _sigint_handler(signum, frame):
    global _running
    print("\n[gravity_impedance] 收到 Ctrl+C，触发停机保护...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


def _on_window_close(event):
    global _running
    print("\n[gravity_impedance] 检测到图形窗口被关闭，触发安全停机...")
    _running = False


# --------------------------------------------------------------------------- #
# 控制器
# --------------------------------------------------------------------------- #

class GravityImpedanceController:
    def __init__(self):
        self.mode = "FREE_DRAG"

        self.q_ref = None

        self.q_prev = None
        self.t_prev = None
        self.qd_filt = None

        self.still_since = None
        self.counter = 0

        self.force_lock_request = False
        self.force_free_request = False

    def request_lock_current(self):
        self.force_lock_request = True

    def request_free_drag(self):
        self.force_free_request = True

    def _estimate_velocity(self, q):
        now = time.time()

        if self.q_prev is None or self.t_prev is None:
            qd = np.zeros_like(q)
        else:
            dt = max(now - self.t_prev, 1e-4)
            qd = (q - self.q_prev) / dt

        if self.qd_filt is None:
            self.qd_filt = qd
        else:
            self.qd_filt = QD_FILTER_ALPHA * qd + (1.0 - QD_FILTER_ALPHA) * self.qd_filt

        self.q_prev = q.copy()
        self.t_prev = now

        return self.qd_filt.copy()

    def _lock_current_position(self, q):
        self.q_ref = q.copy()
        self.mode = "IMPEDANCE_HOLD"
        self.still_since = None

        print("\n[模式切换] 已锁定当前位置 q_ref，进入 IMPEDANCE_HOLD 阻抗保持模式。")
        print("[说明] 重力补偿参数保持原始设置，只额外加入小刚度关节阻抗。")
        print(f"[说明] 当前 KP_HOLD = {KP_HOLD_BASE * KP_HOLD_SCALE}")

    def _switch_to_free_drag(self, q):
        self.q_ref = q.copy()
        self.mode = "FREE_DRAG"
        self.still_since = None

        print("\n[模式切换] 已回到 FREE_DRAG 自由拖拽模式。")
        print("[说明] 当前控制与原始重力补偿代码一致：kp=0, kd=KD_CONFIG, tau=tau_g_safe。")

    def _maybe_auto_lock(self, q, qd_est):
        if not ENABLE_AUTO_LOCK:
            return

        if self.mode != "FREE_DRAG":
            return

        now = time.time()
        elapsed = now - start_time
        max_abs_qd = np.max(np.abs(qd_est[:6]))

        if elapsed > AUTO_LOCK_MIN_START_TIME and max_abs_qd < AUTO_LOCK_VEL_THRESH:
            if self.still_since is None:
                self.still_since = now
            elif now - self.still_since >= AUTO_LOCK_STILL_TIME:
                self._lock_current_position(q)
        else:
            self.still_since = None

    def __call__(self, arm: RobotArm, dt: float) -> None:
        global _running

        if not _running:
            return

        try:
            # -----------------------------------------------------------------
            # 1. 读取关节位置
            # -----------------------------------------------------------------
            q = np.asarray(arm.get_positions(), dtype=float).reshape(-1)
            n = arm.num_joints

            if len(q) < n:
                raise RuntimeError(f"读取到的关节数 {len(q)} 小于 arm.num_joints={n}")

            q = q[:n]

            if not np.all(np.isfinite(q)):
                raise RuntimeError("关节角数据包含 NaN 或 Inf")

            # -----------------------------------------------------------------
            # 2. 估计速度，仅用于自动锁定判断
            # -----------------------------------------------------------------
            qd_est = self._estimate_velocity(q)

            # -----------------------------------------------------------------
            # 3. 原始重力补偿计算：严格保持你的原逻辑
            # -----------------------------------------------------------------
            tau_g_raw = compute_generalized_gravity(q=q[:6])
            tau_g_raw = np.asarray(tau_g_raw, dtype=float).reshape(-1)

            tau_g = tau_g_raw * GRAVITY_SCALES[:n]
            tau_g_safe = np.clip(
                tau_g,
                -TORQUE_LIMITS[:n],
                TORQUE_LIMITS[:n],
            )

            # -----------------------------------------------------------------
            # 4. 初始化 q_ref
            # -----------------------------------------------------------------
            if self.q_ref is None:
                self.q_ref = q.copy()

            # -----------------------------------------------------------------
            # 5. 处理键盘模式请求
            # -----------------------------------------------------------------
            if self.force_free_request:
                self.force_free_request = False
                self._switch_to_free_drag(q)

            if self.force_lock_request:
                self.force_lock_request = False
                self._lock_current_position(q)

            self._maybe_auto_lock(q, qd_est)

            # -----------------------------------------------------------------
            # 6. 根据模式生成 MIT 命令
            # -----------------------------------------------------------------
            if self.mode == "FREE_DRAG":
                # 与原始重力补偿代码一致
                pos_cmd = q.copy()
                vel_cmd = np.zeros(n)
                kp_cmd = np.zeros(n)
                kd_cmd = KD_CONFIG[:n].copy()
                tau_cmd = tau_g_safe.copy()

            elif self.mode == "IMPEDANCE_HOLD":
                # 阻抗保持：
                # 重力补偿 tau 仍然使用原来的 tau_g_safe
                # kd 仍然使用原来的 KD_CONFIG
                # 只额外加入较小 kp，让机械臂回到 q_ref
                pos_cmd = self.q_ref.copy()
                vel_cmd = np.zeros(n)
                kp_cmd = fit_to_n(KP_HOLD_BASE * KP_HOLD_SCALE, n, fill=0.0)
                kd_cmd = KD_CONFIG[:n].copy()
                tau_cmd = tau_g_safe.copy()

            else:
                raise RuntimeError(f"未知控制模式: {self.mode}")

            # -----------------------------------------------------------------
            # 7. 下发 MIT 控制
            # -----------------------------------------------------------------
            arm.mit(
                pos=pos_cmd,
                vel=vel_cmd,
                kp=kp_cmd,
                kd=kd_cmd,
                tau=tau_cmd,
                request_feedback=True,
            )

            # -----------------------------------------------------------------
            # 8. 数据缓存
            # -----------------------------------------------------------------
            current_timestamp = time.time() - start_time

            with data_lock:
                data_time_buffer.append(current_timestamp)
                data_qpos_buffer.append(q[:6].copy())
                data_tau_buffer.append(tau_g_safe[:6].copy())

                if self.mode == "IMPEDANCE_HOLD":
                    data_qref_buffer.append(self.q_ref[:6].copy())
                else:
                    data_qref_buffer.append(np.full(6, np.nan))

                data_mode_buffer.append(self.mode)

            # -----------------------------------------------------------------
            # 9. 打印状态
            # -----------------------------------------------------------------
            self.counter += 1

            if self.counter % 50 == 0:
                print(
                    f"[{self.counter:5d}] mode={self.mode:<15s} "
                    f"tau_g(计算/限幅) = "
                    + " | ".join(
                        f"{r:+.2f}/{s:+.2f}"
                        for r, s in zip(tau_g_raw, tau_g_safe)
                    )
                    + " N·m"
                )

                if self.mode == "IMPEDANCE_HOLD":
                    q_err = self.q_ref[:6] - q[:6]
                    print(
                        " " * 8
                        + "q_ref - q = "
                        + " | ".join(f"{e:+.3f}" for e in q_err)
                        + " rad"
                    )

        except Exception as e:
            print(f"\n[控制异常] {e}")
            _running = False


# --------------------------------------------------------------------------- #
# 键盘事件
# --------------------------------------------------------------------------- #

def make_key_handler(controller: GravityImpedanceController):
    def _on_key_press(event):
        global _running
        global KP_HOLD_SCALE

        if event.key == "h":
            print("\n[键盘] h：锁定当前位置，进入阻抗保持。")
            controller.request_lock_current()

        elif event.key == "f":
            print("\n[键盘] f：回到自由拖拽 / 纯重力补偿。")
            controller.request_free_drag()

        elif event.key == "q":
            print("\n[键盘] q：安全退出。")
            _running = False

        elif event.key == "s":
            print_status(controller)

        elif event.key == "[":
            KP_HOLD_SCALE = max(0.1, KP_HOLD_SCALE * 0.8)
            print(f"\n[调参] 降低阻抗刚度，KP_HOLD_SCALE = {KP_HOLD_SCALE:.3f}")
            print(f"[调参] 当前 KP_HOLD = {KP_HOLD_BASE * KP_HOLD_SCALE}")

        elif event.key == "]":
            KP_HOLD_SCALE = min(5.0, KP_HOLD_SCALE * 1.25)
            print(f"\n[调参] 增大阻抗刚度，KP_HOLD_SCALE = {KP_HOLD_SCALE:.3f}")
            print(f"[调参] 当前 KP_HOLD = {KP_HOLD_BASE * KP_HOLD_SCALE}")

    return _on_key_press


# --------------------------------------------------------------------------- #
# 安全关闭
# --------------------------------------------------------------------------- #

def safe_shutdown(arm: RobotArm):
    print("\n" + "=" * 50)
    print("[退出流程] 正在紧急向全轴电机下发停机安全指令...")

    try:
        plt.ioff()
        plt.close("all")
    except Exception:
        pass

    try:
        if hasattr(arm, "stop_control_loop"):
            print("[退出流程] 正在停止控制循环...")
            arm.stop_control_loop()
            time.sleep(0.1)
    except Exception as ex:
        print(f"[⚠️警告] 停止控制循环失败: {ex}")

    try:
        print("[退出流程] 正在下发【失能 Disable】帧，全轴释放力矩...")
        for _ in range(3):
            arm.disable()
            time.sleep(0.08)
    except Exception as ex:
        print(f"[⚠️警告] 底层电机失能失败: {ex}")

    try:
        arm.disconnect()
        print("[退出流程] 机械臂已安全失能断开。")
    except Exception as ex:
        print(f"[⚠️警告] 断开连接失败: {ex}")

    print("=" * 50 + "\n")


# --------------------------------------------------------------------------- #
# 主程序
# --------------------------------------------------------------------------- #

def main() -> None:
    global _running

    print("=" * 70)
    print("  reBotArm 重力补偿 + 手动拖拽锁定 + 关节阻抗保持")
    print("=" * 70)
    print("[核心说明]")
    print("  重力补偿参数严格沿用原始代码：")
    print(f"  TORQUE_LIMITS  = {TORQUE_LIMITS}")
    print(f"  KD_CONFIG      = {KD_CONFIG}")
    print(f"  GRAVITY_SCALES = {GRAVITY_SCALES}")
    print("-" * 70)
    print("[操作说明]")
    print("  h ：锁定当前位置，进入阻抗保持")
    print("  f ：回到自由拖拽 / 纯重力补偿")
    print("  [ ：降低阻抗刚度")
    print("  ] ：增大阻抗刚度")
    print("  s ：打印状态")
    print("  q ：安全退出")
    print("=" * 70)

    arm = None

    try:
        # 加载动力学模型
        _ = load_dynamics_model()

        # 连接机械臂
        arm = RobotArm()
        arm.connect()
        arm.enable()

        n = arm.num_joints
        print(f"[机械臂] 已连接，关节数: {n}")

        # 初始 MIT 模式与原始重力补偿代码一致
        arm.mode_mit(
            kp=np.zeros(n),
            kd=KD_CONFIG[:n],
        )

        controller = GravityImpedanceController()

        # ------------------------------------------------------------------- #
        # 可视化初始化
        # ------------------------------------------------------------------- #
        plt.ion()
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Arial", "SimHei"]
        plt.rcParams["axes.unicode_minus"] = False

        fig, axes = plt.subplots(2, 3, figsize=(15, 9), sharex=True)
        fig.canvas.manager.set_window_title(
            "reBotArm 重力补偿 + 关节阻抗保持"
        )
        fig.patch.set_facecolor("#F8F9FA")
        axes = axes.flatten()

        fig.canvas.mpl_connect("close_event", _on_window_close)
        fig.canvas.mpl_connect("key_press_event", make_key_handler(controller))

        academic_colors = [
            "#1F77B4",
            "#D62728",
            "#2CA02C",
            "#9467BD",
            "#FF7F0E",
            "#17BECF",
        ]

        q_lines = []
        ref_lines = []

        for i in range(6):
            ax = axes[i]
            ax.set_facecolor("#FFFFFF")

            q_line, = ax.plot(
                [],
                [],
                label=f"Joint {i + 1} q",
                color=academic_colors[i],
                lw=2.8,
                alpha=0.95,
                antialiased=True,
            )

            ref_line, = ax.plot(
                [],
                [],
                label=f"Joint {i + 1} q_ref",
                color="#343A40",
                lw=1.8,
                ls="--",
                alpha=0.85,
            )

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
            "Mode: FREE_DRAG | h=hold, f=free, [/] adjust stiffness, q=quit",
            fontsize=13,
            fontweight="bold",
            color="#212529",
        )

        plt.tight_layout(pad=3.0)

        # ------------------------------------------------------------------- #
        # 启动控制循环
        # ------------------------------------------------------------------- #
        arm.start_control_loop(controller, rate=arm._rate)

        print(f"[控制循环] 启动 @ {arm._rate} Hz")
        print("[提示] 当前是 FREE_DRAG，自由拖拽模式。")
        print("[提示] 拖到目标位置后按 h 锁定。")
        print("-" * 70)

        # ------------------------------------------------------------------- #
        # 主线程刷新图像
        # ------------------------------------------------------------------- #
        while _running:
            if plt.fignum_exists(fig.number):
                with data_lock:
                    if len(data_time_buffer) > 1:
                        t_data = list(data_time_buffer)
                        q_data = np.array(list(data_qpos_buffer))
                        qref_data = np.array(list(data_qref_buffer))
                        mode_now = (
                            data_mode_buffer[-1]
                            if len(data_mode_buffer) > 0
                            else controller.mode
                        )
                    else:
                        t_data = None
                        q_data = None
                        qref_data = None
                        mode_now = controller.mode

                if t_data is not None:
                    for i in range(6):
                        q_lines[i].set_data(t_data, q_data[:, i])
                        ref_lines[i].set_data(t_data, qref_data[:, i])

                        axes[i].set_xlim(t_data[0], t_data[-1] + 0.05)

                        y_all = q_data[:, i]

                        if np.any(np.isfinite(qref_data[:, i])):
                            y_ref_valid = qref_data[np.isfinite(qref_data[:, i]), i]
                            y_all = np.concatenate([y_all, y_ref_valid])

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
                        "h=hold, f=free, [/] stiffness, s=status, q=quit",
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
