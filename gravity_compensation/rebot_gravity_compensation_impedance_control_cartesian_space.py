#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reBotArm 重力补偿 + 手动拖拽锁定 + 笛卡尔空间阻抗保持控制

本脚本以已经验证可行的关节空间阻抗脚本为基础：
1. FREE_DRAG 模式保持原始重力补偿逻辑。
2. CARTESIAN_HOLD 模式锁定当前末端位姿，默认只对末端位置 XYZ 施加虚拟弹簧阻尼。
3. 笛卡尔力通过 J.T @ wrench 映射为关节力矩，再叠加重力补偿力矩。

键盘：
h：锁定当前末端位置，进入笛卡尔阻抗保持
f：回到自由拖拽 / 纯重力补偿
q：安全退出
[：降低笛卡尔刚度
]：增大笛卡尔刚度
s：打印当前状态
o：切换姿态保持开关，默认关闭，首次真机建议保持关闭
"""

import signal
import sys
import time
from collections import deque
from pathlib import Path
from threading import Lock

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RobotArm
from reBotArm_control_py.dynamics import (
    compute_generalized_gravity,
    load_dynamics_model,
)
from reBotArm_control_py.kinematics import get_end_effector_frame_id


# --------------------------------------------------------------------------- #
# 全局控制标志与原始重力补偿参数
# --------------------------------------------------------------------------- #

_running = True

# 与关节空间阻抗验证脚本保持一致。
TORQUE_LIMITS = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
KD_CONFIG = np.array([1.0, 2.5, 1.5, 1.5, 0.8, 0.6])
GRAVITY_SCALES = np.array([1.50, 1.0, 0.95, 0.85, 1.0, 1.0])


# --------------------------------------------------------------------------- #
# 笛卡尔阻抗参数
# --------------------------------------------------------------------------- #

# 单位：
# K_TRANS: N / m
# D_TRANS: N / (m/s)
# K_ROT:   N*m / rad
# D_ROT:   N*m / (rad/s)
#
# 首次真机建议只启用位置阻抗，姿态保持默认关闭。
# K_TRANS_BASE = np.array([35.0, 35.0, 30.0])
# D_TRANS_BASE = np.array([8.0, 8.0, 7.0])
K_TRANS_BASE = np.array([150.0, 150.0, 120.0])
D_TRANS_BASE = np.array([0.5, 0.5, 0.5])

K_ROT_BASE = np.array([0.6, 0.6, 0.4])
D_ROT_BASE = np.array([0.08, 0.08, 0.06])
ENABLE_ORIENTATION_HOLD = False

# 在线整体缩放系数，通过 [ / ] 调整。
CARTESIAN_STIFFNESS_SCALE = 1.0

# 笛卡尔力/力矩与叠加后关节力矩限幅。
MAX_CART_FORCE_N = 12.0
MAX_CART_TORQUE_NM = 1.2
# TASK_TORQUE_LIMITS = np.array([4.0, 4.0, 4.0, 2.0, 2.0, 2.0])
TASK_TORQUE_LIMITS = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

# 速度估计滤波，用于末端阻尼与打印。
QD_FILTER_ALPHA = 0.25


# --------------------------------------------------------------------------- #
# 可视化全局缓存定义
# --------------------------------------------------------------------------- #

WINDOW_SIZE = 500
data_lock = Lock()

data_time_buffer = deque(maxlen=WINDOW_SIZE)
data_xpos_buffer = deque(maxlen=WINDOW_SIZE)
data_xref_buffer = deque(maxlen=WINDOW_SIZE)
data_qpos_buffer = deque(maxlen=WINDOW_SIZE)
data_tau_buffer = deque(maxlen=WINDOW_SIZE)
data_mode_buffer = deque(maxlen=WINDOW_SIZE)

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


def limit_vector_norm(vec, max_norm):
    vec = np.asarray(vec, dtype=float).reshape(-1)
    norm = float(np.linalg.norm(vec))
    if norm > max_norm > 0.0:
        return vec * (max_norm / norm)
    return vec


def rotation_error_world(rot_ref, rot_cur):
    """返回世界坐标表达的姿态误差向量，供 LOCAL_WORLD_ALIGNED 雅可比使用。"""
    err_local = pin.log3(rot_cur.T @ rot_ref)
    return rot_cur @ err_local


def print_status(controller=None):
    print("\n" + "=" * 76)
    print("[当前控制状态]")
    if controller is not None:
        print(f"mode                    = {controller.mode}")
        if controller.x_ref is not None:
            print(f"x_ref                   = {controller.x_ref}")
        print(f"orientation_hold         = {controller.orientation_hold}")
    print(f"TORQUE_LIMITS            = {TORQUE_LIMITS}")
    print(f"KD_CONFIG                = {KD_CONFIG}")
    print(f"GRAVITY_SCALES           = {GRAVITY_SCALES}")
    print(f"K_TRANS_NOW              = {K_TRANS_BASE * CARTESIAN_STIFFNESS_SCALE}")
    print(f"D_TRANS_NOW              = {D_TRANS_BASE * np.sqrt(CARTESIAN_STIFFNESS_SCALE)}")
    print(f"K_ROT_NOW                = {K_ROT_BASE * CARTESIAN_STIFFNESS_SCALE}")
    print(f"D_ROT_NOW                = {D_ROT_BASE * np.sqrt(CARTESIAN_STIFFNESS_SCALE)}")
    print(f"MAX_CART_FORCE_N         = {MAX_CART_FORCE_N}")
    print(f"MAX_CART_TORQUE_NM       = {MAX_CART_TORQUE_NM}")
    print(f"TASK_TORQUE_LIMITS       = {TASK_TORQUE_LIMITS}")
    print("=" * 76 + "\n")


# --------------------------------------------------------------------------- #
# 信号与窗口关闭
# --------------------------------------------------------------------------- #

def _sigint_handler(signum, frame):
    global _running
    print("\n[cartesian_impedance] 收到 Ctrl+C，触发停机保护...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


def _on_window_close(event):
    global _running
    print("\n[cartesian_impedance] 检测到图形窗口被关闭，触发安全停机...")
    _running = False


# --------------------------------------------------------------------------- #
# 控制器
# --------------------------------------------------------------------------- #

class CartesianImpedanceController:
    def __init__(self, model):
        self.mode = "FREE_DRAG"

        self.model = model
        self.data = model.createData()
        self.end_frame_id = get_end_effector_frame_id(model)

        self.q_ref = None
        self.x_ref = None
        self.rot_ref = None

        self.q_prev = None
        self.t_prev = None
        self.qd_filt = None

        self.counter = 0
        self.orientation_hold = ENABLE_ORIENTATION_HOLD

        self.force_lock_request = False
        self.force_free_request = False

    def request_lock_current(self):
        self.force_lock_request = True

    def request_free_drag(self):
        self.force_free_request = True

    def toggle_orientation_hold(self):
        self.orientation_hold = not self.orientation_hold
        state = "开启" if self.orientation_hold else "关闭"
        print(f"\n[调参] 姿态保持已{state}。")

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

    def _compute_kinematics(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        oMf = self.data.oMf[self.end_frame_id]
        jac = pin.getFrameJacobian(
            self.model,
            self.data,
            self.end_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return oMf.translation.copy(), oMf.rotation.copy(), jac.copy()

    def _lock_current_pose(self, q, x, rot):
        self.q_ref = q.copy()
        self.x_ref = x.copy()
        self.rot_ref = rot.copy()
        self.mode = "CARTESIAN_HOLD"

        print("\n[模式切换] 已锁定当前末端位姿，进入 CARTESIAN_HOLD 笛卡尔阻抗保持模式。")
        print("[说明] 默认保持末端 XYZ 位置，姿态保持可按 o 开关。")
        print(f"[说明] x_ref = {self.x_ref}")
        print(f"[说明] K_TRANS = {K_TRANS_BASE * CARTESIAN_STIFFNESS_SCALE}")

    def _switch_to_free_drag(self, q, x, rot):
        self.q_ref = q.copy()
        self.x_ref = x.copy()
        self.rot_ref = rot.copy()
        self.mode = "FREE_DRAG"

        print("\n[模式切换] 已回到 FREE_DRAG 自由拖拽模式。")
        print("[说明] 当前控制与原始重力补偿一致：kp=0, kd=KD_CONFIG, tau=tau_g_safe。")

    def _compute_task_torque(self, qd_est, x, rot, jac, n):
        if self.x_ref is None or self.rot_ref is None:
            return np.zeros(n), np.zeros(6), np.zeros(6)

        spatial_vel = jac @ qd_est[:self.model.nv]
        v_linear = spatial_vel[:3]
        v_angular = spatial_vel[3:]

        pos_err = self.x_ref - x
        k_trans = K_TRANS_BASE * CARTESIAN_STIFFNESS_SCALE
        d_trans = D_TRANS_BASE * np.sqrt(CARTESIAN_STIFFNESS_SCALE)
        force = k_trans * pos_err - d_trans * v_linear
        force = limit_vector_norm(force, MAX_CART_FORCE_N)

        torque_cart = np.zeros(3)
        rot_err = np.zeros(3)
        if self.orientation_hold:
            rot_err = rotation_error_world(self.rot_ref, rot)
            k_rot = K_ROT_BASE * CARTESIAN_STIFFNESS_SCALE
            d_rot = D_ROT_BASE * np.sqrt(CARTESIAN_STIFFNESS_SCALE)
            torque_cart = k_rot * rot_err - d_rot * v_angular
            torque_cart = limit_vector_norm(torque_cart, MAX_CART_TORQUE_NM)

        wrench = np.concatenate([force, torque_cart])
        tau_task = jac.T @ wrench
        tau_task = fit_to_n(tau_task, n, fill=0.0)
        tau_task = np.clip(tau_task, -TASK_TORQUE_LIMITS[:n], TASK_TORQUE_LIMITS[:n])

        err6 = np.concatenate([pos_err, rot_err])
        return tau_task, wrench, err6

    def __call__(self, arm: RobotArm, dt: float) -> None:
        global _running

        if not _running:
            return

        try:
            # 1. 读取关节位置
            q = np.asarray(arm.get_positions(), dtype=float).reshape(-1)
            n = arm.num_joints

            if len(q) < n:
                raise RuntimeError(f"读取到的关节数 {len(q)} 小于 arm.num_joints={n}")

            q = q[:n]

            if not np.all(np.isfinite(q)):
                raise RuntimeError("关节角数据包含 NaN 或 Inf")

            if n < self.model.nq:
                raise RuntimeError(f"机械臂关节数 {n} 小于模型 nq={self.model.nq}")

            q_model = q[:self.model.nq]
            qd_est = self._estimate_velocity(q)
            qd_model = fit_to_n(qd_est, self.model.nv, fill=0.0)

            # 2. 正运动学与末端雅可比
            x, rot, jac = self._compute_kinematics(q_model)

            # 3. 原始重力补偿计算
            tau_g_raw = compute_generalized_gravity(model=self.model, q=q_model)
            tau_g_raw = np.asarray(tau_g_raw, dtype=float).reshape(-1)

            tau_g = tau_g_raw * GRAVITY_SCALES[:self.model.nv]
            tau_g_safe = np.clip(
                fit_to_n(tau_g, n, fill=0.0),
                -TORQUE_LIMITS[:n],
                TORQUE_LIMITS[:n],
            )

            # 4. 初始化参考
            if self.q_ref is None:
                self.q_ref = q.copy()
                self.x_ref = x.copy()
                self.rot_ref = rot.copy()

            # 5. 处理键盘请求
            if self.force_free_request:
                self.force_free_request = False
                self._switch_to_free_drag(q, x, rot)

            if self.force_lock_request:
                self.force_lock_request = False
                self._lock_current_pose(q, x, rot)

            # 6. 根据模式生成 MIT 命令
            tau_task = np.zeros(n)
            wrench = np.zeros(6)
            err6 = np.full(6, np.nan)

            if self.mode == "FREE_DRAG":
                pos_cmd = q.copy()
                vel_cmd = np.zeros(n)
                kp_cmd = np.zeros(n)
                kd_cmd = KD_CONFIG[:n].copy()
                tau_cmd = tau_g_safe.copy()

            elif self.mode == "CARTESIAN_HOLD":
                # 使用纯力矩形式叠加笛卡尔阻抗，关节 kp 保持为 0。
                tau_task, wrench, err6 = self._compute_task_torque(qd_model, x, rot, jac, n)
                tau_cmd = np.clip(
                    tau_g_safe + tau_task,
                    -TORQUE_LIMITS[:n],
                    TORQUE_LIMITS[:n],
                )

                pos_cmd = q.copy()
                vel_cmd = np.zeros(n)
                kp_cmd = np.zeros(n)
                kd_cmd = KD_CONFIG[:n].copy()

            else:
                raise RuntimeError(f"未知控制模式: {self.mode}")

            # 7. 下发 MIT 控制
            arm.mit(
                pos=pos_cmd,
                vel=vel_cmd,
                kp=kp_cmd,
                kd=kd_cmd,
                tau=tau_cmd,
                request_feedback=True,
            )

            # 8. 数据缓存
            current_timestamp = time.time() - start_time

            with data_lock:
                data_time_buffer.append(current_timestamp)
                data_xpos_buffer.append(x.copy())
                data_qpos_buffer.append(q[:6].copy())
                data_tau_buffer.append(tau_cmd[:6].copy())

                if self.mode == "CARTESIAN_HOLD":
                    data_xref_buffer.append(self.x_ref.copy())
                else:
                    data_xref_buffer.append(np.full(3, np.nan))

                data_mode_buffer.append(self.mode)

            # 9. 打印状态
            self.counter += 1

            if self.counter % 50 == 0:
                print(
                    f"[{self.counter:5d}] mode={self.mode:<15s} "
                    f"x = {x[0]:+.3f}, {x[1]:+.3f}, {x[2]:+.3f} m"
                )
                if self.mode == "CARTESIAN_HOLD":
                    print(
                        " " * 8
                        + "x_ref - x = "
                        + " | ".join(f"{e:+.4f}" for e in err6[:3])
                        + " m"
                    )
                    print(
                        " " * 8
                        + "Fxyz = "
                        + " | ".join(f"{f:+.2f}" for f in wrench[:3])
                        + " N, tau_task = "
                        + " | ".join(f"{t:+.2f}" for t in tau_task[:6])
                        + " N*m"
                    )

        except Exception as e:
            print(f"\n[控制异常] {e}")
            _running = False


# --------------------------------------------------------------------------- #
# 键盘事件
# --------------------------------------------------------------------------- #

def make_key_handler(controller: CartesianImpedanceController):
    def _on_key_press(event):
        global _running
        global CARTESIAN_STIFFNESS_SCALE

        if event.key == "h":
            print("\n[键盘] h：锁定当前末端位置，进入笛卡尔阻抗保持。")
            controller.request_lock_current()

        elif event.key == "f":
            print("\n[键盘] f：回到自由拖拽 / 纯重力补偿。")
            controller.request_free_drag()

        elif event.key == "q":
            print("\n[键盘] q：安全退出。")
            _running = False

        elif event.key == "s":
            print_status(controller)

        elif event.key == "o":
            controller.toggle_orientation_hold()

        elif event.key == "[":
            CARTESIAN_STIFFNESS_SCALE = max(0.1, CARTESIAN_STIFFNESS_SCALE * 0.8)
            print(f"\n[调参] 降低笛卡尔刚度，scale = {CARTESIAN_STIFFNESS_SCALE:.3f}")
            print(f"[调参] 当前 K_TRANS = {K_TRANS_BASE * CARTESIAN_STIFFNESS_SCALE}")

        elif event.key == "]":
            CARTESIAN_STIFFNESS_SCALE = min(5.0, CARTESIAN_STIFFNESS_SCALE * 1.25)
            print(f"\n[调参] 增大笛卡尔刚度，scale = {CARTESIAN_STIFFNESS_SCALE:.3f}")
            print(f"[调参] 当前 K_TRANS = {K_TRANS_BASE * CARTESIAN_STIFFNESS_SCALE}")

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
        print(f"[警告] 停止控制循环失败: {ex}")

    try:
        print("[退出流程] 正在下发【失能 Disable】帧，全轴释放力矩...")
        for _ in range(3):
            arm.disable()
            time.sleep(0.08)
    except Exception as ex:
        print(f"[警告] 底层电机失能失败: {ex}")

    try:
        arm.disconnect()
        print("[退出流程] 机械臂已安全失能断开。")
    except Exception as ex:
        print(f"[警告] 断开连接失败: {ex}")

    print("=" * 50 + "\n")


# --------------------------------------------------------------------------- #
# 主程序
# --------------------------------------------------------------------------- #

def main() -> None:
    global _running

    print("=" * 76)
    print("  reBotArm 重力补偿 + 手动拖拽锁定 + 笛卡尔阻抗保持")
    print("=" * 76)
    print("[核心说明]")
    print("  FREE_DRAG 与关节阻抗验证脚本保持一致。")
    print("  CARTESIAN_HOLD 默认只保持末端 XYZ 位置，通过 J.T @ F 叠加任务力矩。")
    print(f"  TORQUE_LIMITS  = {TORQUE_LIMITS}")
    print(f"  KD_CONFIG      = {KD_CONFIG}")
    print(f"  GRAVITY_SCALES = {GRAVITY_SCALES}")
    print(f"  K_TRANS_BASE   = {K_TRANS_BASE}")
    print(f"  D_TRANS_BASE   = {D_TRANS_BASE}")
    print("-" * 76)
    print("[操作说明]")
    print("  h ：锁定当前末端位置，进入笛卡尔阻抗保持")
    print("  f ：回到自由拖拽 / 纯重力补偿")
    print("  [ ：降低笛卡尔刚度")
    print("  ] ：增大笛卡尔刚度")
    print("  o ：开启/关闭姿态保持，默认关闭")
    print("  s ：打印状态")
    print("  q ：安全退出")
    print("=" * 76)

    arm = None

    try:
        # 加载 Pinocchio 动力学/运动学共用模型。
        model = load_dynamics_model()

        # 连接机械臂。
        arm = RobotArm()
        arm.connect()
        arm.enable()

        n = arm.num_joints
        print(f"[机械臂] 已连接，关节数: {n}, model.nq={model.nq}, model.nv={model.nv}")

        # 初始 MIT 模式与原始重力补偿代码一致。
        arm.mode_mit(
            kp=np.zeros(n),
            kd=KD_CONFIG[:n],
        )

        controller = CartesianImpedanceController(model)

        # ------------------------------------------------------------------- #
        # 可视化初始化
        # ------------------------------------------------------------------- #
        plt.ion()
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Arial", "SimHei"]
        plt.rcParams["axes.unicode_minus"] = False

        fig, axes = plt.subplots(2, 3, figsize=(15, 9), sharex=True)
        fig.canvas.manager.set_window_title(
            "reBotArm 重力补偿 + 笛卡尔阻抗保持"
        )
        fig.patch.set_facecolor("#F8F9FA")
        axes = axes.flatten()

        fig.canvas.mpl_connect("close_event", _on_window_close)
        fig.canvas.mpl_connect("key_press_event", make_key_handler(controller))

        colors = ["#1F77B4", "#D62728", "#2CA02C"]
        x_lines = []
        x_ref_lines = []

        for i, name in enumerate(["X", "Y", "Z"]):
            ax = axes[i]
            ax.set_facecolor("#FFFFFF")
            line, = ax.plot([], [], label=f"EE {name}", color=colors[i], lw=2.8)
            ref_line, = ax.plot([], [], label=f"{name}_ref", color="#343A40", lw=1.8, ls="--")
            x_lines.append(line)
            x_ref_lines.append(ref_line)
            ax.set_title(f"End Effector {name}", fontsize=12, fontweight="bold")
            ax.set_ylabel("Position (m)", fontsize=10, fontweight="bold")
            ax.grid(True, which="both", linestyle="--", color="#E9ECEF", linewidth=0.8)
            ax.legend(loc="upper right", frameon=True, facecolor="#FFFFFF", edgecolor="#CED4DA", fontsize=9)
            for spine in ax.spines.values():
                spine.set_linewidth(1.2)
                spine.set_color("#6C757D")

        q_lines = []
        for idx, joint_id in enumerate([0, 1, 2]):
            ax = axes[idx + 3]
            ax.set_facecolor("#FFFFFF")
            line, = ax.plot([], [], label=f"Joint {joint_id + 1}", color=colors[idx], lw=2.5)
            q_lines.append(line)
            ax.set_title(f"Joint {joint_id + 1}", fontsize=12, fontweight="bold")
            ax.set_ylabel("Position (rad)", fontsize=10, fontweight="bold")
            ax.set_xlabel("Time Elapsed (s)", fontsize=11, fontweight="bold")
            ax.grid(True, which="both", linestyle="--", color="#E9ECEF", linewidth=0.8)
            ax.legend(loc="upper right", frameon=True, facecolor="#FFFFFF", edgecolor="#CED4DA", fontsize=9)
            for spine in ax.spines.values():
                spine.set_linewidth(1.2)
                spine.set_color("#6C757D")

        fig.suptitle(
            "Mode: FREE_DRAG | h=hold, f=free, [/] stiffness, o=orientation, q=quit",
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
        print("[提示] 拖到目标末端位置后按 h 锁定。")
        print("[提示] 首次测试建议保持姿态保持关闭，只观察 XYZ 保持效果。")
        print("-" * 76)

        # ------------------------------------------------------------------- #
        # 主线程刷新图像
        # ------------------------------------------------------------------- #
        while _running:
            if plt.fignum_exists(fig.number):
                with data_lock:
                    if len(data_time_buffer) > 1:
                        t_data = list(data_time_buffer)
                        x_data = np.array(list(data_xpos_buffer))
                        xref_data = np.array(list(data_xref_buffer))
                        q_data = np.array(list(data_qpos_buffer))
                        mode_now = (
                            data_mode_buffer[-1]
                            if len(data_mode_buffer) > 0
                            else controller.mode
                        )
                    else:
                        t_data = None
                        x_data = None
                        xref_data = None
                        q_data = None
                        mode_now = controller.mode

                if t_data is not None:
                    for i in range(3):
                        x_lines[i].set_data(t_data, x_data[:, i])
                        x_ref_lines[i].set_data(t_data, xref_data[:, i])
                        axes[i].set_xlim(t_data[0], t_data[-1] + 0.05)

                        y_all = x_data[:, i]
                        if np.any(np.isfinite(xref_data[:, i])):
                            y_ref_valid = xref_data[np.isfinite(xref_data[:, i]), i]
                            y_all = np.concatenate([y_all, y_ref_valid])

                        y_min = np.nanmin(y_all)
                        y_max = np.nanmax(y_all)
                        y_range = y_max - y_min
                        if y_range < 0.01:
                            axes[i].set_ylim(y_min - 0.05, y_min + 0.05)
                        else:
                            axes[i].set_ylim(y_min - y_range * 0.15, y_max + y_range * 0.15)

                    for i in range(3):
                        q_lines[i].set_data(t_data, q_data[:, i])
                        axes[i + 3].set_xlim(t_data[0], t_data[-1] + 0.05)
                        y_min = np.nanmin(q_data[:, i])
                        y_max = np.nanmax(q_data[:, i])
                        y_range = y_max - y_min
                        if y_range < 0.01:
                            axes[i + 3].set_ylim(y_min - 0.2, y_min + 0.2)
                        else:
                            axes[i + 3].set_ylim(y_min - y_range * 0.15, y_max + y_range * 0.15)

                    fig.suptitle(
                        f"Mode: {mode_now} | "
                        "h=hold, f=free, [/] stiffness, o=orientation, s=status, q=quit",
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
