#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reBotArm MuJoCo 仿真：末端笛卡尔阻抗控制 + 重力补偿

功能：
1. 鼠标拖动红色 mocap_target 方块；
2. 机械臂末端 eef_trace_site 通过笛卡尔阻抗跟随 mocap_target；
3. 使用 MuJoCo data.qfrc_bias 实现重力补偿；
4. 通过 data.ctrl 直接下发关节力矩；
5. 保留夹爪开合控制；
6. 支持键盘在线调节阻抗刚度和阻尼。

控制律：
    F_task = Kx * (x_des - x) - Dx * xdot
    tau_task = J_pos.T @ F_task
    tau_cmd = tau_task + tau_gravity - Kq_damp * qvel

其中：
    x_des  : mocap_target 位置
    x      : 末端 site 位置
    J_pos  : 末端位置雅可比矩阵
    tau_g  : MuJoCo qfrc_bias 对应的重力/科氏/离心补偿项

注意：
    这份脚本默认 joint1~joint6 的 actuator 是 torque/motor 类型。
    如果 XML 里 actuator 是 position 类型，那么 data.ctrl 不是力矩，而是目标位置，
    需要先把 XML actuator 改成 motor actuator。
"""

import sys
import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer


# =============================================================================
# 路径与仿真参数
# =============================================================================

ROOT_DIR = Path(__file__).resolve().parents[1]

# 你原代码使用的 XML 路径
XML_PATH = str(
    ROOT_DIR / "xml" / "rebot_gripper" / "sim_reBot_impedance_control.xml"
)

CTRL_FREQ = 200.0
DT = 1.0 / CTRL_FREQ

ARM_DOF = 6


# =============================================================================
# 末端阻抗控制参数
# =============================================================================

# 笛卡尔位置刚度，单位近似 N/m
# 数值越大，末端越努力跟随 mocap_target，感觉越“硬”
# 数值越小，末端越柔顺，但跟踪误差更大
KX_BASE = np.array([120.0, 120.0, 100.0])

# 笛卡尔阻尼，单位近似 N·s/m
# 数值越大，末端运动越稳，但越“粘”
# 数值越小，末端更轻快，但可能振荡
DX_BASE = np.array([18.0, 18.0, 16.0])

# 关节空间附加阻尼，抑制整机晃动
# 这个不要太大，否则又会拖不动
KQ_DAMP_BASE = np.array([0.25, 0.35, 0.35, 0.20, 0.10, 0.08])

# 每个关节最大输出力矩，单位 N·m
TORQUE_LIMITS = np.array([12.0, 12.0, 12.0, 6.0, 6.0, 6.0])

# 力矩变化率限制，单位 N·m/s
TAU_RATE_LIMITS = np.array([50.0, 50.0, 40.0, 25.0, 18.0, 18.0])

# 重力补偿缩放
# 如果仿真中机械臂自然下坠，增大对应项；
# 如果机械臂自己上抬，减小对应项。
GRAVITY_SCALE = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

# 启动渐入时间，避免刚启动瞬间力矩突变
RAMP_TIME = 1.0


# =============================================================================
# 夹爪参数
# =============================================================================

GRIPPER_OPEN = 0.05
GRIPPER_CLOSE = 0.018
GRIPPER_STIFFNESS = 50000.0

current_gripper_target = GRIPPER_OPEN


# =============================================================================
# 在线调参全局变量
# =============================================================================

impedance_enabled = True
gravity_enabled = True

kx_scale = 1.0
dx_scale = 1.0


# =============================================================================
# 工具函数
# =============================================================================

def clamp(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)


class SimpleRateLimiter:
    def __init__(self, frequency):
        self.period = 1.0 / frequency
        self.last_time = time.perf_counter()

    def sleep(self):
        now = time.perf_counter()
        elapsed = now - self.last_time
        sleep_time = self.period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        self.last_time = time.perf_counter()
        return self.period


# =============================================================================
# 夹爪控制器
# =============================================================================

class GripperController:
    def __init__(self, model, data):
        self.model = model
        self.data = data

        self.gripper_act_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper"
        )

        if self.gripper_act_id < 0:
            self.gripper_act_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_motor"
            )

        if self.gripper_act_id < 0:
            print("⚠️ 未找到夹爪执行器，夹爪控制将被禁用。")

        self.left_finger_joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "left_finger"
        )

        self.target_pos = GRIPPER_OPEN
        self.current_pos = GRIPPER_OPEN

    def set_target(self, target):
        self.target_pos = float(np.clip(target, 0.001, 0.05))

    def update(self):
        if self.gripper_act_id < 0:
            return

        if self.left_finger_joint_id >= 0:
            qpos_addr = self.model.jnt_qposadr[self.left_finger_joint_id]
            current_pos = self.data.qpos[qpos_addr]

            pos_error = self.target_pos - current_pos
            control_signal = GRIPPER_STIFFNESS * pos_error

            # 保持你原来的夹爪控制映射方式
            ctrl_value = self.target_pos + control_signal * 0.001
            ctrl_value = float(np.clip(ctrl_value, 0.001, 0.05))

            self.data.ctrl[self.gripper_act_id] = ctrl_value
            self.current_pos = ctrl_value


# =============================================================================
# 笛卡尔阻抗控制器
# =============================================================================

class CartesianImpedanceController:
    def __init__(self, model, data, site_name="eef_trace_site"):
        self.model = model
        self.data = data

        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, site_name
        )
        if self.site_id < 0:
            raise RuntimeError(f"未找到末端 site: {site_name}")

        # 获取 joint1~joint6 的 joint id、qpos 地址、qvel 地址
        self.joint_ids = []
        self.qpos_addr = []
        self.qvel_addr = []

        for i in range(1, ARM_DOF + 1):
            joint_name = f"joint{i}"
            jid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            if jid < 0:
                raise RuntimeError(f"未找到关节: {joint_name}")

            self.joint_ids.append(jid)
            self.qpos_addr.append(model.jnt_qposadr[jid])
            self.qvel_addr.append(model.jnt_dofadr[jid])

        self.qpos_addr = np.array(self.qpos_addr, dtype=int)
        self.qvel_addr = np.array(self.qvel_addr, dtype=int)

        # 获取 joint1~joint6 的 actuator id
        self.actuator_ids = []
        for i in range(1, ARM_DOF + 1):
            act_name_1 = f"joint{i}"
            act_name_2 = f"motor{i}"

            aid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name_1
            )
            if aid < 0:
                aid = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name_2
                )

            if aid < 0:
                raise RuntimeError(
                    f"未找到 joint{i} 对应执行器。请检查 actuator 名称是否为 joint{i} 或 motor{i}。"
                )

            self.actuator_ids.append(aid)

        self.actuator_ids = np.array(self.actuator_ids, dtype=int)

        self.tau_prev = np.zeros(ARM_DOF)
        self.last_time = None
        self.start_time = time.time()

        print("✅ 笛卡尔阻抗控制器初始化完成")
        print(f"   site_id       = {self.site_id}")
        print(f"   joint_ids     = {self.joint_ids}")
        print(f"   qpos_addr     = {self.qpos_addr.tolist()}")
        print(f"   qvel_addr     = {self.qvel_addr.tolist()}")
        print(f"   actuator_ids  = {self.actuator_ids.tolist()}")

    def get_q(self):
        return self.data.qpos[self.qpos_addr].copy()

    def get_qvel(self):
        return self.data.qvel[self.qvel_addr].copy()

    def compute_gravity_compensation(self):
        """
        MuJoCo 中：
            M(q)qdd + qfrc_bias = tau + ...
        静态重力补偿时，通常取：
            tau_g = qfrc_bias
        对于低速运动，qfrc_bias 还包含科氏/离心项。
        """
        tau_g = self.data.qfrc_bias[self.qvel_addr].copy()
        tau_g = tau_g * GRAVITY_SCALE
        return tau_g

    def compute_cartesian_impedance_tau(self, x_des):
        """
        计算末端位置阻抗对应的关节力矩：
            F = Kx(x_des - x) - Dx * xdot
            tau = J.T @ F
        """
        # 当前末端位置
        x = self.data.site_xpos[self.site_id].copy()

        # 末端位置雅可比
        J_pos_full = np.zeros((3, self.model.nv))
        J_rot_full = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(
            self.model,
            self.data,
            J_pos_full,
            J_rot_full,
            self.site_id,
        )

        # 只取 6 个机械臂关节对应列
        J = J_pos_full[:, self.qvel_addr]

        qvel = self.get_qvel()

        # 末端线速度
        xdot = J @ qvel

        # 笛卡尔刚度与阻尼
        Kx = KX_BASE * kx_scale
        Dx = DX_BASE * dx_scale

        pos_error = x_des - x

        # 末端虚拟弹簧阻尼力
        F_task = Kx * pos_error - Dx * xdot

        # 映射为关节力矩
        tau_task = J.T @ F_task

        return tau_task, x, xdot, pos_error, F_task

    def update(self, x_des):
        global impedance_enabled, gravity_enabled

        now = time.time()
        if self.last_time is None:
            dt = DT
        else:
            dt = max(now - self.last_time, 1e-5)
        self.last_time = now

        qvel = self.get_qvel()

        # 1. 重力补偿
        if gravity_enabled:
            tau_g = self.compute_gravity_compensation()
        else:
            tau_g = np.zeros(ARM_DOF)

        # 2. 笛卡尔阻抗
        if impedance_enabled:
            tau_task, x, xdot, pos_error, F_task = self.compute_cartesian_impedance_tau(x_des)
        else:
            tau_task = np.zeros(ARM_DOF)
            x = self.data.site_xpos[self.site_id].copy()
            xdot = np.zeros(3)
            pos_error = x_des - x
            F_task = np.zeros(3)

        # 3. 小关节阻尼，抑制晃动
        tau_damp = -KQ_DAMP_BASE * qvel

        # 4. 合成力矩
        tau_cmd = tau_g + tau_task + tau_damp

        # 5. 启动渐入
        elapsed = now - self.start_time
        ramp = min(1.0, elapsed / RAMP_TIME)
        tau_cmd = ramp * tau_cmd

        # 6. 力矩限幅
        tau_cmd = clamp(tau_cmd, -TORQUE_LIMITS, TORQUE_LIMITS)

        # 7. 力矩变化率限制
        max_delta = TAU_RATE_LIMITS * dt
        tau_cmd = self.tau_prev + clamp(
            tau_cmd - self.tau_prev,
            -max_delta,
            max_delta,
        )
        self.tau_prev = tau_cmd.copy()

        # 8. 下发到 MuJoCo data.ctrl
        for i, act_id in enumerate(self.actuator_ids):
            # 如果 actuator 设置了 ctrlrange，再额外按 XML 范围裁剪
            if self.model.actuator_ctrllimited[act_id]:
                lo, hi = self.model.actuator_ctrlrange[act_id]
                self.data.ctrl[act_id] = float(np.clip(tau_cmd[i], lo, hi))
            else:
                self.data.ctrl[act_id] = float(tau_cmd[i])

        info = {
            "x": x,
            "x_des": x_des,
            "xdot": xdot,
            "pos_error": pos_error,
            "F_task": F_task,
            "tau_g": tau_g,
            "tau_task": tau_task,
            "tau_damp": tau_damp,
            "tau_cmd": tau_cmd,
        }

        return info


# =============================================================================
# 键盘控制
# =============================================================================

def key_callback(key):
    global current_gripper_target
    global impedance_enabled, gravity_enabled
    global kx_scale, dx_scale

    # 上箭头：闭合夹爪
    if key == 265:
        current_gripper_target = GRIPPER_CLOSE
        print("⬆️  夹爪闭合")

    # 下箭头：张开夹爪
    elif key == 264:
        current_gripper_target = GRIPPER_OPEN
        print("⬇️  夹爪张开")

    # i：开启/关闭阻抗跟随
    elif key in [ord("i"), ord("I")]:
        impedance_enabled = not impedance_enabled
        print(f"🧲 阻抗控制: {'ON' if impedance_enabled else 'OFF'}")

    # g：开启/关闭重力补偿
    elif key in [ord("g"), ord("G")]:
        gravity_enabled = not gravity_enabled
        print(f"🌍 重力补偿: {'ON' if gravity_enabled else 'OFF'}")

    # [：降低刚度
    elif key == ord("["):
        kx_scale = max(0.1, kx_scale * 0.8)
        print(f"🔽 Kx scale = {kx_scale:.3f}")

    # ]：提高刚度
    elif key == ord("]"):
        kx_scale = min(10.0, kx_scale * 1.25)
        print(f"🔼 Kx scale = {kx_scale:.3f}")

    # ;：降低阻尼
    elif key == ord(";"):
        dx_scale = max(0.1, dx_scale * 0.8)
        print(f"🔽 Dx scale = {dx_scale:.3f}")

    # '：提高阻尼
    elif key == ord("'"):
        dx_scale = min(10.0, dx_scale * 1.25)
        print(f"🔼 Dx scale = {dx_scale:.3f}")

    # 空格：打印状态
    elif key == 32:
        print(
            f"状态 | impedance={impedance_enabled}, gravity={gravity_enabled}, "
            f"kx_scale={kx_scale:.3f}, dx_scale={dx_scale:.3f}, "
            f"gripper_target={current_gripper_target:.3f}"
        )


# =============================================================================
# 主程序
# =============================================================================

def main():
    global current_gripper_target

    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    model.opt.timestep = DT
    model.opt.iterations = 50
    model.opt.tolerance = 1e-8

    print("=" * 70)
    print("  reBotArm MuJoCo 笛卡尔阻抗控制 + 重力补偿仿真")
    print("=" * 70)
    print(f"XML_PATH = {XML_PATH}")
    print(f"CTRL_FREQ = {CTRL_FREQ} Hz")
    print("=" * 70)

    # 获取 mocap target
    mocap_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "mocap_target"
    )
    if mocap_body_id < 0:
        raise RuntimeError("未找到 body: mocap_target")

    # 一般只有一个 mocap body，对应 mocap_id=0
    mocap_id = 0

    # 初始化机械臂关节
    data.qpos[:ARM_DOF] = np.zeros(ARM_DOF)
    mujoco.mj_forward(model, data)

    # 初始化控制器
    arm_ctrl = CartesianImpedanceController(
        model=model,
        data=data,
        site_name="eef_trace_site",
    )

    gripper_ctrl = GripperController(model, data)

    # 将 mocap_target 初始位置设置为末端当前位置
    start_pos = data.site_xpos[arm_ctrl.site_id].copy()
    data.mocap_pos[mocap_id] = start_pos.copy()
    data.mocap_quat[mocap_id] = np.array([1.0, 0.0, 0.0, 0.0])

    print(f"📍 末端初始位置: {start_pos}")
    print(f"🎯 mocap_target 初始位置已设置为末端位置。")

    # 打印 actuator 信息
    print("\n🔌 执行器信息:")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        limited = bool(model.actuator_ctrllimited[i])
        ctrlrange = model.actuator_ctrlrange[i]
        print(
            f"  actuator {i:2d}: {name}, "
            f"ctrllimited={limited}, ctrlrange={ctrlrange}"
        )

    print("\n" + "=" * 70)
    print("🎮 操作说明")
    print("  鼠标拖动红色 mocap_target 方块：机械臂末端阻抗跟随")
    print("  上箭头：夹爪闭合")
    print("  下箭头：夹爪张开")
    print("  i：开启/关闭末端阻抗")
    print("  g：开启/关闭重力补偿")
    print("  [ / ]：降低 / 提高末端刚度 Kx")
    print("  ; / '：降低 / 提高末端阻尼 Dx")
    print("  空格：打印当前状态")
    print("  ESC：退出")
    print("=" * 70 + "\n")

    rate = SimpleRateLimiter(CTRL_FREQ)

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
        show_right_ui=False,
        key_callback=key_callback,
    ) as viewer:

        last_print_time = time.time()
        frame = 0

        while viewer.is_running():
            frame += 1

            # 必须先 forward，保证 site_xpos、qfrc_bias、Jacobian 使用当前状态
            mujoco.mj_forward(model, data)

            # 读取 mocap 目标位置
            x_des = data.mocap_pos[mocap_id].copy()

            # 更新阻抗控制
            info = arm_ctrl.update(x_des)

            # 更新夹爪
            gripper_ctrl.set_target(current_gripper_target)
            gripper_ctrl.update()

            # 仿真步进
            mujoco.mj_step(model, data)

            # 更新画面
            viewer.sync()

            # 状态打印
            now = time.time()
            if now - last_print_time >= 1.0:
                last_print_time = now

                pos_error_norm = np.linalg.norm(info["pos_error"])
                tau_str = ", ".join([f"{v:+.2f}" for v in info["tau_cmd"]])
                fg_str = ", ".join([f"{v:+.1f}" for v in info["F_task"]])

                print(f"帧: {frame:5d}")
                print(
                    f"  🎯 target: [{info['x_des'][0]:+.3f}, {info['x_des'][1]:+.3f}, {info['x_des'][2]:+.3f}]"
                )
                print(
                    f"  📍 eef   : [{info['x'][0]:+.3f}, {info['x'][1]:+.3f}, {info['x'][2]:+.3f}]"
                )
                print(f"  📏 error : {pos_error_norm:.4f} m")
                print(f"  🧲 F_task: [{fg_str}] N")
                print(f"  🎛️ tau   : [{tau_str}] N·m")
                print(
                    f"  mode: impedance={'ON' if impedance_enabled else 'OFF'}, "
                    f"gravity={'ON' if gravity_enabled else 'OFF'}, "
                    f"kx_scale={kx_scale:.2f}, dx_scale={dx_scale:.2f}"
                )
                print("-" * 70)

            rate.sleep()

    print("👋 退出仿真")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n👋 用户中断，退出程序")
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        import traceback
        traceback.print_exc()
