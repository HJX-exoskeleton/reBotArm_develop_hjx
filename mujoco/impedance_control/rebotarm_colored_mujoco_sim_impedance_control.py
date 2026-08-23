#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reBotArm MuJoCo 仿真：末端笛卡尔阻抗控制 + 重力补偿

功能：
1. 鼠标拖动红色 mocap_target 方块；
2. 机械臂末端 eef_trace_site 通过 6D 笛卡尔阻抗跟随 mocap_target 位姿；
3. 使用 MuJoCo data.qfrc_bias 实现重力补偿；
4. 运行时关闭六轴 position servo，通过 qfrc_applied 下发关节力矩；
5. 保留夹爪开合控制；
6. 支持键盘在线调节阻抗刚度和阻尼。

控制律：
    F_task = Kx * (x_des - x) - Dx * xdot
    M_task = Kr * orientation_error - Dr * omega
    tau_task = J_pos.T @ F_task + J_rot.T @ M_task

其中：
    x_des  : mocap_target 位置
    x      : 末端 site 位置
    J_pos  : 末端位置雅可比矩阵
    J_rot  : 末端旋转雅可比矩阵
    tau_g  : MuJoCo qfrc_bias 对应的重力/科氏/离心补偿项

注意：
    彩色 XML 中六轴是 position actuator。本脚本在运行时将它们的
    gain/bias 清零，然后将力矩写入 qfrc_applied，不会把 N·m 误当成 rad。
"""

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
    ROOT_DIR / "xml" / "rebotarm_b601_colored" / "sim_rebotarm_colored_grasp.xml"
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
KX_BASE = np.array([200.0, 200.0, 170.0])

# 笛卡尔阻尼，单位近似 N·s/m
# 数值越大，末端运动越稳，但越“粘”
# 数值越小，末端更轻快，但可能振荡
DX_BASE = np.array([22, 22, 20])

# 笛卡尔旋转刚度和阻尼，单位近似 N·m/rad 和 N·m·s/rad
KR_BASE = np.array([8.0, 8.0, 6.0])
DR_BASE = np.array([0.9, 0.9, 0.7])

# 零空间姿态保持，防止仅控制 XYZ 时关节慢慢漂移
KQ_NULL = np.array([8.0, 8.0, 7.0, 4.0, 2.0, 1.5])
DQ_NULL = np.array([1.4, 1.6, 1.4, 0.8, 0.45, 0.30])

# 每个关节最大输出力矩，单位 N·m
TORQUE_LIMITS = np.array([25.0, 25.0, 25.0, 12.0, 8.0, 6.0])

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
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_position"
        )

        if self.gripper_act_id < 0:
            self.gripper_act_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_motor"
            )

        if self.gripper_act_id < 0:
            print("⚠️ 未找到夹爪执行器，夹爪控制将被禁用。")

        self.left_finger_joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "finger_left"
        )
        if self.gripper_act_id >= 0:
            self.ctrl_range = model.actuator_ctrlrange[self.gripper_act_id].copy()
        if self.left_finger_joint_id >= 0:
            self.qpos_addr = int(model.jnt_qposadr[self.left_finger_joint_id])

        self.target_pos = GRIPPER_OPEN
        self.current_pos = GRIPPER_OPEN

    def set_target(self, target):
        self.target_pos = float(np.clip(target, 0.001, 0.05))

    def update(self):
        if self.gripper_act_id < 0:
            return

        ctrl_value = float(np.clip(self.target_pos, *self.ctrl_range))
        self.data.ctrl[self.gripper_act_id] = ctrl_value
        if self.left_finger_joint_id >= 0:
            self.current_pos = float(self.data.qpos[self.qpos_addr])


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
            act_name_1 = f"joint{i}_position"
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

        # 彩色 XML 使用 position actuator。禁用其内部位置反馈，
        # 否则 ctrl 会被解释为关节角而不是力矩。
        self.model.actuator_gainprm[self.actuator_ids, :] = 0.0
        self.model.actuator_biasprm[self.actuator_ids, :] = 0.0
        self.data.ctrl[self.actuator_ids] = 0.0

        self.q_reference = self.get_q()
        self.tau_motion_prev = np.zeros(ARM_DOF)
        self.start_time = float(data.time)

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

    def compute_cartesian_impedance_tau(self, x_des, quat_des):
        """
        计算末端 6D 位姿阻抗对应的关节力矩：
            F = Kx(x_des - x) - Dx * xdot
            M = Kr * rotation_error - Dr * omega
            tau = J_pos.T @ F + J_rot.T @ M
        """
        # 当前末端位姿
        x = self.data.site_xpos[self.site_id].copy()
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, self.data.site_xmat[self.site_id])

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
        J_pos = J_pos_full[:, self.qvel_addr]
        J_rot = J_rot_full[:, self.qvel_addr]

        qvel = self.get_qvel()

        # 末端线速度
        xdot = J_pos @ qvel
        omega = J_rot @ qvel

        # 笛卡尔刚度与阻尼
        Kx = KX_BASE * kx_scale
        Dx = DX_BASE * dx_scale
        Kr = KR_BASE * kx_scale
        Dr = DR_BASE * dx_scale

        pos_error = x_des - x
        rot_error = np.empty(3)
        # MuJoCo 返回从当前姿态旋转到目标姿态的最短轴角向量。
        mujoco.mju_subQuat(rot_error, quat_des, quat)

        # 末端虚拟弹簧阻尼力
        F_task = Kx * pos_error - Dx * xdot
        M_task = Kr * rot_error - Dr * omega

        # 映射为关节力矩
        tau_task = J_pos.T @ F_task + J_rot.T @ M_task

        # 位姿任务的零空间只在奇异/降秩时保留姿态约束。
        J_task = np.vstack((J_pos, J_rot))
        J_pinv = np.linalg.pinv(J_task, rcond=0.03)
        nullspace = np.eye(ARM_DOF) - J_pinv @ J_task
        q_error = self.q_reference - self.get_q()
        tau_posture_raw = KQ_NULL * q_error - DQ_NULL * qvel
        tau_null = nullspace.T @ tau_posture_raw

        return (
            tau_task, tau_null, x, quat, xdot, omega,
            pos_error, rot_error, F_task, M_task,
        )

    def update(self, x_des, quat_des):
        global impedance_enabled, gravity_enabled

        qvel = self.get_qvel()

        # 1. 重力补偿
        if gravity_enabled:
            tau_g = self.compute_gravity_compensation()
        else:
            tau_g = np.zeros(ARM_DOF)

        # 2. 笛卡尔阻抗
        if impedance_enabled:
            (
                tau_task, tau_null, x, quat, xdot, omega,
                pos_error, rot_error, F_task, M_task,
            ) = self.compute_cartesian_impedance_tau(x_des, quat_des)
        else:
            tau_task = np.zeros(ARM_DOF)
            tau_null = -DQ_NULL * qvel
            x = self.data.site_xpos[self.site_id].copy()
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, self.data.site_xmat[self.site_id])
            xdot = np.zeros(3)
            omega = np.zeros(3)
            pos_error = x_des - x
            rot_error = np.empty(3)
            mujoco.mju_subQuat(rot_error, quat_des, quat)
            F_task = np.zeros(3)
            M_task = np.zeros(3)

        # 3. 启动渐入只作用于任务力矩。重力补偿立即生效，避免启动下坠。
        elapsed = float(self.data.time) - self.start_time
        ramp = min(1.0, elapsed / RAMP_TIME)
        tau_motion = ramp * (tau_task + tau_null)

        # 4. 仅对运动力矩做变化率限制，不延迟重力补偿。
        max_delta = TAU_RATE_LIMITS * DT
        tau_motion = self.tau_motion_prev + clamp(
            tau_motion - self.tau_motion_prev,
            -max_delta,
            max_delta,
        )
        self.tau_motion_prev = tau_motion.copy()

        # 5. 合成、限幅并下发到六轴 DoF。
        tau_cmd = clamp(tau_g + tau_motion, -TORQUE_LIMITS, TORQUE_LIMITS)
        self.data.qfrc_applied[:] = 0.0
        self.data.qfrc_applied[self.qvel_addr] = tau_cmd

        info = {
            "x": x,
            "x_des": x_des,
            "quat": quat,
            "quat_des": quat_des,
            "xdot": xdot,
            "omega": omega,
            "pos_error": pos_error,
            "rot_error": rot_error,
            "F_task": F_task,
            "M_task": M_task,
            "tau_g": tau_g,
            "tau_task": tau_task,
            "tau_null": tau_null,
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

    # 空格：切换夹爪开合
    elif key == 32:
        if np.isclose(current_gripper_target, GRIPPER_OPEN):
            current_gripper_target = GRIPPER_CLOSE
            print("⏹️  夹爪闭合")
        else:
            current_gripper_target = GRIPPER_OPEN
            print("▶️  夹爪张开")


# =============================================================================
# 主程序
# =============================================================================

def main():
    global current_gripper_target

    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    # 保留 XML 的 1 ms 物理步长，在 200 Hz 控制周期内执行多个物理子步。
    physics_steps_per_control = max(1, round(DT / model.opt.timestep))
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

    mocap_id = int(model.body_mocapid[mocap_body_id])
    if mocap_id < 0:
        raise RuntimeError("mocap_target 不是 mocap body")

    # 初始化机械臂关节
    initial_joints = np.array([0.0, -0.70, -0.80, 0.0, 0.0, 0.0])
    for joint_index, joint_target in enumerate(initial_joints, start=1):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{joint_index}"
        )
        data.qpos[model.jnt_qposadr[joint_id]] = joint_target

    left_finger_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "finger_left"
    )
    right_finger_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "finger_right"
    )
    data.qpos[model.jnt_qposadr[left_finger_id]] = GRIPPER_OPEN
    data.qpos[model.jnt_qposadr[right_finger_id]] = -GRIPPER_OPEN
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
    start_quat = np.empty(4)
    mujoco.mju_mat2Quat(start_quat, data.site_xmat[arm_ctrl.site_id])
    data.mocap_quat[mocap_id] = start_quat

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
    print("  鼠标平移/旋转红色 mocap_target：末端 6D 位姿阻抗跟随")
    print("  上箭头：夹爪闭合")
    print("  下箭头：夹爪张开")
    print("  i：开启/关闭末端阻抗")
    print("  g：开启/关闭重力补偿")
    print("  [ / ]：降低 / 提高末端刚度 Kx")
    print("  ; / '：降低 / 提高末端阻尼 Dx")
    print("  空格：切换夹爪张开/闭合")
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

            # 读取 mocap 目标位姿
            x_des = data.mocap_pos[mocap_id].copy()
            quat_des = data.mocap_quat[mocap_id].copy()

            # 更新阻抗控制
            info = arm_ctrl.update(x_des, quat_des)

            # 更新夹爪
            gripper_ctrl.set_target(current_gripper_target)
            gripper_ctrl.update()

            # 仿真步进
            for _ in range(physics_steps_per_control):
                mujoco.mj_step(model, data)

            # 更新画面
            viewer.sync()

            # 状态打印
            now = time.time()
            if now - last_print_time >= 1.0:
                last_print_time = now

                pos_error_norm = np.linalg.norm(info["pos_error"])
                rot_error_deg = np.degrees(np.linalg.norm(info["rot_error"]))
                tau_str = ", ".join([f"{v:+.2f}" for v in info["tau_cmd"]])
                fg_str = ", ".join([f"{v:+.1f}" for v in info["F_task"]])
                moment_str = ", ".join([f"{v:+.2f}" for v in info["M_task"]])

                print(f"帧: {frame:5d}")
                print(
                    f"  🎯 target: [{info['x_des'][0]:+.3f}, {info['x_des'][1]:+.3f}, {info['x_des'][2]:+.3f}]"
                )
                print(
                    f"  📍 eef   : [{info['x'][0]:+.3f}, {info['x'][1]:+.3f}, {info['x'][2]:+.3f}]"
                )
                print(f"  📏 error : {pos_error_norm:.4f} m")
                print(f"  🔄 attitude error: {rot_error_deg:.2f}°")
                print(f"  🧲 F_task: [{fg_str}] N")
                print(f"  🧭 M_task: [{moment_str}] N·m")
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
