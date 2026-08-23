#!/usr/bin/env python3
"""
reBotArm mocap手动拖拽调试脚本
通过data.ctrl控制执行器，实现稳定的机械臂控制
通过上下箭头控制夹爪开合
"""

import time
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer

try:
    import mink
    MINK_IMPORT_ERROR = None
except Exception as exc:  # Mink 可能被 qpsolvers/JAX 的版本冲突阻断。
    mink = None
    MINK_IMPORT_ERROR = exc

# ================= 系统参数配置 =================
ROOT_DIR = Path(__file__).resolve().parents[1]
XML_PATH = str(ROOT_DIR / "mujoco" / "xml" / "rebotarm_b601_colored" / "sim_rebotarm_grasp.xml")

CTRL_FREQ = 200
dt = 1.0 / CTRL_FREQ

# 模型对象名称（不要假设 qpos/qvel 中的排列顺序）
ARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
ARM_ACTUATOR_NAMES = tuple(f"joint{i}_position" for i in range(1, 7))

# 夹爪控制参数
GRIPPER_OPEN = 0.05
GRIPPER_CLOSE = 0.01

# 控制参数
POSITION_COST = 2.0
ORIENTATION_COST = 0.1
POSTURE_COST = 0.01
LM_DAMPING = 2.0

# ================= 全局状态 =================
current_gripper_target = GRIPPER_OPEN  # 全局夹爪目标
arm_ctrl = None


# ================= 机械臂控制器 =================

class ArmController:
    """使用 MJCF 原生 position actuator 的机械臂控制器。"""

    def __init__(self, model, data):
        self.model = model
        self.data = data

        self.joint_ids = np.array([
            self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in ARM_JOINT_NAMES
        ])
        self.joint_actuator_ids = np.array([
            self._required_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ARM_ACTUATOR_NAMES
        ])
        self.qpos_addresses = model.jnt_qposadr[self.joint_ids].copy()
        self.dof_addresses = model.jnt_dofadr[self.joint_ids].copy()
        print(f"🔧 六轴 qpos 地址: {self.qpos_addresses.tolist()}")
        print(f"🔧 六轴执行器 ID: {self.joint_actuator_ids.tolist()}")

        # 目标关节位置
        self.target_joints = np.zeros(6)
        # 当前关节位置
        self.current_joints = np.zeros(6)

        # 关节限位
        self.joint_limits = model.jnt_range[self.joint_ids].copy()

    def _required_id(self, object_type, name):
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"XML 中缺少必需对象: {name}")
        return object_id

    def set_target(self, target_joints):
        """设置目标关节位置"""
        # 应用关节限位
        for i in range(6):
            self.target_joints[i] = np.clip(target_joints[i],
                                            self.joint_limits[i, 0],
                                            self.joint_limits[i, 1])

    def update(self):
        """位置执行器的 ctrl 就是目标关节角。"""
        self.current_joints = self.data.qpos[self.qpos_addresses].copy()
        self.data.ctrl[self.joint_actuator_ids] = self.target_joints
        # 补偿重力/科氏力，避免位置执行器长时间饱和。
        self.data.qfrc_applied[self.dof_addresses] = self.data.qfrc_bias[self.dof_addresses]


# ================= 夹爪控制器 =================

class GripperController:
    """夹爪控制器"""

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_position")

        if self.gripper_act_id < 0:
            print("❌ 未找到夹爪执行器")
            # 尝试查找其他可能的夹爪执行器名称
            self.gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_motor")
            if self.gripper_act_id < 0:
                raise ValueError("未找到夹爪执行器")

        print(f"🤏 夹爪执行器ID: {self.gripper_act_id}")

        # 控制参数
        self.target_pos = GRIPPER_OPEN
        self.current_pos = GRIPPER_OPEN
        left_finger_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "finger_left"
        )
        if left_finger_id < 0:
            raise ValueError("XML 中缺少夹爪关节 finger_left")
        self.qpos_address = int(self.model.jnt_qposadr[left_finger_id])
        self.ctrl_range = self.model.actuator_ctrlrange[self.gripper_act_id].copy()

    def set_target(self, target):
        """设置目标位置"""
        self.target_pos = np.clip(target, 0.001, 0.05)

    def update(self):
        """夹爪也是 MJCF 位置执行器，直接写入目标开度。"""
        self.target_pos = float(np.clip(self.target_pos, *self.ctrl_range))
        self.data.ctrl[self.gripper_act_id] = self.target_pos
        self.current_pos = float(self.data.qpos[self.qpos_address])

    def get_status(self):
        return {
            'target': self.target_pos,
            'current': self.current_pos
        }


# ================= 键盘回调 =================

def key_callback(key):
    """键盘回调函数，处理键盘输入"""
    global current_gripper_target, arm_ctrl

    # 上箭头：闭合夹爪
    if key == 265:  # 上箭头键码
        current_gripper_target = GRIPPER_CLOSE
        print("⬆️  夹爪闭合")
    # 下箭头：张开夹爪
    elif key == 264:  # 下箭头键码
        current_gripper_target = GRIPPER_OPEN
        print("⬇️  夹爪张开")
    # 空格键：在张开和闭合之间切换
    elif key == 32:  # 空格键码
        if np.isclose(current_gripper_target, GRIPPER_OPEN):
            current_gripper_target = GRIPPER_CLOSE
            print("⏹️  夹爪闭合")
        else:
            current_gripper_target = GRIPPER_OPEN
            print("▶️  夹爪张开")
    # 数字1-6：打印对应关节信息
    elif 49 <= key <= 54:  # 数字1-6的键码
        joint_idx = key - 49
        if arm_ctrl is not None:
            qpos_addr = arm_ctrl.qpos_addresses[joint_idx]
            act_id = arm_ctrl.joint_actuator_ids[joint_idx]
            print(f"🔧 关节{joint_idx + 1}: qpos={data.qpos[qpos_addr]:.3f}, "
                  f"target={data.ctrl[act_id]:.3f}")


# ================= 简单频率限制器 =================

class SimpleRateLimiter:
    """简单的频率限制器"""

    def __init__(self, frequency=200.0):
        self.period = 1.0 / frequency
        self.last_time = time.perf_counter()

    def sleep(self):
        """等待以达到目标频率"""
        current_time = time.perf_counter()
        elapsed = current_time - self.last_time
        sleep_time = self.period - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)

        self.last_time = time.perf_counter()
        return self.period


# ================= 主程序 =================

def main():
    global current_gripper_target, data, arm_ctrl

    # 加载模型
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    # 保留 XML 中 1 ms 物理步长，控制器每 5 ms 更新一次。
    physics_steps_per_control = max(1, round(dt / model.opt.timestep))

    # 优化仿真参数
    model.opt.solver = mujoco.mjtSolver.mjSOL_CG
    model.opt.iterations = 30
    model.opt.tolerance = 1e-8

    print("✅ 仿真环境初始化...")

    # 获取相关ID
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "eef_trace_site")
    mocap_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "mocap_target")

    if site_id < 0 or mocap_body_id < 0:
        raise ValueError("XML 中缺少 eef_trace_site 或 mocap_target")

    # 获取mocap数据ID
    mocap_id = int(model.body_mocapid[mocap_body_id])
    if mocap_id < 0:
        raise ValueError("mocap_target 不是 mocap body")

    arm_ctrl = ArmController(model, data)
    gripper_ctrl = GripperController(model, data)

    # 初始化关节位置
    initial_joints = np.array([0.0, -0.70, -0.80, 0.0, 0.0, 0.0])
    data.qpos[arm_ctrl.qpos_addresses] = initial_joints
    # 初始化夹爪对称关节，避免启动时约束冲击。
    data.qpos[gripper_ctrl.qpos_address] = GRIPPER_OPEN
    right_finger_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "finger_right")
    data.qpos[model.jnt_qposadr[right_finger_id]] = -GRIPPER_OPEN
    arm_ctrl.set_target(initial_joints)
    arm_ctrl.update()
    gripper_ctrl.set_target(GRIPPER_OPEN)
    gripper_ctrl.update()
    mujoco.mj_forward(model, data)

    # 获取初始位置
    start_pos = data.site_xpos[site_id].copy()

    print(f"📍 末端起始位置: {start_pos}")
    print(f"🎯 mocap初始位置: {data.mocap_pos[mocap_id]}")

    # 设置mocap_target的初始位置为当前末端位置
    data.mocap_pos[mocap_id] = start_pos.copy()
    start_quat = np.empty(4)
    mujoco.mju_mat2Quat(start_quat, data.site_xmat[site_id])
    data.mocap_quat[mocap_id] = start_quat

    print(f"🎯 设置mocap初始位置为末端位置: {start_pos}")

    # ================= 初始化 IK =================
    configuration = None
    ee_task = None
    tasks = None
    limits = None
    if mink is not None:
        print("\n🔄 初始化 Mink 控制...")
        configuration = mink.Configuration(model)
        ee_task = mink.FrameTask(
            frame_name="eef_trace_site",
            frame_type="site",
            position_cost=POSITION_COST,
            orientation_cost=ORIENTATION_COST,
            lm_damping=LM_DAMPING,
        )
        posture_task = mink.PostureTask(model, cost=POSTURE_COST)
        tasks = [ee_task, posture_task]
        limits = [mink.ConfigurationLimit(model=model)]
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)
        print("✅ Mink 初始化完成")
    else:
        print(f"\n⚠️  Mink 不可用: {MINK_IMPORT_ERROR}")
        print("✅ 将使用内置阻尼最小二乘位置 IK")

    # 打印执行器信息
    print("\n🔌 执行器信息:")
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  执行器 {i}: {act_name}, 控制范围: {model.actuator_ctrlrange[i]}")

    # 打印使用说明
    print("\n" + "=" * 60)
    print("  手动拖拽调试模式 (通过data.ctrl控制)")
    print("=" * 60)
    print("🎮 控制说明:")
    print("  - 用鼠标拖动红色的mocap_target方块")
    print("  - 观察机械臂末端如何跟随目标")
    print("  - 上箭头键: 闭合夹爪")
    print("  - 下箭头键: 张开夹爪")
    print("  - 数字1-6: 打印对应关节信息")
    print("  - 空格键: 切换夹爪张开/闭合")
    print("  - 按ESC键退出")
    print("\n📊 状态信息（每秒更新）:")

    # 初始化频率限制器
    rate = SimpleRateLimiter(frequency=CTRL_FREQ)
    solver_names = ("daqp", "osqp", "proxqp", "cvxopt")
    selected_solver = None
    solver_failure_reported = False

    with mujoco.viewer.launch_passive(
            model=model,
            data=data,
            show_left_ui=False,
            show_right_ui=False,
            key_callback=key_callback
    ) as viewer:
        time.sleep(1.0)

        last_print_time = time.time()
        frame = 0

        while viewer.is_running():
            frame += 1

            # 1. 获取当前mocap_target的位置
            target_pos = data.mocap_pos[mocap_id].copy()

            # 2. 设置夹爪目标
            gripper_ctrl.set_target(current_gripper_target)

            # 3. 优先用 Mink 求解；不可用时改用内置位置 IK。
            try:
                vel = None
                solver_errors = []
                if mink is not None:
                    ee_task.set_target(
                        mink.SE3.from_mocap_name(model, data, "mocap_target")
                    )
                    candidates = (selected_solver,) if selected_solver else solver_names
                    for solver in candidates:
                        try:
                            vel = mink.solve_ik(
                                configuration,
                                tasks,
                                rate.period,
                                solver,
                                limits=limits,
                                damping=1e-5,
                            )
                            if vel is not None:
                                selected_solver = solver
                                break
                        except Exception as solver_error:
                            solver_errors.append(f"{solver}: {solver_error}")

                if vel is None:
                    if solver_errors and not solver_failure_reported:
                        print("⚠️  Mink QP 求解器不可用，改用阻尼最小二乘位置 IK:")
                        for message in solver_errors:
                            print(f"    {message}")
                        solver_failure_reported = True
                    # 使用简单的数值IK
                    current_pos = data.site_xpos[site_id].copy()
                    error = target_pos - current_pos

                    # 计算雅可比
                    J = np.zeros((3, model.nv))
                    mujoco.mj_jacSite(model, data, J, None, site_id)
                    J = J[:, arm_ctrl.dof_addresses]

                    # 阻尼最小二乘
                    damping = 0.1
                    J_T = J.T
                    J_pinv = J_T @ np.linalg.inv(J @ J_T + damping ** 2 * np.eye(3))

                    # 计算关节速度
                    Kp = 5.0
                    desired_vel = Kp * error
                    q_dot = np.clip(J_pinv @ desired_vel, -1.5, 1.5)
                    target_joints = (
                        data.qpos[arm_ctrl.qpos_addresses] + q_dot * rate.period
                    )
                else:
                    configuration.integrate_inplace(vel, rate.period)
                    target_joints = configuration.q[arm_ctrl.qpos_addresses].copy()

                arm_ctrl.set_target(target_joints)

            except Exception as e:
                print(f"⚠️  IK求解失败: {e}")
                # 继续使用当前配置

            # 5. 更新机械臂控制
            arm_ctrl.update()

            # 6. 夹爪控制
            gripper_ctrl.update()

            # 7. 计算跟踪误差
            current_pos = data.site_xpos[site_id].copy()
            pos_error = np.linalg.norm(target_pos - current_pos)

            # 8. 仿真步进
            for _ in range(physics_steps_per_control):
                mujoco.mj_step(model, data)

            # 9. 更新视图
            viewer.sync()

            # 10. 显示状态（每秒一次）
            current_time = time.time()
            if current_time - last_print_time >= 1.0:
                last_print_time = current_time

                # 计算夹爪闭合百分比
                gripper_status = gripper_ctrl.get_status()
                gripper_percent = ((GRIPPER_OPEN - gripper_status['current']) /
                                   (GRIPPER_OPEN - GRIPPER_CLOSE)) * 100
                gripper_percent = np.clip(gripper_percent, 0, 100)

                # 显示控制信号
                ctrl_str = ", ".join([f"{data.ctrl[i]:.1f}" for i in range(min(7, model.nu))])

                print(f"帧: {frame:4d} | 🎯 mocap位置: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                print(f"      📍 末端位置: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
                print(f"      📏 跟踪误差: {pos_error:.4f}m")
                print(f"      🤏 夹爪: {gripper_percent:.1f}% 闭合")
                joint_positions = data.qpos[arm_ctrl.qpos_addresses]
                print(f"      🎮 关节: {', '.join([f'{q:.2f}' for q in joint_positions])}")
                print(f"      🎛️  控制信号: [{ctrl_str}]")
                print("-" * 60)

            # 11. 频率限制
            rate.sleep()

        print("👋 退出仿真")

    return True


if __name__ == "__main__":
    print("=" * 60)
    print("  reBotArm 手动拖拽调试工具 (通过data.ctrl控制)")
    print("=" * 60)

    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 用户中断，退出程序")
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        import traceback

        traceback.print_exc()
