#!/usr/bin/env python3
"""
reBotArm mocap手动拖拽调试脚本
通过data.ctrl控制执行器，实现稳定的机械臂控制
通过上下箭头控制夹爪开合
"""

import sys
import time
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
import mink

# ================= 系统参数配置 =================
ROOT_DIR = Path(__file__).resolve().parents[1]
XML_PATH = str(ROOT_DIR / "mujoco" / "xml" / "rebot_gripper" / "sim_reBot_grasp.xml")

CTRL_FREQ = 200
dt = 1.0 / CTRL_FREQ

# 夹爪控制参数
GRIPPER_OPEN = 0.05
GRIPPER_CLOSE = 0.018
GRIPPER_STIFFNESS = 50000.0
GRIPPER_DAMPING = 500.0

# 机械臂PD控制参数
KP_JOINTS = np.array([500.0, 500.0, 500.0, 300.0, 300.0, 200.0])  # 位置增益
KD_JOINTS = np.array([30.0, 30.0, 30.0, 20.0, 20.0, 10.0])  # 阻尼增益

# 控制参数
POSITION_COST = 2.0
ORIENTATION_COST = 0.1
POSTURE_COST = 0.01
LM_DAMPING = 2.0

# ================= 全局状态 =================
current_gripper_target = GRIPPER_OPEN  # 全局夹爪目标


# ================= 机械臂控制器 =================

class ArmController:
    """机械臂PD控制器"""

    def __init__(self, model, data):
        self.model = model
        self.data = data

        # 获取关节执行器ID
        self.joint_actuator_ids = []
        for i in range(1, 7):  # joint1 到 joint6
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"joint{i}")
            if actuator_id >= 0:
                self.joint_actuator_ids.append(actuator_id)
            else:
                print(f"⚠️  未找到执行器 joint{i}")
                # 尝试查找其他可能的执行器名称
                actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor{i}")
                if actuator_id >= 0:
                    self.joint_actuator_ids.append(actuator_id)
                    print(f"✅ 找到执行器 motor{i}")
                else:
                    # 创建虚拟执行器ID（如果XML中没有定义）
                    self.joint_actuator_ids.append(-1)

        print(f"🔧 找到的执行器ID: {self.joint_actuator_ids}")

        # 目标关节位置
        self.target_joints = np.zeros(6)
        # 当前关节位置
        self.current_joints = np.zeros(6)

        # 关节限位
        self.joint_limits = np.zeros((6, 2))
        for i in range(6):
            joint_name = f"joint{i + 1}"
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id >= 0:
                self.joint_limits[i] = model.jnt_range[joint_id]
            else:
                self.joint_limits[i] = [-np.pi, np.pi]

    def set_target(self, target_joints):
        """设置目标关节位置"""
        # 应用关节限位
        for i in range(6):
            self.target_joints[i] = np.clip(target_joints[i],
                                            self.joint_limits[i, 0],
                                            self.joint_limits[i, 1])

    def update(self, dt):
        """更新机械臂控制"""
        # 获取当前关节位置和速度
        self.current_joints = self.data.qpos[:6].copy()
        current_vel = self.data.qvel[:6].copy()

        # 计算位置误差
        pos_error = self.target_joints - self.current_joints

        # PD控制
        for i, act_id in enumerate(self.joint_actuator_ids):
            if act_id >= 0 and i < 6:
                # 计算控制信号
                control_signal = (KP_JOINTS[i] * pos_error[i] -
                                  KD_JOINTS[i] * current_vel[i])

                # 应用控制信号
                self.data.ctrl[act_id] = control_signal
            elif i < 6:
                # 如果没有找到执行器，直接设置关节位置（不推荐）
                self.data.qpos[i] = self.target_joints[i]


# ================= 夹爪控制器 =================

class GripperController:
    """夹爪控制器"""

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")

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

    def set_target(self, target):
        """设置目标位置"""
        self.target_pos = np.clip(target, 0.001, 0.05)

    def update(self, dt):
        """更新夹爪控制"""
        # 获取当前夹爪位置
        left_finger_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_finger")
        if left_finger_id >= 0:
            qpos_addr = self.model.jnt_qposadr[left_finger_id]
            if qpos_addr < self.data.qpos.size:
                current_gripper_pos = self.data.qpos[qpos_addr]

                # 计算位置误差
                pos_error = self.target_pos - current_gripper_pos

                # PD控制
                control_signal = GRIPPER_STIFFNESS * pos_error

                # 映射到控制范围
                ctrl_value = self.target_pos + control_signal * 0.001
                ctrl_value = np.clip(ctrl_value, 0.001, 0.05)

                # 设置控制信号
                self.data.ctrl[self.gripper_act_id] = ctrl_value
                self.current_pos = ctrl_value

    def get_status(self):
        return {
            'target': self.target_pos,
            'current': self.current_pos
        }


# ================= 键盘回调 =================

def key_callback(key):
    """键盘回调函数，处理键盘输入"""
    global current_gripper_target

    # 上箭头：闭合夹爪
    if key == 265:  # 上箭头键码
        current_gripper_target = GRIPPER_CLOSE
        print("⬆️  夹爪闭合")
    # 下箭头：张开夹爪
    elif key == 264:  # 下箭头键码
        current_gripper_target = GRIPPER_OPEN
        print("⬇️  夹爪张开")
    # 空格键：打印状态
    elif key == 32:  # 空格键码
        print("⏸️  当前夹爪目标: ", current_gripper_target)
    # 数字1-6：打印对应关节信息
    elif 49 <= key <= 54:  # 数字1-6的键码
        joint_idx = key - 49
        print(f"🔧 关节{joint_idx + 1}: qpos={data.qpos[joint_idx]:.3f}, ctrl={data.ctrl[joint_idx]:.3f}")


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
    global current_gripper_target, data

    # 加载模型
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    model.opt.timestep = dt

    # 优化仿真参数
    model.opt.solver = mujoco.mjtSolver.mjSOL_CG
    model.opt.iterations = 30
    model.opt.tolerance = 1e-8

    print("✅ 仿真环境初始化...")

    # 获取相关ID
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "eef_trace_site")
    mocap_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "mocap_target")

    if mocap_body_id < 0:
        print("❌ 未找到mocap_target")
        return

    # 获取mocap数据ID
    mocap_id = 0

    # 初始化关节位置
    initial_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    data.qpos[:6] = initial_joints
    mujoco.mj_forward(model, data)

    # 获取初始位置
    start_pos = data.site_xpos[site_id].copy()

    print(f"📍 末端起始位置: {start_pos}")
    print(f"🎯 mocap初始位置: {data.mocap_pos[mocap_id]}")

    # 设置mocap_target的初始位置为当前末端位置
    data.mocap_pos[mocap_id] = start_pos.copy()
    data.mocap_quat[mocap_id] = np.array([1, 0, 0, 0])

    print(f"🎯 设置mocap初始位置为末端位置: {start_pos}")

    # 初始化控制器
    arm_ctrl = ArmController(model, data)
    gripper_ctrl = GripperController(model, data)

    # 设置初始目标
    arm_ctrl.set_target(initial_joints)

    # ================= 初始化mink =================
    print("\n🔄 初始化mink控制...")

    # 初始化配置
    configuration = mink.Configuration(model)

    # 创建任务
    tasks = [
        ee_task := mink.FrameTask(
            frame_name="eef_trace_site",
            frame_type="site",
            position_cost=POSITION_COST,
            orientation_cost=ORIENTATION_COST,
            lm_damping=LM_DAMPING,
        ),
        posture_task := mink.PostureTask(model, cost=POSTURE_COST),
    ]

    # 创建限制
    limits = [
        mink.ConfigurationLimit(model=model),
    ]

    # 初始化到初始姿态
    mujoco.mj_forward(model, data)
    configuration.update(data.qpos)
    posture_task.set_target_from_configuration(configuration)

    print("✅ mink初始化完成")

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
    print("  - 空格键: 打印当前状态")
    print("  - 按ESC键退出")
    print("\n📊 状态信息（每秒更新）:")

    # 初始化频率限制器
    rate = SimpleRateLimiter(frequency=CTRL_FREQ)

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
            target_pos = data.mocap_pos[mocap_id]

            # 2. 设置夹爪目标
            gripper_ctrl.set_target(current_gripper_target)

            # 3. 设置任务目标为mocap_target的位置
            try:
                # 使用mocap名称获取目标
                ee_task.set_target(mink.SE3.from_mocap_name(model, data, "mocap_target"))
            except Exception as e:
                # 备用方法
                try:
                    target_se3 = mink.SE3.from_translation(target_pos)
                    ee_task.set_target(target_se3)
                except Exception as e2:
                    print(f"⚠️  设置目标失败: {e2}")
                    continue

            # 4. 使用mink求解逆运动学
            try:
                # 尝试使用可用的求解器
                available_solvers = ["daqp", "osqp", "proxqp", "cvxopt"]
                vel = None

                for solver in available_solvers:
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
                            break
                    except:
                        continue

                if vel is None:
                    # 使用简单的数值IK
                    current_pos = data.site_xpos[site_id].copy()
                    error = target_pos - current_pos

                    # 计算雅可比
                    J = np.zeros((3, model.nv))
                    mujoco.mj_jacSite(model, data, J, None, site_id)
                    J = J[:, :6]

                    # 阻尼最小二乘
                    damping = 0.1
                    J_T = J.T
                    J_pinv = J_T @ np.linalg.inv(J @ J_T + damping ** 2 * np.eye(3))

                    # 计算关节速度
                    Kp = 5.0
                    desired_vel = Kp * error
                    q_dot = J_pinv @ desired_vel

                    # 积分得到新配置
                    vel = np.zeros(model.nv)
                    vel[:6] = q_dot

                # 积分得到新配置
                configuration.integrate_inplace(vel, rate.period)

                # 获取计算出的目标关节位置
                target_joints = configuration.q[:6].copy()
                arm_ctrl.set_target(target_joints)

            except Exception as e:
                print(f"⚠️  IK求解失败: {e}")
                # 继续使用当前配置

            # 5. 更新机械臂控制
            arm_ctrl.update(dt)

            # 6. 夹爪控制
            gripper_ctrl.update(dt)

            # 7. 计算跟踪误差
            current_pos = data.site_xpos[site_id].copy()
            pos_error = np.linalg.norm(target_pos - current_pos)

            # 8. 仿真步进
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
                print(f"      🎮 关节: {', '.join([f'{q:.2f}' for q in data.qpos[:6]])}")
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
