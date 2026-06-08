#!/usr/bin/env python3
"""
reBotArm 柔顺抓取 Demo (优化版)
修复夹爪闭合力度不足和抖动问题
"""

import sys
import time
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
import mujoco
import mujoco.viewer

# ================= 系统参数配置 =================
ROOT_DIR = Path(__file__).resolve().parents[1]
XML_PATH = str(ROOT_DIR / "mujoco" / "xml" / "rebot_gripper" / "reBot-DevArm_gripper.xml")

CTRL_FREQ = 500
LAMBDA = 5.0
Kd = np.diag([800, 800, 800, 600, 600, 600])

# 【优化】夹爪控制参数
GRIPPER_OPEN = 0.05
# 【关键优化】使用更小的闭合位置，增加抓取力
GRIPPER_CLOSE = 0.018  # 从0.0195减小到0.018，增加闭合力度
GRIPPER_SOFT_CLOSE = 0.019  # 软抓取位置
GRIPPER_STIFFNESS = 50000.0  # 【增加】夹爪刚度
GRIPPER_DAMPING = 500.0  # 【增加】夹爪阻尼
GRIPPER_FORCE_SCALE = 1.5  # 【增加】抓取力缩放因子


# ================= 核心算法 =================

def sliding_mode_controller(model, data, qd, qd_dot, qd_ddot):
    nv = model.nv
    q = data.qpos[:6].copy()
    q_dot = data.qvel[:6].copy()

    q_tilde = q - qd
    q_tilde_dot = q_dot - qd_dot
    qr_ddot = qd_ddot - LAMBDA * q_tilde_dot
    s = q_tilde_dot + LAMBDA * q_tilde

    H = np.zeros((nv, nv), dtype=np.float64)
    mujoco.mj_fullM(model, H, data.qM)
    H_joints = H[:6, :6]

    C = np.zeros(nv)
    mujoco.mj_rnePostConstraint(model, data)
    mujoco.mj_rne(model, data, 1, C)

    data.qacc[:] = 0
    data.qfrc_bias[:] = 0
    mujoco.mj_rne(model, data, 0, data.qfrc_bias)
    G = data.qfrc_bias[:6]

    tau = H_joints @ qr_ddot + C[:6] + G - Kd @ s
    return np.clip(tau, -100, 100)


def update_clik(model, data, target_pos, target_vel, qd_prev, dt):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "eef_trace_site")
    current_pos = data.site_xpos[site_id].copy()

    err = target_pos - current_pos

    J = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, J, None, site_id)
    J = J[:, :6]

    damping = 0.1
    Jinv = J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * np.eye(3))

    Kp = 5.0
    x_dot_cmd = target_vel + Kp * err
    qd_dot = Jinv @ x_dot_cmd

    qd = qd_prev + qd_dot * dt
    qd = np.clip(qd, model.jnt_range[:6, 0], model.jnt_range[:6, 1])

    return qd, qd_dot


# ================= 夹爪控制器 =================

class GripperController:
    """优化的夹爪控制器"""

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")

        # 控制参数
        self.target_pos = GRIPPER_OPEN
        self.current_pos = GRIPPER_OPEN
        self.velocity = 0.0

        # 滤波参数
        self.pos_filter_alpha = 0.3  # 位置滤波系数
        self.vel_filter_alpha = 0.5  # 速度滤波系数

        # 状态跟踪
        self.is_grasping = False
        self.grasp_start_time = 0.0
        self.last_grasp_pos = GRIPPER_OPEN

        # 物理参数优化
        self.force_multiplier = 1.0

    def set_target(self, target):
        """设置目标位置"""
        # 确保在控制范围内
        self.target_pos = np.clip(target, 0.001, 0.05)

        # 如果目标位置是闭合位置，标记为抓取状态
        if target <= GRIPPER_CLOSE + 0.001:
            if not self.is_grasping:
                self.is_grasping = True
                self.grasp_start_time = time.time()
                self.last_grasp_pos = self.current_pos
                # 【优化】在抓取时增加力系数
                self.force_multiplier = GRIPPER_FORCE_SCALE
        else:
            self.is_grasping = False
            self.force_multiplier = 1.0

    def update(self, dt):
        """更新夹爪控制"""
        # 计算目标速度（带限制）
        pos_error = self.target_pos - self.current_pos
        max_speed = 0.1  # 最大闭合速度 m/s

        # 【优化】在抓取时降低速度，避免冲击
        if self.is_grasping:
            max_speed = 0.05
            # 在抓取状态下，逐渐增加闭合力度
            elapsed = time.time() - self.grasp_start_time
            if elapsed < 0.5:  # 前0.5秒逐渐增加力度
                self.force_multiplier = 1.0 + (GRIPPER_FORCE_SCALE - 1.0) * (elapsed / 0.5)

        target_vel = np.clip(pos_error / dt, -max_speed, max_speed)

        # 速度滤波
        self.velocity = self.vel_filter_alpha * target_vel + (1 - self.vel_filter_alpha) * self.velocity

        # 更新位置
        new_pos = self.current_pos + self.velocity * dt

        # 位置滤波
        self.current_pos = self.pos_filter_alpha * new_pos + (1 - self.pos_filter_alpha) * self.current_pos

        # 确保在控制范围内
        self.current_pos = np.clip(self.current_pos, 0.001, 0.05)

        # 【关键优化】计算控制信号
        # 使用PD控制器，增加刚度和阻尼
        if self.gripper_act_id >= 0:
            # 获取当前夹爪位置（通过left_finger关节）
            left_finger_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_finger")
            if left_finger_id >= 0:
                qpos_addr = self.model.jnt_qposadr[left_finger_id]
                if qpos_addr < self.data.qpos.size:
                    current_gripper_pos = self.data.qpos[qpos_addr]

                    # 计算PD控制
                    pos_error = self.current_pos - current_gripper_pos
                    vel_error = self.velocity - 0.0  # 假设速度为0

                    # 控制信号 = 刚度 * 位置误差 + 阻尼 * 速度误差
                    control_signal = GRIPPER_STIFFNESS * pos_error + GRIPPER_DAMPING * vel_error

                    # 应用力系数
                    control_signal *= self.force_multiplier

                    # 映射到控制范围
                    ctrl_value = self.current_pos + control_signal * 0.001

                    # 确保在XML控制范围内
                    ctrl_value = np.clip(ctrl_value, 0.001, 0.05)

                    # 设置控制信号
                    self.data.ctrl[self.gripper_act_id] = ctrl_value

                    # 调试信息
                    if self.is_grasping and time.time() - self.grasp_start_time < 1.0:
                        print(f"🤏 抓取中: 目标={self.target_pos:.4f}, 实际={current_gripper_pos:.4f}, "
                              f"控制={ctrl_value:.4f}, 力系数={self.force_multiplier:.2f}")

    def get_status(self):
        """获取夹爪状态"""
        return {
            'target': self.target_pos,
            'current': self.current_pos,
            'velocity': self.velocity,
            'is_grasping': self.is_grasping,
            'force_multiplier': self.force_multiplier
        }


# ================= 轨迹生成 =================

def build_task_trajectory(start_pos, box_pos):
    """构建优化的抓取轨迹"""
    # 【优化】调整时间点，让夹爪闭合更缓慢
    t_points = [0.0, 2.0, 3.5, 4.5, 6.0, 8.0, 9.0]  # 延长闭合时间

    p0 = start_pos
    p1 = box_pos + np.array([0, 0, 0.08])
    p2 = box_pos + np.array([0, 0, 0.01])  # 【优化】稍微提高悬停高度
    p3 = p2
    p4 = box_pos + np.array([0, 0, 0.20])
    p5 = box_pos + np.array([-0.2, 0.2, 0.1])
    p6 = p5

    pos_points = np.vstack([p0, p1, p2, p3, p4, p5, p6])
    cs_pos = CubicSpline(t_points, pos_points, bc_type='clamped')

    # 【优化】夹爪轨迹：缓慢闭合 -> 保持闭合 -> 缓慢张开
    g_points = [GRIPPER_OPEN, GRIPPER_OPEN, GRIPPER_OPEN,
                GRIPPER_SOFT_CLOSE,  # 先软抓取
                GRIPPER_CLOSE,  # 再增加力度
                GRIPPER_CLOSE, GRIPPER_OPEN]

    return cs_pos, t_points, g_points


# ================= 主程序 =================

def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    dt = 1.0 / CTRL_FREQ
    model.opt.timestep = dt

    print("✅ 仿真环境初始化...")

    # 【优化】调整仿真参数
    model.opt.solver = mujoco.mjtSolver.mjSOL_CG  # 使用共轭梯度求解器
    model.opt.iterations = 50
    model.opt.tolerance = 1e-10

    # 启用摩擦锥
    model.opt.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL

    for _ in range(int(0.5 * CTRL_FREQ)):
        mujoco.mj_step(model, data)

    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "red_box")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "eef_trace_site")
    box_pos = data.geom_xpos[box_id].copy()

    data.qpos[:6] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)
    start_pos = data.site_xpos[site_id].copy()

    print("🚀 正在生成优化的3D空间抓取样条轨迹...")
    cs_pos, t_plan, g_plan = build_task_trajectory(start_pos, box_pos)
    cs_vel = cs_pos.derivative(1)

    total_duration = t_plan[-1]
    num_steps = int(total_duration * CTRL_FREQ)

    qd_prev = data.qpos[:6].copy()
    prev_qd_dot = np.zeros(6)

    qd_ddot_filtered = np.zeros(6)
    tau_filtered = np.zeros(6)

    alpha_acc = 0.02
    alpha_tau = 0.10

    # 初始化夹爪控制器
    gripper_ctrl = GripperController(model, data)

    print("🎬 开始执行优化的抓取任务！")
    print(f"夹爪参数: 刚度={GRIPPER_STIFFNESS}, 阻尼={GRIPPER_DAMPING}, 力系数={GRIPPER_FORCE_SCALE}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        time.sleep(1.0)

        # 初始抓取测试
        print("🤏 初始抓取测试...")
        gripper_ctrl.set_target(GRIPPER_CLOSE)
        for _ in range(100):
            gripper_ctrl.update(dt)
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(dt)

        # 张开夹爪
        gripper_ctrl.set_target(GRIPPER_OPEN)
        for _ in range(100):
            gripper_ctrl.update(dt)
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(dt)

        print("✅ 初始测试完成，开始主任务")

        for step in range(num_steps):
            if not viewer.is_running():
                break

            t = step * dt

            target_pos = cs_pos(t)
            target_vel = cs_vel(t)

            qd, qd_dot = update_clik(model, data, target_pos, target_vel, qd_prev, dt)

            # 加速度滤波
            raw_qd_ddot = (qd_dot - prev_qd_dot) * CTRL_FREQ
            qd_ddot_filtered = alpha_acc * raw_qd_ddot + (1 - alpha_acc) * qd_ddot_filtered

            # 计算原始控制力矩
            tau_raw = sliding_mode_controller(model, data, qd, qd_dot, qd_ddot_filtered)

            # 力矩低通滤波
            tau_filtered = alpha_tau * tau_raw + (1 - alpha_tau) * tau_filtered

            data.ctrl[:6] = tau_filtered

            # 【优化】更新夹爪控制
            g_cmd = np.interp(t, t_plan, g_plan)
            gripper_ctrl.set_target(g_cmd)
            gripper_ctrl.update(dt)

            mujoco.mj_step(model, data)
            qd_prev = qd.copy()
            prev_qd_dot = qd_dot.copy()

            if step % int(CTRL_FREQ / 60) == 0:
                viewer.sync()

            # 显示抓取状态
            if step % 500 == 0:
                status = gripper_ctrl.get_status()
                print(f"⏱️  t={t:.2f}s | 夹爪: 目标={status['target']:.4f}, 实际={status['current']:.4f}, "
                      f"抓取={status['is_grasping']}, 力系数={status['force_multiplier']:.2f}")

        print("🎉 抓取任务圆满完成！")

        # 保持抓取状态2秒
        hold_time = 2.0
        hold_steps = int(hold_time * CTRL_FREQ)

        for step in range(hold_steps):
            if not viewer.is_running():
                break

            # 保持当前力矩
            tau_raw = sliding_mode_controller(model, data, qd_prev, np.zeros(6), np.zeros(6))
            tau_filtered = alpha_tau * tau_raw + (1 - alpha_tau) * tau_filtered
            data.ctrl[:6] = tau_filtered

            # 保持夹爪闭合
            gripper_ctrl.update(dt)

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(dt)

        print("✅ 任务完成，夹爪释放...")

        # 缓慢张开夹爪
        release_steps = 200
        for step in range(release_steps):
            if not viewer.is_running():
                break

            # 逐渐张开夹爪
            release_progress = step / release_steps
            release_pos = GRIPPER_CLOSE + (GRIPPER_OPEN - GRIPPER_CLOSE) * release_progress
            gripper_ctrl.set_target(release_pos)
            gripper_ctrl.update(dt)

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(dt)

        print("🎊 所有任务完成！")


if __name__ == "__main__":
    main()
