import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import tkinter as tk
import threading

# 中文字体显示配置
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False
from ik_solver import InverseKinematicsSolver

from pathlib import Path
import sys

ROOT_DIR = Path(__file__).resolve().parents[1]
XML_PATH = str(ROOT_DIR / "xml" / "rebot_gripper" / "reBot-DevArm_gripper.xml")


class SMCController:
    def __init__(self, model, control_mode=4):
        self.model = model
        self.n_joints = 6  # 固定7自由度配置
        self.dt = model.opt.timestep
        self.control_mode = control_mode
        
        # 控制器参数
        self.Lambda = np.diag([30.0]*self.n_joints)
        self.Kv = np.diag([15.0]*self.n_joints)
        self.epsilon_N = 0.2
        self.b_d = 0.3
        self.W_max = 5.0
        self.k = 0.8
        
        # RBF网络参数
        self.input_dim = 5 * 6  # 固定为7自由度输入维度
        self.hidden_dim = 50
        self.centers = np.random.uniform(-1.5, 1.5, (self.hidden_dim, self.input_dim))
        self.bandwidth = np.ones(self.hidden_dim) * 0.6
        self.F = 0.1 * np.eye(self.hidden_dim)  # 自适应增益矩阵
        self.W_hat = np.random.normal(0, 0.1, (self.hidden_dim, self.n_joints))
        
        # 输入动态归一化参数
        self.input_mean = np.zeros(self.input_dim)
        self.input_std = np.ones(self.input_dim)
        self.input_update_count = 0  # 用于动态统计
        
        # 轨迹参数
        self.center = np.array([0.3, -0.2, 0.5])
        self.radius = 0.2
        self.z_amp = 0.05
        self.freq = 0.5
        
        self.ik = InverseKinematicsSolver(model, damping=0.001, max_iter=100)
        self.prev_qd = np.zeros(6)  # 初始参考位置设为0

    def _rbf_activation(self, x):
        """动态更新输入统计量并计算RBF激活"""
        # 前100步收集统计量
        if self.input_update_count < 100:
            self.input_mean = (self.input_mean * self.input_update_count + x) / (self.input_update_count + 1)
            self.input_std = np.sqrt(
                (self.input_std**2 * self.input_update_count + (x - self.input_mean)**2) / 
                (self.input_update_count + 1)
            )
            self.input_update_count += 1
        
        x_norm = (x - self.input_mean) / (self.input_std + 1e-6)
        diff = self.centers - x_norm.reshape(1, -1)
        norms = np.linalg.norm(diff, axis=1)
        return np.exp(-(norms**2) / (self.bandwidth**2))

    def _generate_trajectory(self, t):
        """生成末端轨迹"""
        theta = 2 * np.pi * self.freq * t
        pos = self.center + np.array([
            self.radius * np.cos(theta),
            self.radius * np.sin(theta),
            self.z_amp * np.sin(4 * np.pi * self.freq * t)
        ])
        vel = np.array([
            -2 * np.pi * self.freq * self.radius * np.sin(theta),
            2 * np.pi * self.freq * self.radius * np.cos(theta),
            4 * np.pi * self.freq * self.z_amp * np.cos(4 * np.pi * self.freq * t)
        ])
        acc = np.array([
            -(2 * np.pi * self.freq)**2 * self.radius * np.cos(theta),
            -(2 * np.pi * self.freq)**2 * self.radius * np.sin(theta),
            -(4 * np.pi * self.freq)**2 * self.z_amp * np.sin(4 * np.pi * self.freq * t)
        ])
        return pos, vel, acc

    def _project_weights(self):
        """参数投影保证权值有界"""
        W_norm = np.linalg.norm(self.W_hat, 'fro')
        if W_norm > self.W_max:
            self.W_hat *= self.W_max / W_norm

    def _update_weights(self, r, phi, r_norm):
        """
        更新RBF网络权重参数

        Args:
            r (ndarray): 滑模面误差向量，形状(7,)
            phi (ndarray): RBF激活向量，形状(50,)
            r_norm (float): 滑模面范数值
        """
        """权值更新逻辑（包含两种模式）"""
        assert phi.shape == (self.hidden_dim,), f"phi维度错误: {phi.shape}"
        assert r.shape == (self.n_joints,), f"r维度错误: {r.shape}"
        
        if self.control_mode == 3:
            # 模式3：带阻尼项的自适应律
            dW = self.F @ phi.reshape(-1, 1) @ r.reshape(1, -1)-self.k*self.F*r_norm @ self.W_hat
        else:
            # 默认模式：基本自适应律
            dW = self.F @ phi.reshape(-1, 1) @ r.reshape(1, -1)
            
        self.W_hat += dW * self.dt
        self._project_weights()  # 每次更新后强制投影

    def _compute_control_force(self, q, dq, data):
        """核心控制计算"""
        t = data.time
        target_pos, target_vel, target_acc = self._generate_trajectory(t)
        
        # 逆运动学求解（使用上一时刻解作为初始猜测）
        qd = self.ik.solve(data, target_pos)[0][:6]  # 取前7个关节分量
        dqd = self.ik.solve(data, target_pos)[1][:6]
        self.prev_qd = qd.copy()
        
        # 计算关节空间导数
        # 使用MuJoCo原生接口获取雅可比矩阵
        J = np.zeros((3, self.model.nv))
        mujoco.mj_jac(self.model, data, J, None, data.site(self.ik.ee_site_name).xpos, self.model.site(self.ik.ee_site_name).bodyid[0])
        J_pinv = np.linalg.pinv(J)
        dqd = (J_pinv @ target_vel)[:6]  # 取前7个关节分量
        
        # 获取当前时刻的雅可比矩阵
        J_current = np.zeros((3, self.model.nv))
        mujoco.mj_jac(self.model, data, J_current, None, data.site(self.ik.ee_site_name).xpos, self.model.site(self.ik.ee_site_name).bodyid[0])
        J_current = J_current[:, :6]  # 仅保留前7个关节的雅可比分量
        cartesian_acc = target_acc.reshape(3,) - J_current @ dqd  # 确保维度匹配
        qd_ddot = (J_pinv @ cartesian_acc)[:6]  # 取前7个关节分量
        
        # 误差计算
        e = qd[:6] - q  # 确保仅使用前7个关节
        de = dqd[:6] - dq  # 确保维度匹配
        r = de + self.Lambda @ e
        
        # RBF输入构建
        x = np.concatenate([e, de, qd, dqd, qd_ddot])
        phi = self._rbf_activation(x)
        f_hat = self.W_hat.T @ phi  # (n_joints,)
        
        # 改进的鲁棒项
        if self.control_mode == 4:
            v = -(self.epsilon_N + self.b_d) *np.sign(r)
        else:
            v = np.zeros_like(r)
        
        # 控制律计算
        tau = f_hat + self.Kv @ r - v
        tau = np.clip(tau, -50, 50)  # 限制控制范围
        # 更新权值
        self._update_weights(r, phi, np.linalg.norm(r))
        
        return tau, r, f_hat


# 构造优化后的几何绘制数据准备逻辑（与 viewer 一致）： 在仿真过程中每帧绘制轨迹点
def prepare_red_sphere_geom(geom, pos):
    """填充一个 mjvGeom 实例为红色小球（追踪点）"""
    mujoco.mjv_initGeom(
        geom,
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=np.array([0.002, 0.0, 0.0], dtype=np.float64),
        pos=pos.astype(np.float64),
        mat=np.eye(3).flatten().astype(np.float64),
        rgba=np.array([1.0, 0.0, 0.0, 0.8], dtype=np.float32)
    )


def run_simulation(control_mode=4):
    """
    执行机械臂控制仿真

    Args:
        control_mode (int): 控制模式选择 (1-4)

    Returns:
        dict: 包含时间、关节角度、扭矩等数据的日志字典

    Raises:
        ValueError: 当控制模式不在1-4范围内时抛出
    """

    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    controller = SMCController(model, control_mode)

    # 初始化日志
    log = {
        't': [],
        'target_pos': [],
        'actual_pos': [],
        'err_pos': [],
        'tau': [],
        'joints': [],
        's': [],        # 滑模面
        'f_hat': []  # RBF估计误差
    }

    # 获取 site ID
    site_id = model.site("eef_trace_site").id
    # 存储轨迹
    trajectory = []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        SIM_DURATION = 10.0
        
        while data.time < SIM_DURATION and viewer.is_running():
            # 读取当前状态
            q = data.qpos[:controller.n_joints].copy()
            dq = data.qvel[:controller.n_joints].copy()
            
            # 计算控制力
            tau, r, f_hat = controller._compute_control_force(q, dq, data)
            
            # 应用控制
            data.ctrl[:controller.n_joints] = tau
            
            # 记录数据
            target_pos, _, _ = controller._generate_trajectory(data.time)
            actual_pos = data.site_xpos[model.site(controller.ik.ee_site_name).id]
            
            log['t'].append(data.time)
            log['target_pos'].append(target_pos.copy())
            log['actual_pos'].append(actual_pos.copy())
            log['err_pos'].append(target_pos - actual_pos)
            log['tau'].append(tau.copy())
            log['joints'].append(q.copy())
            log['s'].append(r)
            log['f_hat'].append(np.linalg.norm(f_hat))
            
            # 步进仿真（使用MuJoCo内部时间管理）
            mujoco.mj_step(model, data)
            viewer.sync()

            # 在仿真过程中每帧绘制轨迹点
            pos = data.site_xpos[site_id].copy()  # 获取 site 的当前坐标
            trajectory.append(pos)
            # 控制拖尾点数量
            max_points = 2500  # naive：1000
            if len(trajectory) > max_points:
                trajectory.pop(0)
            # 清除并重绘用户几何体（红点）
            with viewer.lock():
                viewer.user_scn.ngeom = 0  # 清空上帧所有用户几何体
                for p in trajectory:
                    prepare_red_sphere_geom(viewer.user_scn.geoms[viewer.user_scn.ngeom], p)
                    viewer.user_scn.ngeom += 1

        viewer.close()
    
    # 转换为numpy数组
    for key in log:
        log[key] = np.array(log[key])
    return log



def plot_results(log):
    plt.figure(figsize=(18, 12))
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False

    # Trajectory comparison (Target vs Actual)   轨迹对比
    ax1 = plt.subplot(231, projection='3d')
    ax1.plot(log['target_pos'][:,0], log['target_pos'][:,1], log['target_pos'][:,2], 'r--', label='Target Trajectory')
    ax1.plot(log['actual_pos'][:,0], log['actual_pos'][:,1], log['actual_pos'][:,2], 'b-', label='Actual Trajectory')
    ax1.set_xlabel('X axis (m)')
    ax1.set_ylabel('Y axis (m)')
    ax1.set_zlabel('Z axis (m)')
    ax1.set_title('End-Effector Trajectory Tracking')  # 末端轨迹跟踪
    ax1.legend(loc='upper left')

    # Position tracking error norm   跟踪误差
    plt.subplot(232)
    plt.plot(log['t'], np.linalg.norm(log['err_pos'], axis=1))
    plt.title('Position Tracking Error Norm')  # 位置跟踪误差范数
    plt.xlabel('Time (s)')
    plt.ylabel('Error Norm (m)')  # 误差范数 (m)

    # Joint torques  控制力矩
    plt.subplot(233)
    for i in range(6):
        plt.plot(log['t'], log['tau'][:,i], label=f'Joint {i+1}')
    plt.title('Joint Torques')  # 关节力矩
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (N·m)')  # 力矩 (N·m)
    plt.legend(loc='upper right')

    # Sliding surface norm  滑模面
    plt.subplot(234)
    plt.plot(log['t'], np.linalg.norm(log['s'], axis=1))
    plt.title('Sliding Surface Norm Convergence')  # 滑模面范数收敛过程
    plt.xlabel('Time (s)')
    plt.ylabel('Sliding Surface Norm')  # 滑模面范数

    # RBF estimation values  RBF估计
    plt.subplot(235)
    plt.plot(log['t'], log['f_hat'])
    plt.title('RBF Network Estimated Value')  # RBF网络估计值
    plt.xlabel('Time (s)')
    plt.ylabel('Estimated Value')  # 估计值

    # Joint angles  关节角度
    plt.subplot(236)
    for i in range(6):
        plt.plot(log['t'], log['joints'][:,i], label=f'Joint {i+1}', alpha=0.7)
    plt.title('Joint Angle Changes')  # 关节角度变化
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')  # 角度 (rad)
    plt.legend(loc='upper right')

    plt.tight_layout()
    plt.show()


class ControlGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("控制模式选择")
        self.create_buttons()
        self.simulation_thread = None
        self.running = False
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_close(self):
        self.running = False
        if self.simulation_thread and self.simulation_thread.is_alive():
            self.simulation_thread.join(timeout=1)
        self.root.destroy()
        
    def create_buttons(self):
        modes = [
            ("模式1", 1),
            ("模式2", 2),
            ("模式3", 3),
            ("模式4", 4)
        ]
        
        for text, mode in modes:
            btn = tk.Button(self.root, text=text, width=25, height=3,
                          command=lambda m=mode: self.start_simulation(m))
            btn.pack(pady=5)
        
    def start_simulation(self, control_mode):
        if self.running:
            return
        
        def run_thread():
            try:
                log = run_simulation(control_mode)
                self.root.after(0, lambda: plot_results(log))
            finally:
                self.running = False
        
        self.running = True
        self.simulation_thread = threading.Thread(target=run_thread)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()

if __name__ == "__main__":

    
    gui = ControlGUI()
    gui.root.mainloop()