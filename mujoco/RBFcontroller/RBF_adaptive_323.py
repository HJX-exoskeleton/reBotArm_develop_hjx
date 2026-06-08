import sys
# sys.path.append(r'c:\xarm7')
import numpy as np
import mujoco
import mujoco.viewer
from scipy.linalg import pinv
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False
from ik_solver import InverseKinematicsSolver


from pathlib import Path
import sys

ROOT_DIR = Path(__file__).resolve().parents[1]
XML_PATH = str(ROOT_DIR / "xml" / "rebot_gripper" / "reBot-DevArm_gripper.xml")

class TrajectoryGenerator:
    def __init__(self):
        self.target_pos = np.array([0.3, -0.2, 0.5])
        self.radius = 0.2
        self.z_amp = 0.05
        self.freq = 0.5
        
    def get_target(self, t):
        theta = 2 * np.pi * self.freq * t
        offset = np.array([
            self.radius * np.cos(theta),
            self.radius * np.sin(theta),
            self.z_amp * np.sin(4 * np.pi * self.freq * t)
        ])
        return self.target_pos + offset

class RBFNetwork:
    def __init__(self, input_dim, hidden_dim, output_dim):
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.output_dim = output_dim
        
        # RBF参数初始化
        # Initialize multi-dimensional centers with proper joint ranges
        self.centers = np.random.uniform(-np.pi, np.pi, size=(hidden_dim, input_dim))
        self.sigma = 1.0 * np.sqrt(input_dim)  # Scale sigma with input dimension
        self.sigma = 1.0
        self.weights = np.random.randn(hidden_dim, output_dim) * 0.1
        
    def basis_function(self, x):
        # Ensure x is properly broadcastable with centers
        x = np.array(x).reshape(1, -1)
        return np.exp(-np.linalg.norm(x - self.centers, axis=1)**2 / (2*self.sigma**2)).reshape(-1, 1)
    
    def __call__(self, x):
        phi = self.basis_function(x)
        return self.weights.T @ phi

class AdaptiveController:
    def __init__(self, model):
        # 机械臂参数
        self.n_joints = 7
        self.dt = model.opt.timestep

        
        # 初始化RBF网络
        hidden_dim = 20
        self.net_M = RBFNetwork(6, hidden_dim, 36)   # 惯性矩阵M(q) 7x7
        self.net_C = RBFNetwork(12, hidden_dim, 36)    # 科氏力C(q, dq) 7x7 matrix
        self.net_G = RBFNetwork(6, hidden_dim, 6)     # 重力项G(q)
        self.net_F = RBFNetwork(6, hidden_dim, 6)     # 摩擦力F(dq)
        
        # 控制参数
        self.Kv = np.diag([50]*6)        # 阻尼矩阵
        self.Lambda = np.diag([10]*6)    # 滑模面参数
        self.epsilon = 0.01              # 鲁棒项系数
        
        # 自适应律参数
        self.F = 0.1 * np.eye(hidden_dim)
        self.k = 0.01
    
    def update_weights(self, r, q, dq):
        # 更新惯性矩阵网络
        phi_M = self.net_M.basis_function(q)
        # Expand r for M network update
        r_M = np.kron(r, r).flatten()[:self.net_M.output_dim]
        delta_W_M = self.F @ np.outer(phi_M, r_M).reshape(self.net_M.weights.shape) - self.k * np.linalg.norm(r) * self.F @self.net_M.weights
        self.net_M.weights += delta_W_M * self.dt
        
        # 更新科氏力网络
        phi_C = self.net_C.basis_function(np.concatenate([q, dq]))
        # Expand r for C network update
        r_C = np.kron(r, r).flatten()[:self.net_C.output_dim]
        delta_W_C = self.F @ np.outer(phi_C, r_C).reshape(self.net_C.weights.shape) - self.k * np.linalg.norm(r) * self.F @self.net_C.weights
        self.net_C.weights += delta_W_C * self.dt
        
        # 更新重力网络
        phi_G = self.net_G.basis_function(q)
        delta_W_G = self.F @ np.outer(phi_G, r) - self.k * np.linalg.norm(r) * (self.F @ self.net_G.weights[:, :6])
        self.net_G.weights += delta_W_G * self.dt
        
        # 更新摩擦力网络
        phi_F = self.net_F.basis_function(dq)
        delta_W_F = self.F @ np.outer(phi_F, r) - self.k * np.linalg.norm(r) * (self.F @ self.net_F.weights[:, :6])
        self.net_F.weights += delta_W_F * self.dt
        
        # 参数投影
        self._project_weights()
    def _project_weights(self, max_norm=10.0):
        self.net_M.weights = self._norm_clip(self.net_M.weights, max_norm)
        self.net_C.weights = self._norm_clip(self.net_C.weights, max_norm)
        self.net_G.weights = self._norm_clip(self.net_G.weights, max_norm)
        self.net_F.weights = self._norm_clip(self.net_F.weights, max_norm)
    def _norm_clip(self, W, threshold):
        norm = np.linalg.norm(W)
        return W * threshold / norm if norm > threshold else W


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



def main():
    # 加载机械臂模型
    # 初始化系统
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    ik_solver = InverseKinematicsSolver(model=model, damping=0.001, max_iter=100)
    traj_gen = TrajectoryGenerator()
    controller = AdaptiveController(model)

    # 控制参数
    K_v = np.diag([80]*6)
    Lambda = np.diag([15]*6)
    epsilon = 0.02
    dt = model.opt.timestep

    # 初始化数据记录数组
    # 初始化数据记录数组
    time_steps = []
    joint_angles = []
    torques = []
    tracking_errors = []
    target_trajectory = []
    actual_trajectory = []
    f_hat_values = []  # 新增摩擦力估计记录

    # 获取 site ID
    site_id = model.site("eef_trace_site").id
    # 存储轨迹
    trajectory = []

    # 轨迹跟踪主循环
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 初始化时间记录
        start_time = data.time  # 记录初始仿真时间
        duration = 10.0         # 设定10秒时长
        # Initialize with explicit 7-element arrays
        prev_q_d = np.zeros(6)
        q_d = np.zeros(6)
        
        # Verify model configuration
        assert model.nq >= 6, "Model must have at least 6 joints"
        
        # 轨迹跟踪主循环
        while viewer.is_running() and (data.time - start_time) < duration:
            t = data.time
            
            # 1. 生成末端目标轨迹
            target_pos = traj_gen.get_target(t)
            target_trajectory.append(target_pos.copy())
            
            # 2. 逆运动学求解关节目标
            q_d, _ = ik_solver.solve(data, target_pos)
            
            # 3. 计算目标关节速度（数值微分）
            q_d_dot = (q_d - prev_q_d) / dt
            prev_q_d = q_d.copy()
            
            # 4. 获取当前状态
            q = data.qpos[:6].copy()
            q_dot = data.qvel[:6].copy()
            
            # Dimension validation
            assert q.shape == (6,), f"Invalid joint position shape: {q.shape}"
            
            # 5. 计算跟踪误差
            e = q_d - q
            e_dot = q_d_dot - q_dot
            r = e_dot + Lambda @ e
            
            # 6. 计算控制律
            zeta1 = (q_d_dot - q_dot) + Lambda @ e_dot
            zeta2 = q_d_dot + Lambda @ e
            
            # 神经网络近似项
            M_hat = controller.net_M(q).reshape(6,6)
            C_hat = controller.net_C(np.concatenate([q, q_dot])).reshape(6,6)
            G_hat = controller.net_G(q)
            F_hat = controller.net_F(q_dot)
            
            # 鲁棒项
            v = epsilon * np.tanh(10 * r)  # 连续化符号函数
            
            # 控制输入
            tau = (M_hat @ zeta1).flatten() + (C_hat @ zeta2.reshape(-1, 1)).flatten() + G_hat.flatten() + F_hat.flatten() + (K_v @ r).flatten() - v
            tau=np.clip(tau, -50, 50)
            # 7. 应用控制量
            data.ctrl[:6] = tau
            
            # 8. 更新神经网络权重
            controller.update_weights(r, q, q_dot)
            
            # 记录神经网络误差
            # 9. 物理步进
            mujoco.mj_step(model, data)
            viewer.sync()

            # 记录当前数据
            # 在控制循环中记录数据
            time_steps.append(t)
            joint_angles.append(q.copy())
            torques.append(tau.copy())
            tracking_errors.append(target_pos - data.site('eef_trace_site').xpos)
            actual_trajectory.append(data.site('eef_trace_site').xpos.copy())
            f_hat_values.append(controller.net_F(q_dot).flatten().copy())  # 记录摩擦力估计值
            # 10. 轨迹修正（每1秒重置逆运动学）
            if int(t*100) % 100 == 0:
                q_d, _ = ik_solver.solve(data, target_pos)

            # 在仿真过程中每帧绘制轨迹点
            pos = data.site_xpos[site_id].copy()  # 获取 site 的当前坐标
            trajectory.append(pos)
            # 控制拖尾点数量
            max_points = 1000  # naive：1000
            if len(trajectory) > max_points:
                trajectory.pop(0)
            # 清除并重绘用户几何体（红点）
            with viewer.lock():
                viewer.user_scn.ngeom = 0  # 清空上帧所有用户几何体
                for p in trajectory:
                    prepare_red_sphere_geom(viewer.user_scn.geoms[viewer.user_scn.ngeom], p)
                    viewer.user_scn.ngeom += 1
            
        # 主动关闭查看器（可选）
        viewer.close()

    # 仿真结束后绘图
    # 转换跟踪误差为numpy数组
    tracking_errors = np.array(tracking_errors)

    # Plot trajectory tracking results
    plt.figure(figsize=(18, 18))

    # 3D trajectory comparison  三维轨迹对比
    ax1 = plt.subplot(2, 2, 1, projection='3d')
    target_xyz = np.array(target_trajectory)
    actual_xyz = np.array(actual_trajectory)
    ax1.plot(target_xyz[:, 0], target_xyz[:, 1], target_xyz[:, 2], 'b--', label='Target Trajectory')
    ax1.plot(actual_xyz[:, 0], actual_xyz[:, 1], actual_xyz[:, 2], 'r-', label='Actual Trajectory', alpha=0.6)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory Comparison of End-Effector')  # 末端执行器三维轨迹对比
    ax1.legend()

    # End-effector tracking error  末端轨迹跟踪误差
    plt.subplot(2, 2, 2)
    for i in range(3):
        plt.plot(time_steps, tracking_errors[:, i], label=f'Axis {i}')
    plt.title('End-Effector Tracking Error')  # 末端轨迹跟踪误差
    plt.ylabel('Error (m)')  # 误差 (m)
    plt.legend()

    # Joint angle trajectories  关节角度变化
    plt.subplot(2, 2, 3)
    angles = np.array(joint_angles)
    for j in range(6):
        plt.plot(time_steps, angles[:, j], label=f'Joint {j + 1}')
    plt.title('Joint Angle Trajectories')  # 关节角度变化
    plt.ylabel('Angle (rad)')  # 角度 (rad)
    plt.legend()

    # Joint torques  关节力矩
    plt.subplot(2, 2, 4)
    torques_arr = np.array(torques)
    for j in range(6):
        plt.plot(time_steps, torques_arr[:, j], label=f'Joint {j + 1}')
    plt.title('Joint Torques')  # 关节力矩
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (Nm)')  # 力矩 (Nm)
    plt.legend()

    # RBF estimated friction values
    plt.figure(figsize=(16, 16))
    f_hat_arr = np.array(f_hat_values)
    for j in range(6):
        plt.plot(time_steps, f_hat_arr[:, j], label=f'Joint {j + 1}')
    plt.title('RBF Network Friction Estimation')  # RBF网络摩擦力估计值
    plt.xlabel('Time (s)')
    plt.ylabel('Estimated Friction (Nm)')  # 摩擦力估计 (Nm)
    plt.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
