import sys
# sys.path.append(r'c:\xarm7')
import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
from ik_solver import InverseKinematicsSolver

from pathlib import Path
import sys

ROOT_DIR = Path(__file__).resolve().parents[1]
XML_PATH = str(ROOT_DIR / "xml" / "rebot_gripper" / "reBot-DevArm_gripper.xml")


class TrajectoryGenerator:
    def __init__(self, dt=0.002):
        self.radius = 0.2
        self.freq = 0.5
        self.center = np.array([0.3, -0.2, 0.5])
        self.z_amp = 0.05
        self.t = 0.0
        self.dt = dt
        
    def get_trajectory(self):
        self.t += self.dt
        theta = 2*np.pi*self.freq*self.t
        
        pos = self.center + [
            self.radius*np.cos(theta),
            self.radius*np.sin(theta),
            self.z_amp*np.sin(4*np.pi*self.freq*self.t)
        ]
        
        vel = [
            -2*np.pi*self.freq*self.radius*np.sin(theta),
            2*np.pi*self.freq*self.radius*np.cos(theta),
            4*np.pi*self.freq*self.z_amp*np.cos(4*np.pi*self.freq*self.t)
        ]
        
        acc = [
            -(2*np.pi*self.freq)**2*self.radius*np.cos(theta),
            -(2*np.pi*self.freq)**2*self.radius*np.sin(theta),
            -(4*np.pi*self.freq)**2*self.z_amp*np.sin(4*np.pi*self.freq*self.t)
        ]
        
        return np.array(pos), np.array(vel), np.array(acc)


class HeartTrajectoryGenerator:
    def __init__(self, dt=0.002):
        self.freq = 0.2  # 频率控制心形的速度
        self.center = np.array([0.3, -0.2, 0.5])  # 心形轨迹中心
        self.z_amp = 0.02  # Z方向振幅
        self.t = 0.0
        self.dt = dt

    def get_trajectory(self):
        self.t += self.dt
        theta = 2 * np.pi * self.freq * self.t

        # 心形轨迹在XY平面
        x = 16 * np.sin(theta)**3
        y = 13 * np.cos(theta) - 5 * np.cos(2 * theta) - 2 * np.cos(3 * theta) - np.cos(4 * theta)

        # 缩放并平移心形图案
        x = 0.01 * x + self.center[0]
        y = 0.01 * y + self.center[1]
        z = self.center[2] + self.z_amp * np.sin(4 * np.pi * self.freq * self.t)

        pos = np.array([x, y, z])

        # 一阶导数（速度）
        dtheta = 2 * np.pi * self.freq
        dx = 16 * 3 * np.sin(theta)**2 * np.cos(theta) * dtheta * 0.01
        dy = (-13 * np.sin(theta)
              + 10 * np.sin(2 * theta)
              + 6 * np.sin(3 * theta)
              + 4 * np.sin(4 * theta)) * dtheta * 0.01
        dz = 4 * np.pi * self.freq * self.z_amp * np.cos(4 * np.pi * self.freq * self.t)

        vel = np.array([dx, dy, dz])

        # 二阶导数（加速度）
        ddx = 16 * 3 * (2 * np.sin(theta) * np.cos(theta)**2 - np.sin(theta)**3) * (dtheta**2) * 0.01
        ddy = (-13 * np.cos(theta)
               + 20 * np.cos(2 * theta)
               + 18 * np.cos(3 * theta)
               + 16 * np.cos(4 * theta)) * (dtheta**2) * 0.01
        ddz = -(4 * np.pi * self.freq)**2 * self.z_amp * np.sin(4 * np.pi * self.freq * self.t)

        acc = np.array([ddx, ddy, ddz])

        return pos, vel, acc


class SMCController:
    def __init__(self, n_joints=6, dt=0.002):
        self.n = n_joints
        self.c = 3.0*np.ones(n_joints)  # 增大滑模面系数加快收敛速度
        self.eta = 15.0*np.ones(n_joints)  # 增大切换增益抑制抖振
        self.gamma = 0.02
        self.phi = 0.05
        self.dt = dt
        
        # RBF参数
        self.n_centers = 25  # 增加RBF中心数量提升函数逼近能力
        self.centers = np.stack([
            np.column_stack([
                np.linspace(-np.pi, np.pi, self.n_centers),
                np.linspace(-5, 5, self.n_centers)
            ]) for _ in range(n_joints)
        ])
        self.widths = 0.5*np.ones(self.n_centers)
        self.W = np.zeros((n_joints, self.n_centers))

    def rbf(self, x, j):
        return np.exp(-np.linalg.norm(x - self.centers[j], axis=1)**2 / (2 * self.widths**2))
    
    def control_law(self, q, dq, q_d, dq_d, ddq_d):
        if not isinstance(dq_d, np.ndarray):
            dq_d = np.array(dq_d)
        if not isinstance(ddq_d, np.ndarray):
            ddq_d = np.array(ddq_d)
        assert dq_d.shape == (6,) and ddq_d.shape == (6,)  # 添加维度校验断言
        e = q - q_d
        edot = dq - dq_d
        s = self.c*e + edot
        
        u = np.zeros(self.n)
        f_hat_values = np.zeros(self.n)
        for j in range(self.n):
            h = self.rbf(np.array([q[j], dq[j]]), j)
            f_hat = self.W[j] @ h
            f_hat_values[j] = f_hat
            
            # 带边界层的符号函数
            sat = np.sign(s[j])
            
            u[j] = -self.c[j]*edot[j] - f_hat + ddq_d[j] - self.eta[j]*sat
            self.W[j] += self.gamma * s[j] * h * self.dt
            
        return np.clip(u, -50, 50), f_hat_values, s


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


# 初始化系统
model = mujoco.MjModel.from_xml_path(XML_PATH)
if model is None:
    raise ValueError('模型文件加载失败，请检查scene.xml路径是否正确')
print(f"模型加载成功，物体数量: {model.nbody}，关节数量: {model.njnt}")  # 添加调试信息
data = mujoco.MjData(model)
if data is None:
    raise RuntimeError('数据初始化失败')
print("数据初始化成功")  # 添加调试信息

controller = SMCController(dt=model.opt.timestep)
traj_gen = TrajectoryGenerator(model.opt.timestep)
# traj_gen = HeartTrajectoryGenerator(model.opt.timestep)
ik_solver = InverseKinematicsSolver(model)

# 初始位置设置
data.qpos[:6] = ik_solver.solve(data, traj_gen.center)[0]
data.qpos[6:] = 0
mujoco.mj_forward(model, data)

# 数据记录
log = {
    'time': [], 'target': [], 'actual': [],
    'error': [], 'control': [], 'joints': [],
    's': [], 'f_hat': [], 'tau_saturation': []
}

# 获取 site ID
site_id = model.site("eef_trace_site").id
# 存储轨迹
trajectory = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 调试：打印模型和数据状态
    print(f"模型加载状态：{model is not None}")
    print(f"数据初始化状态：{data is not None}")
    # 精确计时器配置
    SIM_DURATION = 10.0  # 精确5秒
    num_steps = int(SIM_DURATION / model.opt.timestep)
    time_compensation = 0.0
    step = 0

    # 严格5秒终止条件（允许1个时间步误差）
    while step < num_steps and viewer.is_running():
        # 轨迹生成
        target_pos, target_vel, target_acc = traj_gen.get_trajectory()
        
        # 逆运动学
        q_d = ik_solver.solve(data, target_pos)[0]
        
        # 使用轨迹生成器提供的真实导数
        jac_pos = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jac_pos, None, model.site(ik_solver.ee_site_name).id)
        J = jac_pos[:, :6]
        dq_d = np.linalg.pinv(J) @ target_vel.reshape(3,1)  # 将 target_vel 转换为列向量
        dq_d = dq_d.flatten()
        assert isinstance(dq_d, np.ndarray)
        ddq_d = np.linalg.pinv(J) @ (target_acc - J @ dq_d)
        ddq_d = ddq_d.flatten()
        assert isinstance(ddq_d, np.ndarray)
        
        # 控制计算
        u, f_hat_values, s_values = controller.control_law(data.qpos[:6], data.qvel[:6], q_d, dq_d.copy(), ddq_d.copy())
        data.ctrl[:6] = u
        
        # 记录数据
        log['time'].append(data.time)
        log['target'].append(target_pos.copy())
        log['actual'].append(data.site(model.site(ik_solver.ee_site_name).id).xpos.copy())
        log['error'].append(np.linalg.norm(log['target'][-1]-log['actual'][-1]))
        log['control'].append(u.copy())
        log['joints'].append(data.qpos[:6].copy())
        log['s'].append(s_values)
        log['f_hat'].append(f_hat_values.copy())
        log['tau_saturation'].append(np.sum(np.abs(u) >= 49.9))
        
        # 仿真步进
        mujoco.mj_step(model, data)
        step += 1

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

        viewer.sync()



# Visualization analysis
plt.figure(figsize=(16,12))

# Trajectory tracking  轨迹跟踪
ax1 = plt.subplot(3,2,1, projection='3d')
target = np.array(log['target'])
actual = np.array(log['actual'])
ax1.plot(target[:,0], target[:,1], target[:,2], 'r--', label='Desired trajectory')
ax1.plot(actual[:,0], actual[:,1], actual[:,2], 'b-', label='Actual trajectory')
ax1.set_title('End-effector Trajectory Tracking')  # 末端执行器轨迹跟踪
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.legend()

# Tracking error  跟踪误差
ax2 = plt.subplot(3,2,2)
ax2.plot(log['time'], np.array(log['error'])*1000, 'g-')
ax2.set_title('Position Tracking Error')  # 位置跟踪误差
ax2.set_xlabel('Time (s)')  # 时间(s)
ax2.set_ylabel('Error (m)')  # 误差 (m)

# Control input  控制输入
ax3 = plt.subplot(3,2,3)
ctrl = np.array(log['control'])
for j in range(6):
    ax3.plot(log['time'], ctrl[:,j], label=f'Joint {j+1}')
ax3.set_title('Control Torque')  # 控制力矩
ax3.set_xlabel('Time (s)')  # 时间(s)
ax3.set_ylabel('Torque (N·m)')  # 控制力矩 (N·m)
ax3.legend()

# Joint angles  关节角度
ax4 = plt.subplot(3,2,4)
q = np.array(log['joints'])
t = np.array(log['time'])
for j in range(6):
    ax4.plot(t, q[:,j], label=f'Joint {j+1}')
ax4.set_title('Joint Angle Variation')  # 关节角度变化
ax4.set_xlabel('Time (s)')  # 时间(s)
ax4.set_ylabel('Joint Angle (rad)')  # 关节角度 (rad)
ax4.legend()

# Sliding surface  滑模面
ax5 = plt.subplot(3,2,5)
s = np.array(log['s'])
ax5.plot(t, np.linalg.norm(s, axis=1), 'k-')
ax5.set_title('Sliding Surface s')  # 滑模面s
ax5.set_xlabel('Time (s)')  # 时间(s)
ax5.set_ylabel('Norm of s (unitless)')  # 滑模面范数 (无量纲)

# RBF estimation vs. actual  RBF估计误差与真实误差
ax6 = plt.subplot(3,2,6)
f_hat = np.array(log['f_hat'])
ax6.plot(t, np.linalg.norm(f_hat, axis=1), 'r-', label='Estimated value')
ax6.set_title('RBF Estimation')  # RBF估计值
ax6.set_xlabel('Time (s)')  # 时间(s)
ax6.set_ylabel('Estimated Value')  # 估计值

plt.tight_layout()
plt.show()
