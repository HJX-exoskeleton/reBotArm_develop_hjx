import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from matplotlib import rcParams
from pathlib import Path
import sys

# 中文字体设置
# rcParams['font.sans-serif'] = ['SimHei']
# rcParams['axes.unicode_minus'] = False

# 系统参数配置
ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR))

SIM_DURATION = 10.0  # 仿真总时长(s)
CTRL_FREQ = 500  # 控制频率(Hz)
LAMBDA = 15.0  # 滑模面系数
Kd = np.diag([500] * 6)  # 滑模控制增益矩阵


def inverse_kinematics(model, data, target_pos, target_vel=None, dt=0.001, max_iter=100):
    """带速度解的改进逆运动学"""
    q = data.qpos[:6].copy()
    ee_id = model.body("gripper_eef_trace_site").id  # naive: right_finger_link

    for _ in range(max_iter):
        mujoco.mj_forward(model, data)
        current_pos = data.body(ee_id).xpos
        err = target_pos - current_pos

        # 计算雅可比矩阵
        J = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, J, None, ee_id)
        J = J[:, :6]

        # 阻尼伪逆解
        damping = 0.001
        Jinv = np.linalg.pinv(J.T @ J + damping ** 2 * np.eye(6)) @ J.T
        delta_q = Jinv @ err
        q += delta_q * dt

        # 关节限位保护
        q = np.clip(q, model.jnt_range[:6, 0], model.jnt_range[:6, 1])
        data.qpos[:6] = q

        if np.linalg.norm(err) < 1e-4:
            break

    # 速度计算
    if target_vel is not None and J is not None:
        qd = Jinv @ target_vel
    else:
        qd = np.zeros(6)

    return q.copy(), qd.copy()


def sliding_mode_controller(model, data, qd, qd_dot, qd_ddot):
    """改进的滑模控制器"""
    nv = model.nv

    # 当前状态
    q = data.qpos[:6].copy()
    q_dot = data.qvel[:6].copy()

    # 误差计算
    q_tilde = q - qd
    q_tilde_dot = q_dot - qd_dot

    # 参考轨迹
    qr_dot = qd_dot - LAMBDA * q_tilde
    qr_ddot = qd_ddot - LAMBDA * q_tilde_dot

    # 滑模面
    s = q_tilde_dot + LAMBDA * q_tilde

    # 获取动力学参数
    H = np.zeros((nv, nv), dtype=np.float64)
    mujoco.mj_fullM(model, H, data.qM)
    H_joints = H[:6, :6]

    # 计算科里奥利力
    C = np.zeros(nv)
    mujoco.mj_rnePostConstraint(model, data)
    mujoco.mj_rne(model, data, 1, C)

    # 计算重力项
    data.qacc[:] = 0
    data.qfrc_bias[:] = 0
    mujoco.mj_rne(model, data, 0, data.qfrc_bias)
    G = data.qfrc_bias[:6]

    # 控制律
    tau = H_joints @ qr_ddot + C[:6] + G - Kd @ s
    return np.clip(tau, -300, 300)


# 圆形轨迹
def circular_trajectory(t, radius=0.2, freq=0.5):
    """生成带速度加速度的轨迹"""
    omega = 2 * np.pi * freq
    theta = omega * t
    pos = np.array([
        radius * np.cos(theta) + 0.3,
        radius * np.sin(theta) - 0.2,
        0.3  # 固定高度
    ])
    vel = np.array([
        -radius * omega * np.sin(theta),
        radius * omega * np.cos(theta),
        0
    ])
    acc = np.array([
        -radius * omega ** 2 * np.cos(theta),
        -radius * omega ** 2 * np.sin(theta),
        0
    ])
    return pos, vel, acc


# 三角形轨迹
def triangle_trajectory(t, side_length=0.3, freq=0.1):
    """
    Generate a 3D triangle trajectory (equilateral triangle in XY plane).

    Parameters:
        t (float): Current time in seconds
        side_length (float): Length of each triangle side (m)
        freq (float): Number of triangle cycles per second (Hz)

    Returns:
        pos (np.ndarray): 3D position [x, y, z]
        vel (np.ndarray): 3D velocity [vx, vy, vz]
        acc (np.ndarray): 3D acceleration [ax, ay, az] (set to zero for simplicity)
    """
    # Triangle has 3 segments, compute which segment we're in
    T = 1.0 / freq  # period of one full triangle
    t_mod = t % T  # wrap around to period
    segment_time = T / 3  # duration of each side
    segment = int(t_mod / segment_time)
    tau = (t_mod % segment_time) / segment_time  # normalized progress [0, 1]
    # Define triangle corners (equilateral triangle in XY)
    h = np.sqrt(3) / 2 * side_length
    p1 = np.array([0.0, 0.0, 0.6])
    p2 = np.array([side_length, 0.0, 0.6])
    p3 = np.array([0.5 * side_length, h, 0.6])
    # Pick segment
    if segment == 0:
        start, end = p1, p2
    elif segment == 1:
        start, end = p2, p3
    else:
        start, end = p3, p1
    # Linear interpolation
    pos = (1 - tau) * start + tau * end
    vel = (end - start) / segment_time
    acc = np.zeros(3)  # constant velocity, zero acceleration
    return pos, vel, acc


# 正方形轨迹
def square_trajectory(t, side_length=0.3, freq=0.1):
    """
    Generate a square trajectory in XY plane (Z fixed).
    """
    T = 1.0 / freq
    t_mod = t % T
    segment_time = T / 4
    segment = int(t_mod / segment_time)
    tau = (t_mod % segment_time) / segment_time
    # Define corners (square, counter-clockwise)
    z = 0.6
    p1 = np.array([0.0, 0.0, z])
    p2 = np.array([side_length, 0.0, z])
    p3 = np.array([side_length, side_length, z])
    p4 = np.array([0.0, side_length, z])
    points = [p1, p2, p3, p4]
    start = points[segment]
    end = points[(segment + 1) % 4]
    pos = (1 - tau) * start + tau * end
    vel = (end - start) / segment_time
    acc = np.zeros(3)
    return pos, vel, acc


# 螺旋轨迹
def spiral_trajectory(t, r_max=0.3, height=0.2, num_turns=2, duration=5.0):
    """
    Generate a 3D spiral trajectory in cylinder shape.
    - r_max: maximum radius
    - height: vertical span
    - num_turns: number of full revolutions over duration
    - duration: total time to complete spiral
    """
    tau = (t % duration) / duration  # normalized time
    theta = 2 * np.pi * num_turns * tau
    r = r_max * tau
    z = 0.2 + height * tau
    pos = np.array([
        r * np.cos(theta) + 0.3,
        r * np.sin(theta) - 0.2,
        z
    ])
    omega = 2 * np.pi * num_turns / duration
    r_dot = r_max / duration
    vel = np.array([
        r_dot * np.cos(theta) - r * omega * np.sin(theta),
        r_dot * np.sin(theta) + r * omega * np.cos(theta),
        height / duration
    ])
    acc = np.zeros(3)  # optional: can compute analytically if needed
    return pos, vel, acc


# 星形轨迹
def star_trajectory(t, radius=0.2, freq=0.1, points=5):
    """
    Generate a 2D star-shaped trajectory (like a 5-pointed star).
    """
    # Star path by radius modulation: R(θ) = R0 * cos(kθ)
    omega = 2 * np.pi * freq
    theta = omega * t
    k = points  # Number of star points
    R = radius * np.cos(k * theta)
    x = R * np.cos(theta)
    y = R * np.sin(theta)
    pos = np.array([x + 0.3, y - 0.2, 0.4])
    # Derivatives (velocity) approximated numerically for simplicity
    dt = 1e-4
    theta_dt = omega * (t + dt)
    R_dt = radius * np.cos(k * theta_dt)
    x_dt = R_dt * np.cos(theta_dt)
    y_dt = R_dt * np.sin(theta_dt)
    vel = (np.array([x_dt, y_dt]) - np.array([x, y])) / dt
    vel = np.array([vel[0], vel[1], 0])
    acc = np.zeros(3)  # optional
    return pos, vel, acc


# 心形轨迹（2D）
def heart_trajectory(t, scale=0.01, freq=0.5):
    """
    Generate a 3D heart-shaped trajectory with velocity and acceleration.
    - scale: size of the heart
    - freq: how fast to trace the shape (Hz)
    """
    omega = 2 * np.pi * freq
    theta = omega * t
    # 基本爱心轨迹（单位心形）
    x = 16 * np.sin(theta)**3
    y = 13 * np.cos(theta) - 5 * np.cos(2*theta) - 2 * np.cos(3*theta) - np.cos(4*theta)
    # 一阶导数（速度）
    dx = 48 * np.sin(theta)**2 * np.cos(theta)
    dy = (-13 * np.sin(theta) +
          10 * np.sin(2*theta) +
          6 * np.sin(3*theta) +
          4 * np.sin(4*theta))
    # 二阶导数（加速度）
    ddx = 48 * (2 * np.sin(theta) * np.cos(theta)**2 - np.sin(theta)**3)
    ddy = (-13 * np.cos(theta) +
           20 * np.cos(2*theta) +
           18 * np.cos(3*theta) +
           16 * np.cos(4*theta))
    # 缩放 + 平移 + 添加高度
    pos = np.array([
        scale * x + 0.3,     # X 平移至 0.3
        scale * y - 0.2,     # Y 平移至 -0.2
        0.520                  # 固定高度
    ])
    vel = np.array([
        scale * omega * dx,
        scale * omega * dy,
        0
    ])
    acc = np.array([
        scale * omega**2 * ddx,
        scale * omega**2 * ddy,
        0
    ])
    return pos, vel, acc


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


# 初始化模型
XML_PATH = str(ROOT_DIR / "xml" / "rebot_gripper" / "reBot-DevArm_gripper.xml")
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)
model.opt.timestep = 1 / CTRL_FREQ
mujoco.mj_resetDataKeyframe(model, data, 0)

# 获取 site ID
site_id = model.site("eef_trace_site").id
# 存储轨迹
trajectory = []

# 数据记录
num_steps = round(SIM_DURATION * CTRL_FREQ)
time_axis = np.linspace(0, SIM_DURATION, num_steps)

log = {
    'target_pos': np.zeros((num_steps, 3)),
    'actual_pos': np.zeros((num_steps, 3)),
    'tau': np.zeros((num_steps, 6)),
    'q_error': np.zeros((num_steps, 6))
}

# 主控制循环
step = 0
prev_qd_dot = np.zeros(6)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 直接使用步数作为循环条件
    while step < num_steps and viewer.is_running():
        # 生成目标轨迹
        t = step / CTRL_FREQ
        # target_pos, target_vel, target_acc = circular_trajectory(t)  # 圆形轨迹
        # target_pos, target_vel, target_acc = triangle_trajectory(t)  # 三角形轨迹
        # target_pos, target_vel, target_acc = square_trajectory(t)  # 正方形轨迹
        # target_pos, target_vel, target_acc = spiral_trajectory(t)  # 螺旋轨迹
        # target_pos, target_vel, target_acc = star_trajectory(t)  # 星形轨迹
        target_pos, target_vel, target_acc = heart_trajectory(t)  # 心形轨迹
        log['target_pos'][step] = target_pos

        # 逆运动学求解
        qd, qd_dot = inverse_kinematics(model, data, target_pos, target_vel)

        # 计算关节加速度
        if step == 0:
            qd_ddot = np.zeros(6)
        else:
            qd_ddot = (qd_dot - prev_qd_dot) * CTRL_FREQ
        prev_qd_dot = qd_dot.copy()

        # 计算控制力矩
        tau = sliding_mode_controller(model, data, qd, qd_dot, qd_ddot)

        # 应用控制
        data.ctrl[:6] = tau

        # 记录数据
        log['actual_pos'][step] = data.body("gripper_eef_trace_site").xpos  # naive: right_finger_link
        log['tau'][step] = tau
        log['q_error'][step] = data.qpos[:6] - qd

        # 仿真步进
        mujoco.mj_step(model, data)
        viewer.sync()
        step += 1

        # 在仿真过程中每帧绘制轨迹点
        pos = data.site_xpos[site_id].copy()  # 获取 site 的当前坐标
        trajectory.append(pos)
        # 控制拖尾点数量
        max_points = 1000
        if len(trajectory) > max_points:
            trajectory.pop(0)
        # 清除并重绘用户几何体（红点）
        with viewer.lock():
            viewer.user_scn.ngeom = 0  # 清空上帧所有用户几何体
            for p in trajectory:
                prepare_red_sphere_geom(viewer.user_scn.geoms[viewer.user_scn.ngeom], p)
                viewer.user_scn.ngeom += 1

    # 自动关闭前同步最后一次状态
    viewer.sync()


# 可视化结果
plt.figure(figsize=(14, 10))

# 三维轨迹跟踪
ax1 = plt.subplot(2, 2, 1, projection='3d')
ax1.plot(log['target_pos'][:, 0], log['target_pos'][:, 1], log['target_pos'][:, 2],
         'r--', label='Target Trajectory')
ax1.plot(log['actual_pos'][:, 0], log['actual_pos'][:, 1], log['actual_pos'][:, 2],
         'b-', alpha=0.5, label='Actual Trajectory')
ax1.set_title('3D Trajectory Tracking')
ax1.legend()

# 位置误差
ax2 = plt.subplot(2, 2, 2)
pos_error = np.linalg.norm(log['actual_pos'] - log['target_pos'], axis=1)
ax2.plot(time_axis, pos_error * 1000)
ax2.set_title('End-effector Position Tracking Error')
ax2.set_ylabel('Error (mm)')

# 控制力矩
ax3 = plt.subplot(2, 2, 3)
for i in range(6):
    ax3.plot(time_axis, log['tau'][:, i], label=f'Joint {i + 1}')
ax3.set_title('Joint Torque Commands')
ax3.set_ylabel('Torque (N·m)')
ax3.legend()

# 关节角度误差
ax4 = plt.subplot(2, 2, 4)
for i in range(6):
    ax4.plot(time_axis, np.degrees(log['q_error'][:, i]), label=f'Joint {i + 1}')
ax4.set_title('Joint Angle Tracking Error')
ax4.set_ylabel('Error (°)')

# 添加图例（调整到最佳显示位置）
ax4.legend(
    loc='upper right',  # 定位在右上角
    ncol=2,  # 分2列显示
    fontsize=8,  # 缩小字体
    framealpha=0.5  # 半透明背景
)

plt.tight_layout()
plt.show()

