"""ArmEndPos — 末端位置控制器（IK  + 轨迹规划二合一）。

统一的 posvel 控制器，同时支持两种运动模式：

  - ``move_to_ik(...)``   即时 IK 求解，关节角度一步到位（无轨迹平滑）。
  - ``move_to_traj(...)`` SE(3) 测地线规划 + CLIK 跟踪，末端沿平滑轨迹运动。

使用示例::

    arm = RobotArm()
    Arm_endpos_control = ArmEndPos(arm)
    Arm_endpos_control.start()

    # 即时 IK（适合近距离小幅运动）
    Arm_endpos_control.move_to_ik(x=0.3, y=0.0, z=0.3)

    # 带轨迹规划（平滑、可控时长）
    Arm_endpos_control.move_to_traj(x=0.3, y=0.0, z=0.3,
                        roll=0, pitch=0.4, yaw=0,
                        duration=2.0)

    Arm_endpos_control.end()

上下文管理器::

    with ArmEndPos(arm) as endpos:
        endpos.move_to_ik(x=0.3, y=0.0, z=0.3)
"""

from __future__ import annotations

import threading
import time

import numpy as np

from ..kinematics import (
    compute_fk,
    pos_rot_to_se3,
    get_end_effector_frame_id,
    load_robot_model,
)
from ..kinematics.inverse_kinematics import (
    solve_ik,
    IKParams as TrajIKParams,
)
from ..trajectory import (
    TrajProfile,
    TrajPlanParams,
    IKParams as ClikIKParams,
    plan_cartesian_geodesic_trajectory,
    track_trajectory,
)
from ..actuator import RobotArm


class ArmEndPos:

    def __init__(
        self,
        arm: RobotArm,
        dt: float = 0.02,
        profile: TrajProfile = TrajProfile.MIN_JERK,
    ) -> None:
        """初始化控制器。

        参数:
            arm:     RobotArm 实例。
            dt:      控制循环周期（秒），默认 20 ms。
            profile: 轨迹时间轮廓类型，默认 MIN_JERK（最小 jerk）。
        """
        self.arm = arm
        self._n = arm.num_joints
        self._dt = dt
        self._model = load_robot_model()
        self._end_frame_id = get_end_effector_frame_id(self._model)
        self._data = self._model.createData()

        self._pv_vlim = np.array([j.vlim for j in arm._joints], dtype=np.float64)

        self._traj_params = TrajPlanParams(dt=dt, profile=profile)
        self._ik_solver_params = TrajIKParams(
            max_iter=200, tolerance=1e-4, step_size=0.5, damping=1e-6,
        )
        self._clik_params = ClikIKParams(
            max_iter=200, tolerance=1e-4, damping=1e-6, step_size=0.8,
        )

        self._q_target = np.zeros(self._n)
        self._running = False

        # ── 轨迹规划状态（仅 move_to_traj 使用） ──────────────────────────
        self._traj: list[np.ndarray] = []
        self._traj_idx = 0
        self._moving = False
        self._send_thread: threading.Thread | None = None
        self._stop_send = threading.Event()

        # ── 回零安全参数 ──────────────────────────────────────────────────
        self._home_vel: float = 0.3   # rad/s，回零限速
        self._vlim_override: np.ndarray | None = None   # safe_home 期间生效

    # ── 生命周期 ───────────────────────────────────────────────────────────

    def start(self) -> None:
        """连接、切换模式、使能、启动控制循环。"""
        self.arm.connect()
        self.arm.mode_pos_vel()
        self.arm.enable()
        self.arm.start_control_loop(self._loop_cb)
        self._running = True

    def end(self) -> None:
        """安全回零后再断开连接。"""
        if not self._running:
            return
        self.safe_home()
        self.arm.disconnect()
        self._running = False

    def __enter__(self) -> "ArmEndPos":
        return self

    def __exit__(self, *args) -> None:
        self.end()

    # ── 公共 API ───────────────────────────────────────────────────────────

    def safe_home(self, vlim: float | None = None) -> None:
        """驱动机械臂以安全速度返回零位。"""
        if not self._running:
            return
        v = self._home_vel if vlim is None else float(vlim)
        self._vlim_override = np.full(self._n, v, dtype=np.float64)
        self._q_target[:] = 0.0
        self._stop_send.set()
        if self._send_thread is not None:
            self._send_thread.join()

        deadline = time.monotonic() + 30.0
        while True:
            q, _, _ = self.arm.get_state()
            if np.max(np.abs(q)) < 0.01:
                break
            if time.monotonic() > deadline:
                print("[ArmEndPos] safe_home 超时")
                break
            time.sleep(self._dt)
        self._vlim_override = None

    def move_to_ik(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> bool:
        """IK 求解并驱动机械臂移动到目标位姿（无轨迹平滑）。

        参数:
            x, y, z:        目标末端位置（米）。
            roll, pitch, yaw: 目标姿态欧拉角（弧度），默认零姿态。

        返回:
            IK 求解成功返回 ``True``，否则 ``False``。
        """
        if not self._running:
            return False

        q_curr, _, _ = self.arm.get_state()
        T_target = pos_rot_to_se3(np.array([x, y, z]), roll=roll, pitch=pitch, yaw=yaw)

        result = solve_ik(
            self._model, self._data, self._end_frame_id,
            T_target, q_curr, self._ik_solver_params,
        )
        if not result.success:
            print(f"[ArmEndPos/IK] IK 未收敛  err={result.error:.3e}")
            return False

        self._q_target = result.q.copy()
        return True

    def move_to_traj(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        duration: float = 2.0,
    ) -> bool:
        """SE(3) 测地线规划 + CLIK 跟踪，驱动机械臂沿平滑轨迹运动。

        参数:
            x, y, z:        目标末端位置（米）。
            roll, pitch, yaw: 目标姿态欧拉角（弧度），默认零姿态。
            duration:       运动时长（秒）。若 ``<= 0`` 则根据末端移动距离自动估算。

        返回:
            规划与求解成功返回 ``True``，否则 ``False``。
        """
        if not self._running:
            return False

        q_start, _, _ = self.arm.get_state()

        T_target = pos_rot_to_se3(
            np.array([x, y, z]), roll=roll, pitch=pitch, yaw=yaw,
        )

        ik_result = solve_ik(
            self._model, self._data, self._end_frame_id,
            T_target, q_start, self._ik_solver_params,
        )
        if not ik_result.success:
            print(f"[ArmEndPos/Traj] IK 失败  err={ik_result.error:.4f}")
            return False

        q_end = ik_result.q

        T_start = compute_fk(self._model, q_start)[2]
        T_end = compute_fk(self._model, q_end)[2]

        if duration <= 0:
            dist = float(np.linalg.norm(T_target.translation() - T_start.translation()))
            duration = max(1.0, dist / 0.1)

        cart_traj = plan_cartesian_geodesic_trajectory(
            T_start, T_end, duration, self._traj_params,
        )

        joint_traj = track_trajectory(
            self._model, self._end_frame_id,
            cart_traj.trajectory, q_start, self._clik_params,
            null_gain=0.1,
        )
        if not joint_traj:
            print("[ArmEndPos/Traj] 轨迹为空")
            return False

        pts = [pt.q.copy() for pt in joint_traj]

        self._stop_send.set()
        if self._send_thread is not None:
            self._send_thread.join()

        self._traj = pts
        self._traj_idx = 0
        self._moving = True
        self._stop_send.clear()
        self._send_thread = threading.Thread(
            target=self._send_loop, args=(duration,), daemon=True,
        )
        self._send_thread.start()
        return True

    # ── 控制循环 ───────────────────────────────────────────────────────────

    def _loop_cb(self, _: RobotArm, dt: float) -> None:
        vlim = self._vlim_override if self._vlim_override is not None else self._pv_vlim
        self.arm.pos_vel(self._q_target, vlim=vlim)

    # ── 轨迹发送线程 ──────────────────────────────────────────────────────

    def _send_loop(self, duration: float) -> None:
        n = len(self._traj)
        interval = duration / n if n > 0 else self._dt
        for i in range(n):
            if self._stop_send.is_set():
                return
            self._q_target[:] = self._traj[i]
            time.sleep(interval)
        self._moving = False
