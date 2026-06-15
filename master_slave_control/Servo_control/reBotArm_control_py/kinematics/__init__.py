"""Kinematics 运动学库 — 基于 Pinocchio 的正逆运动学计算。"""

from .robot_model import (
    load_robot_model,
    get_joint_names,
    get_joint_limits,
    get_frame_id,
    get_end_effector_frame_id,
    get_all_frame_names,
    _get_default_urdf_path,
)
from .forward_kinematics import compute_fk, joint_to_pose
from .inverse_kinematics import (
    compute_ik,
    solve_ik_with_retry,
    pos_rot_to_se3,
    IKResult,
    IKSolverParams,
)

__all__ = [
    # robot_model
    "load_robot_model",
    "get_joint_names",
    "get_joint_limits",
    "get_frame_id",
    "get_end_effector_frame_id",
    "get_all_frame_names",
    # 正运动学
    "compute_fk",
    "joint_to_pose",
    # 逆运动学
    "compute_ik",
    "solve_ik_with_retry",
    "pos_rot_to_se3",
    "IKResult",
    "IKSolverParams",
]
