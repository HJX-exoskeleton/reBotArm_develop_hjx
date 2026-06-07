"""reBot-DevArm 机器人模型加载模块。

本模块负责从 URDF 文件加载 Pinocchio 刚体模型，
并提供关节信息查询工具。
机器人构型：6 个旋转关节（joint1–joint6）+ 1 个固定末端关节（end_joint）。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List, Tuple

import numpy as np
import pinocchio as pin


# --------------------------------------------------------------------------- #
# 路径工具
# --------------------------------------------------------------------------- #

def _get_default_urdf_path() -> str:
    """返回内置 reBot-DevArm URDF 文件的绝对路径，并确保包名符号链接存在。"""
    base = Path(__file__).resolve().parents[2] / "urdf"
    urdf_pkg_in_urdf = "reBot-DevArm_description_fixend"
    link_path = base / urdf_pkg_in_urdf
    real_path = base / "reBot-DevArm_fixend_description"
    if not link_path.exists() and real_path.exists():
        os.symlink(real_path.name, str(link_path))
    return str(base / "reBot-DevArm_fixend_description" / "urdf" / "reBot-DevArm_fixend.urdf")


# --------------------------------------------------------------------------- #
# 模型加载
# --------------------------------------------------------------------------- #

def load_robot_model(
    urdf_path: str | None = None,
) -> pin.Model:
    """从 reBot-DevArm URDF 构建 Pinocchio 模型。

    参数:
        urdf_path: URDF 文件的绝对或相对路径。
                   默认为内置 URDF 文件。

    返回:
        pin.Model，包含 6 个旋转关节（nq=6, nv=6）。
        固定 ``end_joint`` 被当作操作空间帧处理，不占用位形变量。
    """
    if urdf_path is None:
        urdf_path = _get_default_urdf_path()

    # 相对路径以工作目录为基准解析。
    if not os.path.isabs(urdf_path):
        urdf_path = str(Path.cwd() / urdf_path)

    model = pin.buildModelFromUrdf(urdf_path)
    return model


# --------------------------------------------------------------------------- #
# 查询工具
# --------------------------------------------------------------------------- #

def get_joint_names(model: pin.Model) -> List[str]:
    """返回所有非固定关节的名称列表（跳过 ``universe``）。"""
    return [
        name
        for name, jtype in zip(model.names[1:], model.joints[1:])
        if jtype.idx_q >= 0  # idx_q < 0 表示固定关节
    ]


def get_joint_limits(model: pin.Model) -> List[Tuple[float, float]]:
    """返回每个非固定关节的位置限位 (下限, 上限)。

    URDF 中无限位定义的连续旋转关节返回 ``(-inf, inf)``。
    """
    names = get_joint_names(model)
    limits = []
    for name in names:
        joint_id = model.getJointId(name)
        lo = float(model.lowerPositionLimit[joint_id])
        hi = float(model.upperPositionLimit[joint_id])
        if np.isinf(lo) and np.isinf(hi):
            limits.append((-np.inf, np.inf))
        else:
            limits.append((lo, hi))
    return limits


def get_frame_id(model: pin.Model, frame_name: str) -> int:
    """返回指定名称帧的索引。"""
    return model.getFrameId(frame_name)


def get_end_effector_frame_id(model: pin.Model) -> int:
    """返回末端操作帧 ``end_link`` 的索引。"""
    return model.getFrameId("end_link")


def get_all_frame_names(model: pin.Model) -> List[str]:
    """返回模型中所有已注册帧的名称。"""
    return [f.name for f in model.frames]
