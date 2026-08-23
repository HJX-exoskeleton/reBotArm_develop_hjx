#!/usr/bin/env python3
"""ROS-free physical-contact pick-and-place demo for the reBot B601.

The arm path uses damped-least-squares Jacobian IK at ``grasp_center``. The
yellow cylinder is lifted only by bilateral finger contact and MuJoCo friction;
the script never attaches the object or writes its simulated pose.
"""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
import time

import mujoco
import numpy as np

from rebotarm_control_demo import ARM_JOINT_NAMES, RebotArmSimulation, default_model_path

GRASP_SITE_NAME = "grasp_center"


@dataclass(frozen=True)
class MotionPhase:
    name: str
    arm_target: np.ndarray
    gripper_target: float
    duration: float


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


class JacobianIK:
    """Pose IK solver adapted from the original simulation task server."""

    def __init__(self, simulation: RebotArmSimulation) -> None:
        self.model = simulation.model
        self.data = mujoco.MjData(self.model)
        self.grasp_site_id = self._id(mujoco.mjtObj.mjOBJ_SITE, GRASP_SITE_NAME)
        self.joint_ids = np.array(
            [self._id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in ARM_JOINT_NAMES]
        )
        self.qpos_addresses = self.model.jnt_qposadr[self.joint_ids]
        self.dof_addresses = self.model.jnt_dofadr[self.joint_ids]
        self.lower = self.model.jnt_range[self.joint_ids, 0]
        self.upper = self.model.jnt_range[self.joint_ids, 1]

    def _id(self, object_type: mujoco.mjtObj, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"MuJoCo object {name!r} was not found")
        return object_id

    def forward_pose(self, joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_addresses] = joints
        mujoco.mj_forward(self.model, self.data)
        return (
            self.data.site_xpos[self.grasp_site_id].copy(),
            self.data.site_xmat[self.grasp_site_id].reshape(3, 3).copy(),
        )

    @staticmethod
    def orientation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
        return 0.5 * (
            np.cross(current[:, 0], target[:, 0])
            + np.cross(current[:, 1], target[:, 1])
            + np.cross(current[:, 2], target[:, 2])
        )

    def solve(
        self,
        target: np.ndarray,
        initial: np.ndarray,
        target_orientation: np.ndarray | None = None,
        *,
        locked_joints: dict[int, float] | None = None,
        iterations: int = 360,
        tolerance: float = 0.002,
        damping: float = 0.035,
        orientation_weight: float = 0.75,
        orientation_tolerance: float = 0.05,
    ) -> np.ndarray:
        joints = initial.copy()
        locked_joints = locked_joints or {}
        active_indices = np.array(
            [index for index in range(len(joints)) if index not in locked_joints],
            dtype=np.int32,
        )
        if len(active_indices) < 3:
            raise ValueError("IK requires at least three unlocked arm joints")
        for index, value in locked_joints.items():
            joints[index] = float(np.clip(value, self.lower[index], self.upper[index]))
        best_joints = joints.copy()
        best_error = float("inf")
        for _ in range(iterations):
            position, orientation = self.forward_pose(joints)
            error_vector = target - position
            position_error = float(np.linalg.norm(error_vector))
            rotation_error = (
                self.orientation_error(orientation, target_orientation)
                if target_orientation is not None
                else np.zeros(3, dtype=np.float64)
            )
            orientation_error = float(np.linalg.norm(rotation_error))
            score = float(np.hypot(position_error, orientation_weight * orientation_error))
            if score < best_error:
                best_error = score
                best_joints = joints.copy()
            if position_error <= tolerance and (
                target_orientation is None or orientation_error <= orientation_tolerance
            ):
                return joints

            jacobian_position = np.zeros((3, self.model.nv), dtype=np.float64)
            jacobian_rotation = np.zeros((3, self.model.nv), dtype=np.float64)
            mujoco.mj_jacSite(
                self.model,
                self.data,
                jacobian_position,
                jacobian_rotation,
                self.grasp_site_id,
            )
            jacobian = jacobian_position[:, self.dof_addresses[active_indices]]
            solve_error = error_vector
            if target_orientation is not None:
                jacobian = np.vstack(
                    [
                        jacobian,
                        orientation_weight
                        * jacobian_rotation[:, self.dof_addresses[active_indices]],
                    ]
                )
                solve_error = np.r_[error_vector, orientation_weight * rotation_error]
            regularized = (
                jacobian @ jacobian.T + damping * damping * np.eye(jacobian.shape[0])
            )
            try:
                delta = jacobian.T @ np.linalg.solve(regularized, solve_error)
            except np.linalg.LinAlgError as error_detail:
                raise RuntimeError("IK Jacobian solve failed") from error_detail
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > 0.10:
                delta *= 0.10 / delta_norm
            joints[active_indices] = np.clip(
                joints[active_indices] + delta,
                self.lower[active_indices],
                self.upper[active_indices],
            )
            for index, value in locked_joints.items():
                joints[index] = value

        best_position, best_orientation_matrix = self.forward_pose(best_joints)
        best_position_error = float(np.linalg.norm(target - best_position))
        best_orientation_error = (
            float(np.linalg.norm(self.orientation_error(best_orientation_matrix, target_orientation)))
            if target_orientation is not None
            else 0.0
        )
        raise RuntimeError(
            f"IK failed for target {target.tolist()}: position error="
            f"{best_position_error * 1000:.1f} mm, orientation error={best_orientation_error:.3f}"
        )


class PhysicalGraspMonitor:
    """Observe bilateral MuJoCo contacts without modifying object state."""

    def __init__(self, simulation: RebotArmSimulation, body_name: str) -> None:
        self.simulation = simulation
        self.model = simulation.model
        self.data = simulation.data
        self.body_id = self._id(mujoco.mjtObj.mjOBJ_BODY, body_name)
        self.grasp_site_id = self._id(mujoco.mjtObj.mjOBJ_SITE, GRASP_SITE_NAME)
        joint_id = int(self.model.body_jntadr[self.body_id])
        if joint_id < 0 or self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(f"Body {body_name!r} must have a free joint")
        self.qpos_address = int(self.model.jnt_qposadr[joint_id])
        self.object_geom_ids = set(
            range(
                int(self.model.body_geomadr[self.body_id]),
                int(self.model.body_geomadr[self.body_id] + self.model.body_geomnum[self.body_id]),
            )
        )
        self.left_geom_ids = self._geom_ids(
            "finger_left_collision", "finger_left_rear_collision"
        )
        self.right_geom_ids = self._geom_ids(
            "finger_right_collision", "finger_right_rear_collision"
        )
        self.bilateral_contact_seen = False
        self.contact_announced = False

    def _id(self, object_type: mujoco.mjtObj, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"MuJoCo object {name!r} was not found")
        return object_id

    @property
    def object_position(self) -> np.ndarray:
        return self.data.qpos[self.qpos_address : self.qpos_address + 3].copy()

    @property
    def grasp_position(self) -> np.ndarray:
        return self.data.site_xpos[self.grasp_site_id].copy()

    def _geom_ids(self, *names: str) -> set[int]:
        result = set()
        for name in names:
            geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if geom_id >= 0:
                result.add(geom_id)
        if not result:
            raise ValueError(f"None of the required contact geoms exist: {names}")
        return result

    def contact_state(self) -> tuple[bool, bool, float, float]:
        left_contact = False
        right_contact = False
        left_force = 0.0
        right_force = 0.0
        wrench = np.zeros(6, dtype=np.float64)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            pair = {int(contact.geom1), int(contact.geom2)}
            if not pair.intersection(self.object_geom_ids):
                continue
            finger_geoms = pair - self.object_geom_ids
            mujoco.mj_contactForce(self.model, self.data, index, wrench)
            normal_force = max(float(wrench[0]), 0.0)
            if finger_geoms.intersection(self.left_geom_ids):
                left_contact = True
                left_force += normal_force
            if finger_geoms.intersection(self.right_geom_ids):
                right_contact = True
                right_force += normal_force
        bilateral = left_contact and right_contact
        self.bilateral_contact_seen = self.bilateral_contact_seen or bilateral
        return left_contact, right_contact, left_force, right_force

    def contact_diagnostics(self) -> tuple[float, float]:
        """Return minimum target/finger contact distance and finger travel."""
        distances: list[float] = []
        finger_geoms = self.left_geom_ids | self.right_geom_ids
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            pair = {int(contact.geom1), int(contact.geom2)}
            if pair.intersection(self.object_geom_ids) and pair.intersection(finger_geoms):
                distances.append(float(contact.dist))
        left_joint = self._id(mujoco.mjtObj.mjOBJ_JOINT, "finger_left")
        finger_qpos = float(self.data.qpos[self.model.jnt_qposadr[left_joint]])
        return (min(distances) if distances else float("nan"), finger_qpos)

    def physical_close_target(self, preload: float = 0.0003) -> float:
        """Compute a near-contact command from current MJCF collision geometry."""
        object_geom_id = next(iter(self.object_geom_ids))
        if self.model.geom_type[object_geom_id] != mujoco.mjtGeom.mjGEOM_CYLINDER:
            raise ValueError("Automatic physical close target currently requires a cylinder")
        cylinder_radius = float(self.model.geom_size[object_geom_id, 0])

        left_geom_id = self._id(mujoco.mjtObj.mjOBJ_GEOM, "finger_left_rear_collision")
        left_body_id = int(self.model.geom_bodyid[left_geom_id])
        collision_center_y = abs(
            float(self.model.body_pos[left_body_id, 1])
            + float(self.model.geom_pos[left_geom_id, 1])
        )
        collision_half_width = float(self.model.geom_size[left_geom_id, 1])
        closed_inner_surface = collision_center_y - collision_half_width
        contact_qpos = cylinder_radius - closed_inner_surface
        command = contact_qpos - max(preload, 0.0)
        command = float(np.clip(command, self.simulation.ctrl_min[6], self.simulation.ctrl_max[6]))
        print(
            "[plan] physical gripper: "
            f"radius={cylinder_radius * 1000:.1f} mm, "
            f"contact_qpos={contact_qpos * 1000:.2f} mm, "
            f"command={command * 1000:.2f} mm"
        )
        return command


def build_phases(
    simulation: RebotArmSimulation,
    pick: np.ndarray,
    place: np.ndarray,
    grasp_gripper: float,
) -> list[MotionPhase]:
    ik = JacobianIK(simulation)
    # Straight wrist: joint4, joint5 and joint6 remain at zero throughout.
    home = np.array([0.0, -0.70, -0.80, 0.0, 0.0, 0.0], dtype=np.float64)
    wrist_locks = {3: 0.0, 4: 0.0, 5: 0.0}
    # Read the user's current MJCF range instead of assuming an older opening.
    open_gripper = float(simulation.ctrl_max[6])
    empty_closed_gripper = float(simulation.ctrl_min[6])

    # grasp_center is the end effector. Approach from directly above in world Z;
    # do not derive the path from the legacy TCP or a tool-local offset.
    at_pick = pick.copy()
    pre_pick = pick + np.array([0.0, 0.0, 0.12])
    lifted = pick + np.array([0.0, 0.0, 0.15])
    at_place = place.copy()
    pre_place = place + np.array([0.0, 0.0, 0.15])

    q_pre_pick = ik.solve(pre_pick, home, locked_joints=wrist_locks)
    q_pick = ik.solve(at_pick, q_pre_pick, locked_joints=wrist_locks)
    planned_grasp_position, _grasp_orientation = ik.forward_pose(q_pick)
    planned_error = float(np.linalg.norm(planned_grasp_position - at_pick))
    if planned_error > 0.0021:
        raise RuntimeError(
            f"{GRASP_SITE_NAME} IK verification failed: "
            f"error={planned_error * 1000:.1f} mm"
        )
    print(
        f"[plan] {GRASP_SITE_NAME} -> {at_pick.round(4).tolist()}, "
        f"IK error={planned_error * 1000:.2f} mm"
    )
    q_lift = ik.solve(lifted, q_pick, locked_joints=wrist_locks)
    q_pre_place = ik.solve(pre_place, q_lift, locked_joints=wrist_locks)
    q_place = ik.solve(at_place, q_pre_place, locked_joints=wrist_locks)

    return [
        MotionPhase("move above cylinder", q_pre_pick, open_gripper, 3.0),
        MotionPhase("lower grasp_center", q_pick, open_gripper, 2.5),
        MotionPhase("close gripper", q_pick, grasp_gripper, 1.5),
        MotionPhase("lift", q_lift, grasp_gripper, 2.5),
        MotionPhase("transfer", q_pre_place, grasp_gripper, 3.5),
        MotionPhase("lower", q_place, grasp_gripper, 2.0),
        MotionPhase("open gripper", q_place, open_gripper, 1.5),
        MotionPhase("retreat", q_pre_place, open_gripper, 2.0),
        MotionPhase("return home", home, open_gripper, 3.0),
        MotionPhase("close gripper at home", home, empty_closed_gripper, 1.5),
    ]


def run(args: argparse.Namespace) -> None:
    simulation = RebotArmSimulation(args.model, gravity_compensation=True)
    home = np.array(
        [0.0, -0.70, -0.80, 0.0, 0.0, 0.0, float(simulation.ctrl_max[6])]
    )
    simulation.set_target(home)
    simulation.reset_to_target()
    grasp = PhysicalGraspMonitor(simulation, args.object)

    # Pick the selected object's live center; defaults target yellow_cylinder.
    pick = grasp.object_position
    place = np.asarray(args.place, dtype=np.float64)
    phases = build_phases(
        simulation,
        pick,
        place,
        grasp_gripper=grasp.physical_close_target(),
    )

    viewer = None
    if not args.headless:
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(simulation.model, simulation.data)
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -25.0
        viewer.cam.distance = 1.25
        viewer.cam.lookat[:] = (0.36, 0.0, 0.18)

    phase_index = 0
    phase_start_time = float(simulation.data.time)
    phase_start_target = simulation.target.copy()
    print(f"[phase 1/{len(phases)}] {phases[0].name}")
    wall_start = time.perf_counter()
    next_sync = wall_start

    try:
        while (viewer is None or viewer.is_running()) and phase_index < len(phases):
            phase = phases[phase_index]
            elapsed = float(simulation.data.time - phase_start_time)
            blend = smoothstep(elapsed / phase.duration)
            destination = np.r_[phase.arm_target, phase.gripper_target]
            simulation.set_target((1.0 - blend) * phase_start_target + blend * destination)

            lock = viewer.lock() if viewer is not None else nullcontext()
            with lock:
                simulation.step()
                left, right, left_force, right_force = grasp.contact_state()
                if left and right and not grasp.contact_announced:
                    grasp.contact_announced = True
                    print(
                        "[contact] bilateral physical grasp: "
                        f"left={left_force:.2f} N, right={right_force:.2f} N"
                    )

                if elapsed >= phase.duration:
                    if phase.name == "close gripper" and not (left and right):
                        distance = float(
                            np.linalg.norm(grasp.object_position - grasp.grasp_position)
                        )
                        raise RuntimeError(
                            "Physical grasp failed: both fingers are not contacting the object "
                            f"(distance={distance * 1000:.1f} mm, "
                            f"grasp_center={grasp.grasp_position.round(4).tolist()}, "
                            f"object={grasp.object_position.round(4).tolist()})"
                        )
                    if phase.name == "close gripper":
                        min_distance, finger_qpos = grasp.contact_diagnostics()
                        print(
                            "[contact] settled geometry: "
                            f"penetration={max(-min_distance, 0.0) * 1000:.3f} mm, "
                            f"finger_qpos={finger_qpos:.5f} m"
                        )
                    if phase.name == "lift":
                        lift_height = float(grasp.object_position[2] - pick[2])
                        if lift_height < 0.05 or not (left and right):
                            raise RuntimeError(
                                "Physical lift failed: object was not retained by contact "
                                f"(lift={lift_height * 1000:.1f} mm, "
                                f"left_contact={left}, right_contact={right})"
                            )
                        print(f"[lift] physical grasp retained for {lift_height * 1000:.1f} mm")
                    if phase.name == "open gripper":
                        print(
                            f"[release] contacts left={left}, right={right}, "
                            f"object={grasp.object_position.round(4).tolist()}"
                        )
                    phase_index += 1
                    if phase_index < len(phases):
                        phase_start_time = float(simulation.data.time)
                        phase_start_target = simulation.target.copy()
                        print(f"[phase {phase_index + 1}/{len(phases)}] {phases[phase_index].name}")

            now = time.perf_counter()
            if viewer is not None and now >= next_sync:
                viewer.sync()
                next_sync = now + 1.0 / args.render_hz
            if args.realtime:
                remaining = simulation.model.opt.timestep - (time.perf_counter() - now)
                if remaining > 0.0:
                    time.sleep(remaining)

        # Let the released object settle on the table.
        settle_until = simulation.data.time + args.settle_time
        while simulation.data.time < settle_until and (viewer is None or viewer.is_running()):
            simulation.step()
            if viewer is not None:
                viewer.sync()
            if args.realtime:
                time.sleep(simulation.model.opt.timestep)
    finally:
        if viewer is not None:
            viewer.close()

    final_position = grasp.object_position
    planar_error = float(np.linalg.norm(final_position[:2] - place[:2]))
    left_joint_id = mujoco.mj_name2id(
        simulation.model, mujoco.mjtObj.mjOBJ_JOINT, "finger_left"
    )
    right_joint_id = mujoco.mj_name2id(
        simulation.model, mujoco.mjtObj.mjOBJ_JOINT, "finger_right"
    )
    final_left = float(
        simulation.data.qpos[simulation.model.jnt_qposadr[left_joint_id]]
    )
    final_right = float(
        simulation.data.qpos[simulation.model.jnt_qposadr[right_joint_id]]
    )
    print(
        f"Pick-and-place finished: object={final_position.round(4).tolist()}, "
        f"planar_place_error={planar_error * 1000:.1f} mm, "
        f"final_gripper=[{final_left:.5f}, {final_right:.5f}] m"
    )
    if planar_error > 0.08:
        raise RuntimeError("Object finished outside the requested place region")
    if abs(final_left) > 0.001 or abs(final_right) > 0.001:
        raise RuntimeError(
            "Arm returned home but gripper did not fully close: "
            f"left={final_left:.5f}, right={final_right:.5f} m"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=default_model_path(), help="MJCF XML path")
    parser.add_argument("--object", default="yellow_cylinder", help="Free-body object name")
    parser.add_argument(
        "--place",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.34, 0.15, 0.065),
        help="Object-center placement target in metres",
    )
    parser.add_argument("--headless", action="store_true", help="Run without viewer")
    parser.add_argument("--no-realtime", dest="realtime", action="store_false", help="Run as fast as possible")
    parser.add_argument("--render-hz", type=float, default=60.0, help="Viewer synchronization rate")
    parser.add_argument("--settle-time", type=float, default=1.0, help="Physics settling time after release")
    parser.set_defaults(realtime=True)
    args = parser.parse_args()
    if args.render_hz <= 0.0 or args.settle_time < 0.0:
        parser.error("render rate must be positive and settle time non-negative")
    return args


if __name__ == "__main__":
    run(parse_args())
