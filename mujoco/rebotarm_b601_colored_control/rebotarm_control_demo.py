#!/usr/bin/env python3
"""Standalone MuJoCo control example for the reBot B601 arm.

This example intentionally has no ROS dependency. The MJCF position actuators
provide PD feedback, while this script adds MuJoCo bias-force compensation in
the same spirit as the original project's ``qfrc_bias + PD`` control loop.
"""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
import time

import mujoco
import numpy as np


ACTUATOR_NAMES = (
    "joint1_position",
    "joint2_position",
    "joint3_position",
    "joint4_position",
    "joint5_position",
    "joint6_position",
    "gripper_position",
)
ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


@dataclass(frozen=True)
class Waypoint:
    """A seven-axis target and the travel time to the next target."""

    target: tuple[float, float, float, float, float, float, float]
    duration: float


def default_model_path() -> Path:
    return Path(__file__).resolve().parents[1] / "xml" / "rebotarm_b601_colored" / "rebotarm_b601_colored_optimized.xml"


class RebotArmSimulation:
    """Small reusable interface around the standalone reBot MuJoCo model."""

    def __init__(self, model_path: Path, gravity_compensation: bool = True) -> None:
        self.model_path = model_path.resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.gravity_compensation = gravity_compensation

        self.actuator_ids = np.array(
            [self._named_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ACTUATOR_NAMES],
            dtype=np.int32,
        )
        self.arm_dof_addresses = np.array(
            [self.model.jnt_dofadr[self._named_id(mujoco.mjtObj.mjOBJ_JOINT, name)]
             for name in ARM_JOINT_NAMES],
            dtype=np.int32,
        )
        self.ctrl_min = self.model.actuator_ctrlrange[self.actuator_ids, 0].copy()
        self.ctrl_max = self.model.actuator_ctrlrange[self.actuator_ids, 1].copy()
        self.target = np.zeros(len(ACTUATOR_NAMES), dtype=np.float64)
        self.left_finger_joint_id = self._named_id(
            mujoco.mjtObj.mjOBJ_JOINT, "finger_left"
        )
        self.right_finger_joint_id = self._named_id(
            mujoco.mjtObj.mjOBJ_JOINT, "finger_right"
        )
        self.left_finger_qpos_address = int(
            self.model.jnt_qposadr[self.left_finger_joint_id]
        )
        self.right_finger_qpos_address = int(
            self.model.jnt_qposadr[self.right_finger_joint_id]
        )
        self._validate_control_model()

    def _validate_control_model(self) -> None:
        if not np.all(self.model.actuator_ctrllimited[self.actuator_ids]):
            raise ValueError("All seven reBot actuators must define ctrlrange limits")
        if np.any(self.ctrl_max <= self.ctrl_min):
            raise ValueError(f"Invalid actuator ranges: {self.ctrl_min} .. {self.ctrl_max}")
        left_range = self.model.jnt_range[self.left_finger_joint_id]
        right_range = self.model.jnt_range[self.right_finger_joint_id]
        if not (
            np.isclose(self.ctrl_min[6], left_range[0])
            and np.isclose(self.ctrl_max[6], left_range[1])
            and np.isclose(right_range[0], -left_range[1])
            and np.isclose(right_range[1], -left_range[0])
        ):
            raise ValueError(
                "Gripper actuator and symmetric finger joint ranges are inconsistent"
            )

    def _named_id(self, object_type: mujoco.mjtObj, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"Required MuJoCo object {name!r} is missing from {self.model_path}")
        return object_id

    def set_target(self, target: np.ndarray | tuple[float, ...] | list[float]) -> None:
        """Set [joint1..joint6, gripper] targets, clipping to MJCF limits."""
        command = np.asarray(target, dtype=np.float64)
        if command.shape != (len(ACTUATOR_NAMES),):
            raise ValueError(f"Expected 7 targets, received shape {command.shape}")
        self.target[:] = np.clip(command, self.ctrl_min, self.ctrl_max)

    def reset_to_target(self) -> None:
        """Reset and initialize the robot at the target without a startup impulse."""
        mujoco.mj_resetData(self.model, self.data)
        for index, joint_name in enumerate(ARM_JOINT_NAMES):
            joint_id = self._named_id(mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            self.data.qpos[self.model.jnt_qposadr[joint_id]] = self.target[index]
        self.data.qpos[self.left_finger_qpos_address] = self.target[6]
        self.data.qpos[self.right_finger_qpos_address] = -self.target[6]
        self.data.ctrl[self.actuator_ids] = self.target
        mujoco.mj_forward(self.model, self.data)

    def step(self) -> None:
        """Apply a target, compensate arm bias forces, and advance one MJCF step."""
        self.data.ctrl[self.actuator_ids] = self.target
        self.data.qfrc_applied[:] = 0.0
        if self.gravity_compensation:
            mujoco.mj_forward(self.model, self.data)
            self.data.qfrc_applied[self.arm_dof_addresses] = self.data.qfrc_bias[
                self.arm_dof_addresses
            ]
        mujoco.mj_step(self.model, self.data)

    @property
    def gripper_position(self) -> float:
        """Actual single-finger travel in metres."""
        return float(self.data.qpos[self.left_finger_qpos_address])


def build_demo_waypoints(simulation: RebotArmSimulation) -> tuple[Waypoint, ...]:
    """Build a safe cycle from the current MJCF actuator ranges."""
    gripper_closed = float(simulation.ctrl_min[6])
    gripper_open = float(simulation.ctrl_max[6])

    # Keep joint4-6 straight at home, consistent with the pick/place demo.
    home = (0.0, -0.70, -0.80, 0.0, 0.0, 0.0)
    left_pose = (-0.40, -1.00, -1.20, 0.20, 0.0, 0.0)
    right_pose = (0.40, -0.85, -1.05, -0.18, 0.0, 0.0)

    raw = (
        Waypoint((*home, gripper_closed), 1.5),
        Waypoint((*home, gripper_open), 1.5),
        Waypoint((*left_pose, gripper_open), 3.0),
        Waypoint((*left_pose, gripper_closed), 1.5),
        Waypoint((*right_pose, gripper_closed), 3.0),
        Waypoint((*right_pose, gripper_open), 1.5),
        Waypoint((*home, gripper_open), 3.0),
        Waypoint((*home, gripper_closed), 1.5),
    )
    return tuple(
        Waypoint(tuple(np.clip(item.target, simulation.ctrl_min, simulation.ctrl_max)), item.duration)
        for item in raw
    )


class CyclicWaypointTrajectory:
    """Smooth, cyclic joint-space trajectory used by the demo."""

    def __init__(self, waypoints: tuple[Waypoint, ...]) -> None:
        if len(waypoints) < 2:
            raise ValueError("At least two waypoints are required")
        self.waypoints = waypoints
        self.segment = 0
        self.segment_start = 0.0

    def sample(self, simulation_time: float) -> np.ndarray:
        current = self.waypoints[self.segment]
        while simulation_time - self.segment_start >= current.duration:
            self.segment_start += current.duration
            self.segment = (self.segment + 1) % len(self.waypoints)
            current = self.waypoints[self.segment]
        following = self.waypoints[(self.segment + 1) % len(self.waypoints)]
        phase = np.clip((simulation_time - self.segment_start) / current.duration, 0.0, 1.0)
        blend = phase * phase * (3.0 - 2.0 * phase)  # cubic smoothstep
        return (1.0 - blend) * np.asarray(current.target) + blend * np.asarray(following.target)


def run(args: argparse.Namespace) -> None:
    simulation = RebotArmSimulation(args.model, not args.no_gravity_compensation)
    waypoints = build_demo_waypoints(simulation)
    trajectory = CyclicWaypointTrajectory(waypoints)
    simulation.set_target(waypoints[0].target)
    simulation.reset_to_target()
    print(
        "Loaded control model: "
        f"gripper_range=[{simulation.ctrl_min[6]:.4f}, {simulation.ctrl_max[6]:.4f}] m, "
        f"initial_gripper={simulation.gripper_position:.4f} m"
    )

    viewer = None
    if not args.headless:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(simulation.model, simulation.data)
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -25.0
        viewer.cam.distance = 1.35
        viewer.cam.lookat[:] = (0.30, 0.0, 0.25)

    wall_start = time.perf_counter()
    sync_period = 1.0 / args.render_hz
    next_sync = wall_start
    try:
        while viewer is None or viewer.is_running():
            if args.duration > 0.0 and simulation.data.time >= args.duration:
                break

            step_start = time.perf_counter()
            simulation.set_target(trajectory.sample(simulation.data.time))
            lock = viewer.lock() if viewer is not None else nullcontext()
            with lock:
                simulation.step()

            now = time.perf_counter()
            if viewer is not None and now >= next_sync:
                viewer.sync()
                next_sync = now + sync_period

            if args.realtime:
                remaining = simulation.model.opt.timestep - (time.perf_counter() - step_start)
                if remaining > 0.0:
                    time.sleep(remaining)
    finally:
        if viewer is not None:
            viewer.close()

    elapsed = time.perf_counter() - wall_start
    print(
        f"Finished: simulated={simulation.data.time:.3f}s, wall={elapsed:.3f}s, "
        f"realtime_factor={simulation.data.time / max(elapsed, 1e-9):.2f}, "
        f"gripper={simulation.gripper_position:.5f} m"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=default_model_path(), help="MJCF XML path")
    parser.add_argument("--headless", action="store_true", help="Run without opening the viewer")
    parser.add_argument("--duration", type=float, default=0.0, help="Stop after N simulated seconds; 0 runs until viewer closes")
    parser.add_argument("--render-hz", type=float, default=60.0, help="Viewer synchronization rate")
    parser.add_argument("--no-realtime", dest="realtime", action="store_false", help="Run as fast as possible")
    parser.add_argument("--no-gravity-compensation", action="store_true", help="Disable qfrc_bias feedforward")
    parser.set_defaults(realtime=True)
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration must be non-negative")
    if args.render_hz <= 0.0:
        parser.error("--render-hz must be positive")
    return args


if __name__ == "__main__":
    run(parse_args())
