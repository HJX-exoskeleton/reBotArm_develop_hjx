#!/usr/bin/env python3
"""Automatic reBot + right AeroHand cylinder pick-and-place demo.

The scripted state machine approaches the cylinder with the thumb pointing up,
rotates the thumb into opposition, closes the fingers, lifts the object, moves
above the fixed target disk, lowers, and releases. The task succeeds only when
MuJoCo reports contact between ``red_box`` and ``target_box``.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import rebotarm_aerohand_right_cv_control_sim as teleop


DEFAULT_MODEL = teleop.DEFAULT_TASK_MODEL


@dataclass(frozen=True)
class Stage:
    name: str
    duration: float
    tool_target: np.ndarray | None
    hand_target: np.ndarray


def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


class AutoGraspDemo:
    def __init__(self, model, data, arm, hand, speed: float):
        self.model = model
        self.data = data
        self.arm = arm
        self.hand = hand
        self.speed = max(float(speed), 0.05)
        mj = teleop.hand_cv.mujoco

        self.object_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "box")
        self.object_geom = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "red_box")
        self.target_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "target_box")
        self.target_geom = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "target_box")
        self.table_geom = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "tabletop")
        if min(self.object_body, self.object_geom, self.target_body, self.target_geom) < 0:
            raise RuntimeError("Model must contain box/red_box and target_box")

        # The XML marker is visual-only. Enable its collision at runtime so the
        # success condition is a real MuJoCo contact rather than an XY heuristic.
        model.geom_contype[self.target_geom] = 1
        model.geom_conaffinity[self.target_geom] = 1
        mj.mj_forward(model, data)
        object_joint = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "red_box_joint")
        self.object_qpos = int(model.jnt_qposadr[object_joint])
        self.object_dof = int(model.jnt_dofadr[object_joint])

        hand_root = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "tetheria_mount")
        hand_bodies = set()
        for body_id in range(model.nbody):
            ancestor = body_id
            while ancestor > 0 and ancestor != hand_root:
                ancestor = int(model.body_parentid[ancestor])
            if ancestor == hand_root:
                hand_bodies.add(body_id)
        self.hand_geoms = {
            geom_id for geom_id in range(model.ngeom)
            if int(model.geom_bodyid[geom_id]) in hand_bodies
        }
        hand_joint_ids = [
            joint_id for joint_id in range(model.njnt)
            if int(model.jnt_bodyid[joint_id]) in hand_bodies
        ]
        self.hand_qpos_adr = model.jnt_qposadr[hand_joint_ids].astype(np.int32)
        self.hand_dof_adr = model.jnt_dofadr[hand_joint_ids].astype(np.int32)
        self.unsafe_contacts: set[str] = set()
        self.max_hand_object_penetration = 0.0
        self.penetration_by_stage: dict[str, float] = {}

        obj = data.xpos[self.object_body].copy()
        goal = data.xpos[self.target_body].copy()
        cylinder_half_height = float(model.geom_size[self.object_geom, 1])
        cylinder_radius = float(model.geom_size[self.object_geom, 0])
        disk_half_height = float(model.geom_size[self.target_geom, 1])
        # Keep dynamics consistent when the XML cylinder height is edited.
        mass = float(model.body_mass[self.object_body])
        full_height = 2.0 * cylinder_half_height
        inertia_xy = mass * (3.0 * cylinder_radius ** 2 + full_height ** 2) / 12.0
        inertia_z = 0.5 * mass * cylinder_radius ** 2
        model.body_inertia[self.object_body] = np.asarray(
            [inertia_xy, inertia_xy, inertia_z]
        )

        # The fingers extend along +X and curl toward +Y against the palm. Put
        # the cylinder between palm (Y=0) and closed fingertips (Y~=0.05), then
        # approach laterally from -Y so open fingertips do not push it forward.
        mount_to_grasp_x = 0.100
        # Raise the mount 25 mm: the lowest pinky geometry then stays clear of
        # the tabletop while the 120 mm tall cylinder remains inside the hand.
        grasp_tool = obj + np.asarray([-mount_to_grasp_x, -0.050, 0.042])
        # Pre-shape the tiger mouth behind and slightly to the side of the
        # cylinder. After thumb rotation, approach straight along +X (the
        # fingers' forward direction), rather than sliding laterally past it.
        contact_tool = grasp_tool + np.asarray([-0.015, 0.0, 0.0])
        pregrasp_behind = contact_tool + np.asarray([-0.10, -0.020, 0.0])
        pregrasp_high = pregrasp_behind + np.asarray([0.0, 0.0, 0.16])
        lift_tool = grasp_tool + np.asarray([0.0, 0.0, 0.20])
        # Stop with the cylinder bottom above the disk.  The hand opens and
        # retreats at this clearance; only then is the cylinder released to
        # fall onto the target.  This keeps even the low pinky visual mesh away
        # from the disk during the whole placement motion.
        release_clearance = 0.008
        goal_object_z = (
            goal[2] + disk_half_height + cylinder_half_height + release_clearance
        )
        place_tool = np.asarray(
            [goal[0] - mount_to_grasp_x, goal[1] - 0.050, goal_object_z + 0.042]
        )
        goal_high = place_tool + np.asarray([0.0, 0.0, 0.20])
        retreat_tool = place_tool + np.asarray([-0.10, -0.04, 0.08])

        opened = hand.open_ctrl
        thumb_opposed = opened.copy()
        thumb_opposed[4] = min(1.35, hand.ctrl_max[4])
        # Full closure is for an empty hand and can pull links through a 50 mm
        # object. Stop the tendons at a contact-safe partial grasp instead.
        closed = opened.copy()
        finger_tendons = np.asarray([0, 1, 2, 3], dtype=np.int32)
        thumb_tendons = np.asarray([5, 6], dtype=np.int32)
        closed[finger_tendons] += 0.58 * (
            hand.closed_ctrl[finger_tendons] - opened[finger_tendons]
        )
        closed[thumb_tendons] += 0.55 * (
            hand.closed_ctrl[thumb_tendons] - opened[thumb_tendons]
        )
        closed[4] = min(1.35, hand.ctrl_max[4])
        release_shaped = opened.copy()
        release_shaped[4] = min(1.35, hand.ctrl_max[4])
        # First relax all four fingers together while the cylinder is still
        # carried.  This unloads contact without the unnatural look of opening
        # the pinky separately.  Full opening starts only after the object is
        # supported at the release pose.
        loosened = closed + 0.55 * (release_shaped - closed)

        def seconds(value: float) -> float:
            return value / self.speed

        self.stages = [
            Stage("settle/open hand", seconds(0.8), None, opened),
            Stage("move above side-rear pregrasp", seconds(2.8), pregrasp_high, opened),
            Stage("lower behind cylinder", seconds(2.0), pregrasp_behind, opened),
            Stage("rotate thumb at safe distance", seconds(1.2), pregrasp_behind, thumb_opposed),
            Stage("approach forward with shaped tiger mouth", seconds(2.0), contact_tool, thumb_opposed),
            Stage("envelop cylinder at contact", seconds(2.4), contact_tool, closed),
            Stage("settle grasp contact", seconds(1.2), grasp_tool, closed),
            Stage("lift cylinder", seconds(2.4), lift_tool, closed),
            Stage("move above target disk", seconds(3.0), goal_high, closed),
            Stage("stabilize upright above target", seconds(1.5), goal_high, closed),
            Stage("lower upright cylinder", seconds(4.0), place_tool, closed),
            Stage("relax four-finger grasp", seconds(0.9), place_tool, loosened),
            Stage("release fingers/keep tiger mouth", seconds(2.2), place_tool, release_shaped),
            Stage("retreat with tiger mouth fixed", seconds(1.8), retreat_tool, release_shaped),
            Stage("return thumb after retreat", seconds(1.2), retreat_tool, opened),
            Stage("drop and verify contact", seconds(2.0), retreat_tool, opened),
        ]
        self.stage_index = 0
        self.stage_start_time = float(data.time)
        self.start_tool = arm.target_pos.copy()
        self.start_hand = opened.copy()
        self.hand_command = opened.copy()
        self.contact_seen = False
        self.finished = False
        self.success = False
        self.carry_offset: np.ndarray | None = None
        self.carry_offset_local: np.ndarray | None = None
        self.release_anchor: np.ndarray | None = None
        self.goal_xy = goal[:2].copy()
        self.goal_object_z = goal_object_z
        print(f"[Object] position={np.round(obj, 4)}, half-height={cylinder_half_height:.3f}")
        print(f"[Object] corrected inertia={np.round(model.body_inertia[self.object_body], 8)}")
        print(f"[Target] position={np.round(goal, 4)}, radius={model.geom_size[self.target_geom, 0]:.3f}")
        self._announce()

    def _announce(self) -> None:
        stage = self.stages[self.stage_index]
        obj = self.data.xpos[self.object_body]
        object_rot = self.data.xmat[self.object_body].reshape(3, 3)
        tilt = np.degrees(np.arccos(np.clip(object_rot[2, 2], -1.0, 1.0)))
        print(
            f"[AUTO {self.stage_index + 1}/{len(self.stages)}] {stage.name} "
            f"| cylinder={np.round(obj, 4)} | tilt={tilt:.2f} deg"
        )

    def _center_transport_targets(self) -> None:
        """Compensate the actual post-grasp object offset for centered placement."""
        tool_pos, _ = self.arm._tool_pose(self.data)
        carry_offset = (
            self.arm.target_rot @ self.carry_offset_local
            if self.carry_offset_local is not None
            else self.data.xpos[self.object_body] - tool_pos
        )
        place_tool = np.asarray(
            [
                self.goal_xy[0] - carry_offset[0],
                self.goal_xy[1] - carry_offset[1],
                self.goal_object_z - carry_offset[2],
            ]
        )
        goal_high = place_tool + np.asarray([0.0, 0.0, 0.20])
        for candidate in self.stages:
            if candidate.name in ("move above target disk", "stabilize upright above target"):
                candidate.tool_target[:] = goal_high
            elif candidate.name == "lower upright cylinder":
                candidate.tool_target[:] = place_tool
            elif candidate.name in (
                "relax four-finger grasp",
                "release fingers/keep tiger mouth",
            ):
                candidate.tool_target[:] = place_tool
            elif candidate.name in (
                "retreat with tiger mouth fixed",
                "return thumb after retreat",
                "drop and verify contact",
            ):
                candidate.tool_target[:] = place_tool + np.asarray([-0.10, -0.04, 0.08])
        print(f"[Placement] compensated carry offset={np.round(carry_offset, 4)}")

    def _capture_grasp_offset(self) -> None:
        tool_pos, tool_rot = self.arm._tool_pose(self.data)
        self.carry_offset = self.data.xpos[self.object_body].copy() - tool_pos
        self.carry_offset_local = tool_rot.T @ self.carry_offset
        print(f"[Grasp] measured physical carry offset={np.round(self.carry_offset, 4)}")

    def _has_object_target_contact(self) -> bool:
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            pair = {int(contact.geom1), int(contact.geom2)}
            if pair == {self.object_geom, self.target_geom}:
                return True
        return False

    def monitor_safety_contacts(self) -> None:
        """Record any forbidden hand contact with the table or target disk."""
        forbidden = {self.table_geom: "tabletop", self.target_geom: "target disk"}
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if (
                (geom1 in self.hand_geoms and geom2 == self.object_geom)
                or (geom2 in self.hand_geoms and geom1 == self.object_geom)
            ):
                self.max_hand_object_penetration = max(
                    self.max_hand_object_penetration,
                    max(0.0, -float(contact.dist)),
                )
                stage_name = self.stages[self.stage_index].name
                self.penetration_by_stage[stage_name] = max(
                    self.penetration_by_stage.get(stage_name, 0.0),
                    max(0.0, -float(contact.dist)),
                )
            if geom1 in self.hand_geoms and geom2 in forbidden:
                label = forbidden[geom2]
            elif geom2 in self.hand_geoms and geom1 in forbidden:
                label = forbidden[geom1]
            else:
                continue
            if label not in self.unsafe_contacts:
                print(f"[Safety] forbidden hand contact with {label}")
            self.unsafe_contacts.add(label)

    def apply_carry_upright_stabilization(self) -> None:
        """Leave the cylinder fully dynamic throughout grasp and transport.

        This hook is intentionally retained at the call site so the simulation
        loop stays simple, but it must never write object or finger qpos/qvel.
        Grasp success now comes only from actuator forces, contacts and friction.
        """
        torque_slice = slice(self.object_dof + 3, self.object_dof + 6)
        self.data.qfrc_applied[torque_slice] = 0.0

    def update(self) -> None:
        if self.finished:
            return
        stage = self.stages[self.stage_index]
        elapsed = float(self.data.time) - self.stage_start_time
        blend = smoothstep(elapsed / max(stage.duration, 1e-6))
        if stage.tool_target is not None:
            self.arm.target_pos[:] = (
                self.start_tool + blend * (stage.tool_target - self.start_tool)
            )
        self.hand_command = self.start_hand + blend * (
            stage.hand_target - self.start_hand
        )

        if self.stage_index >= 7 and self._has_object_target_contact():
            if not self.contact_seen:
                print("[Contact] cylinder touched target disk")
            self.contact_seen = True

        if elapsed < stage.duration:
            return
        if stage.tool_target is not None:
            self.arm.target_pos[:] = stage.tool_target
        self.hand_command = stage.hand_target.copy()
        self.stage_index += 1
        if self.stage_index >= len(self.stages):
            self.finished = True
            self.success = (
                (self.contact_seen or self._has_object_target_contact())
                and not self.unsafe_contacts
                and self.max_hand_object_penetration <= 0.006
            )
            result = "SUCCESS" if self.success else "FAILED"
            obj = self.data.xpos[self.object_body]
            print(f"[TASK {result}] final cylinder position={np.round(obj, 4)}")
            print(
                "[Contact quality] maximum hand-object penetration="
                f"{1000.0 * self.max_hand_object_penetration:.3f} mm"
            )
            print(
                "[Contact quality] penetration by stage="
                + str({k: round(1000.0 * v, 3) for k, v in self.penetration_by_stage.items()})
            )
            return
        self.stage_start_time = float(self.data.time)
        self.start_tool = self.arm.target_pos.copy()
        self.start_hand = self.hand_command.copy()
        if self.stages[self.stage_index].name == "envelop cylinder at contact":
            self.grasp_anchor = self.data.xpos[self.object_body].copy()
            print(
                "[Grasp] physical enveloping closure begins at="
                f"{np.round(self.grasp_anchor, 4)}"
            )
        elif self.stages[self.stage_index].name == "lift cylinder":
            self._capture_grasp_offset()
        elif self.stages[self.stage_index].name == "move above target disk":
            self._center_transport_targets()
        elif self.stages[self.stage_index].name == "release fingers/keep tiger mouth":
            self.carry_offset = None
            self.carry_offset_local = None
            self.release_anchor = None
            print("[Release] fingers opening; cylinder dynamics released immediately")
        elif self.stages[self.stage_index].name == "drop and verify contact":
            self.carry_offset = None
            self.carry_offset_local = None
            self.release_anchor = None
            print("[Release] hand clear; verifying free cylinder contact")
        self._announce()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--speed", type=float, default=1.0,
                        help="trajectory speed multiplier")
    parser.add_argument("--arm-control-hz", type=float, default=100.0)
    parser.add_argument("--ik-damping", type=float, default=0.03)
    parser.add_argument("--max-joint-step", type=float, default=0.02)
    parser.add_argument("--arm-gravcomp", type=float, default=1.0)
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--stay-open", action="store_true",
                        help="keep viewer open after success/failure")
    parser.add_argument("--seed", type=int, default=None,
                        help="random seed for cylinder initialization")
    parser.add_argument("--object-random-radius", type=float, default=0.025,
                        help="maximum cylinder XY offset in metres")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mj = teleop.hand_cv.mujoco
    model_path = teleop.hand_cv.resolve_model_path(args.model)
    if not model_path.is_file():
        raise FileNotFoundError(f"MuJoCo model not found: {model_path}")
    model = mj.MjModel.from_xml_path(str(model_path))
    data = mj.MjData(model)
    object_joint = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "red_box_joint")
    if object_joint < 0:
        raise RuntimeError("Model is missing free joint 'red_box_joint'")
    object_qpos = int(model.jnt_qposadr[object_joint])
    rng = np.random.default_rng(args.seed)
    random_radius = max(0.0, float(args.object_random_radius)) * np.sqrt(rng.random())
    random_angle = rng.uniform(-np.pi, np.pi)
    random_offset = random_radius * np.asarray(
        [np.cos(random_angle), np.sin(random_angle)]
    )
    data.qpos[object_qpos:object_qpos + 2] += random_offset
    mj.mj_forward(model, data)
    print(
        f"[Randomization] seed={args.seed}, cylinder XY offset="
        f"{np.round(random_offset, 4)}"
    )
    arm = teleop.KeyboardArmController(
        model,
        data,
        translation_step=0.003,
        rotation_step=0.02,
        damping=args.ik_damping,
        max_joint_step=args.max_joint_step,
        gravcomp=args.arm_gravcomp,
    )
    hand = teleop.hand_cv.SimHandMapper(model)
    demo = AutoGraspDemo(model, data, arm, hand, args.speed)
    hand.write(data, hand.open_ctrl)
    mj.mj_forward(model, data)

    arm_period = 1.0 / max(args.arm_control_hz, 1.0)
    next_arm_update = 0.0
    realtime_factor = max(args.realtime_factor, 0.05)

    def simulation_step() -> None:
        nonlocal next_arm_update
        if data.time >= next_arm_update:
            demo.update()
            arm.step(data)
            next_arm_update = data.time + arm_period
        hand.write(data, demo.hand_command)
        demo.apply_carry_upright_stabilization()
        mj.mj_step(model, data)
        demo.monitor_safety_contacts()

    viewer_context = (
        teleop.ArmControlViewer(model, data, lambda _key: None)
        if not args.headless else None
    )
    try:
        viewer = viewer_context.__enter__() if viewer_context is not None else None
        wall_start = time.perf_counter()
        sim_start = float(data.time)
        while viewer is None or viewer.is_running():
            if viewer is None:
                simulation_step()
            else:
                # swap_buffers is synchronized to the monitor (normally 60 Hz),
                # whereas this model uses a 1 ms timestep. Advance all MuJoCo
                # steps due for the current video frame before rendering it.
                target_sim_time = sim_start + (
                    time.perf_counter() - wall_start
                ) * realtime_factor
                steps = 0
                while data.time < target_sim_time and steps < 100:
                    simulation_step()
                    steps += 1
                viewer.sync()
                if steps == 0:
                    time.sleep(0.001)

            if demo.finished and (args.headless or not args.stay_open):
                # Let the released object settle briefly before returning.
                if data.time - demo.stage_start_time > 0.25:
                    break
        return 0 if demo.success else 1
    finally:
        if viewer_context is not None:
            viewer_context.__exit__(None, None, None)


if __name__ == "__main__":
    raise SystemExit(main())
