#!/usr/bin/env python3
"""Keyboard 6-DoF reBot Cartesian control + CV-controlled right AeroHand.

The MuJoCo viewer keyboard controls only the reBot end-effector pose. The
OpenCV camera window controls only the simulated AeroHand through MediaPipe.

MuJoCo viewer keys
------------------
W/S             : world -X / +X
A/D             : world -Y / +Y
R/F             : world +Z / -Z
UP/DOWN         : local -pitch / +pitch
LEFT/RIGHT      : local -roll / +roll
Q/E             : local +yaw / -yaw
Z               : return the arm to the bent home pose
X               : synchronize the IK target to the current tool pose
P               : print current and target end-effector poses

OpenCV window keys
------------------
SPACE           : enable/disable CV hand control
O / C           : open/close the simulated hand
V               : show/hide diagnostics
Q or ESC        : quit
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import cv2
import glfw
import numpy as np

import aerohand_right_cv_control_sim as hand_cv


ARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
ARM_HOME = np.asarray([0.0, -1.0, -1.0, 0.0, 0.0, 0.0], dtype=np.float64)
TOOL_BODY_NAME = "tetheria_mount"
DEFAULT_TASK_MODEL = (
    hand_cv.WORKSPACE_DIR
    / "xml"
    / "rebot_aerohand_right"
    / "mujoco_xml"
    / "rebotarm_aerohand_sim_transfer_cube.xml"
)


class ArmControlViewer:
    """Minimal GLFW MuJoCo viewer with no built-in keyboard shortcuts.

    This follows the viewer design used by lerobot-mujoco-rebot: GLFW events
    are registered directly, so control keys are consumed only by our arm
    callback. Mouse orbit/pan/zoom remains available.
    """

    def __init__(self, model, data, key_callback):
        self.model = model
        self.data = data
        self.key_callback = key_callback
        self.window = None
        self.left_pressed = False
        self.right_pressed = False
        self.last_x = 0.0
        self.last_y = 0.0

        if not glfw.init():
            raise RuntimeError("GLFW initialization failed")
        # GLFW window hints persist across create_window calls. An offscreen
        # mujoco.Renderer (--save-videos) may have set VISIBLE=0 beforehand,
        # which would otherwise make this window invisible.
        glfw.default_window_hints()
        monitor = glfw.get_primary_monitor()
        mode = glfw.get_video_mode(monitor) if monitor is not None else None
        width = min(1400, mode.size.width) if mode is not None else 1280
        height = min(900, mode.size.height) if mode is not None else 800
        self.window = glfw.create_window(
            width, height, "reBot Arm Keyboard + AeroHand CV", None, None
        )
        if self.window is None:
            glfw.terminate()
            raise RuntimeError("Could not create the MuJoCo GLFW window")
        # Center the window and request focus so it is not left hidden behind
        # a maximized terminal window.
        pos_x = max(0, (mode.size.width - width) // 2) if mode is not None else 60
        pos_y = max(0, (mode.size.height - height) // 2) if mode is not None else 60
        glfw.set_window_pos(self.window, pos_x, pos_y)
        glfw.focus_window(self.window)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        self.camera = hand_cv.mujoco.MjvCamera()
        self.option = hand_cv.mujoco.MjvOption()
        self.scene = hand_cv.mujoco.MjvScene(model, maxgeom=10000)
        self.context = hand_cv.mujoco.MjrContext(
            model, hand_cv.mujoco.mjtFontScale.mjFONTSCALE_150
        )
        self.camera.type = hand_cv.mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.azimuth = 135.0
        self.camera.elevation = -25.0
        self.camera.distance = 1.25
        self.camera.lookat[:] = np.asarray([0.0, 0.0, 0.28])

        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_mouse_button_callback(self.window, self._on_mouse_button)
        glfw.set_cursor_pos_callback(self.window, self._on_cursor)
        glfw.set_scroll_callback(self.window, self._on_scroll)

    def _on_key(self, _window, key, _scancode, action, _mods):
        if action not in (glfw.PRESS, glfw.REPEAT):
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(self.window, True)
            return
        self.key_callback(key)

    def _on_mouse_button(self, window, _button, _action, _mods):
        self.left_pressed = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        self.right_pressed = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        self.last_x, self.last_y = glfw.get_cursor_pos(window)

    def _on_cursor(self, window, xpos, ypos):
        if not (self.left_pressed or self.right_pressed):
            return
        width, height = glfw.get_window_size(window)
        dx = (xpos - self.last_x) / max(1, height)
        dy = (ypos - self.last_y) / max(1, height)
        self.last_x, self.last_y = xpos, ypos
        shift = any(
            glfw.get_key(window, key) == glfw.PRESS
            for key in (glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT)
        )
        if self.right_pressed:
            action = hand_cv.mujoco.mjtMouse.mjMOUSE_MOVE_H if shift else hand_cv.mujoco.mjtMouse.mjMOUSE_MOVE_V
        else:
            action = hand_cv.mujoco.mjtMouse.mjMOUSE_ROTATE_H if shift else hand_cv.mujoco.mjtMouse.mjMOUSE_ROTATE_V
        hand_cv.mujoco.mjv_moveCamera(
            self.model, action, dx, dy, self.scene, self.camera
        )

    def _on_scroll(self, _window, _xoffset, yoffset):
        hand_cv.mujoco.mjv_moveCamera(
            self.model,
            hand_cv.mujoco.mjtMouse.mjMOUSE_ZOOM,
            0.0,
            -0.05 * yoffset,
            self.scene,
            self.camera,
        )

    def is_running(self) -> bool:
        return self.window is not None and not glfw.window_should_close(self.window)

    def sync(self) -> None:
        glfw.make_context_current(self.window)
        width, height = glfw.get_framebuffer_size(self.window)
        viewport = hand_cv.mujoco.MjrRect(0, 0, width, height)
        hand_cv.mujoco.mjv_updateScene(
            self.model,
            self.data,
            self.option,
            None,
            self.camera,
            hand_cv.mujoco.mjtCatBit.mjCAT_ALL,
            self.scene,
        )
        hand_cv.mujoco.mjr_render(viewport, self.scene, self.context)
        glfw.swap_buffers(self.window)
        glfw.poll_events()

    def close(self) -> None:
        if self.window is not None:
            glfw.make_context_current(self.window)
            self.context.free()
            glfw.destroy_window(self.window)
            self.window = None
            glfw.terminate()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


def rotation_matrix(axis: int, angle: float) -> np.ndarray:
    """Return a 3x3 right-handed rotation about local x/y/z."""
    c, s = float(np.cos(angle)), float(np.sin(angle))
    if axis == 0:
        return np.asarray([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)
    if axis == 1:
        return np.asarray([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)
    return np.asarray([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def orientation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Small-angle world-frame orientation error compatible with mj_jacBody."""
    return 0.5 * sum(
        np.cross(current[:, axis], target[:, axis]) for axis in range(3)
    )


class KeyboardArmController:
    """Incremental Cartesian target and damped-least-squares arm IK."""

    def __init__(
        self,
        model,
        data,
        *,
        translation_step: float,
        rotation_step: float,
        damping: float,
        max_joint_step: float,
        gravcomp: float = 1.0,
    ):
        self.model = model
        self.translation_step = float(translation_step)
        self.rotation_step = float(rotation_step)
        self.damping = max(float(damping), 1e-6)
        self.max_joint_step = max(float(max_joint_step), 1e-5)
        self.lock = threading.Lock()
        self.pending_pos = np.zeros(3, dtype=np.float64)
        self.pending_rpy = np.zeros(3, dtype=np.float64)
        self.home_requested = False
        self.sync_requested = False
        self.print_requested = False

        self.joint_ids = np.asarray(
            [
                hand_cv.mujoco.mj_name2id(
                    model, hand_cv.mujoco.mjtObj.mjOBJ_JOINT, name
                )
                for name in ARM_JOINT_NAMES
            ],
            dtype=np.int32,
        )
        if np.any(self.joint_ids < 0):
            raise RuntimeError("The model must contain joint1 through joint6")
        self.qpos_adr = model.jnt_qposadr[self.joint_ids].astype(np.int32)
        self.dof_adr = model.jnt_dofadr[self.joint_ids].astype(np.int32)
        self.q_min = model.jnt_range[self.joint_ids, 0].copy()
        self.q_max = model.jnt_range[self.joint_ids, 1].copy()

        # Compensate the complete payload below joint1, including the hand.
        # This is MuJoCo's built-in body gravity compensation and avoids the
        # large steady-state Cartesian sag of a pure position servo.
        arm_root_body = int(model.jnt_bodyid[self.joint_ids[0]])
        for body_id in range(1, model.nbody):
            ancestor = body_id
            while ancestor > 0 and ancestor != arm_root_body:
                ancestor = int(model.body_parentid[ancestor])
            if ancestor == arm_root_body:
                model.body_gravcomp[body_id] = float(np.clip(gravcomp, 0.0, 1.0))

        actuator_ids = []
        for joint_id, name in zip(self.joint_ids, ARM_JOINT_NAMES):
            # Tendon actuators also use actuator_trnid[:, 0], but that value is
            # a tendon id rather than a joint id. Filter by transmission type so
            # an AeroHand tendon whose numeric id equals an arm joint id cannot
            # be mistaken for an arm actuator.
            matches = np.flatnonzero(
                (model.actuator_trntype == hand_cv.mujoco.mjtTrn.mjTRN_JOINT)
                & (model.actuator_trnid[:, 0] == joint_id)
            )
            if len(matches) != 1:
                raise RuntimeError(
                    f"Expected one actuator transmitted by {name}, found {len(matches)}"
                )
            actuator_ids.append(int(matches[0]))
        self.actuator_ids = np.asarray(actuator_ids, dtype=np.int32)

        self.body_id = hand_cv.mujoco.mj_name2id(
            model, hand_cv.mujoco.mjtObj.mjOBJ_BODY, TOOL_BODY_NAME
        )
        if self.body_id < 0:
            raise RuntimeError(f"Model is missing tool body {TOOL_BODY_NAME!r}")

        self.jac_pos = np.zeros((3, model.nv), dtype=np.float64)
        self.jac_rot = np.zeros((3, model.nv), dtype=np.float64)
        self.ik_data = hand_cv.mujoco.MjData(model)
        self.target_pos = np.zeros(3, dtype=np.float64)
        self.target_rot = np.eye(3, dtype=np.float64)
        self.command_q = data.qpos[self.qpos_adr].copy()
        self.set_home(data)

    def _tool_pose(self, data) -> tuple[np.ndarray, np.ndarray]:
        return (
            data.xpos[self.body_id].copy(),
            data.xmat[self.body_id].reshape(3, 3).copy(),
        )

    def _pose_at_q(self, data, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        saved_qpos = data.qpos.copy()
        saved_qvel = data.qvel.copy()
        data.qpos[self.qpos_adr] = np.clip(q, self.q_min, self.q_max)
        data.qvel[:] = 0.0
        hand_cv.mujoco.mj_forward(self.model, data)
        pose = self._tool_pose(data)
        data.qpos[:] = saved_qpos
        data.qvel[:] = saved_qvel
        hand_cv.mujoco.mj_forward(self.model, data)
        return pose

    def set_home(self, data) -> None:
        home = np.clip(ARM_HOME, self.q_min, self.q_max)
        data.qpos[self.qpos_adr] = home
        data.qvel[self.dof_adr] = 0.0
        data.ctrl[self.actuator_ids] = home
        hand_cv.mujoco.mj_forward(self.model, data)
        self.command_q = home.copy()
        self.target_pos, self.target_rot = self._tool_pose(data)

    def synchronize(self, data) -> None:
        hand_cv.mujoco.mj_forward(self.model, data)
        self.target_pos, self.target_rot = self._tool_pose(data)
        self.command_q = data.qpos[self.qpos_adr].copy()

    def key_callback(self, keycode: int) -> None:
        """MuJoCo passive-viewer callback; GLFW supplies press/repeat events."""
        dp = np.zeros(3, dtype=np.float64)
        dr = np.zeros(3, dtype=np.float64)
        if keycode == glfw.KEY_S:
            dp[0] += self.translation_step
        elif keycode == glfw.KEY_W:
            dp[0] -= self.translation_step
        elif keycode == glfw.KEY_A:
            dp[1] -= self.translation_step
        elif keycode == glfw.KEY_D:
            dp[1] += self.translation_step
        elif keycode == glfw.KEY_R:
            dp[2] += self.translation_step
        elif keycode == glfw.KEY_F:
            dp[2] -= self.translation_step
        elif keycode == glfw.KEY_LEFT:
            dr[0] -= self.rotation_step
        elif keycode == glfw.KEY_RIGHT:
            dr[0] += self.rotation_step
        elif keycode == glfw.KEY_UP:
            dr[1] -= self.rotation_step
        elif keycode == glfw.KEY_DOWN:
            dr[1] += self.rotation_step
        elif keycode == glfw.KEY_Q:
            dr[2] += self.rotation_step
        elif keycode == glfw.KEY_E:
            dr[2] -= self.rotation_step

        with self.lock:
            self.pending_pos += dp
            self.pending_rpy += dr
            if keycode == glfw.KEY_Z:
                self.home_requested = True
            elif keycode == glfw.KEY_X:
                self.sync_requested = True
            elif keycode == glfw.KEY_P:
                self.print_requested = True

    def apply_pending(self, data) -> None:
        with self.lock:
            dp = self.pending_pos.copy()
            dr = self.pending_rpy.copy()
            do_home = self.home_requested
            do_sync = self.sync_requested
            do_print = self.print_requested
            self.pending_pos.fill(0.0)
            self.pending_rpy.fill(0.0)
            self.home_requested = False
            self.sync_requested = False
            self.print_requested = False

        if do_home:
            self.set_home(data)
            print("[Arm] HOME", self.command_q)
        elif do_sync:
            self.synchronize(data)
            print("[Arm] IK target synchronized to current pose")

        self.target_pos += dp
        # Match the reference controller: post-multiply local rotations.
        for axis in range(3):
            if dr[axis] != 0.0:
                self.target_rot = self.target_rot @ rotation_matrix(axis, dr[axis])

        if do_print:
            current_pos, current_rot = self._tool_pose(data)
            print("[Arm] current position:", np.round(current_pos, 5))
            print("[Arm] target  position:", np.round(self.target_pos, 5))
            print("[Arm] position error:", np.round(self.target_pos - current_pos, 5))
            print("[Arm] rotation error:", np.round(
                orientation_error(current_rot, self.target_rot), 5
            ))

    def step(self, data) -> None:
        self.apply_pending(data)
        # Solve on a kinematic shadow state. This makes command_q converge to a
        # fixed IK solution instead of integrating errors from gravity and the
        # position actuators' transient tracking lag.
        self.ik_data.qpos[:] = data.qpos
        self.ik_data.qpos[self.qpos_adr] = self.command_q
        self.ik_data.qvel[:] = 0.0
        hand_cv.mujoco.mj_forward(self.model, self.ik_data)
        current_pos, current_rot = self._tool_pose(self.ik_data)
        error = np.concatenate(
            [
                self.target_pos - current_pos,
                orientation_error(current_rot, self.target_rot),
            ]
        )
        hand_cv.mujoco.mj_jacBody(
            self.model, self.ik_data, self.jac_pos, self.jac_rot, self.body_id
        )
        jac = np.vstack([self.jac_pos, self.jac_rot])[:, self.dof_adr]
        regularized = jac @ jac.T + (self.damping ** 2) * np.eye(6)
        try:
            dq = jac.T @ np.linalg.solve(regularized, error)
        except np.linalg.LinAlgError:
            dq = jac.T @ np.linalg.lstsq(regularized, error, rcond=None)[0]
        dq = np.clip(dq, -self.max_joint_step, self.max_joint_step)
        # Integrate the kinematic solution in command space.
        self.command_q = np.clip(self.command_q + dq, self.q_min, self.q_max)
        data.ctrl[self.actuator_ids] = self.command_q


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_TASK_MODEL)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--process-width", type=int, default=384)
    parser.add_argument("--vision-hz", type=float, default=20.0)
    parser.add_argument("--model-complexity", type=int, choices=(0, 1), default=0)
    parser.add_argument("--track-hand", choices=("right", "left", "any"), default="right")
    parser.add_argument("--landmark-alpha", type=float, default=0.70)
    parser.add_argument("--command-alpha", type=float, default=0.25)
    parser.add_argument("--open-deadband", type=float, default=0.10)
    parser.add_argument("--response-gamma", type=float, default=1.35)
    parser.add_argument("--lost-hand-open-delay", type=float, default=0.35)
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    parser.add_argument("--gl-mode", choices=("software", "hardware"), default=hand_cv.EARLY_GL_MODE)
    parser.add_argument("--start-enabled", action="store_true")
    parser.add_argument("--no-mirror", action="store_true")
    parser.add_argument("--translation-step", type=float, default=0.003)
    parser.add_argument("--rotation-step", type=float, default=0.02)
    parser.add_argument("--arm-control-hz", type=float, default=100.0)
    parser.add_argument("--arm-gravcomp", type=float, default=1.0)
    parser.add_argument("--ik-damping", type=float, default=0.03)
    parser.add_argument("--max-joint-step", type=float, default=0.025)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model_path = hand_cv.resolve_model_path(args.model)
    if not model_path.is_file():
        raise FileNotFoundError(f"MuJoCo model not found: {model_path}")

    model = hand_cv.mujoco.MjModel.from_xml_path(str(model_path))
    data = hand_cv.mujoco.MjData(model)
    hand_mapper = hand_cv.SimHandMapper(model)
    arm = KeyboardArmController(
        model,
        data,
        translation_step=args.translation_step,
        rotation_step=args.rotation_step,
        damping=args.ik_damping,
        max_joint_step=args.max_joint_step,
        gravcomp=args.arm_gravcomp,
    )
    hand_target = hand_mapper.open_ctrl
    hand_filtered = hand_target.copy()
    hand_mapper.write(data, hand_target)
    hand_cv.mujoco.mj_forward(model, data)

    grabber = hand_cv.LatestFrameGrabber(
        args.camera, args.width, args.height, args.camera_fps
    )
    grabber.start()
    vision = None
    cv_enabled = bool(args.start_enabled)
    show_details = True
    last_sequence = -1
    last_frame = None
    last_frame_time = 0.0
    last_label = "NONE"
    last_ratios = np.zeros(16, dtype=np.float64)
    last_detection_time = time.perf_counter()
    frame_times: list[float] = []
    command_alpha = float(np.clip(args.command_alpha, 0.01, 1.0))
    sim_period = float(model.opt.timestep) / max(0.05, args.realtime_factor)
    next_step = time.perf_counter()
    arm_control_period = 1.0 / max(1.0, args.arm_control_hz)
    next_arm_update = 0.0

    print(f"[Model] {model_path}")
    print(f"[Model] nq={model.nq} nv={model.nv} nu={model.nu}")
    print(f"[Camera] {grabber.negotiated()}")
    print("[Arm keys/viewer] W/S X | A/D Y | R/F Z | arrows roll/pitch | Q/E yaw")
    print("[Arm keys/viewer] Z home | X sync target | P print pose")
    print("[Hand keys/camera] SPACE CV | O open | C close | V details | Q/ESC quit")

    try:
        with ArmControlViewer(model, data, arm.key_callback) as viewer:
            vision = hand_cv.VisionProcessor(grabber, args)
            vision.start()
            while viewer.is_running():
                if grabber.error is not None:
                    raise RuntimeError(f"Camera thread failed: {grabber.error}")
                if vision.error is not None:
                    raise RuntimeError(f"Vision thread failed: {vision.error}")

                new_frame = False
                packet = vision.latest(last_sequence)
                if packet is not None:
                    frame, capture_time, last_sequence, label, ratios, detected = packet
                    last_frame = frame
                    last_frame_time = capture_time
                    last_label = label
                    new_frame = True
                    if detected:
                        last_ratios = ratios
                        last_detection_time = time.perf_counter()
                        if cv_enabled:
                            hand_target = hand_mapper.ratios_to_ctrl(ratios)
                    frame_times.append(time.perf_counter())
                    frame_times = frame_times[-30:]

                if (
                    cv_enabled
                    and time.perf_counter() - last_detection_time
                    >= max(0.0, args.lost_hand_open_delay)
                ):
                    hand_target = hand_mapper.open_ctrl
                    last_ratios.fill(0.0)

                now = time.perf_counter()
                steps = 0
                while now >= next_step and steps < 5:
                    if data.time >= next_arm_update:
                        arm.step(data)
                        next_arm_update = data.time + arm_control_period
                    hand_filtered += command_alpha * (hand_target - hand_filtered)
                    hand_mapper.write(data, hand_filtered)
                    hand_cv.mujoco.mj_step(model, data)
                    next_step += sim_period
                    steps += 1
                if now - next_step > 0.05:
                    next_step = now
                viewer.sync()

                if last_frame is not None and new_frame:
                    display = last_frame.copy()
                    fps = 0.0
                    if len(frame_times) >= 2:
                        fps = (len(frame_times) - 1) / max(
                            frame_times[-1] - frame_times[0], 1e-6
                        )
                    age_ms = (time.perf_counter() - last_frame_time) * 1000.0
                    status = "ON" if cv_enabled else "OFF"
                    cv2.putText(
                        display,
                        f"HAND CV {status} | {last_label} | {fps:.1f} FPS | {age_ms:.0f} ms",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                        (40, 230, 60) if cv_enabled else (80, 190, 255),
                        2, cv2.LINE_AA,
                    )
                    if show_details:
                        curls = [
                            last_ratios[0],
                            np.mean(last_ratios[4:7]),
                            np.mean(last_ratios[7:10]),
                            np.mean(last_ratios[10:13]),
                            np.mean(last_ratios[13:16]),
                        ]
                        cv2.putText(
                            display,
                            "curl T/I/M/R/P " + " / ".join(f"{v:.2f}" for v in curls),
                            (12, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                            (240, 240, 240), 1, cv2.LINE_AA,
                        )
                    cv2.imshow("reBot Arm Keyboard + AeroHand CV", display)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key == ord(" "):
                    cv_enabled = not cv_enabled
                    print(f"[Hand CV] {'ON' if cv_enabled else 'OFF'}")
                elif key in (ord("o"), ord("O")):
                    cv_enabled = False
                    hand_target = hand_mapper.open_ctrl
                    print("[Hand] OPEN")
                elif key in (ord("c"), ord("C")):
                    cv_enabled = False
                    hand_target = hand_mapper.closed_ctrl
                    print("[Hand] CLOSED")
                elif key in (ord("v"), ord("V")):
                    show_details = not show_details

                time.sleep(0.001)
    finally:
        if vision is not None:
            vision.close()
        grabber.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
