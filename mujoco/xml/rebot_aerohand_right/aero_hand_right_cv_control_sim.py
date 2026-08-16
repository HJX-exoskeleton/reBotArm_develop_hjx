#!/usr/bin/env python3
"""Webcam mapping control for the simulated right AeroHand on reBot.

Install
-------
pip install mujoco mediapipe opencv-python numpy

Run from the project root
-------------------------
python xml/aerohand_right/aero_hand_right_cv_control_sim.py

Run with explicit right-hand model and Linux low-latency camera settings
------------------------------------------------------------------------
python xml/aerohand_right/aero_hand_right_cv_control_sim.py --start-enabled

Start with webcam control enabled
---------------------------------
python xml/aerohand_right/aero_hand_right_cv_control_sim.py --start-enabled

Linux OpenGL modes
------------------
python xml/aerohand_right/aero_hand_right_cv_control_sim.py --gl-mode software
python xml/aerohand_right/aero_hand_right_cv_control_sim.py --gl-mode hardware

Pipeline
--------
camera -> MediaPipe 21 landmarks -> wrist-aligned landmarks -> normalized
finger curls -> the seven named AeroHand actuators in MuJoCo.

Only the simulated hand is commanded.  The six reBot actuator controls are
left unchanged, and no robot SDK, ROS package, or physical hand is used.

Keys (focus the OpenCV camera window)
-------------------------------------
SPACE : enable/disable live control
O     : command simulated hand OPEN
C     : command simulated hand CLOSED
R     : reset landmark and command filters
V     : show/hide diagnostic overlay
Q/ESC : quit
"""

from __future__ import annotations

import argparse
import os
import platform
import sys
import threading
import time
from pathlib import Path

# Select the OpenGL implementation before importing cv2, MediaPipe, GLFW, or
# MuJoCo. Hardware GLFW rendering is the responsive default. The optional
# software mode uses Mesa llvmpipe and removes stale library paths locally.
def _early_option(name: str, default: str) -> str:
    args = sys.argv[1:]
    for index, item in enumerate(args):
        if item == name and index + 1 < len(args):
            return args[index + 1]
        if item.startswith(name + "="):
            return item.split("=", 1)[1]
    return default


EARLY_GL_MODE = _early_option("--gl-mode", "hardware").lower()
if platform.system() == "Linux":
    os.environ.setdefault("MUJOCO_GL", "glfw")
    if EARLY_GL_MODE == "software":
        old_paths = os.environ.get("LD_LIBRARY_PATH", "").split(":")
        clean_paths = [
            item for item in old_paths
            if item
            and ".mujoco/mujoco-" not in item
            and item.rstrip("/") != "/usr/lib/nvidia"
        ]
        os.environ["LD_LIBRARY_PATH"] = ":".join(clean_paths)
        os.environ["LIBGL_DRIVERS_PATH"] = "/usr/lib/x86_64-linux-gnu/dri"
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
        os.environ["MESA_LOADER_DRIVER_OVERRIDE"] = "swrast"

import cv2
import mediapipe as mp
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent

try:
    import mujoco
    import mujoco.viewer
except ImportError as exc:
    raise SystemExit("MuJoCo is required: pip install mujoco") from exc

# Calibration in retarget_25 order: thumb(4), index(3), middle(3), ring(3), pinky(3).
MEDIAPIPE_VALLEY = np.asarray(
    [
        0.0, 0.51, 0.0, 0.0,
        0.708, 0.02, 0.25,
        0.708, 0.02, 0.25,
        0.708, 0.02, 0.25,
        0.47, 0.005, 0.24,
    ],
    dtype=np.float64,
)
MEDIAPIPE_PEAK = np.asarray(
    [
        0.705, 0.956, 0.70, 0.78,
        1.20, 1.57, 1.57,
        1.20, 1.57, 1.57,
        1.20, 1.57, 1.57,
        1.571, 1.57, 1.36,
    ],
    dtype=np.float64,
)
MEDIAPIPE_SPAN = np.maximum(MEDIAPIPE_PEAK - MEDIAPIPE_VALLEY, 1e-6)

HAND_ACTUATOR_NAMES = (
    "right_index_A_tendon",
    "right_middle_A_tendon",
    "right_ring_A_tendon",
    "right_pinky_A_tendon",
    "right_thumb_A_cmc_abd",
    "right_th1_A_tendon",
    "right_th2_A_tendon",
)

def unit_vector(value: np.ndarray, eps: float = 1e-9) -> np.ndarray | None:
    norm = float(np.linalg.norm(value))
    if norm <= eps:
        return None
    return value / norm


def canonicalize_landmarks(landmarks: np.ndarray, hand_label: str):
    """Wrist-center and align MediaPipe coordinates like webcam_mocap.py."""
    x_axis = unit_vector(landmarks[5] - landmarks[13])
    z_axis = unit_vector(landmarks[9] - landmarks[0])
    if x_axis is None or z_axis is None:
        return None
    if hand_label == "left":
        x_axis = -x_axis
    y_axis = unit_vector(np.cross(z_axis, x_axis))
    if y_axis is None:
        return None
    x_axis = unit_vector(np.cross(y_axis, z_axis))
    if x_axis is None:
        return None
    rotation = np.asarray([x_axis, y_axis, z_axis], dtype=np.float64).T
    return (landmarks - landmarks[0]) @ rotation


def make_25_landmarks(landmarks: np.ndarray) -> np.ndarray:
    return np.asarray(
        [
            *landmarks[0:5],
            landmarks[0], *landmarks[5:9],
            landmarks[0], *landmarks[9:13],
            landmarks[0], *landmarks[13:17],
            landmarks[0], *landmarks[17:21],
        ],
        dtype=np.float64,
    )


def angle_three(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    ba = a - b
    bc = c - b
    denom = float(np.linalg.norm(ba) * np.linalg.norm(bc))
    if denom <= 1e-9:
        return 0.0
    cosine = float(np.clip(np.dot(ba, bc) / denom, -1.0, 1.0))
    return float(np.arccos(cosine))


def finger_joints(points: np.ndarray) -> list[float]:
    return [
        np.pi - angle_three(points[0], points[1], points[2]),
        np.pi - angle_three(points[1], points[2], points[3]),
        np.pi - angle_three(points[2], points[3], points[4]),
    ]


def thumb_joints(points: np.ndarray) -> list[float]:
    a = np.asarray([points[2, 0], points[1, 1], 0.0])
    b = np.asarray([points[1, 0], points[1, 1], 0.0])
    c = np.asarray([points[2, 0], points[2, 1], 0.0])
    return [
        angle_three(a, b, c),
        np.pi - angle_three(points[0], points[1], points[2]),
        np.pi - angle_three(points[1], points[2], points[3]),
        np.pi - angle_three(points[2], points[3], points[4]),
    ]


def retarget_25(landmarks: np.ndarray) -> np.ndarray:
    result = thumb_joints(landmarks[0:5])
    result += finger_joints(landmarks[5:10])
    result += finger_joints(landmarks[10:15])
    result += finger_joints(landmarks[15:20])
    result += finger_joints(landmarks[20:25])
    return np.asarray(result, dtype=np.float64)


class EMA:
    def __init__(self, alpha: float):
        self.alpha = float(np.clip(alpha, 0.01, 1.0))
        self.value = None

    def reset(self):
        self.value = None

    def update(self, value: np.ndarray) -> np.ndarray:
        value = np.asarray(value, dtype=np.float64)
        if self.value is None:
            self.value = value.copy()
        else:
            self.value += self.alpha * (value - self.value)
        return self.value.copy()


class LatestFrameGrabber(threading.Thread):
    def __init__(self, index: int, width: int, height: int, fps: float):
        super().__init__(daemon=True)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.frame = None
        self.timestamp = 0.0
        self.sequence = 0
        self.error = None

        backend = cv2.CAP_V4L2 if platform.system() == "Linux" else cv2.CAP_DSHOW
        self.cap = cv2.VideoCapture(int(index), backend)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = cv2.VideoCapture(int(index))
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index={index}")

        # Linux V4L2 should select compressed transport before size/FPS.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        self.cap.set(cv2.CAP_PROP_FPS, float(fps))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def negotiated(self) -> str:
        code = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))
        return (
            f"backend={self.cap.getBackendName()} "
            f"size={int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
            f"fps={self.cap.get(cv2.CAP_PROP_FPS):.1f} fourcc={fourcc!r}"
        )

    def run(self):
        try:
            while not self.stop_event.is_set():
                ok, frame = self.cap.read()
                if not ok:
                    time.sleep(0.003)
                    continue
                with self.lock:
                    self.frame = frame
                    self.timestamp = time.perf_counter()
                    self.sequence += 1
        except Exception as exc:
            self.error = exc

    def latest(self, last_sequence: int):
        with self.lock:
            if self.frame is None or self.sequence == last_sequence:
                return None
            return self.frame.copy(), self.timestamp, self.sequence

    def close(self):
        self.stop_event.set()
        self.join(timeout=1.0)
        self.cap.release()


class VisionProcessor(threading.Thread):
    """Run MediaPipe off the MuJoCo/GLFW viewer thread."""

    def __init__(self, grabber: LatestFrameGrabber, args):
        super().__init__(daemon=True)
        self.grabber = grabber
        self.args = args
        self.stop_event = threading.Event()
        self.reset_event = threading.Event()
        self.lock = threading.Lock()
        self.sequence = 0
        self.frame = None
        self.timestamp = 0.0
        self.label = "NONE"
        self.ratios = np.zeros(16, dtype=np.float64)
        self.detected = False
        self.error = None

    def latest(self, last_sequence: int):
        with self.lock:
            if self.frame is None or self.sequence == last_sequence:
                return None
            return (
                self.frame.copy(), self.timestamp, self.sequence,
                self.label, self.ratios.copy(), self.detected,
            )

    def run(self):
        last_camera_sequence = -1
        landmark_filter = EMA(self.args.landmark_alpha)
        period = 1.0 / max(1.0, self.args.vision_hz)
        next_process = time.perf_counter()
        try:
            with mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                model_complexity=self.args.model_complexity,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            ) as hands:
                while not self.stop_event.is_set():
                    if self.reset_event.is_set():
                        landmark_filter.reset()
                        self.reset_event.clear()
                    now = time.perf_counter()
                    if now < next_process:
                        time.sleep(min(next_process - now, 0.005))
                        continue
                    packet = self.grabber.latest(last_camera_sequence)
                    if packet is None:
                        time.sleep(0.002)
                        continue
                    frame, capture_time, last_camera_sequence = packet
                    next_process = now + period
                    if not self.args.no_mirror:
                        frame = cv2.flip(frame, 1)

                    h, w = frame.shape[:2]
                    process_width = min(w, max(256, int(self.args.process_width)))
                    if process_width != w:
                        process_frame = cv2.resize(
                            frame, (process_width, max(1, round(h * process_width / w))),
                            interpolation=cv2.INTER_AREA,
                        )
                    else:
                        process_frame = frame
                    rgb = cv2.cvtColor(process_frame, cv2.COLOR_BGR2RGB)
                    rgb.flags.writeable = False
                    results = hands.process(rgb)
                    selected = None
                    if results.multi_hand_landmarks and results.multi_hand_world_landmarks:
                        for lm2d, world, handedness in zip(
                            results.multi_hand_landmarks,
                            results.multi_hand_world_landmarks,
                            results.multi_handedness,
                        ):
                            label = handedness.classification[0].label.lower()
                            mp.solutions.drawing_utils.draw_landmarks(
                                frame, lm2d, mp.solutions.hands.HAND_CONNECTIONS
                            )
                            if self.args.track_hand == "any" or label == self.args.track_hand:
                                selected = world, label
                                break

                    detected = False
                    label_text = "NONE"
                    ratios = self.ratios.copy()
                    if selected is not None:
                        world, label = selected
                        points = np.asarray(
                            [[-lm.x, lm.y, lm.z] for lm in world.landmark],
                            dtype=np.float64,
                        )
                        canonical = canonicalize_landmarks(points, label)
                        if canonical is not None:
                            smooth = landmark_filter.update(make_25_landmarks(canonical))
                            raw = retarget_25(smooth)
                            ratios = np.clip(
                                (raw - MEDIAPIPE_VALLEY) / MEDIAPIPE_SPAN, 0.0, 1.0
                            )
                            ratios[ratios < self.args.open_deadband] = 0.0
                            ratios = np.power(
                                ratios, max(0.2, float(self.args.response_gamma))
                            )
                            detected = True
                            label_text = label.upper()

                    with self.lock:
                        self.frame = frame
                        self.timestamp = capture_time
                        self.label = label_text
                        self.ratios = ratios
                        self.detected = detected
                        self.sequence += 1
        except Exception as exc:
            self.error = exc

    def close(self):
        self.stop_event.set()
        self.join(timeout=2.0)

    def reset_filter(self):
        self.reset_event.set()


class SimHandMapper:
    """Map 16 normalized MediaPipe joints directly to seven sim controls."""

    def __init__(self, model):
        self.model = model
        ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
               for n in HAND_ACTUATOR_NAMES]
        if any(i < 0 for i in ids):
            missing = [n for n, i in zip(HAND_ACTUATOR_NAMES, ids) if i < 0]
            raise RuntimeError(f"MuJoCo model is missing hand actuators: {missing}")
        self.ids = np.asarray(ids, dtype=np.int32)
        self.ctrl_min = model.actuator_ctrlrange[self.ids, 0].copy()
        self.ctrl_max = model.actuator_ctrlrange[self.ids, 1].copy()
        # Tendons follow the reference simulation: high length=open and low
        # length=closed.  Thumb abduction instead increases while closing.
        self._open_ctrl = self.ctrl_max.copy()
        self._closed_ctrl = self.ctrl_min.copy()
        self._open_ctrl[4] = 0.0
        self._closed_ctrl[4] = min(1.5, self.ctrl_max[4])

    def ratios_to_ctrl(self, ratios: np.ndarray) -> np.ndarray:
        """Convert normalized joint curls (0=open, 1=closed) to XML ctrl."""
        ratios = np.clip(np.asarray(ratios, dtype=np.float64), 0.0, 1.0)
        if ratios.shape != (16,):
            raise ValueError(f"Expected 16 joint ratios, got {ratios.shape}")

        closure = np.asarray(
            [
                np.mean(ratios[4:7]),    # index
                np.mean(ratios[7:10]),   # middle
                np.mean(ratios[10:13]),  # ring
                np.mean(ratios[13:16]),  # pinky
                ratios[0],               # thumb abduction
                np.mean(ratios[1:3]),    # thumb proximal tendon
                np.mean(ratios[2:4]),    # thumb distal tendon
            ],
            dtype=np.float64,
        )
        return self._open_ctrl + closure * (self._closed_ctrl - self._open_ctrl)

    @property
    def open_ctrl(self) -> np.ndarray:
        return self._open_ctrl.copy()

    @property
    def closed_ctrl(self) -> np.ndarray:
        return self._closed_ctrl.copy()

    def write(self, data, ctrl: np.ndarray):
        data.ctrl[self.ids] = np.clip(ctrl, self.ctrl_min, self.ctrl_max)


def parse_args():
    parser = argparse.ArgumentParser(description="Right AeroHand-only CV control in MuJoCo")
    parser.add_argument(
        "--model",
        default=str(SCRIPT_DIR / "mujoco_xml" / "rebot_arm_right_hand.xml"),
    )
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
    parser.add_argument(
        "--response-gamma", type=float, default=1.35,
        help="Greater than 1 makes nearly-open gestures map closer to fully open.",
    )
    parser.add_argument(
        "--lost-hand-open-delay", type=float, default=0.35,
        help="Open the simulated hand after detection is lost for this many seconds.",
    )
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    parser.add_argument(
        "--gl-mode",
        choices=("software", "hardware"),
        default=EARLY_GL_MODE,
        help="Linux viewer renderer; hardware is faster, software is the fallback.",
    )
    parser.add_argument("--start-enabled", action="store_true")
    parser.add_argument("--no-mirror", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    model_path = Path(args.model).expanduser().resolve()
    if not model_path.exists():
        raise FileNotFoundError(f"MuJoCo model not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mapper = SimHandMapper(model)
    open_ctrl = mapper.open_ctrl
    closed_ctrl = mapper.closed_ctrl
    target_ctrl = open_ctrl.copy()
    filtered_ctrl = open_ctrl.copy()
    mapper.write(data, open_ctrl)
    mujoco.mj_forward(model, data)

    grabber = LatestFrameGrabber(
        args.camera, args.width, args.height, args.camera_fps
    )
    print(f"[Model] {model_path}")
    print(f"[Model] nq={model.nq} nv={model.nv} nu={model.nu}")
    print(f"[Camera] {grabber.negotiated()}")
    print("[Keys] SPACE control | O open | C closed | R reset | V details | Q/ESC quit")
    grabber.start()

    # Start MediaPipe only after GLFW owns its context, and keep inference off
    # the viewer thread so camera processing cannot stall mouse interaction.
    vision = None
    enabled = bool(args.start_enabled)
    show_details = True
    last_sequence = -1
    last_frame = None
    last_frame_time = 0.0
    last_label = "NONE"
    last_ratios = np.zeros(16, dtype=np.float64)
    last_detection_time = time.perf_counter()
    frame_times = []
    command_alpha = float(np.clip(args.command_alpha, 0.01, 1.0))
    sim_period = float(model.opt.timestep) / max(0.05, args.realtime_factor)
    next_step = time.perf_counter()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print(f"[Viewer] OpenGL mode={args.gl_mode}; MuJoCo viewer created")
            vision = VisionProcessor(grabber, args)
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
                    new_frame = True
                    last_frame_time = capture_time
                    last_label = label
                    if detected:
                        last_ratios = ratios
                        last_detection_time = time.perf_counter()
                        if enabled:
                            target_ctrl = mapper.ratios_to_ctrl(ratios)

                    frame_times.append(time.perf_counter())
                    frame_times = frame_times[-30:]

                if (enabled and time.perf_counter() - last_detection_time
                        >= max(0.0, args.lost_hand_open_delay)):
                    target_ctrl = open_ctrl.copy()
                    last_ratios.fill(0.0)

                now = time.perf_counter()
                steps = 0
                while now >= next_step and steps < 5:
                    filtered_ctrl += command_alpha * (target_ctrl - filtered_ctrl)
                    mapper.write(data, filtered_ctrl)
                    mujoco.mj_step(model, data)
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
                    status = "ON" if enabled else "OFF"
                    cv2.putText(
                        display,
                        f"SIM CONTROL {status} | {last_label} | {fps:.1f} FPS | age {age_ms:.0f} ms",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                        (40, 230, 60) if enabled else (80, 190, 255), 2, cv2.LINE_AA,
                    )
                    if show_details:
                        curls = [last_ratios[0], *[np.mean(last_ratios[s]) for s in (
                            slice(4, 7), slice(7, 10), slice(10, 13), slice(13, 16)
                        )]]
                        cv2.putText(
                            display,
                            "curl T/I/M/R/P " + " / ".join(f"{v:.2f}" for v in curls),
                            (12, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                            (240, 240, 240), 1, cv2.LINE_AA,
                        )
                    cv2.imshow("Aero Hand Right CV Control (non-ROS)", display)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key == ord(" "):
                    enabled = not enabled
                    print(f"[Control] {'ON' if enabled else 'OFF'}")
                elif key in (ord("o"), ord("O")):
                    enabled = False
                    target_ctrl = open_ctrl.copy()
                    print("[Control] OPEN")
                elif key in (ord("c"), ord("C")):
                    enabled = False
                    target_ctrl = closed_ctrl.copy()
                    print("[Control] CLOSED")
                elif key in (ord("r"), ord("R")):
                    if vision is not None:
                        vision.reset_filter()
                    target_ctrl = open_ctrl.copy()
                    filtered_ctrl = open_ctrl.copy()
                    print("[Control] filters reset")
                elif key in (ord("v"), ord("V")):
                    show_details = not show_details

                time.sleep(0.001)
    finally:
        if vision is not None:
            vision.close()
        grabber.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
