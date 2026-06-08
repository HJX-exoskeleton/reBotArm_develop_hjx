#!/usr/bin/env python3
"""reBotArm 重力补偿 + MuJoCo real2sim 数字孪生 (7轴全物理拖拽示教版)。

功能特性:
1. 1-6 轴运行 MIT 模式重力补偿，支持零力拖拽。
2. 第 7 轴（夹爪）运行 MIT 零力矩模式（纯软化），支持手动开合。
3. 仿真系统实时读取 1-7 轴的真实物理反馈，并在 MuJoCo 窗口中 1:1 同步再现。
"""

from __future__ import annotations

import argparse
import importlib.util
import signal
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# --------------------------------------------------------------------------- #
# 配置参数
# --------------------------------------------------------------------------- #
ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_XML = ROOT_DIR / "mujoco" / "xml" / "rebot_gripper" / "reBot-DevArm_gripper.xml"
DEFAULT_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
DEFAULT_GRIPPER_CFG = ROOT_DIR / "config" / "gripper.yaml"

# 【标定参数】：用于将真机弧度(rad)转换为仿真位移(m)
GRIPPER_MOTOR_NAME = "gripper"
# 假设真机夹爪全开是 0.8 rad，仿真全开是 0.035 m
GRIPPER_REAL_MAX_RAD = 1.8
GRIPPER_SIM_MAX_METER = 0.5

TORQUE_LIMITS = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
KD_CONFIG = np.array([1.0, 2.0, 1.5, 1.0, 0.8, 0.6])
GRAVITY_SCALES = np.array([1.50, 0.75, 0.75, 0.75, 1.0, 1.0])

_running = True


def _sigint_handler(signum, frame) -> None:
    global _running
    print("\n[real2sim] 收到退出信号，准备安全关闭...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)


def _load_robot_arm_class():
    arm_py = ROOT_DIR / "reBotArm_control_py" / "actuator" / "arm.py"
    spec = importlib.util.spec_from_file_location("_rebotarm_actuator_arm", arm_py)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.RobotArm


def _load_gripper_cfg_func():
    gripper_py = ROOT_DIR / "reBotArm_control_py" / "actuator" / "gripper.py"
    spec = importlib.util.spec_from_file_location("_rebotarm_actuator_gripper", gripper_py)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.load_cfg


def _load_gravity_functions():
    from reBotArm_control_py.dynamics import load_dynamics_model, compute_generalized_gravity
    return load_dynamics_model, compute_generalized_gravity


class RealToSimMapper:
    def __init__(self, model: mujoco.MjModel, joint_names: tuple[str, ...]) -> None:
        self.model = model
        self.joint_names = joint_names
        self.joint_ids = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names],
                                  dtype=np.int32)
        self.qpos_addrs = np.array([model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.actuator_ids = []
        for jid in self.joint_ids:
            for act_id in range(model.nu):
                if int(model.actuator_trntype[act_id]) == int(mujoco.mjtTrn.mjTRN_JOINT) and int(
                        model.actuator_trnid[act_id, 0]) == jid:
                    self.actuator_ids.append(act_id)
                    break
            else:
                self.actuator_ids.append(None)

    def apply(self, data: mujoco.MjData, q_real: np.ndarray) -> np.ndarray:
        q_sim = np.asarray(q_real, dtype=np.float64)[: len(self.joint_names)]
        data.qpos[self.qpos_addrs] = q_sim
        for i, act_id in enumerate(self.actuator_ids):
            if act_id is not None:
                data.ctrl[act_id] = float(q_sim[i])
        mujoco.mj_forward(self.model, data)
        return q_sim


class GravityCompensationState:
    def __init__(self, num_joints: int) -> None:
        self._lock = threading.Lock()
        self.q = np.zeros(num_joints, dtype=np.float64)
        self.tau_g = np.zeros(num_joints, dtype=np.float64)
        self.has_feedback = False

    def update(self, q: np.ndarray, tau_g: np.ndarray) -> None:
        with self._lock:
            self.q = np.asarray(q, dtype=np.float64).copy()
            self.tau_g = np.asarray(tau_g, dtype=np.float64).copy()
            self.has_feedback = True

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, bool]:
        with self._lock:
            return self.q.copy(), self.tau_g.copy(), self.has_feedback


def make_gravity_compensation_controller(state: GravityCompensationState, compute_generalized_gravity):
    def controller(arm, dt: float) -> None:
        if not _running: return
        q, _, _ = arm.get_state()
        q_arm_6d = q[:6]

        tau_g = compute_generalized_gravity(q=q_arm_6d) * GRAVITY_SCALES[:6]
        tau_g_safe = np.clip(tau_g, -TORQUE_LIMITS[:6], TORQUE_LIMITS[:6])

        # 1. 逐个对前 6 轴底层电机下发 MIT 重力补偿
        for i, jname in enumerate(DEFAULT_JOINT_NAMES):
            try:
                mot = arm._motor_map.get(jname)
                if mot:
                    mot.send_mit(float(q_arm_6d[i]), 0.0, 0.0, float(KD_CONFIG[i]), float(tau_g_safe[i]))
            except Exception:
                pass

        # 2. 🚨【核心修改】对第 7 轴（夹爪）下发纯 0 力矩的 MIT 指令，使其保持软绵绵状态
        try:
            mot_g = arm._motor_map.get(GRIPPER_MOTOR_NAME)
            if mot_g:
                # pos=0, vel=0, kp=0, kd=0, tau=0 -> 完全失去刚度，任人拖拽
                mot_g.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass

        # 统一进行总线收发轮询
        try:
            for mot in arm._motor_map.values():
                mot.request_feedback()
            for ctrl in arm._ctrl_map.values():
                ctrl.poll_feedback_once()
        except Exception:
            pass

        state.update(q_arm_6d, tau_g_safe)

    return controller


def main() -> None:
    global _running
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML)
    parser.add_argument("--cfg", type=Path, default=None)
    parser.add_argument("--gripper-cfg", type=Path, default=DEFAULT_GRIPPER_CFG)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--print-every", type=int, default=60)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    mapper = RealToSimMapper(model, DEFAULT_JOINT_NAMES)

    gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_actuator")
    if gripper_act_id < 0:
        gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")

    RobotArm = _load_robot_arm_class()
    load_gripper_cfg = _load_gripper_cfg_func()
    load_dynamics_model, compute_generalized_gravity = _load_gravity_functions()
    load_dynamics_model()

    g_cfg = load_gripper_cfg(str(args.gripper_cfg))["gripper"]
    arm = RobotArm(cfg_path=str(args.cfg) if args.cfg is not None else None)
    state = GravityCompensationState(6)

    # 存放夹爪电机实例
    gripper_motor_obj = None

    print("\n" + "=" * 60)
    print("  reBotArm Real2Sim: 7轴全物理拖拽示教 (大臂重力补偿 + 夹爪零力)")
    print("=" * 60)

    try:
        arm.connect()
        arm.enable()
        time.sleep(0.2)

        # =================================================================== #
        # 将夹爪注册到共享总线，并将其配置为 MIT 模式
        # =================================================================== #
        if "damiao" in arm._ctrl_map:
            shared_damiao_controller = arm._ctrl_map["damiao"]
            g_mot = shared_damiao_controller.add_damiao_motor(g_cfg.motor_id, g_cfg.feedback_id, g_cfg.model)
            arm._motor_map[g_cfg.name] = g_mot
            gripper_motor_obj = g_mot

            try:
                from motorbridge import Mode
                # 🚨【核心修改】夹爪也配置为 MIT 模式，而不是 POS_VEL
                g_mot.ensure_mode(Mode.MIT, 1000)
                shared_damiao_controller.enable_all()
                time.sleep(0.2)
                print("✅ [真机 J7] 独立夹爪已切入 MIT 零力模式，现在可以用手掰动它了！")
            except Exception as e:
                print(f"❌ [错误] 夹爪配置失败: {e}")
        # =================================================================== #

        arm.mode_mit(kp=np.zeros(6), kd=KD_CONFIG[:6])
        arm.start_control_loop(make_gravity_compensation_controller(state, compute_generalized_gravity))

        frame = 0
        period = 1.0 / args.rate

        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("\n👀 视窗已启动！现在你可以随意推拉大臂和夹爪，仿真会 1:1 完美同步！")
            while _running and viewer.is_running():
                t0 = time.perf_counter()
                q_real_6d, tau_g, has_feedback = state.snapshot()

                # 读取并映射夹爪真实物理角度
                q_gripper_real = 0.0
                sim_gripper_cmd = 0.0
                if gripper_motor_obj is not None:
                    # 获取电机的最新状态
                    st = gripper_motor_obj.get_state()
                    if st is not None:
                        q_gripper_real = st.pos

                        # 【线性映射】将真机的弧度(rad)等比例映射为仿真的位移(m)
                        # 公式：(当前真机角度 / 真机最大开合角度) * 仿真最大位移
                        ratio = abs(q_gripper_real) / GRIPPER_REAL_MAX_RAD
                        sim_gripper_cmd = float(np.clip(ratio * GRIPPER_SIM_MAX_METER, 0.0, GRIPPER_SIM_MAX_METER))

                        if gripper_act_id >= 0:
                            data.ctrl[gripper_act_id] = sim_gripper_cmd

                if has_feedback:
                    mapper.apply(data, q_real_6d)
                    viewer.sync()

                    if args.print_every > 0 and frame % args.print_every == 0:
                        q_str = " ".join(f"{v:+.3f}" for v in q_real_6d)
                        sys.stdout.write("\033[K")
                        print(
                            f"[{frame:06d}] 6D_Arm=[{q_str}] | J7_Real_Rad={q_gripper_real:+.3f} -> Sim_m={sim_gripper_cmd:.4f}",
                            end="\r")

                frame += 1
                time.sleep(max(0, period - (time.perf_counter() - t0)))

    except KeyboardInterrupt:
        _running = False
    finally:
        print("\n\n[退出流程] 正在释放总线并关闭系统...")
        try:
            arm.stop_control_loop()
            arm.disable()
            arm.disconnect()
        except Exception:
            pass
        print("[退出流程] 安全退出。")


if __name__ == "__main__":
    main()
