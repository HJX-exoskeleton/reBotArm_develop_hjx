#!/usr/bin/env python3
"""reBotArm 重力补偿 + MuJoCo real2sim 数字孪生 (7轴全物理拖拽示教版)。"""

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

# 配置参数
ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_XML = ROOT_DIR / "mujoco" / "xml" / "rebot_gripper" / "reBot-DevArm_gripper.xml"
DEFAULT_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
DEFAULT_GRIPPER_CFG = ROOT_DIR / "config" / "gripper.yaml"

# 夹爪标定参数
GRIPPER_MOTOR_NAME = "gripper"
GRIPPER_REAL_CLOSED_RAD = 0.0  # 真机闭合角度
GRIPPER_REAL_OPEN_RAD = -5.8  # 真机张开角度
GRIPPER_SIM_CLOSED_METER = 0.001  # 仿真闭合位置
GRIPPER_SIM_OPEN_METER = 0.05  # 仿真张开位置

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

        for i, jname in enumerate(DEFAULT_JOINT_NAMES):
            try:
                mot = arm._motor_map.get(jname)
                if mot:
                    mot.send_mit(float(q_arm_6d[i]), 0.0, 0.0, float(KD_CONFIG[i]), float(tau_g_safe[i]))
            except Exception:
                pass

        try:
            mot_g = arm._motor_map.get(GRIPPER_MOTOR_NAME)
            if mot_g:
                mot_g.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass

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
    parser.add_argument("--smooth", type=float, default=0.02, help="平滑因子 (0.0-1.0)，越小越平滑")
    parser.add_argument("--max-speed", type=float, default=0.15, help="最大夹爪闭合速度(m/s)")
    args = parser.parse_args()

    # 加载模型
    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    mapper = RealToSimMapper(model, DEFAULT_JOINT_NAMES)

    # 查找夹爪执行器
    gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")
    if gripper_act_id < 0:
        print("❌ 未找到夹爪执行器 'gripper'")
        return

    # 获取执行器控制范围
    ctrlrange = model.actuator_ctrlrange[gripper_act_id]
    print(f"✅ 找到夹爪执行器，控制范围: [{ctrlrange[0]:.4f}, {ctrlrange[1]:.4f}] m")

    # 加载机器人控制类
    RobotArm = _load_robot_arm_class()
    load_gripper_cfg = _load_gripper_cfg_func()
    load_dynamics_model, compute_generalized_gravity = _load_gravity_functions()
    load_dynamics_model()

    g_cfg = load_gripper_cfg(str(args.gripper_cfg))["gripper"]
    arm = RobotArm(cfg_path=str(args.cfg) if args.cfg is not None else None)
    state = GravityCompensationState(6)

    print("\n" + "=" * 60)
    print("  reBotArm Real2Sim: 7轴全物理拖拽示教")
    print("=" * 60)
    print(f"[夹爪标定] 真机: [{GRIPPER_REAL_CLOSED_RAD:.1f}, {GRIPPER_REAL_OPEN_RAD:.1f}] rad")
    print(f"[夹爪标定] 仿真: [{GRIPPER_SIM_CLOSED_METER:.3f}, {GRIPPER_SIM_OPEN_METER:.3f}] m")
    print(f"[控制参数] 平滑因子: {args.smooth:.2f} | 最大速度: {args.max_speed:.2f} m/s")

    try:
        arm.connect()
        arm.enable()
        time.sleep(0.2)

        # 配置夹爪电机
        if "damiao" in arm._ctrl_map:
            shared_damiao_controller = arm._ctrl_map["damiao"]
            g_mot = shared_damiao_controller.add_damiao_motor(g_cfg.motor_id, g_cfg.feedback_id, g_cfg.model)
            arm._motor_map[g_cfg.name] = g_mot
            gripper_motor_obj = g_mot

            try:
                from motorbridge import Mode
                g_mot.ensure_mode(Mode.MIT, 1000)
                shared_damiao_controller.enable_all()
                time.sleep(0.2)
                print("✅ 夹爪已切入 MIT 零力模式")
            except Exception as e:
                print(f"❌ 夹爪配置失败: {e}")

        arm.mode_mit(kp=np.zeros(6), kd=KD_CONFIG[:6])
        arm.start_control_loop(make_gravity_compensation_controller(state, compute_generalized_gravity))

        frame = 0
        period = 1.0 / args.rate
        last_gripper_real_pos = 0.0
        current_ctrl_value = 0.05  # 初始为张开状态
        last_gripper_cmd = 0.05
        smooth_factor = args.smooth
        max_displacement_per_frame = args.max_speed * period  # 每帧最大位移

        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("\n👀 视窗已启动！")
            print("💡 可以随意拖拽机械臂和夹爪，仿真会实时同步")

            # 先设置一个初始控制值
            data.ctrl[gripper_act_id] = current_ctrl_value
            mujoco.mj_step(model, data)
            viewer.sync()

            while _running and viewer.is_running():
                t0 = time.perf_counter()
                q_real_6d, tau_g, has_feedback = state.snapshot()

                # 读取夹爪真实角度
                q_gripper_real = 0.0
                if gripper_motor_obj is not None and gripper_act_id >= 0:
                    st = gripper_motor_obj.get_state()
                    if st is not None:
                        q_gripper_real = st.pos

                        # 线性映射：真机角度 -> 仿真目标控制位置
                        normalized = (q_gripper_real - GRIPPER_REAL_CLOSED_RAD) / (
                                    GRIPPER_REAL_OPEN_RAD - GRIPPER_REAL_CLOSED_RAD)
                        target_ctrl_value = GRIPPER_SIM_CLOSED_METER + normalized * (
                                    GRIPPER_SIM_OPEN_METER - GRIPPER_SIM_CLOSED_METER)

                        # 确保在XML控制范围内
                        target_ctrl_value = max(target_ctrl_value, 0.001)
                        target_ctrl_value = min(target_ctrl_value, 0.05)

                        # 计算真机夹爪速度
                        gripper_real_velocity = (q_gripper_real - last_gripper_real_pos) / period
                        last_gripper_real_pos = q_gripper_real

                        # 【关键】平滑过渡到目标值
                        # 方法1：线性插值平滑
                        diff = target_ctrl_value - current_ctrl_value

                        # 限制最大位移
                        if abs(diff) > max_displacement_per_frame:
                            if diff > 0:
                                current_ctrl_value += max_displacement_per_frame
                            else:
                                current_ctrl_value -= max_displacement_per_frame
                        else:
                            # 使用平滑因子
                            current_ctrl_value = current_ctrl_value * (
                                        1 - smooth_factor) + target_ctrl_value * smooth_factor

                        # 确保最终值在控制范围内
                        current_ctrl_value = max(current_ctrl_value, 0.001)
                        current_ctrl_value = min(current_ctrl_value, 0.05)

                        # 设置控制信号
                        data.ctrl[gripper_act_id] = current_ctrl_value

                        # 显示状态
                        if frame % 100 == 0:  # 每2秒显示一次
                            gripper_percent = normalized * 100
                            velocity_ms = (current_ctrl_value - last_gripper_cmd) / period
                            print(
                                f"[{frame:06d}] 夹爪:{gripper_percent:5.1f}% | 目标:{target_ctrl_value:.4f}m | 实际:{current_ctrl_value:.4f}m | 速度:{velocity_ms:.2f}m/s")
                            last_gripper_cmd = current_ctrl_value

                if has_feedback:
                    # 应用机械臂位置
                    mapper.apply(data, q_real_6d)

                    # 推进仿真
                    mujoco.mj_step(model, data)

                    # 更新视图
                    viewer.sync()

                frame += 1
                time.sleep(max(0, period - (time.perf_counter() - t0)))

    except KeyboardInterrupt:
        _running = False
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        _running = False
    finally:
        print("\n\n[退出流程] 正在关闭系统...")
        try:
            arm.stop_control_loop()
            arm.disable()
            arm.disconnect()
        except Exception:
            pass
        print("[退出流程] 安全退出。")


if __name__ == "__main__":
    main()