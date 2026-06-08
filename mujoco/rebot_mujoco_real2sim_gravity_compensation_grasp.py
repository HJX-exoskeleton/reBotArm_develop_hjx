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


def main() -> None:
    global _running

    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML)
    parser.add_argument("--cfg", type=Path, default=None)
    parser.add_argument("--gripper-cfg", type=Path, default=DEFAULT_GRIPPER_CFG)
    parser.add_argument("--rate", type=float, default=200.0, help="控制频率(Hz)")
    args = parser.parse_args()

    # 加载模型
    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)

    # 查找所有执行器
    print("\n🔍 查找所有执行器:")
    actuator_dict = {}  # 执行器名称 -> ID
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if act_name:
            actuator_dict[act_name] = i
            print(f"  [{i}] {act_name}")

    # 查找机械臂执行器 - 简化查找逻辑
    arm_actuator_ids = []
    for joint_name in DEFAULT_JOINT_NAMES:
        # 直接查找与关节同名的执行器
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
        if act_id >= 0:
            arm_actuator_ids.append(act_id)
            print(f"✅ 为关节 {joint_name} 找到执行器: {joint_name} (ID: {act_id})")
        else:
            # 如果没有同名的执行器，尝试通过关节ID查找
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            found = False
            if joint_id >= 0:
                for i in range(model.nu):
                    trn_type = model.actuator_trntype[i]
                    if trn_type == mujoco.mjtTrn.mjTRN_JOINT:
                        trn_id = model.actuator_trnid[i, 0]
                        if trn_id == joint_id:
                            act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                            arm_actuator_ids.append(i)
                            print(f"✅ 为关节 {joint_name} 找到执行器: {act_name} (ID: {i})")
                            found = True
                            break
            if not found:
                print(f"⚠️  未找到关节 {joint_name} 的执行器")
                arm_actuator_ids.append(-1)

    # 查找夹爪执行器
    gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")
    if gripper_act_id < 0:
        print("❌ 未找到夹爪执行器 'gripper'")
        return

    print(f"\n✅ 找到夹爪执行器: 'gripper' (ID: {gripper_act_id})")

    # 获取执行器控制范围
    ctrlrange = model.actuator_ctrlrange[gripper_act_id]
    print(f"✅ 夹爪控制范围: [{ctrlrange[0]:.4f}, {ctrlrange[1]:.4f}] m")

    # 加载机器人控制类
    RobotArm = _load_robot_arm_class()
    load_gripper_cfg = _load_gripper_cfg_func()
    load_dynamics_model, compute_generalized_gravity = _load_gravity_functions()
    load_dynamics_model()

    g_cfg = load_gripper_cfg(str(args.gripper_cfg))["gripper"]
    arm = RobotArm(cfg_path=str(args.cfg) if args.cfg is not None else None)

    print("\n" + "=" * 60)
    print("  reBotArm Real2Sim: 7轴全物理拖拽示教")
    print("=" * 60)
    print(f"[夹爪标定] 真机: [{GRIPPER_REAL_CLOSED_RAD:.1f}, {GRIPPER_REAL_OPEN_RAD:.1f}] rad")
    print(f"[夹爪标定] 仿真: [{GRIPPER_SIM_CLOSED_METER:.3f}, {GRIPPER_SIM_OPEN_METER:.3f}] m")

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

                # 读取初始位置
                st = g_mot.get_state()
                if st is not None:
                    print(f"✅ 夹爪初始位置: {st.pos:.3f} rad")
            except Exception as e:
                print(f"❌ 夹爪配置失败: {e}")

        # 启动控制线程
        from threading import Lock
        state_lock = Lock()
        q_real_6d = np.zeros(6)
        has_feedback = False

        def gravity_compensation_worker():
            nonlocal q_real_6d, has_feedback
            while _running:
                try:
                    q, _, _ = arm.get_state()
                    q_arm_6d = q[:6]

                    # 计算重力补偿力矩
                    tau_g = compute_generalized_gravity(q=q_arm_6d) * GRAVITY_SCALES[:6]
                    tau_g_safe = np.clip(tau_g, -TORQUE_LIMITS[:6], TORQUE_LIMITS[:6])

                    # 发送MIT控制命令
                    for i, jname in enumerate(DEFAULT_JOINT_NAMES):
                        try:
                            mot = arm._motor_map.get(jname)
                            if mot:
                                mot.send_mit(float(q_arm_6d[i]), 0.0, 0.0, float(KD_CONFIG[i]), float(tau_g_safe[i]))
                        except Exception:
                            pass

                    # 夹爪零力控制
                    try:
                        mot_g = arm._motor_map.get(GRIPPER_MOTOR_NAME)
                        if mot_g:
                            mot_g.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
                    except Exception:
                        pass

                    # 轮询反馈
                    try:
                        for mot in arm._motor_map.values():
                            mot.request_feedback()
                        for ctrl in arm._ctrl_map.values():
                            ctrl.poll_feedback_once()
                    except Exception:
                        pass

                    with state_lock:
                        q_real_6d = q_arm_6d.copy()
                        has_feedback = True

                except Exception as e:
                    print(f"控制线程异常: {e}")
                time.sleep(0.02)  # 50Hz

        # 启动控制线程
        control_thread = threading.Thread(target=gravity_compensation_worker, daemon=True)
        control_thread.start()
        time.sleep(0.5)  # 等待线程启动

        frame = 0
        period = 1.0 / args.rate
        last_gripper_real_pos = 0.0
        gripper_cmd_history = []

        with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
            print("\n👀 视窗已启动！")
            print("💡 可以随意拖拽机械臂和夹爪，仿真会实时同步")

            while _running and viewer.is_running():
                t0 = time.perf_counter()

                with state_lock:
                    current_q_real_6d = q_real_6d.copy()
                    current_has_feedback = has_feedback

                # 读取夹爪真实角度
                q_gripper_real = 0.0
                target_sim_gripper_cmd = GRIPPER_SIM_OPEN_METER
                normalized = 0.0

                if gripper_motor_obj is not None:
                    st = gripper_motor_obj.get_state()
                    if st is not None:
                        q_gripper_real = st.pos

                        # 线性映射：真机角度 -> 仿真控制位置
                        normalized = (q_gripper_real - GRIPPER_REAL_CLOSED_RAD) / (
                                    GRIPPER_REAL_OPEN_RAD - GRIPPER_REAL_CLOSED_RAD)
                        target_sim_gripper_cmd = GRIPPER_SIM_CLOSED_METER + normalized * (
                                    GRIPPER_SIM_OPEN_METER - GRIPPER_SIM_CLOSED_METER)

                        # 确保在XML控制范围内
                        target_sim_gripper_cmd = max(target_sim_gripper_cmd, 0.001)
                        target_sim_gripper_cmd = min(target_sim_gripper_cmd, 0.05)

                        # 【关键】直接设置控制信号，不添加额外的平滑
                        # 参考代码中就是这样直接设置的
                        data.ctrl[gripper_act_id] = target_sim_gripper_cmd

                        # 记录历史用于调试
                        gripper_cmd_history.append((q_gripper_real, target_sim_gripper_cmd))
                        if len(gripper_cmd_history) > 100:
                            gripper_cmd_history.pop(0)

                if current_has_feedback:
                    # 设置机械臂控制信号
                    for i, act_id in enumerate(arm_actuator_ids):
                        if act_id >= 0 and i < len(current_q_real_6d):
                            data.ctrl[act_id] = float(current_q_real_6d[i])

                    # 推进仿真
                    mujoco.mj_step(model, data)

                    # 更新视图
                    viewer.sync()

                    # 显示状态
                    if frame % 200 == 0:  # 每1秒显示一次
                        gripper_percent = normalized * 100
                        print(f"[{frame:06d}] 夹爪:{gripper_percent:5.1f}% | 控制:{target_sim_gripper_cmd:.4f}m")

                frame += 1

                # 精确控制循环频率
                elapsed = time.perf_counter() - t0
                if elapsed < period:
                    time.sleep(period - elapsed)
                else:
                    print(f"⚠️  控制循环超时: {elapsed * 1000:.1f}ms > {period * 1000:.1f}ms")

    except KeyboardInterrupt:
        _running = False
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        import traceback
        traceback.print_exc()
        _running = False
    finally:
        print("\n\n[退出流程] 正在关闭系统...")
        _running = False
        time.sleep(0.5)  # 等待控制线程结束
        try:
            arm.disable()
            arm.disconnect()
        except Exception:
            pass
        print("[退出流程] 安全退出。")


if __name__ == "__main__":
    main()