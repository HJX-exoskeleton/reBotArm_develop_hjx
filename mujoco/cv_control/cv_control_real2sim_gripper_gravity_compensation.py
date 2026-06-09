import os
import sys
import logging
import time
import math
import threading
from threading import Lock
import numpy as np
from pathlib import Path

# ==========================================
# --- 0. 图形上下文隔离与环境配置 ---
# ==========================================
os.environ["WAYLAND_DISPLAY"] = ""
os.environ["MUJOCO_GL"] = "glfw"
os.environ["GLOG_minloglevel"] = "2"
os.environ["OPENCV_LOG_LEVEL"] = "FATAL"

import mujoco
import mujoco.viewer
import cv2

logging.getLogger().setLevel(logging.ERROR)


# ==========================================
# --- 1. 原生频率控制器 ---
# ==========================================
class RateLimiter:
    def __init__(self, frequency):
        self.period = 1.0 / frequency
        self.last_time = time.perf_counter()

    def sleep(self):
        current_time = time.perf_counter()
        elapsed = current_time - self.last_time
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        self.last_time = time.perf_counter()


# ==========================================
# --- 2. 动态路径解析与标准导包 (修复核心) ---
# ==========================================
CURRENT_FILE = Path(__file__).resolve()
PROJECT_ROOT_DIR = CURRENT_FILE.parents[2]

# 【关键修复】将项目根目录加入 sys.path，抛弃 importlib 动态加载
if str(PROJECT_ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT_DIR))

# 现在可以优雅且安全地使用标准包导入，完美解决相对导入报错！
try:
    from reBotArm_control_py.actuator.arm import RobotArm
    from reBotArm_control_py.actuator.gripper import load_cfg as load_gripper_cfg
    from reBotArm_control_py.dynamics import load_dynamics_model, compute_generalized_gravity
except ImportError as e:
    print(f"\n❌ 致命错误: 找不到依赖包，请检查路径。详细报错: {e}")
    sys.exit()

DEFAULT_GRIPPER_CFG = PROJECT_ROOT_DIR / "config" / "gripper.yaml"
DEFAULT_ARM_CFG = None
XML_PATH = PROJECT_ROOT_DIR / "mujoco" / "xml" / "rebot_gripper" / "sim_reBot_grasp.xml"

# ---------------- A. 机械臂重力补偿参数 ----------------
DEFAULT_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
TORQUE_LIMITS = np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
KD_CONFIG = np.array([1.0, 2.0, 1.5, 1.0, 0.8, 0.6])
GRAVITY_SCALES = np.array([1.50, 0.75, 0.75, 0.75, 1.0, 1.0])

# ---------------- B. 夹爪控制与视觉参数 ----------------
VISUAL_DIST_CLOSED = 30.0
VISUAL_DIST_OPEN = 180.0
GRIPPER_REAL_CLOSED_RAD = 0.45
GRIPPER_REAL_OPEN_RAD = -5.8
GRIPPER_SIM_CLOSED_METER = 0.001
GRIPPER_SIM_OPEN_METER = 0.05
GRIPPER_KP = 4.0
GRIPPER_KD = 0.5
GRIPPER_TAU_FF = 0.0
SMOOTHING_FACTOR = 0.15

_running = True

# ==========================================
# --- 3. 主程序入口 ---
# ==========================================
if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("  7轴全物理: 手势控夹爪 + 拖拽重力补偿 -> MuJoCo 孪生")
    print("=" * 60)

    # --- 阶段 A：初始化 MuJoCo 仿真并查找全轴执行器 ---
    print("\n[1/4] 正在加载 MuJoCo 模型并映射控制轴...")
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)
    model.opt.timestep = 0.005  # mujoco 200 Hz (内部计算)

    # 1. 映射机械臂 6 轴
    arm_actuator_ids = []
    for joint_name in DEFAULT_JOINT_NAMES:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
        if act_id >= 0:
            arm_actuator_ids.append(act_id)
        else:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            found = False
            if joint_id >= 0:
                for i in range(model.nu):
                    if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[i, 0] == joint_id:
                        arm_actuator_ids.append(i)
                        found = True
                        break
            if not found:
                arm_actuator_ids.append(-1)
                print(f"⚠️  警告: 仿真模型中未找到关节 {joint_name} 的执行器")

    # 2. 映射夹爪
    gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")
    if gripper_act_id < 0:
        print("❌ 致命错误: 未找到仿真夹爪执行器 'gripper'")
        sys.exit()

    # --- 阶段 B：初始化真机硬件与动力学模型 ---
    print("\n[2/4] 正在连接真机硬件与加载动力学...")
    try:
        load_dynamics_model()  # 加载重力补偿模型

        if not DEFAULT_GRIPPER_CFG.exists():
            print(f"⚠️ 警告: 找不到夹爪配置文件 {DEFAULT_GRIPPER_CFG}")

        g_cfg = load_gripper_cfg(str(DEFAULT_GRIPPER_CFG))["gripper"]
        arm = RobotArm(cfg_path=str(DEFAULT_ARM_CFG) if DEFAULT_ARM_CFG else None)

        arm.connect()
        arm.enable()
        time.sleep(0.2)

        shared_damiao_controller = arm._ctrl_map["damiao"]
        g_mot = shared_damiao_controller.add_damiao_motor(g_cfg.motor_id, g_cfg.feedback_id, g_cfg.model)
        arm._motor_map[g_cfg.name] = g_mot
        gripper_motor_obj = g_mot

        from motorbridge import Mode

        g_mot.ensure_mode(Mode.MIT, 1000)
        shared_damiao_controller.enable_all()
        time.sleep(0.2)
        print("✅ 全局硬件已激活，电机切入 MIT 模式！")

    except Exception as e:
        print(f"❌ 硬件初始化失败: {e}")
        sys.exit()

    # --- 阶段 C：启动重力补偿后台线程 ---
    print("\n[3/4] 正在启动 6 轴重力补偿后台守护线程...")
    state_lock = Lock()
    q_real_6d = np.zeros(6)
    has_feedback = False


    def gravity_compensation_worker():
        global q_real_6d, has_feedback, _running
        while _running:
            try:
                # 1. 读取真机 6 轴位置
                q, _, _ = arm.get_state()
                q_arm_6d = q[:6]

                # 2. 计算并下发前馈重力补偿 (跳过夹爪，夹爪由CV主线程控制)
                tau_g = compute_generalized_gravity(q=q_arm_6d) * GRAVITY_SCALES[:6]
                tau_g_safe = np.clip(tau_g, -TORQUE_LIMITS[:6], TORQUE_LIMITS[:6])

                for i, jname in enumerate(DEFAULT_JOINT_NAMES):
                    try:
                        mot = arm._motor_map.get(jname)
                        if mot:
                            mot.send_mit(float(q_arm_6d[i]), 0.0, 0.0, float(KD_CONFIG[i]), float(tau_g_safe[i]))
                    except Exception:
                        pass

                # 3. 统一执行硬件轮询，刷新所有电机最新状态
                try:
                    for mot in arm._motor_map.values():
                        mot.request_feedback()
                    for ctrl in arm._ctrl_map.values():
                        ctrl.poll_feedback_once()
                except Exception:
                    pass

                # 4. 更新共享状态供主线程孪生映射
                with state_lock:
                    q_real_6d = q_arm_6d.copy()
                    has_feedback = True

            except Exception as e:
                pass
            time.sleep(0.02)  # 50Hz (后台动力学刷新率)  真机动力学与重力补偿


    control_thread = threading.Thread(target=gravity_compensation_worker, daemon=True)
    control_thread.start()
    time.sleep(0.5)

    # --- 阶段 D：启动视觉模块与孪生主循环 ---
    print("\n[4/4] 正在启动视觉引擎与 MuJoCo 视窗...")
    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)

    import HandTrackingModule as htm

    detector = htm.handDetector(detectionCon=0.7, trackCon=0.7)
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    st = gripper_motor_obj.get_state()
    current_smoothed_rad = st.pos if st is not None else GRIPPER_REAL_OPEN_RAD

    print("\n🚀 7 轴数字孪生系统就绪！")
    print("   👉 您可以：1. 左手拖拽机械臂 2. 右手控制夹爪")
    print("   👉 仿真视窗将实时 1:1 还原所有物理动作！(按 'q' 键退出)")

    pTime = 0
    rate = RateLimiter(frequency=30.0)  # 前台视觉限制为 30Hz  视觉感知与真机夹爪控制

    try:
        while _running and viewer.is_running():
            success, img = cap.read()
            if not success:
                break

            # ------------------------------------------------
            # 1. 视觉检测 -> 夹爪目标指令计算
            # ------------------------------------------------
            img = detector.findHands(img, draw=True)
            lmList = detector.findPosition(img, draw=False)

            if len(lmList) != 0:
                x1, y1 = lmList[4][1], lmList[4][2]
                x2, y2 = lmList[8][1], lmList[8][2]
                xc, yc = (x1 + x2) // 2, (y1 + y2) // 2
                length = math.hypot(x2 - x1, y2 - y1)

                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.circle(img, (x1, y1), 10, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 10, (255, 0, 255), cv2.FILLED)
                cv2.putText(img, f'Dist: {int(length)}', (xc + 20, yc), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)

                target_rad = np.interp(
                    length,
                    [VISUAL_DIST_CLOSED, VISUAL_DIST_OPEN],
                    [GRIPPER_REAL_CLOSED_RAD, GRIPPER_REAL_OPEN_RAD]
                )
                current_smoothed_rad = (SMOOTHING_FACTOR * target_rad) + ((1 - SMOOTHING_FACTOR) * current_smoothed_rad)

            # ------------------------------------------------
            # 2. 下发视觉指令到真机夹爪 (机械臂交由子线程处理)
            # ------------------------------------------------
            try:
                gripper_motor_obj.send_mit(
                    float(current_smoothed_rad), 0.0, float(GRIPPER_KP), float(GRIPPER_KD), float(GRIPPER_TAU_FF)
                )
            except Exception:
                pass

            # ------------------------------------------------
            # 3. 读取全系统反馈 -> 同步孪生至 MuJoCo
            # ------------------------------------------------
            # 3.1 获取夹爪物理状态
            q_gripper_real = current_smoothed_rad
            st = gripper_motor_obj.get_state()
            if st is not None:
                q_gripper_real = st.pos

            normalized = (q_gripper_real - GRIPPER_REAL_CLOSED_RAD) / (GRIPPER_REAL_OPEN_RAD - GRIPPER_REAL_CLOSED_RAD)
            target_sim_val = GRIPPER_SIM_CLOSED_METER + normalized * (GRIPPER_SIM_OPEN_METER - GRIPPER_SIM_CLOSED_METER)
            target_sim_val = max(GRIPPER_SIM_CLOSED_METER, min(GRIPPER_SIM_OPEN_METER, target_sim_val))

            # 3.2 注入全轴仿真控制量
            data.ctrl[gripper_act_id] = target_sim_val

            with state_lock:
                current_q_real_6d = q_real_6d.copy()
                current_has_feedback = has_feedback

            if current_has_feedback:
                for i, act_id in enumerate(arm_actuator_ids):
                    if act_id >= 0 and i < len(current_q_real_6d):
                        data.ctrl[act_id] = float(current_q_real_6d[i])

            # 物理引擎步进与渲染
            mujoco.mj_step(model, data)
            viewer.sync()

            # ------------------------------------------------
            # 4. 画面刷新与运行频率控制
            # ------------------------------------------------
            cTime = time.time()
            fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
            pTime = cTime

            cv2.putText(img, f'FPS: {int(fps)}', (10, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.putText(img, f'Grip Rad: {q_gripper_real:.2f}', (10, 80), cv2.FONT_HERSHEY_PLAIN, 2, (0, 165, 255), 2)
            cv2.putText(img, f'Arm J1: {current_q_real_6d[0]:.2f}', (10, 120), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0),2)

            cv2.imshow("7-Axis CV -> Real -> Sim", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                _running = False

            rate.sleep()

    except KeyboardInterrupt:
        print("\n收到退出信号...")
    except Exception as e:
        print(f"\n❌ 运行异常: {e}")
    finally:
        print("\n[退出流程] 正在切断电机动力并安全关闭系统...")
        _running = False
        time.sleep(0.1)  # 等待后台线程自然结束
        try:
            for jname in DEFAULT_JOINT_NAMES:
                mot = arm._motor_map.get(jname)
                if mot: mot.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            gripper_motor_obj.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)

            time.sleep(0.1)
            arm.disable()
            arm.disconnect()
            print("✅ 硬件连接已断开")
        except:
            pass

        if 'cap' in locals() and cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if 'viewer' in locals() and viewer.is_running():
            viewer.close()
        print("👋 拜拜！")
