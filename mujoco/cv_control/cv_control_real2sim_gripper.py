import os
import sys
import logging
import time
import math
import importlib.util
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
import numpy as np

logging.getLogger().setLevel(logging.ERROR)


# ==========================================
# --- 1. 原生频率控制器 ---
# ==========================================
class RateLimiter:
    """原生的循环频率控制器，取代外部依赖，防止 CPU 空转"""

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
# --- 2. 动态路径解析与参数配置 ---
# ==========================================
CURRENT_FILE = Path(__file__).resolve()
PROJECT_ROOT_DIR = CURRENT_FILE.parents[2]

CONTROL_LIB_DIR = PROJECT_ROOT_DIR / "reBotArm_control_py"
DEFAULT_GRIPPER_CFG = PROJECT_ROOT_DIR / "config" / "gripper.yaml"
DEFAULT_ARM_CFG = None

XML_PATH = PROJECT_ROOT_DIR / "mujoco" / "xml" / "rebot_gripper" / "sim_reBot_grasp.xml"

if not (CONTROL_LIB_DIR / "actuator" / "arm.py").exists():
    print(f"\n❌ 致命错误: 找不到控制库 {CONTROL_LIB_DIR / 'actuator' / 'arm.py'}")
    sys.exit()

# ---------------- 控制参数 ----------------
# A. 视觉距离 (像素)
VISUAL_DIST_CLOSED = 30.0
VISUAL_DIST_OPEN = 180.0

# B. 真机控制范围 (Rad)
GRIPPER_REAL_CLOSED_RAD = 0.45
GRIPPER_REAL_OPEN_RAD = -5.8

# C. 仿真控制范围 (Meter)
GRIPPER_SIM_CLOSED_METER = 0.001
GRIPPER_SIM_OPEN_METER = 0.05

# D. 动力学与平滑系数
GRIPPER_KP = 4.0
GRIPPER_KD = 0.5
GRIPPER_TAU_FF = 0.0
SMOOTHING_FACTOR = 0.15


# ==========================================
# --- 3. 硬件加载函数 ---
# ==========================================
def _load_robot_arm_class():
    arm_py = CONTROL_LIB_DIR / "actuator" / "arm.py"
    spec = importlib.util.spec_from_file_location("_rebotarm_actuator_arm", arm_py)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.RobotArm


def _load_gripper_cfg_func():
    gripper_py = CONTROL_LIB_DIR / "actuator" / "gripper.py"
    spec = importlib.util.spec_from_file_location("_rebotarm_actuator_gripper", gripper_py)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.load_cfg


# ==========================================
# --- 4. 主程序入口 ---
# ==========================================
if __name__ == "__main__":
    print("\n" + "=" * 50)
    print("  视觉手势 -> 真机控制 -> MuJoCo 孪生")
    print("=" * 50)

    # --- 阶段 A：初始化 MuJoCo 仿真 ---
    print("\n[1/4] 正在加载 MuJoCo 模型...")
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)
    model.opt.timestep = 0.005
    gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")

    if gripper_act_id < 0:
        print("❌ 未找到仿真夹爪执行器 'gripper'")
        sys.exit()

    # --- 阶段 B：初始化真机硬件 ---
    print("\n[2/4] 正在加载并连接真机硬件...")
    try:
        RobotArm = _load_robot_arm_class()
        load_gripper_cfg = _load_gripper_cfg_func()

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
        print("✅ 夹爪电机已激活，切入 MIT 控制模式！")

    except Exception as e:
        print(f"❌ 硬件初始化失败: {e}")
        sys.exit()

    # --- 阶段 C：启动视窗与视觉模块 ---
    print("\n[3/4] 正在启动 MuJoCo 视窗...")
    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)

    print("\n[4/4] 正在启动 OpenCV 与 MediaPipe...")
    import HandTrackingModule as htm

    detector = htm.handDetector(detectionCon=0.7, trackCon=0.7)

    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    # 读取起始位置作为平滑基准
    st = gripper_motor_obj.get_state()
    current_smoothed_rad = st.pos if st is not None else GRIPPER_REAL_OPEN_RAD

    print("\n🚀 系统完全就绪！请在镜头前展示手势。按 'q' 键退出。")

    pTime = 0
    _running = True
    rate = RateLimiter(frequency=30.0)

    try:
        # --- 阶段 D：核心闭环主循环 ---
        while _running and viewer.is_running():
            success, img = cap.read()
            if not success:
                print("⚠️ 无法读取摄像头画面")
                break

            # ------------------------------------------------
            # 1. CV 视觉检测 -> 目标控制量计算
            # ------------------------------------------------
            img = detector.findHands(img, draw=True)
            lmList = detector.findPosition(img, draw=False)

            if len(lmList) != 0:
                x1, y1 = lmList[4][1], lmList[4][2]
                x2, y2 = lmList[8][1], lmList[8][2]
                xc, yc = (x1 + x2) // 2, (y1 + y2) // 2

                # 计算手势距离
                length = math.hypot(x2 - x1, y2 - y1)

                # UI 绘制
                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.circle(img, (x1, y1), 10, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 10, (255, 0, 255), cv2.FILLED)
                cv2.putText(img, f'Dist: {int(length)}', (xc + 20, yc), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)

                # 插值映射并进行 EMA 平滑
                target_rad = np.interp(
                    length,
                    [VISUAL_DIST_CLOSED, VISUAL_DIST_OPEN],
                    [GRIPPER_REAL_CLOSED_RAD, GRIPPER_REAL_OPEN_RAD]
                )
                current_smoothed_rad = (SMOOTHING_FACTOR * target_rad) + ((1 - SMOOTHING_FACTOR) * current_smoothed_rad)

            # ------------------------------------------------
            # 2. 下发指令到真机
            # ------------------------------------------------
            try:
                gripper_motor_obj.send_mit(
                    float(current_smoothed_rad), 0.0, float(GRIPPER_KP), float(GRIPPER_KD), float(GRIPPER_TAU_FF)
                )
            except Exception:
                pass

            # ------------------------------------------------
            # 3. 读取真机反馈 -> 同步孪生至仿真
            # ------------------------------------------------
            q_gripper_real = current_smoothed_rad
            st = gripper_motor_obj.get_state()
            if st is not None:
                q_gripper_real = st.pos

            # 计算归一化开合度，并映射至仿真模型的控制距离(Meter)
            normalized = (q_gripper_real - GRIPPER_REAL_CLOSED_RAD) / (GRIPPER_REAL_OPEN_RAD - GRIPPER_REAL_CLOSED_RAD)
            target_sim_val = GRIPPER_SIM_CLOSED_METER + normalized * (GRIPPER_SIM_OPEN_METER - GRIPPER_SIM_CLOSED_METER)
            target_sim_val = max(GRIPPER_SIM_CLOSED_METER, min(GRIPPER_SIM_OPEN_METER, target_sim_val))

            # 物理引擎步进与渲染 (变量名已修正)
            data.ctrl[gripper_act_id] = target_sim_val
            mujoco.mj_step(model, data)
            viewer.sync()

            # ------------------------------------------------
            # 4. 画面刷新与运行频率控制
            # ------------------------------------------------
            cTime = time.time()
            fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
            pTime = cTime

            cv2.putText(img, f'FPS: {int(fps)}', (10, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.putText(img, f'Real Rad: {q_gripper_real:.2f}', (10, 80), cv2.FONT_HERSHEY_PLAIN, 2, (0, 165, 255), 2)
            cv2.putText(img, f'Sim Pos: {target_sim_val:.4f}m', (10, 120), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

            cv2.imshow("CV -> Real -> Sim", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                _running = False

            rate.sleep()

    except KeyboardInterrupt:
        print("\n收到退出信号...")
    except Exception as e:
        print(f"\n❌ 运行异常: {e}")
    finally:
        print("\n[退出流程] 正在切断电机动力并安全关闭系统...")
        try:
            # 安全释放：令真机电机进入零力拖拽软化状态
            gripper_motor_obj.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)
            arm.disable()
            arm.disconnect()
            print("✅ 硬件连接已断开")
        except:
            pass

        # 释放视觉与仿真资源
        if 'cap' in locals() and cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if 'viewer' in locals() and viewer.is_running():
            viewer.close()
        print("👋 拜拜！")