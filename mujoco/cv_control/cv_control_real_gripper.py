import os
import cv2
import numpy as np
import time
import math
import sys
import importlib.util
from pathlib import Path

# ==========================================
# --- 1. 配置参数与动态路径解析 (优雅版) ---
# ==========================================
# 获取当前脚本的绝对路径
# .../reBotArm_develop_hjx/mujoco/cv_control/cv_control_real_gripper.py
CURRENT_FILE = Path(__file__).resolve()

# 动态向上跳 3 级，精准定位到项目根目录: reBotArm_develop_hjx
# parents[0]=cv_control, parents[1]=mujoco, parents[2]=reBotArm_develop_hjx
PROJECT_ROOT_DIR = CURRENT_FILE.parents[2]

# 基于项目根目录，推导控制库和配置文件的位置
CONTROL_LIB_DIR = PROJECT_ROOT_DIR / "reBotArm_control_py"
DEFAULT_GRIPPER_CFG = PROJECT_ROOT_DIR / "config" / "gripper.yaml"
DEFAULT_ARM_CFG = None

# 启动前自检，确保动态推导的路径正确无误
if not (CONTROL_LIB_DIR / "actuator" / "arm.py").exists():
    print(f"\n❌ 致命错误: 动态路径解析失败，找不到 {CONTROL_LIB_DIR / 'actuator' / 'arm.py'}")
    sys.exit()

# ==========================================
# --- 2. 映射与硬件控制参数 ---
# ==========================================
# A. 视觉手势距离标定 (像素)
VISUAL_DIST_CLOSED = 30.0  # 手指闭合时的像素距离
VISUAL_DIST_OPEN = 180.0  # 手指完全张开时的像素距离

# B. 真机夹爪控制范围 (弧度 Rad)
GRIPPER_REAL_CLOSED_RAD = 0.45  # 真机闭合  0.0
GRIPPER_REAL_OPEN_RAD = -5.8  # 真机张开

# C. MIT 模式位置控制参数 (PD 控制)
# 注意：Kp 和 Kd 决定了夹爪跟随手势的“硬度”和“力量”。
GRIPPER_KP = 4.0
GRIPPER_KD = 0.5
GRIPPER_TAU_FF = 0.0

# D. 动作平滑系数 (0.0 到 1.0)
SMOOTHING_FACTOR = 0.15

# ==========================================
# --- 3. 硬件环境加载函数 ---
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
    print("  视觉手势 -> 真机夹爪控制启动")
    print("=" * 50)

    # --- 阶段 A：初始化硬件 ---
    print("\n[1/3] 正在加载并连接真机硬件...")
    try:
        RobotArm = _load_robot_arm_class()
        load_gripper_cfg = _load_gripper_cfg_func()

        if not DEFAULT_GRIPPER_CFG.exists():
            print(f"⚠️ 警告: 找不到配置文件 {DEFAULT_GRIPPER_CFG}")

        g_cfg = load_gripper_cfg(str(DEFAULT_GRIPPER_CFG))["gripper"]
        arm = RobotArm(cfg_path=str(DEFAULT_ARM_CFG) if DEFAULT_ARM_CFG else None)

        arm.connect()
        arm.enable()
        time.sleep(0.2)

        # 挂载达妙(Damiao)电机并切入 MIT 模式
        shared_damiao_controller = arm._ctrl_map["damiao"]
        g_mot = shared_damiao_controller.add_damiao_motor(g_cfg.motor_id, g_cfg.feedback_id, g_cfg.model)
        arm._motor_map[g_cfg.name] = g_mot
        gripper_motor_obj = g_mot

        from motorbridge import Mode

        # 激活 MIT 模式
        g_mot.ensure_mode(Mode.MIT, 1000)
        shared_damiao_controller.enable_all()
        time.sleep(0.2)
        print("✅ 夹爪电机已激活，切入 MIT 控制模式！")

    except Exception as e:
        print(f"❌ 硬件初始化失败: {e}")
        sys.exit()

    # --- 阶段 B：初始化视觉 ---
    print("\n[2/3] 正在启动 OpenCV 与 MediaPipe...")

    try:
        import HandTrackingModule as htm
    except ImportError:
        print("❌ 找不到 HandTrackingModule.py，请确保它在此脚本的同一目录下或在环境变量中。")
        sys.exit()

    detector = htm.handDetector(detectionCon=0.8, trackCon=0.8)

    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    # 获取电机当前真实位置，作为平滑控制的起点
    st = gripper_motor_obj.get_state()
    current_smoothed_rad = st.pos if st is not None else GRIPPER_REAL_OPEN_RAD

    print("\n[3/3] 系统就绪！请在镜头前展示手势。按 'q' 键安全退出。")

    pTime = 0
    _running = True

    try:
        # --- 阶段 C：主控制循环 ---
        while _running:
            success, img = cap.read()
            if not success:
                print("⚠️ 无法获取摄像头画面")
                break

            img = detector.findHands(img, draw=True)
            lmList = detector.findPosition(img, draw=False)

            if len(lmList) != 0:
                # 1. 提取大拇指(4)和食指(8)的坐标
                x1, y1 = lmList[4][1], lmList[4][2]
                x2, y2 = lmList[8][1], lmList[8][2]
                xc, yc = (x1 + x2) // 2, (y1 + y2) // 2

                # 2. 计算像素距离
                length = math.hypot(x2 - x1, y2 - y1)

                # 3. 视觉绘制
                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.circle(img, (x1, y1), 10, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 10, (255, 0, 255), cv2.FILLED)
                cv2.putText(img, f'Dist: {int(length)}', (xc + 20, yc), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)

                # 4. 核心映射：像素距离 -> 真机弧度
                target_rad = np.interp(
                    length,
                    [VISUAL_DIST_CLOSED, VISUAL_DIST_OPEN],
                    [GRIPPER_REAL_CLOSED_RAD, GRIPPER_REAL_OPEN_RAD]
                )

                # 5. EMA 低通滤波 (防抖)
                current_smoothed_rad = (SMOOTHING_FACTOR * target_rad) + ((1 - SMOOTHING_FACTOR) * current_smoothed_rad)

            # 6. 下发指令到真机 (MIT 模式位置跟踪)
            try:
                gripper_motor_obj.send_mit(
                    float(current_smoothed_rad),
                    0.0,
                    float(GRIPPER_KP),
                    float(GRIPPER_KD),
                    float(GRIPPER_TAU_FF)
                )
            except Exception:
                pass

            # 7. UI 更新
            cTime = time.time()
            fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
            pTime = cTime

            cv2.putText(img, f'FPS: {int(fps)}', (10, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.putText(img, f'Cmd: {current_smoothed_rad:.2f} rad', (10, 80), cv2.FONT_HERSHEY_PLAIN, 2, (0, 165, 255), 2)

            cv2.imshow("Real Hardware Control", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                _running = False

    except KeyboardInterrupt:
        print("\n收到退出信号...")
    except Exception as e:
        print(f"\n❌ 运行异常: {e}")
    finally:
        # --- 阶段 D：安全退出 ---
        print("\n[退出流程] 正在切断电机动力并安全关闭系统...")
        try:
            # 零力软化电机
            gripper_motor_obj.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)
            arm.disable()
            arm.disconnect()
            print("✅ 硬件连接已断开")
        except:
            pass

        cap.release()
        cv2.destroyAllWindows()
        print("👋 拜拜！")

