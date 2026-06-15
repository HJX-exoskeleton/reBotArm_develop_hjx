import time
import sys
import os
import cv2
import math
import mediapipe as mp

# ================= 动态环境变量注入 =================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
project_root = os.path.dirname(parent_dir)
sdk_path = os.path.join(project_root, 'STservo_sdk')

if project_root not in sys.path:
    sys.path.append(project_root)

if sdk_path not in sys.path:
    sys.path.append(sdk_path)

from STservo_sdk import *


# ================= 1. 硬件参数配置 =================
# 必须和 ESP32 透传程序 Serial.begin(...) 一致
BAUDRATE = 115200

# Linux
DEVICENAME = '/dev/ttyUSB0'
# Windows 示例
# DEVICENAME = 'COM6'

CAMERA_ID = 0

# ================= 只控制这两个关节 =================
ID_BASE = 1      # 第 1 关节：左右
ID_WRIST = 5     # 第 5 关节：点头

CONTROL_SERVO_IDS = [ID_BASE, ID_WRIST]

# ================= 软件限位 =================
# 来自你的实际标定
ID1_MIN_DEG = 50.0
ID1_MAX_DEG = 300.0

ID5_MIN_DEG = 90.0
ID5_MAX_DEG = 270.0

# 安全余量，防止贴近机械极限
SAFETY_MARGIN_DEG = 3.0

ID1_SAFE_MIN_DEG = ID1_MIN_DEG + SAFETY_MARGIN_DEG
ID1_SAFE_MAX_DEG = ID1_MAX_DEG - SAFETY_MARGIN_DEG

ID5_SAFE_MIN_DEG = ID5_MIN_DEG + SAFETY_MARGIN_DEG
ID5_SAFE_MAX_DEG = ID5_MAX_DEG - SAFETY_MARGIN_DEG

# Home 位
ID1_CENTER_DEG = 180.0
ID5_CENTER_DEG = 180.0

# ================= 控制参数 =================
# ID1 左右追踪参数
KP_ID1 = 0.02
ID1_DIR = -1

# ID5 上下追踪参数
KP_ID5 = 0.018
ID5_DIR = 1

# 视觉死区，单位像素
DEADZONE = 25

# 单帧最大角度变化，防止突跳
MAX_DELTA_ID1_DEG = 2.5
MAX_DELTA_ID5_DEG = 2.5

# 指令下发频率，避免每个摄像头帧都发舵机指令
COMMAND_HZ = 30.0
COMMAND_DT = 1.0 / COMMAND_HZ


# ================= 2. 辅助函数与手势识别 =================
def angle_to_pos(angle):
    return int((angle / 360.0) * 4095.0)


def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))


def limit_delta(delta, max_abs_delta):
    return clamp(delta, -max_abs_delta, max_abs_delta)


def limit_id1_angle(angle):
    return clamp(angle, ID1_SAFE_MIN_DEG, ID1_SAFE_MAX_DEG)


def limit_id5_angle(angle):
    return clamp(angle, ID5_SAFE_MIN_DEG, ID5_SAFE_MAX_DEG)


def detect_gesture(hand_landmarks):
    """
    轻量级手势判断：
    STOP：张开手掌
    GOOD：竖大拇指
    """
    lm = hand_landmarks.landmark

    fingers_up = 0

    for tip in [8, 12, 16, 20]:
        if lm[tip].y < lm[tip - 2].y:
            fingers_up += 1

    thumb_up = (lm[4].y < lm[3].y) and (lm[4].y < lm[5].y)

    if fingers_up >= 4:
        return "STOP"
    elif fingers_up == 0 and thumb_up:
        return "GOOD"

    return "NONE"


# ================= 3. 硬件初始化 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)

if not portHandler.openPort():
    print(f"❌ 舵机串口打开失败: {DEVICENAME}")
    sys.exit()

if not portHandler.setBaudRate(BAUDRATE):
    print(f"❌ 舵机串口波特率设置失败: {BAUDRATE}")
    portHandler.closePort()
    sys.exit()

print(f"✅ 串口已打开: {DEVICENAME}")
print(f"✅ 波特率已设置: {BAUDRATE}")

print("等待 ESP32 进入透传状态...")
time.sleep(2.5)

try:
    portHandler.ser.reset_input_buffer()
    portHandler.ser.reset_output_buffer()
    print("✅ 已清空串口缓冲区")
except Exception as e:
    print(f"⚠️ 清空串口缓冲区失败，可忽略: {e}")


# ================= 4. 舵机控制函数 =================
def enable_control_torque():
    """
    只开启 ID1 和 ID5 的力矩。
    不控制 ID2、ID3、ID4、ID6、ID7。
    """
    print("\n正在开启 ID1 和 ID5 力矩...")

    for servo_id in CONTROL_SERVO_IDS:
        result, error = scs.write1ByteTxRx(
            servo_id,
            STS_TORQUE_ENABLE,
            1
        )

        if result == COMM_SUCCESS:
            print(f"✅ ID={servo_id} 力矩已开启")
        else:
            print(f"⚠️ ID={servo_id} 力矩开启失败: {scs.getTxRxResult(result)}")

        time.sleep(0.03)


def disable_control_torque():
    """
    只释放 ID1 和 ID5 的力矩。
    """
    print("\n正在释放 ID1 和 ID5 力矩...")

    for servo_id in CONTROL_SERVO_IDS:
        result, error = scs.write1ByteTxRx(
            servo_id,
            STS_TORQUE_ENABLE,
            0
        )

        if result == COMM_SUCCESS:
            print(f"✅ ID={servo_id} 力矩已释放")
        else:
            print(f"⚠️ ID={servo_id} 力矩释放失败: {scs.getTxRxResult(result)}")

        time.sleep(0.03)


def sync_write_id1_id5(id1_deg, id5_deg, speed=0, acc=0, use_safe_limit=True):
    """
    只同步下发 ID1 和 ID5。
    其他关节不会收到任何位置指令。
    """
    if use_safe_limit:
        id1_deg = limit_id1_angle(id1_deg)
        id5_deg = limit_id5_angle(id5_deg)
    else:
        id1_deg = clamp(id1_deg, ID1_MIN_DEG, ID1_MAX_DEG)
        id5_deg = clamp(id5_deg, ID5_MIN_DEG, ID5_MAX_DEG)

    scs.groupSyncWrite.clearParam()

    scs.SyncWritePosEx(
        ID_BASE,
        angle_to_pos(id1_deg),
        speed,
        acc
    )

    scs.SyncWritePosEx(
        ID_WRIST,
        angle_to_pos(id5_deg),
        speed,
        acc
    )

    result = scs.groupSyncWrite.txPacket()

    return result, id1_deg, id5_deg


def home_servos():
    """
    只让 ID1 和 ID5 回到 Home 位。
    """
    print("\n🏠 机械臂 ID1 和 ID5 回到 Home 位...")

    enable_control_torque()

    result, id1_home, id5_home = sync_write_id1_id5(
        ID1_CENTER_DEG,
        ID5_CENTER_DEG,
        speed=1500,
        acc=40,
        use_safe_limit=False
    )

    if result == COMM_SUCCESS:
        print(f"✅ Home 指令已下发: ID1={id1_home:.1f}°, ID5={id5_home:.1f}°")
    else:
        print(f"⚠️ Home 指令下发异常: {scs.getTxRxResult(result)}")

    time.sleep(1.5)


# ================= 5. 宠物大脑主循环 =================
def pet_loop():
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    FRAME_W, FRAME_H = 640, 480
    CENTER_X, CENTER_Y = FRAME_W // 2, FRAME_H // 2

    mp_face = mp.solutions.face_detection.FaceDetection(
        model_selection=0,
        min_detection_confidence=0.7
    )

    mp_hands = mp.solutions.hands.Hands(
        model_complexity=0,
        max_num_hands=1,
        min_detection_confidence=0.7
    )

    current_id1_deg = ID1_CENTER_DEG
    current_id5_deg = ID5_CENTER_DEG

    emotion_state = "TRACKING"
    emotion_start_time = 0.0
    EMOTION_DURATION = 1.5

    last_command_time = 0.0

    print("\n" + "=" * 60)
    print("🤖 电子宠物已上线")
    print("当前只控制机械臂 ID1 和 ID5")
    print("ID1：人脸左右跟随 / STOP 手势摇头")
    print("ID5：人脸上下跟随 / GOOD 手势点头")
    print("ID2、ID3、ID4、ID6、ID7：完全不下发指令")
    print("按 q 退出")
    print("=" * 60)

    while True:
        success, img = cap.read()

        if not success:
            print("❌ 摄像头读取失败")
            break

        img = cv2.flip(img, 1)
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        face_results = mp_face.process(imgRGB)
        hand_results = mp_hands.process(imgRGB)

        # ================= 手势识别 =================
        if emotion_state == "TRACKING":
            gesture = "NONE"

            if hand_results.multi_hand_landmarks:
                for handLms in hand_results.multi_hand_landmarks:
                    mp.solutions.drawing_utils.draw_landmarks(
                        img,
                        handLms,
                        mp.solutions.hands.HAND_CONNECTIONS
                    )
                    gesture = detect_gesture(handLms)

            if gesture == "GOOD":
                emotion_state = "NODDING"
                emotion_start_time = time.time()
                print("触发情绪：开心！ID5 点头")

            elif gesture == "STOP":
                emotion_state = "SHAKING"
                emotion_start_time = time.time()
                print("触发情绪：拒绝！ID1 摇头")

        # ================= 动作执行层 =================
        final_id1_deg = current_id1_deg
        final_id5_deg = current_id5_deg

        ui_text = "STATE: TRACKING"
        ui_color = (0, 255, 0)

        # ---------- 状态 A：正常追踪人脸 ----------
        if emotion_state == "TRACKING":
            if face_results.detections:
                bboxC = face_results.detections[0].location_data.relative_bounding_box

                cx = int((bboxC.xmin + bboxC.width / 2) * FRAME_W)
                cy = int((bboxC.ymin + bboxC.height / 2) * FRAME_H)

                cv2.circle(img, (cx, cy), 5, (0, 0, 255), cv2.FILLED)

                error_x = cx - CENTER_X
                error_y = cy - CENTER_Y

                if abs(error_x) > DEADZONE:
                    delta_id1 = error_x * KP_ID1 * ID1_DIR
                    delta_id1 = limit_delta(delta_id1, MAX_DELTA_ID1_DEG)

                    current_id1_deg += delta_id1
                    current_id1_deg = limit_id1_angle(current_id1_deg)

                if abs(error_y) > DEADZONE:
                    delta_id5 = error_y * KP_ID5 * ID5_DIR
                    delta_id5 = limit_delta(delta_id5, MAX_DELTA_ID5_DEG)

                    current_id5_deg += delta_id5
                    current_id5_deg = limit_id5_angle(current_id5_deg)

            final_id1_deg = current_id1_deg
            final_id5_deg = current_id5_deg

        # ---------- 状态 B：开心点头，只动 ID5 ----------
        elif emotion_state == "NODDING":
            elapsed = time.time() - emotion_start_time

            if elapsed < EMOTION_DURATION:
                offset = math.sin(elapsed * 25.0) * 20.0

                final_id1_deg = current_id1_deg
                final_id5_deg = current_id5_deg + offset

                ui_text = "STATE: HAPPY (ID5 NODDING)"
                ui_color = (0, 255, 255)

            else:
                emotion_state = "TRACKING"

        # ---------- 状态 C：拒绝摇头，只动 ID1 ----------
        elif emotion_state == "SHAKING":
            elapsed = time.time() - emotion_start_time

            if elapsed < EMOTION_DURATION:
                offset = math.sin(elapsed * 25.0) * 20.0

                final_id1_deg = current_id1_deg + offset
                final_id5_deg = current_id5_deg

                ui_text = "STATE: REFUSE (ID1 SHAKING)"
                ui_color = (0, 0, 255)

            else:
                emotion_state = "TRACKING"

        # ================= 软件限位 =================
        final_id1_deg = limit_id1_angle(final_id1_deg)
        final_id5_deg = limit_id5_angle(final_id5_deg)

        # ================= 指令限频下发 =================
        now = time.time()

        if now - last_command_time >= COMMAND_DT:
            result, safe_id1, safe_id5 = sync_write_id1_id5(
                final_id1_deg,
                final_id5_deg,
                speed=0,
                acc=0,
                use_safe_limit=True
            )

            if result == COMM_SUCCESS:
                final_id1_deg = safe_id1
                final_id5_deg = safe_id5

            last_command_time = now

        # ================= UI 渲染 =================
        cv2.putText(
            img,
            ui_text,
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            ui_color,
            2
        )

        cv2.putText(
            img,
            f"ID1={final_id1_deg:.1f} deg | ID5={final_id5_deg:.1f} deg",
            (20, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            img,
            "Only ID1 and ID5 are controlled | Press q to quit",
            (20, FRAME_H - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1
        )

        cv2.imshow("Emotional Pet Arm - ID1 and ID5 Only", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    mp_face.close()
    mp_hands.close()
    cv2.destroyAllWindows()


# ================= 6. 执行流程 =================
try:
    home_servos()
    pet_loop()

finally:
    print("\n宠物进入休眠，ID1 和 ID5 回 Home 并释放力矩...")

    try:
        sync_write_id1_id5(
            ID1_CENTER_DEG,
            ID5_CENTER_DEG,
            speed=1200,
            acc=40,
            use_safe_limit=False
        )
        time.sleep(1.0)

    except Exception as e:
        print(f"⚠️ 回 Home 异常: {e}")

    disable_control_torque()
    portHandler.closePort()
    print("✨ 设备已安全释放。")