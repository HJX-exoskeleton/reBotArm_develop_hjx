import time
import sys
import os
import cv2
import mediapipe as mp
import numpy as np

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


# ================= 1. 基础硬件参数 =================
# 必须和 ESP32 透传程序 Serial.begin(...) 一致
# 如果 ESP32 是 Serial.begin(115200)，这里就是 115200
# 如果 ESP32 是 Serial.begin(1000000)，这里改成 1000000
BAUDRATE = 115200

# Linux
DEVICENAME = '/dev/ttyUSB0'
# Windows 示例
# DEVICENAME = 'COM6'

CAMERA_ID = 0

# 本程序只控制这两个关节
ID_BASE = 1
ID_WRIST = 5

CONTROL_SERVO_IDS = [ID_BASE, ID_WRIST]

SERVO_DIGITAL_RANGE = 4095.0
SERVO_ANGLE_RANGE = 360.0


def angle_to_pos(angle):
    return int((angle / SERVO_ANGLE_RANGE) * SERVO_DIGITAL_RANGE)


def pos_to_angle(pos):
    return (pos / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE


def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))


# ================= 2. 软件限位 =================
# 这里保留你的完整 7 自由度限位，但本程序只使用 ID1 和 ID5
JOINT_LIMITS_DEG = {
    1: {
        "name": "ID1",
        "desc_min": "左偏",
        "desc_max": "右偏",
        "min_deg": 50.0,
        "max_deg": 300.0,
        "home_deg": 180.0,
    },
    2: {
        "name": "ID2",
        "desc_min": "前倾",
        "desc_max": "初始",
        "min_deg": 10.0,
        "max_deg": 180.0,
        "home_deg": 180.0,
    },
    3: {
        "name": "ID3",
        "desc_min": "抬头极限",
        "desc_max": "初始",
        "min_deg": 22.0,
        "max_deg": 180.0,
        "home_deg": 180.0,
    },
    4: {
        "name": "ID4",
        "desc_min": "低头",
        "desc_max": "抬头",
        "min_deg": 100.0,
        "max_deg": 270.0,
        "home_deg": 180.0,
    },
    5: {
        "name": "ID5",
        "desc_min": "右偏",
        "desc_max": "左偏",
        "min_deg": 90.0,
        "max_deg": 270.0,
        "home_deg": 180.0,
    },
    6: {
        "name": "ID6",
        "desc_min": "逆转",
        "desc_max": "顺转",
        "min_deg": 90.0,
        "max_deg": 270.0,
        "home_deg": 180.0,
    },
    7: {
        "name": "ID7",
        "desc_min": "极限位置",
        "desc_max": "初始",
        "min_deg": 90.0,
        "max_deg": 180.0,
        "home_deg": 180.0,
    },
}

# 软件限位安全余量，避免贴近机械极限
SAFETY_MARGIN_DEG = 3.0
USE_SAFETY_MARGIN = True


def get_safe_min_deg(servo_id):
    raw_min = JOINT_LIMITS_DEG[servo_id]["min_deg"]
    if USE_SAFETY_MARGIN:
        return raw_min + SAFETY_MARGIN_DEG
    return raw_min


def get_safe_max_deg(servo_id):
    raw_max = JOINT_LIMITS_DEG[servo_id]["max_deg"]
    if USE_SAFETY_MARGIN:
        return raw_max - SAFETY_MARGIN_DEG
    return raw_max


def limit_angle(servo_id, angle, use_margin=True):
    if use_margin and USE_SAFETY_MARGIN:
        min_deg = get_safe_min_deg(servo_id)
        max_deg = get_safe_max_deg(servo_id)
    else:
        min_deg = JOINT_LIMITS_DEG[servo_id]["min_deg"]
        max_deg = JOINT_LIMITS_DEG[servo_id]["max_deg"]

    return clamp(angle, min_deg, max_deg)


def print_control_limits():
    print("\n" + "=" * 70)
    print("当前程序只控制 ID1 和 ID5")
    print("=" * 70)

    for servo_id in CONTROL_SERVO_IDS:
        cfg = JOINT_LIMITS_DEG[servo_id]
        print(
            f"ID={servo_id} | "
            f"机械范围: {cfg['min_deg']:.1f}° ~ {cfg['max_deg']:.1f}° | "
            f"软件安全范围: {get_safe_min_deg(servo_id):.1f}° ~ {get_safe_max_deg(servo_id):.1f}° | "
            f"Home: {cfg['home_deg']:.1f}°"
        )

    print("ID2、ID3、ID4、ID6、ID7：本程序不下发任何位置指令")
    print("=" * 70 + "\n")


# ================= 3. 视觉跟随控制参数 =================
FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

# 死区，单位：像素
DEADZONE_X = 20
DEADZONE_Y = 20

# 指令发送频率，建议 20~30Hz
COMMAND_HZ = 30.0
COMMAND_DT = 1.0 / COMMAND_HZ

# 卡尔曼预测提前量
LEAD_FRAMES = 0.0

# ================= 3.1 ID1 左右控制 =================
# ID1 控制画面左右误差
ID1_KP = 0.012
ID1_KD = 0.035

# 如果 ID1 左右方向反了，改成 1 或 -1
ID1_DIR = -1

# ================= 3.2 ID5 上下控制 =================
# ID5 控制画面上下误差
# 如果你实际发现 ID5 更适合做左右补偿，也可以把 error_y 改成 error_x
ID5_KP = 0.010
ID5_KD = 0.030

# 如果 ID5 上下方向反了，改成 1 或 -1
ID5_DIR = 1

# 每帧最大角度变化，防止突跳
MAX_DELTA_ID1_DEG = 2.0
MAX_DELTA_ID5_DEG = 2.0


def limit_delta(delta, max_abs_delta):
    return clamp(delta, -max_abs_delta, max_abs_delta)


# ================= 4. 卡尔曼滤波器 =================
class KalmanTracker:
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)

        self.kf.measurementMatrix = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
            ],
            np.float32
        )

        self.kf.transitionMatrix = np.array(
            [
                [1, 0, 1, 0],
                [0, 1, 0, 1],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            np.float32
        )

        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 30.0

        self.initialized = False

    def initialize(self, x, y):
        self.kf.statePre = np.array(
            [[np.float32(x)], [np.float32(y)], [0.0], [0.0]],
            dtype=np.float32
        )
        self.kf.statePost = np.array(
            [[np.float32(x)], [np.float32(y)], [0.0], [0.0]],
            dtype=np.float32
        )
        self.initialized = True

    def predict(self):
        pred = self.kf.predict()
        return float(pred[0]), float(pred[1]), float(pred[2]), float(pred[3])

    def correct(self, x, y):
        if not self.initialized:
            self.initialize(x, y)

        measurement = np.array(
            [[np.float32(x)], [np.float32(y)]],
            dtype=np.float32
        )

        self.kf.correct(measurement)


# ================= 5. 硬件初始化 =================
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

print_control_limits()


# ================= 6. 舵机底层控制函数 =================
def enable_control_torque():
    """
    只开启 ID1 和 ID5 的力矩。
    不碰 ID2、ID3、ID4、ID6、ID7。
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
    不碰其他关节。
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


def get_home_angle_dict():
    return {
        ID_BASE: JOINT_LIMITS_DEG[ID_BASE]["home_deg"],
        ID_WRIST: JOINT_LIMITS_DEG[ID_WRIST]["home_deg"],
    }


def sync_write_control_angles(angle_dict, speed=0, acc=0, use_margin=True):
    """
    只同步下发 ID1 和 ID5。
    ID2、ID3、ID4、ID6、ID7 不会收到任何位置指令。
    """
    scs.groupSyncWrite.clearParam()

    safe_angle_dict = {}

    for servo_id in CONTROL_SERVO_IDS:
        raw_angle = angle_dict[servo_id]
        safe_angle = limit_angle(
            servo_id,
            raw_angle,
            use_margin=use_margin
        )

        safe_angle_dict[servo_id] = safe_angle

        target_pos = angle_to_pos(safe_angle)

        scs.SyncWritePosEx(
            servo_id,
            int(target_pos),
            speed,
            acc
        )

    result = scs.groupSyncWrite.txPacket()

    return result, safe_angle_dict


def home_control_joints():
    """
    只让 ID1 和 ID5 回到 Home。
    """
    print("\n🏠 ID1 和 ID5 回到安全 Home 位...")

    enable_control_torque()

    home_angles = get_home_angle_dict()

    result, safe_angles = sync_write_control_angles(
        home_angles,
        speed=1500,
        acc=40,
        use_margin=False
    )

    if result == COMM_SUCCESS:
        print("✅ Home 指令已下发:")
        for servo_id in CONTROL_SERVO_IDS:
            print(f"  ID={servo_id}: {safe_angles[servo_id]:.1f}°")
    else:
        print(f"⚠️ Home 指令下发异常: {scs.getTxRxResult(result)}")

    time.sleep(1.5)


# ================= 7. 视觉跟随主循环 =================
def visual_tracking_loop():
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    mp_face_detection = mp.solutions.face_detection
    face_detection = mp_face_detection.FaceDetection(
        model_selection=0,
        min_detection_confidence=0.6
    )

    tracker = KalmanTracker()

    current_cmd_deg = get_home_angle_dict()

    last_error_x = 0.0
    last_error_y = 0.0

    last_command_time = 0.0

    print("\n" + "=" * 70)
    print("🎯 7 自由度机械臂视觉跟随启动")
    print("当前只控制：ID1 和 ID5")
    print("ID1：根据画面左右误差运动")
    print("ID5：根据画面上下误差运动")
    print("ID2、ID3、ID4、ID6、ID7：不下发控制指令")
    print("按 q 退出")
    print("=" * 70)

    while True:
        success, img = cap.read()

        if not success:
            print("❌ 摄像头读取失败")
            break

        img = cv2.flip(img, 1)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        results = face_detection.process(img_rgb)

        observed = False

        if results.detections:
            bboxC = results.detections[0].location_data.relative_bounding_box

            raw_cx = int((bboxC.xmin + bboxC.width / 2) * FRAME_W)
            raw_cy = int((bboxC.ymin + bboxC.height / 2) * FRAME_H)

            x1 = int(bboxC.xmin * FRAME_W)
            y1 = int(bboxC.ymin * FRAME_H)
            x2 = int((bboxC.xmin + bboxC.width) * FRAME_W)
            y2 = int((bboxC.ymin + bboxC.height) * FRAME_H)

            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 1)
            cv2.circle(img, (raw_cx, raw_cy), 4, (0, 0, 255), cv2.FILLED)

            tracker.correct(raw_cx, raw_cy)
            observed = True

        if tracker.initialized:
            pred_x, pred_y, vel_x, vel_y = tracker.predict()
        else:
            pred_x, pred_y, vel_x, vel_y = CENTER_X, CENTER_Y, 0.0, 0.0

        target_x = int(pred_x + vel_x * LEAD_FRAMES)
        target_y = int(pred_y + vel_y * LEAD_FRAMES)

        # ================== 可视化 ==================
        cv2.drawMarker(
            img,
            (int(pred_x), int(pred_y)),
            (0, 255, 0),
            cv2.MARKER_CROSS,
            20,
            2
        )

        cv2.line(
            img,
            (int(pred_x), int(pred_y)),
            (target_x, target_y),
            (0, 255, 255),
            2
        )

        cv2.circle(img, (target_x, target_y), 5, (0, 255, 255), 2)

        cv2.drawMarker(
            img,
            (CENTER_X, CENTER_Y),
            (255, 255, 255),
            cv2.MARKER_CROSS,
            30,
            1
        )

        # ================== 计算误差 ==================
        error_x = float(target_x - CENTER_X)
        error_y = float(target_y - CENTER_Y)

        d_error_x = error_x - last_error_x
        d_error_y = error_y - last_error_y

        last_error_x = error_x
        last_error_y = error_y

        now = time.time()

        if now - last_command_time >= COMMAND_DT and tracker.initialized:
            update_arm = False

            # ========== ID1：左右跟随 ==========
            if abs(error_x) > DEADZONE_X:
                delta_id1 = (
                    error_x * ID1_KP
                    + d_error_x * ID1_KD
                ) * ID1_DIR

                delta_id1 = limit_delta(delta_id1, MAX_DELTA_ID1_DEG)

                current_cmd_deg[ID_BASE] += delta_id1
                update_arm = True

            # ========== ID5：上下跟随 ==========
            if abs(error_y) > DEADZONE_Y:
                delta_id5 = (
                    error_y * ID5_KP
                    + d_error_y * ID5_KD
                ) * ID5_DIR

                delta_id5 = limit_delta(delta_id5, MAX_DELTA_ID5_DEG)

                current_cmd_deg[ID_WRIST] += delta_id5
                update_arm = True

            # ========== 软件限位 ==========
            current_cmd_deg[ID_BASE] = limit_angle(
                ID_BASE,
                current_cmd_deg[ID_BASE],
                use_margin=True
            )

            current_cmd_deg[ID_WRIST] = limit_angle(
                ID_WRIST,
                current_cmd_deg[ID_WRIST],
                use_margin=True
            )

            # ========== 只下发 ID1 和 ID5 ==========
            if update_arm:
                result, safe_angles = sync_write_control_angles(
                    current_cmd_deg,
                    speed=0,
                    acc=0,
                    use_margin=True
                )

                if result == COMM_SUCCESS:
                    current_cmd_deg = safe_angles
                else:
                    print(f"⚠️ 同步下发异常: {scs.getTxRxResult(result)}")

            last_command_time = now

        # ================== UI 显示 ==================
        status_txt = "TRACKING" if observed else "WAITING"
        status_col = (0, 255, 0) if observed else (0, 165, 255)

        cv2.putText(
            img,
            status_txt,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            status_col,
            2
        )

        cv2.putText(
            img,
            f"err_x={error_x:.1f}, err_y={error_y:.1f}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2
        )

        cv2.putText(
            img,
            f"ID1={current_cmd_deg[ID_BASE]:.1f} deg | ID5={current_cmd_deg[ID_WRIST]:.1f} deg",
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            img,
            "Only ID1 & ID5 are controlled | Press q to quit",
            (10, FRAME_H - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1
        )

        cv2.imshow("7DOF Arm Visual Tracking - ID1 and ID5 Only", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    face_detection.close()
    cv2.destroyAllWindows()


# ================= 8. 执行流程 =================
try:
    home_control_joints()
    visual_tracking_loop()

finally:
    print("\n🛑 视觉跟随结束，ID1 和 ID5 回到 Home 并释放力矩...")

    try:
        home_angles = get_home_angle_dict()
        sync_write_control_angles(
            home_angles,
            speed=1200,
            acc=40,
            use_margin=False
        )
        time.sleep(1.0)
    except Exception as e:
        print(f"⚠️ 回 Home 异常: {e}")

    disable_control_torque()
    portHandler.closePort()
    print("✨ 设备已安全释放。")
