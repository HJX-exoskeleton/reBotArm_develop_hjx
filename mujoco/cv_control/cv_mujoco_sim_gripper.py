import os
import cv2
import numpy as np
import time
import math
import mujoco
import mujoco.viewer
import sys
import logging
from pathlib import Path

# 设置日志级别，避免输出过多的底层日志信息
logging.getLogger().setLevel(logging.ERROR)

ROOT_DIR = Path(__file__).resolve().parents[1]
XML_PATH = str(ROOT_DIR / "xml" / "rebot_gripper" / "sim_reBot_grasp.xml")

# ==========================================
# --- 映射与平滑配置参数 ---
# ==========================================
# 视觉手势距离标定 (像素)
VISUAL_DIST_CLOSED = 30.0  # 手指闭合时的像素距离
VISUAL_DIST_OPEN = 180.0  # 手指完全张开时的像素距离

# 仿真夹爪控制范围 (米) - 请根据您的 XML 实际 actuator_ctrlrange 调整
SIM_GRIPPER_CLOSED = 0.001
SIM_GRIPPER_OPEN = 0.05

# 动作平滑系数 (0.0 到 1.0)
# 越小越平滑但延迟越高，越大跟随越紧但越容易抖动
SMOOTHING_FACTOR = 0.15

# === 1. 初始化 MuJoCo 模型与数据 ===
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)
model.opt.timestep = 0.005

# 获取夹爪执行器 ID 并打印其控制范围供参考
gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")
if gripper_actuator_id >= 0:
    ctrl_range = model.actuator_ctrlrange[gripper_actuator_id]
    print(f"模型加载成功！夹爪执行器 ID: {gripper_actuator_id}, 控制范围: {ctrl_range}")
else:
    print("⚠️ 未能在模型中找到名为 'gripper' 的执行器，请检查 XML 文件！")
    sys.exit()

# === 2. 主程序入口 ===
if __name__ == "__main__":
    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    print("MuJoCo 3D 渲染窗口已启动...")

    print("正在初始化手势识别模块...")
    import HandTrackingModule as htm

    # 提高 detectionCon 也可以在一定程度上减少识别抖动
    detector = htm.handDetector(detectionCon=0.8, trackCon=0.8)

    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    pTime = 0

    # 记录上一次的夹爪目标值，用于平滑滤波
    current_smoothed_cmd = SIM_GRIPPER_OPEN

    while viewer.is_running():
        success, img = cap.read()
        if not success:
            break

        # ==========================================
        # --- A. 视觉检测与手势控制核心逻辑 ---
        # ==========================================
        img = detector.findHands(img, draw=True)
        lmList = detector.findPosition(img, draw=False)

        if len(lmList) != 0:
            # 1. 提取关键点坐标 (大拇指 4, 食指 8)
            x1, y1 = lmList[4][1], lmList[4][2]
            x2, y2 = lmList[8][1], lmList[8][2]
            xc, yc = (x1 + x2) // 2, (y1 + y2) // 2

            # 2. 计算手指间距 (像素)
            length = math.hypot(x2 - x1, y2 - y1)

            # 3. 在画面中绘制连线和指尖圆点
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.circle(img, (x1, y1), 10, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), 10, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (xc, yc), 8, (255, 0, 255), cv2.FILLED)

            cv2.putText(img, f'Dist: {int(length)}', (xc + 20, yc),
                        cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)

            # 4. 映射到仿真控制 (像素距离 -> 仿真物理位置)
            # 使用 np.interp 会自动处理越界（钳位/Clamp）问题
            target_sim_val = np.interp(
                length,
                [VISUAL_DIST_CLOSED, VISUAL_DIST_OPEN],
                [SIM_GRIPPER_CLOSED, SIM_GRIPPER_OPEN]
            )

            # 5. 应用低通滤波平滑处理
            current_smoothed_cmd = (SMOOTHING_FACTOR * target_sim_val) + ((1 - SMOOTHING_FACTOR) * current_smoothed_cmd)

            # 将平滑后的值发送给 MuJoCo 执行器
            data.ctrl[gripper_actuator_id] = current_smoothed_cmd

            # 6. 闭合提示 (UI)
            if length < VISUAL_DIST_CLOSED:
                cv2.circle(img, (xc, yc), 12, (0, 255, 0), cv2.FILLED)

        # ==========================================
        # --- B. 仿真物理步进 ---
        # ==========================================
        mujoco.mj_step(model, data)

        # ==========================================
        # --- C. 画面同步与显示 ---
        # ==========================================
        viewer.sync()  # 同步状态到 MuJoCo 渲染器

        # FPS 计算与显示
        cTime = time.time()
        fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
        pTime = cTime
        cv2.putText(img, f'FPS: {int(fps)}', (10, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

        # 在画面左上角显示当前发给 MuJoCo 的控制指令，方便调试
        cv2.putText(img, f'Ctrl: {current_smoothed_cmd:.4f}m', (10, 80), cv2.FONT_HERSHEY_PLAIN, 2, (0, 165, 255), 2)

        # 显示手势控制监控画面
        cv2.imshow("Hand Control View", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    viewer.close()
