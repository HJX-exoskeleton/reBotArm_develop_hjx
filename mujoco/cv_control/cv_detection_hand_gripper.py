import cv2  # pip install opencv-python
import HandTrackingModule as htm
import numpy as np
import time
import math

# 1. 摄像头参数设置
wCam, hCam = 640, 480
cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)

pTime = 0
detector = htm.handDetector()

while True:
    success, img = cap.read()
    if not success:
        break

    # 2. 检测手势
    img = detector.findHands(img)
    lmList = detector.findPosition(img, draw=False)

    if len(lmList) != 0:
        # 获取大拇指 (ID 4) 和食指 (ID 8) 指尖坐标
        x1, y1 = lmList[4][1], lmList[4][2]
        x2, y2 = lmList[8][1], lmList[8][2]

        # 计算中心点，用于放置文字标签
        xc, yc = (x2 + x1) // 2, (y2 + y1) // 2

        # 3. 计算开合距离
        length = math.hypot(x2 - x1, y2 - y1)

        # 4. 视觉增强：画出连线和指尖圆点
        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
        cv2.circle(img, (x1, y1), 10, (255, 0, 255), cv2.FILLED)
        cv2.circle(img, (x2, y2), 10, (255, 0, 255), cv2.FILLED)

        # 5. 【核心修改】在画面中实时显示距离数值
        # 我们把文字放在两个手指的中心位置 (xc, yc)，并稍微偏移一点点
        cv2.putText(img, f'Dist: {int(length)}', (xc + 20, yc),
                    cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

        # 终端同时也保持打印
        print(f"Distance: {int(length)}")

        # 触发效果：距离极小时改变颜色
        if length < 30:
            cv2.circle(img, (xc, yc), 15, (0, 255, 0), cv2.FILLED)

    # 6. 显示 FPS
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, f'FPS: {int(fps)}', (10, 40), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    # 7. 窗口展示
    cv2.imshow("Hand Distance Tracker", img)

    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
