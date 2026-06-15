import cv2
import time

# 打开摄像头（索引为2的外接摄像头）
capture = cv2.VideoCapture(0)

# 设置视频编码格式为MJPG (使用标准宏定义替代数字 6)
capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

frame_count = 0
start_time = time.time()
fps_display = "0.00"  # 用于保存并持续显示的 FPS 文本

while True:
    ret, frame = capture.read()
    if not ret:
        break

    frame_count += 1

    # 每隔10帧更新一次 FPS 数值
    if frame_count % 10 == 0:
        end_time = time.time()
        fps = frame_count / (end_time - start_time)
        fps_display = "{:.2f}".format(fps)

        # 重置计数器和时间
        frame_count = 0
        start_time = time.time()

    # 绘制 FPS (每一帧都画，使用刚刚更新的数值)
    cv2.putText(frame, "FPS: " + fps_display, (10, 50), cv2.FONT_ITALIC, 1, (0, 255, 0), 2)

    # [核心修复] 将显示画面的代码移到外层，保证每帧刷新
    cv2.imshow('CameraPreview', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()

