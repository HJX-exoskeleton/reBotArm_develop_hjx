# 导入OpenCV库，用于图像和视频处理
import cv2  # pip install opencv-python
# 导入time模块，用于计算帧率（FPS）
import time

# 使用VideoCapture函数打开摄像头（索引为0的默认摄像头）
capture = cv2.VideoCapture(0)  # 电脑自带摄像头
# capture = cv2.VideoCapture(2)  # 电脑外接摄像头

# 尝试设置不同的分辨率，找到最适合的组合
# 设置摄像头捕获的帧宽为640像素
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# 设置摄像头捕获的帧高为480像素
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 设置MJPG编码（通常比YUYV更快）
# 注意：这个设置可能不被所有摄像头支持
capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

# 尝试设置更高的帧率
capture.set(cv2.CAP_PROP_FPS, 30)

# 设置缓冲区大小（减少延迟）
capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# 初始化帧率计算变量
frame_count = 0
fps = 0
last_fps_time = time.time()
fps_update_interval = 0.5  # 每0.5秒更新一次FPS显示

print("按 'q' 键退出")

while True:
    # 记录开始时间
    start_time = time.time()

    # 读取一帧图像
    ret, frame = capture.read()

    # 若读取帧失败，则跳出循环
    if not ret:
        print("无法从摄像头读取帧")
        break

    # 帧计数器递增
    frame_count += 1

    # 计算当前时间
    current_time = time.time()

    # 定期更新FPS显示
    if current_time - last_fps_time >= fps_update_interval:
        fps = frame_count / (current_time - last_fps_time)
        frame_count = 0
        last_fps_time = current_time
        print(f"当前FPS: {fps:.2f}")

    # 在帧上添加显示FPS的文字
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # 添加分辨率显示
    height, width = frame.shape[:2]
    cv2.putText(frame, f"Resolution: {width}x{height}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # 显示当前帧
    cv2.imshow('Camera Preview', frame)

    # 计算处理时间
    processing_time = time.time() - start_time

    # 控制显示帧率（非阻塞）
    # 等待1ms，但如果处理时间超过1ms，则立即处理下一帧
    wait_time = max(1, int(1000 / 30 - processing_time * 1000))  # 目标30fps
    key = cv2.waitKey(wait_time) & 0xFF

    # 检查按键，如果按下'q'键，则退出循环
    if key == ord('q'):
        break
    # 可选：按's'键保存当前帧
    elif key == ord('s'):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"screenshot_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"已保存截图: {filename}")

# 关闭摄像头
capture.release()
# 关闭所有OpenCV窗口
cv2.destroyAllWindows()
print("程序结束")