# reBot + AeroHand 右手视觉映射仿真

本目录包含 reBot 六自由度机械臂与 AeroHand 右手灵巧手的 MuJoCo 组合模型，以及基于摄像头和 MediaPipe 的纯仿真手势映射程序。

视觉程序只控制仿真中的 7 个 AeroHand 执行器，不控制真实灵巧手，也不会修改机械臂的 6 路控制量。程序不依赖 ROS、串口或 AeroHand 实机 SDK。

## 目录结构

```text
rebot_aerohand_right/
├── aero_hand_right_cv_control_sim.py       # 摄像头手势到仿真灵巧手的映射程序
├── README.md
└── mujoco_xml/
    ├── rebot_arm_right_hand.xml            # 推荐加载的完整组合模型
    ├── rebor_arm_6dof.xml                  # reBot 机械臂定义
    ├── aerohand_right.xml                  # 灵巧手资产、肌腱、执行器和约束
    ├── aerohand_right_body.xml             # 直接安装在 link6 下的手部 body 树
    ├── assets/                             # AeroHand 网格
    └── rebot_6dof/                         # reBot 网格和纹理
```

组合模型使用 `include` 加载机械臂和灵巧手公共定义。`tetheria_mount` 是 `link6` 的直接子 body，不使用 `freejoint` 或 `weld`，因此模型具有真实的 22 个关节位置自由度：机械臂 6 个、灵巧手 16 个。

## 环境依赖

建议在项目使用的 Python 虚拟环境中安装：

```bash
pip install mujoco mediapipe opencv-python numpy
```

程序需要：

- 可正常加载组合 XML 的 MuJoCo Python 包；
- OpenCV 可访问的摄像头；
- 支持 MediaPipe Hands 的 Python 环境；
- GLFW/OpenGL 图形环境。

## 快速运行

在当前工作区根目录执行：

```bash
python3 xml/rebot_aerohand_right/aero_hand_right_cv_control_sim.py \
  --start-enabled \
  --gl-mode hardware
```

脚本默认加载：

```text
xml/rebot_aerohand_right/mujoco_xml/rebot_arm_right_hand.xml
```

如果不使用 `--start-enabled`，程序启动后视觉控制处于暂停状态，需要在 OpenCV 摄像头窗口中按空格启用。

## 基本操作

按键需要在 OpenCV 摄像头窗口获得焦点时使用。

| 按键 | 功能 |
|---|---|
| `SPACE` | 启用或暂停实时视觉映射 |
| `O` | 暂停视觉映射并命令仿真灵巧手打开 |
| `C` | 暂停视觉映射并命令仿真灵巧手闭合 |
| `R` | 清除视觉滤波状态，并将目标控制重置为打开 |
| `V` | 显示或隐藏手指弯曲比例诊断信息 |
| `Q` / `ESC` | 退出程序 |

`O` 和 `C` 是排查模型与视觉映射问题的重要工具：如果这两个按键可以正常开合，说明 XML 执行器工作正常，问题通常位于摄像头识别或归一化参数。

## 控制数据流程

```text
摄像头
  → 最新帧采集线程
  → MediaPipe Hands 后台推理线程
  → 21 个世界坐标关键点
  → 手腕中心化与掌面坐标对齐
  → 扩展为 25 点手指分组
  → 计算 16 个关节弯曲角
  → valley/peak 归一化到 0～1
  → 开手死区和响应曲线
  → 7 路 AeroHand MuJoCo 控制量
```

其中：

- `0` 弯曲比例表示关节打开；
- `1` 弯曲比例表示关节闭合；
- 归一化结果会限制在 `[0, 1]`；
- 小于开手死区的数值会直接设置为 `0`；
- 响应指数大于 1 时，接近打开的手势会更接近完全打开控制。

## 16 关节到 7 执行器的映射

MediaPipe 重定向后的关节顺序为：

```text
thumb(4), index(3), middle(3), ring(3), pinky(3)
```

映射关系如下：

| MuJoCo 执行器 | 映射来源 |
|---|---|
| `right_index_A_tendon` | 食指 3 个弯曲关节的平均值 |
| `right_middle_A_tendon` | 中指 3 个弯曲关节的平均值 |
| `right_ring_A_tendon` | 无名指 3 个弯曲关节的平均值 |
| `right_pinky_A_tendon` | 小指 3 个弯曲关节的平均值 |
| `right_thumb_A_cmc_abd` | 拇指外展关节比例 |
| `right_th1_A_tendon` | 拇指第 2、3 个关节比例的平均值 |
| `right_th2_A_tendon` | 拇指第 3、4 个关节比例的平均值 |

程序通过执行器名称查询 MuJoCo ID。当前完整模型中，机械臂执行器为索引 `0～5`，灵巧手执行器为索引 `6～12`。程序只写入后者，因此不会覆盖机械臂命令。

## 肌腱控制方向

四指和拇指肌腱使用绝对肌腱长度位置控制，与 `aero_hand_right_sim2real_control_test.py` 的仿真语义一致：

- 肌腱长度较大：手指打开；
- 肌腱长度较小：手指闭合；
- 拇指外展关节例外：控制角度增大时拇指向闭合抓取方向运动。

实际端点如下：

| 控制项 | 打开 | 闭合 |
|---|---:|---:|
| 食指肌腱 | `0.110387` | `0.058520` |
| 中指肌腱 | `0.110387` | `0.058520` |
| 无名指肌腱 | `0.110387` | `0.058520` |
| 小指肌腱 | `0.110387` | `0.058520` |
| 拇指外展 | `0` | `1.5` |
| 拇指肌腱 1 | `0.038389` | `0.026152` |
| 拇指肌腱 2 | `0.112138` | `0.081568` |

控制插值采用：

```python
ctrl = open_ctrl + closure_ratio * (closed_ctrl - open_ctrl)
```

因此四指的 `closure_ratio` 增大时，肌腱控制长度会自动减小。

## XML 关键帧

`rebot_arm_right_hand.xml` 包含两个关键帧：

- `Key 0 = closed`
- `Key 1 = open`

在 MuJoCo 原生界面的 `Simulation` 面板中选择 Key，然后点击 `Load key` 即可切换。关键帧中的控制量与视觉程序使用的打开/闭合端点一致。

## 性能与线程设计

为避免转动 MuJoCo 视角时卡顿，程序将耗时工作与 viewer 主线程分离：

- 摄像头线程持续读取最新帧，并丢弃过期帧；
- MediaPipe 在独立视觉线程中运行；
- 默认只以 `20 Hz` 执行视觉推理；
- MuJoCo/GLFW 主线程只负责控制滤波、仿真步进和 `viewer.sync()`；
- 每轮最多补偿 5 个落后的仿真步，避免长时间追赶阻塞界面；
- OpenCV 窗口只在视觉线程产生新结果时刷新，不会在每个 MuJoCo 步重复绘制。

默认使用硬件 OpenGL：

```bash
--gl-mode hardware
```

只有硬件驱动无法创建窗口时才使用软件渲染：

```bash
--gl-mode software
```

软件模式使用 Mesa llvmpipe，CPU 占用较高，MuJoCo 视角拖动通常会比硬件模式慢。

## 常用命令行参数

查看全部参数：

```bash
python3 xml/rebot_aerohand_right/aero_hand_right_cv_control_sim.py --help
```

常用参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--model` | 组合模型路径 | 指定 MuJoCo XML |
| `--camera` | `0` | OpenCV 摄像头编号 |
| `--width` | `640` | 摄像头请求宽度 |
| `--height` | `480` | 摄像头请求高度 |
| `--camera-fps` | `30` | 摄像头请求帧率 |
| `--process-width` | `384` | MediaPipe 输入宽度 |
| `--vision-hz` | `20` | MediaPipe 最大推理频率 |
| `--model-complexity` | `0` | MediaPipe 模型复杂度 |
| `--track-hand` | `right` | 跟踪 `right`、`left` 或 `any` |
| `--landmark-alpha` | `0.70` | 关键点 EMA 滤波系数 |
| `--command-alpha` | `0.25` | 控制量平滑系数 |
| `--open-deadband` | `0.10` | 小弯曲量归零阈值 |
| `--response-gamma` | `1.35` | 开手区域响应指数 |
| `--lost-hand-open-delay` | `0.35` | 丢失检测后自动打开的延迟，单位秒 |
| `--realtime-factor` | `1.0` | 仿真实时倍率 |
| `--gl-mode` | `hardware` | OpenGL 渲染方式 |
| `--start-enabled` | 关闭 | 启动时立即启用视觉控制 |
| `--no-mirror` | 关闭 | 禁用摄像头水平镜像 |

## 推荐配置

一般使用：

```bash
python3 xml/rebot_aerohand_right/aero_hand_right_cv_control_sim.py \
  --start-enabled \
  --gl-mode hardware \
  --vision-hz 20 \
  --process-width 384
```

CPU 性能有限或界面仍卡顿：

```bash
python3 xml/rebot_aerohand_right/aero_hand_right_cv_control_sim.py \
  --start-enabled \
  --gl-mode hardware \
  --vision-hz 15 \
  --process-width 320 \
  --model-complexity 0
```

真人手已经张开，但仿真手仍有小幅弯曲：

```bash
python3 xml/rebot_aerohand_right/aero_hand_right_cv_control_sim.py \
  --start-enabled \
  --open-deadband 0.15 \
  --response-gamma 1.5
```

## 常见问题

### 1. 灵巧手只能闭合，不能重新张开

先在摄像头窗口按 `O`：

- 如果能够打开，XML 和肌腱执行器正常，应调整视觉归一化参数；
- 如果不能打开，检查是否加载了本目录的最新 `rebot_arm_right_hand.xml`；
- 确认四指打开控制是较大的肌腱长度，而不是 `0`。

可以适当提高：

```bash
--open-deadband 0.15 --response-gamma 1.5
```

### 2. 做出张手动作后仍保持上一次握拳

程序检测到手时会持续更新目标；检测丢失超过默认 `0.35 s` 后会自动打开。如果需要更快恢复：

```bash
--lost-hand-open-delay 0.2
```

观察摄像头窗口顶部标签。如果持续显示 `NONE`，可尝试：

```bash
--track-hand any
```

### 3. MuJoCo 窗口转动视角很卡

优先确认使用：

```bash
--gl-mode hardware
```

然后降低视觉负载：

```bash
--vision-hz 15 --process-width 320 --model-complexity 0
```

不要在硬件 OpenGL 可用时使用 `--gl-mode software`。

### 4. 摄像头无法打开

尝试其他编号：

```bash
--camera 1
```

Linux 下可先查看摄像头设备：

```bash
ls /dev/video*
```

### 5. 左右手标签与画面不一致

程序默认镜像摄像头画面，并跟踪 `right`。可以临时使用：

```bash
--track-hand any
```

如果摄像头或上游程序已经完成镜像，可增加：

```bash
--no-mirror
```

### 6. MuJoCo 硬件窗口创建失败

使用软件模式排查：

```bash
--gl-mode software
```

软件模式能够运行但明显卡顿，通常说明系统 OpenGL/GPU 驱动需要进一步配置。

## 修改灵巧手安装距离

灵巧手直接安装在 `link6` 下，只需修改：

```text
mujoco_xml/aerohand_right_body.xml
```

当前根节点类似：

```xml
<body name="tetheria_mount"
      pos="0.010000174 -0.000418407 -0.025"
      quat="0.405604 0.579211 -0.579208 0.405609"
      childclass="tetheria_rh">
```

第三个 `pos` 分量控制沿机械臂末端轴线的距离。数值每减少 `0.001`，灵巧手靠近机械臂约 `1 mm`。直接嵌套结构不需要同步修改关键帧或其他 XML。

## 验证建议

每次修改模型或控制映射后，建议按以下顺序检查：

1. 加载 XML，确认没有编译错误；
2. 在 MuJoCo 中加载 `closed` 和 `open` 关键帧；
3. 运行视觉程序，按 `C` 和 `O` 验证纯仿真开合；
4. 启用视觉映射，观察 `T/I/M/R/P` 弯曲比例；
5. 确认机械臂 6 路控制量未被视觉程序改写；
6. 最后调整死区、响应指数和视觉频率。

## License

AeroHand 仿真模型保留原项目的 Apache License 2.0 授权。使用或分发模型资产时，请同时遵守目录内 `mujoco_xml/LICENSE` 的条款。
