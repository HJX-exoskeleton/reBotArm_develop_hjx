# reBotArm MuJoCo 独立控制示例

本目录提供不依赖 ROS 的 reBotArm MuJoCo 控制示例，模型位于 `../xml`。
程序使用 MJCF 中定义的 7 个原生位置执行器，并为机械臂 6 个关节增加
MuJoCo 偏置力（包括重力）补偿。

## 基础关节控制示例

启动交互式 MuJoCo Viewer：

```bash
cd /reBotArm_develop_hjx/mujoco/rebotarm_b601_colored_control
python3 rebotarm_control_demo.py
```

执行短时间无界面测试：

```bash
python3 rebotarm_control_demo.py --headless --no-realtime --duration 2
```

7 个控制目标的顺序为：

```text
[joint1, joint2, joint3, joint4, joint5, joint6, gripper]
```

机械臂关节位置单位为弧度，夹爪位置表示单侧手指的移动距离，单位为米。
程序会从当前 MJCF 执行器范围中自动读取夹爪的闭合和完全打开位置，目前为：

```text
完全闭合：0.0000 m
完全打开：0.0485 m
```

程序启动时会校验夹爪执行器范围与左右手指关节范围是否一致，并根据当前
XML 动态生成演示轨迹。初始姿态中 `joint4～joint6` 均为 0，夹爪处于闭合状态。

基础演示的动作顺序为：

```text
初始位置，夹爪闭合
→ 原地打开夹爪
→ 移动到左侧姿态
→ 闭合夹爪
→ 闭合状态下移动到右侧姿态
→ 打开夹爪
→ 返回初始位置
→ 在初始位置闭合夹爪
```

进行二次开发时，可以使用自己的规划器替换 `CyclicWaypointTrajectory` 输出，
并在每个 `step()` 前调用：

```python
simulation.set_target([
    joint1,
    joint2,
    joint3,
    joint4,
    joint5,
    joint6,
    gripper,
])
simulation.step()
```

`rebotarm_control_demo.py` 采用纯关节空间控制，不读取 XML 中的 `tcp` 或
`grasp_center` site。

## 物理抓取放置示例

启动黄色圆柱抓取放置任务：

```bash
cd /reBotArm_develop_hjx/mujoco/rebotarm_b601_colored_control
python3 rebotarm_pick_place_demo.py
```

执行无界面回归测试：

```bash
python3 rebotarm_pick_place_demo.py --headless --no-realtime
```

抓取示例直接将用户在 MJCF 中设置的 `grasp_center` site 规划到物体实时中心。
夹爪打开位置从当前 MJCF 执行器范围读取；闭合目标则根据黄色圆柱半径和当前
手指碰撞盒自动计算，并只设置 `0.3 mm` 的预紧量。程序不会盲目命令夹爪运动
到 0，从而避免手指严重穿入物体。

只有当 MuJoCo 同时检测到物体与左、右手指碰撞体接触时，程序才会判定抓取
成功。物体完全依靠接触力和摩擦力被抬升，不使用虚拟吸附，不重写物体位姿，
也不会关闭夹爪碰撞。

抓取示例中所有末端 FK、Jacobian 和 IK 计算均直接使用 `grasp_center`，不会
读取旧的 `tcp` site。`mj_jacSite` 会直接计算所选 site 的世界位姿及 Jacobian，
其中已包含它相对 `end_link` 的偏移，因此不需要额外进行 TCP 补偿。

抓取过程中 `joint4`、`joint5` 和 `joint6` 始终保持为 0，使腕部在接近、抓取、
搬运和释放过程中保持摆正。机械臂先移动到黄色圆柱正上方，随后让
`grasp_center` 沿世界坐标 Z 方向下降到圆柱中心，再闭合夹爪。

完整动作顺序为：

```text
移动到圆柱正上方
→ grasp_center 下降到圆柱中心
→ 闭合夹爪并确认左右物理接触
→ 抬升并确认圆柱被实际带离桌面
→ 搬运到放置区域上方
→ 下降
→ 打开夹爪并释放圆柱
→ 保持张开状态撤离
→ 返回初始关节位置
→ 空载完全闭合夹爪
```

自定义放置位置：

```bash
python3 rebotarm_pick_place_demo.py --place 0.34 0.15 0.065
```

使用其他具有自由关节的物体：

```bash
python3 rebotarm_pick_place_demo.py --object yellow_cylinder
```
