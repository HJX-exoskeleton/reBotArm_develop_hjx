# reBot Arm B601-DM 的 二次开发

<p align="center">
    <a href="./LICENSE">
        <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT">
    </a>
    <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python Version">
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu-orange.svg" alt="Platform">
    <img src="https://img.shields.io/badge/Framework-Pinocchio-yellow.svg" alt="Pinocchio">
</p>

<p align="center">
  <strong>6 自由度机械臂 · 多电机支持 · 运动学求解 · 轨迹规划 · 完全开源</strong>
</p>

---

## 📖 项目简介

**reBotArm develop** 是一个面向 reBot Arm B601 系列机械臂的 Python 二次开发项目，由hjx作者创建。

### ✨ 核心特性

- 🦾 **达妙型号** — B601-DM（达妙电机）机械臂
- 🧮 **重力补偿** — 微调版本的重力补偿
- 🛤️ **mujoco** — mujoco仿真sim2real以及real2sim
- 🔧 **灵活配置** — YAML 配置文件，快速适配不同硬件

---

## ⚙️ 快速开始

### 环境要求

| 项目 | 要求                   |
|------|----------------------|
| **Python** | 3.10+                |
| **操作系统** | Ubuntu 20.04+        |
| **通信接口** | USB2CAN 串口桥 或 CAN 接口 |

### 安装步骤

#### 步骤 1. 安装 uv（如未安装）

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

#### 步骤 2. 同步环境（安装所有依赖）

```bash
git clone https://github.com/vectorBH6/reBotArm_control_py.git
cd reBotArm_control_py
uv sync
```

:::tip
`uv sync` 会自动创建虚拟环境（如不存在）并根据 `pyproject.toml` 和 `uv.lock` 安装所有依赖。
:::

---

## 🔌 硬件配置

### 默认配置：达妙 USB2CAN 串口桥

reBot Arm B601-DM 默认使用达妙 USB2CAN 串口桥模块。

**硬件连接**：
1. 将 USB2CAN 模块通过 USB 线连接到计算机
2. 系统会自动识别为 `/dev/ttyACM0` 设备

**配置验证**：
```bash
# 检查设备
ls /dev/ttyACM0

# 扫描电机
motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600
```

### 可选配置：标准 CAN 接口

使用其他 USB-CAN 适配器（CANable、PCAN 等）：

```bash
# 启动 CAN 接口
sudo ip link set can0 up type can bitrate 500000

# 验证接口
ip -details link show can0
```

### 电机品牌配置

| 电机品牌 | 传输方式 | 配置参数 | 波特率 |
|----------|---------|---------|--------|
| **达妙 (Damiao)** | 串口桥 | `dm-serial` | 921600 |
| **达妙 (Damiao)** | CAN 接口 | `socketcan` | 500000 |
| **RobStride** | CAN 接口 | `socketcan` | 500000 |

:::tip
- 达妙电机使用串口桥时，必须设置 `--transport dm-serial`
- 反馈 ID 规则：`feedback_id = motor_id + 0x10`
:::

---


## 🎮 示例程序

### 调试工具

#### 零点校准与角度监控 (`rebot_zero_and_read.py`)

自动设置所有关节零点，实时显示关节角度。

**运行方式**：
```bash
uv run python gravity_compensation/rebot_zero_and_read.py
```

---


### 实机控制

:::tip 权限设置
运行实机控制示例前，需要设置设备权限：

```bash
# 设置串口设备权限（达妙 USB2CAN）
sudo chmod 666 /dev/ttyACM0

# 或设置 CAN 设备权限（如 can0）
sudo chmod 666 /dev/can0
```
:::

#### 重力补偿控制 (`rebot_gravity_compensation_control.py`)

使用 Pinocchio 动力学模型补偿关节重力。

**控制律**：
```
tau = g(q)          — 重力前馈
pos = 当前电机位置   — 关节位置跟随当前位置
kp = 2,  kd = 1     — 所有关节统一刚度/阻尼
```

**预期行为**：
- 机械臂可以在任意姿态下"漂浮"
- 松开后不会因自重坠落
- 可以手动掰动到任意位置

**运行方式**：
```bash
uv run python gravity_compensation/rebot_gravity_compensation_control.py
```

**输出**：
- 实时显示各关节期望力矩（N·m）
- 按 `Ctrl+C` 停止并断开连接

---

:::

#### mujoco仿真 (`rebot_mujoco`)

基于 mujoco 仿真的机械臂运动控制以及数字孪生。

**运行方式**：
```bash
# mujoco real2sim
cd reBotArm_develop_hjx
python mujoco/rebot_mujoco_real2sim_gravity_compensation.py

# mujoco sim2real
uv run python mujoco/rebot_mujoco_sim2real_position_setting.py --start-from-keyframe --calibrate-current-as-keyframe

uv run python mujoco/rebot_mujoco_sim2real_control_dance.py --sim-only
uv run python mujoco/rebot_mujoco_sim2real_control_dance.py --start-from-keyframe --calibrate-current-as-keyframe
```
**输出**：运行real2sim以及sim2real代码将会出现mujoco仿真画面

![image](https://github.com/reBotArm_develop_hjx/image/rebot_mujoco.jpg)

---

## 📄 License

本项目采用 **MIT 许可证** 开源。

---

## 参考项目

- **reBot Arm B601-DM 的 Pinocchio 与 MeshCat 入门指南**: [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py)
- **reBot-DevArm: 为每个开发者开源的机械臂**: [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm)

---

<p align="center">
  <strong>🌟 如果本项目对你有帮助，请给个 Star 支持一下！</strong>
</p>
