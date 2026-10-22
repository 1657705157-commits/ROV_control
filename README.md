# ROV_control — ArduSub 二次开发固件

针对 **CUAV Pixhawk V6X（ArduPilot board ID 53）** 的水下机器人固件二次开发补丁。

基于 **ArduSub 4.5.7**（commit `abe1721cf5`）。

---

## 1. 硬件配置

| 项目 | 配置 |
|---|---|
| 飞控 | CUAV Pixhawk V6X (Pixhawk6X, APJ_BOARD_ID 53) |
| 推进器 | 7 个双向无刷（ESC 中值 1500μs） |
| PWM 输出 | M1~M7 推进器，M8 补光灯 |
| 深度传感器 | MS5837（I2C3，`BARO_PRIMARY=2`） |
| 机体 | Roll 90° 侧躺工作姿态，飞控随 ROV 一起侧躺 |

### 推进器布局（相对机体坐标，mm）

```
M1 (150, -550) ∥ Z 轴   左前
M2 (150, +550) ∥ Z 轴   右前
M3 (-150, -550) ∥ Z 轴  左后
M4 (-150, +550) ∥ Z 轴  右后
M5 (0, -386)  ∥ X 轴    左侧前后
M6 (0, +386)  ∥ X 轴    右侧前后
M7 (0, 0)     ∥ Y 轴    中心横移
```

---

## 2. 改动内容

共 4 个文件，`+140 / -22` 行。

### 2.1 `libraries/AP_Motors/AP_Motors6DOF.cpp` / `.h`

**① 自定义 7 推进器混控矩阵**（`SUB_FRAME_CUSTOM`，`FRAME_CONFIG=7`）

原本是空实现（`//break;` 被注释掉，会 fallthrough 到 `SIMPLEROV_3`）。

**② 双混控矩阵，按飞行模式自动切换**

ROV 有两种工作形态，机体坐标系不同：

| 模式 | ROV 姿态 | 混控矩阵 |
|---|---|---|
| `MANUAL` | 水平 | M1~M4 垂直 → Roll/Pitch/Climb<br>M5/M6 前后 → Yaw/Forward<br>M7 横移 → Lateral |
| `ALT_HOLD` | 侧躺 90° | M1~M4 水平侧推 → Roll/Yaw<br>M5/M6 前后 → Pitch/Forward<br>M7 垂直 → Climb（深度） |

切换由 `ArduSub/motors.cpp` 的 `motors_output()` 每周期调用
`configure_custom_frame(control_mode == ALT_HOLD)` 完成。

**水平矩阵**：

| 电机 | roll | pitch | yaw | climb | forward | lateral |
|---|---|---|---|---|---|---|
| M1 | -1.000 | -1.000 | 0 | +1.000 | 0 | 0 |
| M2 | +1.000 | -1.000 | 0 | +1.000 | 0 | 0 |
| M3 | -1.000 | +1.000 | 0 | +1.000 | 0 | 0 |
| M4 | +1.000 | +1.000 | 0 | +1.000 | 0 | 0 |
| M5 | 0 | 0 | +0.702 | 0 | +1.000 | 0 |
| M6 | 0 | 0 | -0.702 | 0 | +1.000 | 0 |
| M7 | 0 | 0 | 0 | 0 | 0 | +1.000 |

**侧躺 90° 矩阵**（`AHRS_ORIENTATION=16` 时）：

| 电机 | roll | pitch | yaw | climb | forward | lateral |
|---|---|---|---|---|---|---|
| M1 | -1.000 | 0 | -0.273 | 0 | 0 | 0 |
| M2 | +1.000 | 0 | -0.273 | 0 | 0 | 0 |
| M3 | -1.000 | 0 | +0.273 | 0 | 0 | 0 |
| M4 | +1.000 | 0 | +0.273 | 0 | 0 | 0 |
| M5 | 0 | -0.702 | 0 | 0 | +1.000 | 0 |
| M6 | 0 | +0.702 | 0 | 0 | +1.000 | 0 |
| M7 | 0 | 0 | 0 | +1.000 | 0 | 0 |

### 2.2 `ArduSub/motors.cpp`

- `motors_output()` 中按 `control_mode` 调用 `configure_custom_frame()`
- `init_motor_test()` 中**注释掉了硬件安全开关检查**（未安装物理开关时无法测电机）

### 2.3 `ArduSub/mode_althold.cpp`

**去掉 ALT_HOLD 的航向闭环**

原逻辑在摇杆回中后会以「上次摇杆动的航向」为目标持续修正 yaw，
导致水流/机械臂反作用力下飞控持续输出 yaw 修正、M5/M6 反向。

改为：摇杆回中时只把 yaw 角速率目标设为 0，不做绝对航向保持。
原代码已注释保留，可随时恢复。

---

## 3. 关键参数

```
FRAME_CONFIG        = 7       (CUSTOM)
AHRS_ORIENTATION    = 16      (Roll90，飞控把侧躺视为水平)
MOT_PWM_MIN         = 1400
MOT_PWM_MAX         = 1600
JS_BUTTON7          = 2       (k_arm_toggle，ArduSub 解锁前置条件)
BATT_MONITOR        = 0       (无电池监测时禁用)
BRD_SAFETY_DEFLT    = 0       (禁用硬件安全开关)

# 深度环（纯 P）
PSC_POSZ_P          = 3
PSC_VELZ_P          = 8
PSC_VELZ_I          = 0
PSC_VELZ_D          = 0
PSC_ACCZ_P          = 0.5
PSC_ACCZ_I          = 0
```

---

## 4. 编译

需要 ARM GCC 工具链（>= 10.2.1，推荐 xPack 10.3.1）。

```bash
git clone --recurse-submodules -b Sub-4.5 https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# 应用本补丁
git apply /path/to/firmware-changes.patch

# 编译
./waf configure --board Pixhawk6X
./waf sub

# 烧录
./waf --upload sub
```

产物：`build/Pixhawk6X/bin/ardusub.apj`

---

## 5. 目录说明

```
.
├── README.md
├── firmware-changes.patch      # 完整 git diff，可直接 git apply
└── ardupilot-patch/            # 改动后的完整文件（便于阅读/对照）
    ├── ArduSub/
    │   ├── mode_althold.cpp
    │   └── motors.cpp
    └── libraries/
        └── AP_Motors/
            ├── AP_Motors6DOF.cpp
            └── AP_Motors6DOF.h
```

---

## 6. 使用流程

```
MANUAL 模式（ROV 水平）
  → 传统布局：Throttle 控制升降、Lateral 控制横移
  → 移动到目标位置

切 ALT_HOLD
  → 飞控驱动 ROV 自动翻转 90°
  → 侧躺矩阵生效：深度闭环走 M7，姿态闭环走 M1~M4
  → 松手保持深度和姿态
```

---

## 7. 注意事项

1. **`AHRS_ORIENTATION=16` 后必须重新校准**加速度计 / 罗盘 / 水平
2. **模式切换瞬间有推力跳变**（两套矩阵系数不同）
3. **yaw 力矩较弱**（力臂 150mm vs roll 550mm，系数仅 0.273）
4. ALT_HOLD 在空气中无法验证深度闭环（水压计读大气压）
