# ROV_control

**水下清洗机器人 · ArduSub 固件二次开发**

适用于**侧向翻转 90° 工作姿态**的水下清洗机器人（7 推进器布局），
基于 **ArduSub 4.5.7**，运行于 **CUAV Pixhawk V6X** 飞控。

---

## 简介

常规 ROV 水平姿态作业，而清洗机器人需要**侧躺 90°** 贴附在船体/管道表面工作。
本项目在 ArduSub 上实现了：

- **双工作形态**：水平移动 → 侧躺清洗，按飞行模式自动切换推进器布局
- **7 推进器自定义混控**：4 垂直 + 2 前后 + 1 横移
- **侧躺姿态闭环**：飞控跟随机体翻转，仍能保持姿态和深度

---

## 硬件

| 项目 | 配置 |
|---|---|
| 飞控 | CUAV Pixhawk V6X（`Pixhawk6X`, board ID 53） |
| 推进器 | 7 × 双向无刷（ESC 中值 1500μs） |
| 补光灯 | M8 通道 |
| 深度传感器 | MS5837（I2C3） |
| 工作姿态 | Roll 90°（飞控随 ROV 一起侧躺） |

### 推进器布局

```
        M1(150,-550)      M2(150,+550)      ← 垂直（水平姿态时）
        M3(-150,-550)     M4(-150,+550)

              M5(0,-386)  M6(0,+386)       ← 前后

                   M7(0,0)                 ← 横移
```

---

## 两种工作形态

### ① 水平姿态（MANUAL 模式）

ROV 正常航行，移动到目标位置：

| 摇杆 | 动作 | 电机 |
|---|---|---|
| Throttle | 升降 | M1~M4 |
| Forward | 前后 | M5/M6 |
| Lateral | 横移 | M7 |
| Roll/Pitch | 姿态 | M1~M4 |

### ② 侧躺 90°（ALT_HOLD 模式）

贴附工作面清洗，飞控自动翻转并保持：

| 摇杆 / 闭环 | 动作 | 电机 |
|---|---|---|
| **深度闭环** | 保持深度 | **M7** |
| **Roll 闭环** | 保持工作姿态 | M1~M4 |
| Forward | 前后移动 | M5/M6 |
| Yaw | 开环（不保持航向） | M1~M4 |

**切换方式**：在控制界面点 `ALT_HOLD` 按钮，飞控会驱动 ROV 自动翻转 90°。

---

## 关键改动

| 文件 | 改动 |
|---|---|
| `libraries/AP_Motors/AP_Motors6DOF.cpp/.h` | 自定义 7 推进器混控，双矩阵按模式切换 |
| `ArduSub/motors.cpp` | `motors_output()` 按 `control_mode` 切换混控 |
| `ArduSub/mode_althold.cpp` | 去掉摇杆回中后的绝对航向保持（yaw 开环） |

详见 [`CHANGELOG.md`](CHANGELOG.md)。

---

## 编译

```bash
git clone --recurse-submodules -b Sub-4.5 https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git apply /path/to/firmware-changes.patch

./waf configure --board Pixhawk6X
./waf sub
./waf --upload sub
```

工具链：ARM GCC ≥ 10.2.1（推荐 xPack 10.3.1）

产物：`build/Pixhawk6X/bin/ardusub.apj`

---

## 关键参数

```
FRAME_CONFIG       = 7       CUSTOM（自定义混控）
AHRS_ORIENTATION   = 16      Roll90（侧躺视为水平）
MOT_PWM_MIN        = 1400
MOT_PWM_MAX        = 1600
JS_BUTTON7         = 2       k_arm_toggle（ArduSub 解锁必需）
BATT_MONITOR       = 0       无电池监测时禁用
BRD_SAFETY_DEFLT   = 0       禁用硬件安全开关

# 深度环（纯 P 调试）
PSC_POSZ_P = 3    PSC_VELZ_P = 8    PSC_ACCZ_P = 0.5
PSC_VELZ_I = 0    PSC_VELZ_D = 0    PSC_ACCZ_I = 0
```

---

## 使用流程

```
1. 上电，ROV 保持水平
2. 切 MANUAL，用摇杆航行到目标位置
3. 点 ALT_HOLD → 飞控驱动 ROV 翻转 90°
4. 松手保持深度和姿态，开始清洗作业
5. 完成后再切回 MANUAL → ROV 翻回水平
```

---

## 注意事项

1. **修改 `AHRS_ORIENTATION` 后必须重新校准**（加速度计 / 罗盘 / 水平）
2. **模式切换瞬间有推力跳变**（两套矩阵系数不同），水下操作时注意
3. **yaw 力矩较弱**（力臂 150mm，系数 0.273）
4. ALT_HOLD 的深度闭环需要真的在水里才能验证

---

## 目录

```
.
├── README.md               # 本文件
├── CHANGELOG.md            # 详细修改记录（开发者用）
├── firmware-changes.patch  # 完整 git diff，可直接 apply
└── ardupilot-patch/        # 改动后的完整源文件
    ├── ArduSub/
    └── libraries/AP_Motors/
```
