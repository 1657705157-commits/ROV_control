# 开发修改记录（开发者用）

本文件记录所有对 ArduSub 源码和飞控参数的改动，含**原因、调试过程和踩过的坑**。
面向外部读者的项目说明见 [`README.md`](README.md)。

---

## 基础环境

| 项目 | 版本 / 配置 |
|---|---|
| ArduSub | 4.5.7（commit `abe1721cf5`） |
| 分支 | `Sub-4.5` |
| 飞控 | CUAV Pixhawk V6X（`Pixhawk6X`, APJ_BOARD_ID 53, STM32H743） |
| 工具链 | xPack arm-none-eabi-gcc 10.3.1（原生 arm64） |
| 编译 | `./waf configure --board Pixhawk6X && ./waf sub` |
| Flash 占用 | 1,670,592 / 1,966,080 B（剩约 288 KB） |

> GCC 13.2 会在 ChibiOS 上报 `-Werror=address` 编译失败，必须用 10.3.x。

---

## 改动总览

| # | 文件 | 内容 |
|---|---|---|
| 1 | `libraries/AP_Motors/AP_Motors6DOF.cpp/.h` | 7 电机自定义混控 + 双矩阵切换 |
| 2 | `ArduSub/motors.cpp` | 按模式切换混控；电机测试绕过安全开关 |
| 3 | `ArduSub/mode_althold.cpp` | 去掉航向闭环 |

---

## 一、7 推进器混控矩阵

### 背景

`SUB_FRAME_CUSTOM` 在 ArduSub 4.5.7 里是空实现：

```cpp
case SUB_FRAME_CUSTOM:
    // Put your custom motor setup here
    //break;        // ← break 被注释掉，会 fallthrough 到 SIMPLEROV_3
```

必须补全，否则 `FRAME_CONFIG=7` 实际跑的是 3 推进器布局。

### 硬件布局

机体坐标（X 前 / Y 右 / Z 下），单位 mm：

```
M1 (150, -550) ∥ Z    左前
M2 (150, +550) ∥ Z    右前
M3 (-150, -550) ∥ Z   左后
M4 (-150, +550) ∥ Z   右后
M5 (0, -386) ∥ X      左侧前后
M6 (0, +386) ∥ X      右侧前后
M7 (0, 0) ∥ Y         中心横移
```

### 力矩计算

`τ = r × F`，按最大力臂 550mm 归一化：

| 电机 | 力矩分量 | 归一化系数 |
|---|---|---|
| M1~M4 | Roll: z=±550<br>Pitch: x=∓150 | Roll ±1.000<br>Pitch ∓0.273 |
| M5/M6 | Yaw: y=∓386 | Yaw ∓0.702 |
| M7 | Lateral: 1.0 | Lateral 1.000 |

### 踩坑 1：pitch 力矩过弱

**现象**：pitch 拉满时电机 PWM 变化只有 ±3，roll 有 ±38。

**原因**：几何归一化让 `pitch = 150/550 = 0.273`，但本机是 1:3.67 的长方形布局，
pitch 最大力矩只有 roll 的 **7.4%**。

**解决**：改用「方案 B · 等推力幅度」，pitch 系数提到 ±1.000（力矩提升到 27%）。

```cpp
// 方案 A（几何归一化，弃用）
add_motor_raw_6dof(MOT_1, -1.000f, -0.273f, ...);

// 方案 B（等推力幅度，采用）
add_motor_raw_6dof(MOT_1, -1.000f, -1.000f, ...);
```

---

## 二、双混控矩阵（水平 / 侧躺 90°）

### 背景

ROV 两种工作形态的**机体坐标系不同**：

```
① 水平姿态（MANUAL）：机体 Z 轴 = 世界垂直
   M1~M4 垂直 → Roll / Pitch / Climb
   M5/M6 前后 → Yaw / Forward
   M7   横移 → Lateral

② 侧躺 90°（ALT_HOLD）：AHRS_ORIENTATION=16 让飞控旋转坐标系
   新 Z' = 原 +Y → M7 变成垂直
   新 Y' = 原 -Z → M1~M4 变成水平侧推
```

**同一套矩阵不可能同时满足两种姿态。**

### 现象（改动前）

侧躺时切 ALT_HOLD，深度闭环持续输出给 M1~M4（错误地认为是垂直推进器），
但它们实际推的是水平方向 → 深度不变 → 误差累积 → **输出饱和到 `MOT_PWM_MIN`**：

```
M1: 1400  M3: 1400   ← 顶到下限
M2: 1558  M4: 1573
M7: 1500             ← 真正的垂直推进器没动
```

### 实现

**`AP_Motors6DOF.h`**：新增切换接口

```cpp
void configure_custom_frame(bool rolled_90);   // public

private:
    bool _rolled_90 = false;
    void set_motor_factors(int8_t motor_num, float roll_fac, float pitch_fac,
                           float yaw_fac, float climb_fac,
                           float forward_fac, float lat_fac);
    void apply_custom_frame(bool rolled_90);
```

**`AP_Motors6DOF.cpp`**：

- `setup_motors()` 的 `SUB_FRAME_CUSTOM` 初始配为**水平矩阵**，`_rolled_90 = false`
- `apply_custom_frame()` 内直接写系数数组（**不能调 `add_motor_raw()`**，
  它会在 `initialised_ok()` 后直接 return）
- `configure_custom_frame()` 做状态缓存，只在变化时重配

**`ArduSub/motors.cpp`**：

```cpp
if (g.frame_configuration == (int8_t)AP_Motors6DOF::SUB_FRAME_CUSTOM) {
    motors.configure_custom_frame(control_mode == Mode::Number::ALT_HOLD);
}
```

### 两套矩阵

**水平**：

| 电机 | roll | pitch | yaw | climb | forward | lateral |
|---|---|---|---|---|---|---|
| M1 | -1.000 | -1.000 | 0 | +1.000 | 0 | 0 |
| M2 | +1.000 | -1.000 | 0 | +1.000 | 0 | 0 |
| M3 | -1.000 | +1.000 | 0 | +1.000 | 0 | 0 |
| M4 | +1.000 | +1.000 | 0 | +1.000 | 0 | 0 |
| M5 | 0 | 0 | +0.702 | 0 | +1.000 | 0 |
| M6 | 0 | 0 | -0.702 | 0 | +1.000 | 0 |
| M7 | 0 | 0 | 0 | 0 | 0 | +1.000 |

**侧躺 90°**：

| 电机 | roll | pitch | yaw | climb | forward | lateral |
|---|---|---|---|---|---|---|
| M1 | -1.000 | 0 | -0.273 | 0 | 0 | 0 |
| M2 | +1.000 | 0 | -0.273 | 0 | 0 | 0 |
| M3 | -1.000 | 0 | +0.273 | 0 | 0 | 0 |
| M4 | +1.000 | 0 | +0.273 | 0 | 0 | 0 |
| M5 | 0 | -0.702 | 0 | 0 | +1.000 | 0 |
| M6 | 0 | +0.702 | 0 | 0 | +1.000 | 0 |
| M7 | 0 | 0 | 0 | +1.000 | 0 | 0 |

### 验证结果

```
MANUAL + throttle 20%：
  M1~M4 全部 1525（+25）  ✓ 水平矩阵生效
  M7 1500

ALT_HOLD 摇杆中位：
  M1: 1403  M2: 1596  M3: 1403  M4: 1596   ← 姿态修正
  M7: 1467                                  ← 深度闭环
  ✓ 侧躺矩阵生效
```

---

## 三、ALT_HOLD 去掉航向闭环

### 现象

Alt_Hold 摇杆全中位时，M5/M6 反向偏转：

```
M5: 1530 (+0.30)
M6: 1469 (-0.31)     ← 差值 61
```

但 RC4（yaw）实测 = 1500（中位）。

### 根因

`mode_althold.cpp` 原逻辑（第 88-99 行）：

```cpp
} else { // hold current heading
    if (tnow < sub.last_pilot_yaw_input_ms + 250) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(...);
    } else {
        // 摇杆回中后，保持"上次动杆时的绝对航向"
        attitude_control->input_euler_angle_roll_pitch_yaw(
            target_roll, target_pitch, sub.last_pilot_heading, true);
    }
}
```

ROV 在水流/机械臂反作用下航向漂移 → 飞控持续输出 yaw 修正。

### 改动

```cpp
} else {
    // 自定义：不保持绝对航向（yaw 不做位置闭环）
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
        target_roll, target_pitch, 0.0f);
    sub.last_pilot_heading = ahrs.yaw_sensor;
}
```

原代码以注释形式保留在文件里，可随时恢复。

### 验证

改前 `M5=1530 M6=1469`（Δ61）→ 改后 `M5=1500 M6=1499`（Δ1）✅

---

## 四、电机测试绕过硬件安全开关

### 背景

ArduSub 的 Motor Test 要求硬件安全开关处于「已解除」状态：

```cpp
if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
    gcs().send_text(MAV_SEVERITY_CRITICAL,
        "Disarm hardware safety switch before testing motors.");
    return false;
}
```

本机未安装物理安全开关，`BRD_SAFETY_DEFLT = 0` 实测未能让
`safety_switch_state()` 返回 `SAFETY_ARMED`（Pixhawk6X 的 PF5 引脚为
`INPUT PULLDOWN`，电平判断与实际硬件不符）。

### 改动

`ArduSub/motors.cpp` 的 `init_motor_test()` 中**注释掉该检查**，
原代码保留注释。

---

## 五、飞控参数变更

| 参数 | 原值 | 新值 | 原因 |
|---|---|---|---|
| `FRAME_CONFIG` | 1 | **7** | 启用 CUSTOM 混控 |
| `AHRS_ORIENTATION` | 0 | **16** | Roll90，把侧躺视为水平 |
| `MOT_PWM_MIN` | 1000 | **1400** | 缩小行程，提高分辨率 |
| `MOT_PWM_MAX` | 2000 | **1600** | 同上 |
| `JS_BUTTON7` | 0 | **2** | `k_arm_toggle`，ArduSub 解锁前置条件 |
| `BATT_MONITOR` | 8 | **0** | 无 DroneCAN 电池，避免解锁被拦 |
| `BRD_SAFETY_DEFLT` | 1 | **0** | 禁用硬件安全开关 |
| `PSC_ACCZ_I` | 0.1 | **0** | 深度环改纯 P 调试 |

### 关于 `JS_BUTTON7 = 2`

`AP_Arming_Sub.cpp` 强制要求：

```cpp
bool AP_Arming_Sub::has_disarm_function() const {
    if (sub.jsbutton_function_is_assigned(JSButton::k_arm_toggle) ||
        sub.jsbutton_function_is_assigned(JSButton::k_disarm)) {
        return true;
    }
    return false;
}
```

没有配置 `JS_BUTTONx`（游戏手柄按钮）就无法解锁。

---

## 六、调试过程中遇到的问题

### 6.1 MAVLink 的 `DO_MOTOR_TEST` 有三个坑

1. **ArduSub 的 `param1` 是 0-based**（MAVLink 文档写 1-based）：
   `MOT_n` 要传 `n-1`，否则 7 号电机越界失败
2. **必须用 `COMMAND_INT`**（不是 `COMMAND_LONG`）：
   `test_type = command.y` 需要 `MOTOR_TEST_ORDER_BOARD = 2`
3. **每次测试后飞控自动上锁**，下一个电机测试前要重新 arm

### 6.2 `MANUAL` 模式的 RC 通道顺序（反直觉）

`ArduSub/radio.cpp`：

```cpp
channel_pitch    = rc_channel(0);   // RC1 = PITCH  ← 不是 roll！
channel_roll     = rc_channel(1);   // RC2 = ROLL   ← 不是 pitch！
channel_throttle = rc_channel(2);   // RC3
channel_yaw      = rc_channel(3);   // RC4
channel_forward  = rc_channel(4);   // RC5
channel_lateral  = rc_channel(5);   // RC6
```

发送 `RC_CHANNELS_OVERRIDE` 时顺序必须是 `pitch, roll, throttle, yaw, forward, lateral`。

### 6.3 `AHRS_ORIENTATION` 是启动参数

改了必须重启，且改后**加速度计校准数据失效**，必须重新校准
（加速度计 / 罗盘 / 水平）。不校准就切 ALT_HOLD，姿态闭环会朝错误方向修正。

### 6.4 `CTRL` 类参数 UDP 读不到

`set_param.py` 默认走 UDP（香橙派中继），中继挂掉时全部报「收不到心跳」。
调试时加 `--usb` 走 USB 直连。

### 6.5 测试脚本自身的 bug

- `read_servos()` 每轮循环把 throttle 重置成 1500 → MANUAL 测试全是 1500
- 解锁期间不发 `HEARTBEAT` → `MYGCS: 255, heartbeat lost` → 解锁失败

---

## 七、待办 / 已知限制

- [ ] **横移能力消失**：侧躺矩阵里 M7 变成垂直推进器，Lateral 摇杆无响应。
      如需横移，可给 M1~M4 加 `lateral` 系数（四路同向推）。
- [ ] **yaw 力矩弱**（系数 0.273），可考虑提到 ±1.0。
- [ ] **模式切换瞬间有推力跳变**，可加斜坡过渡。
- [ ] **深度环纯 P 未在水里验证**（台架上水压计读大气压，看不出真实行为）。
- [ ] **姿态环 I 项仍在**（`ATC_RAT_RLL_I = 0.046`），如遇长期漂移可清零。

---

## 八、相关工具

上位机脚本在 `onboard_computer/`（未纳入本仓库）：

| 脚本 | 用途 |
|---|---|
| `rov_console.py` | 图形控制台（6 摇杆 + 状态面板 + 模式切换 + UDP 广播） |
| `manual_axis_test.py` | MANUAL 逐轴验证混控矩阵 |
| `verify_dual_matrix.py` | 验证双矩阵按模式切换 |
| `verify_althold.py` | 验证 ALT_HOLD 航向闭环已去除 |
| `set_param.py` | 参数读写（支持 UDP / `--usb`） |
| `diag_arm.py` | 解锁失败诊断（收集 PreArm 消息） |
