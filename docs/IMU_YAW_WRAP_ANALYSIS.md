# IMU YAW ±180° 翻转为何不影响当前里程计

## 问题描述

YIS130 IMU 输出的 `YAW_ANGLE` 范围为 **[-180°, +180°]**。当旋转经过 +180° 时，数值会突变为 -180°（反之亦然）。  
直觉上这个跳变似乎会"污染"里程计——但实际分析表明：**对当前 IM_TEST 模型中的 X/Y 积分毫无影响**。

## 数据链路

```
IMU CAN帧 → YAW_ANGLE (度, [-180,+180])
                 ↓
         REAL_YAW = YAW_ANGLE
                 ↓
         rtU.DEG = REAL_YAW
                 ↓
    [IM_TEST_step()]
         θ = DEG × 0.0174532924   (转弧度)
         ↓
    Y增量 = sin(θ + 45°) × W1 + sin(θ + 135°) × W2
    X增量 = cos(θ + 45°) × W1 + cos(θ + 135°) × W2
         ↓
    X_tt += X增量
    Y_tt += Y增量
```

## 为什么 sin/cos 天然免疫翻转

### 数学本质

sin 和 cos 是 **2π 周期函数**，对于任何角度 α：

$$
\sin(\alpha) = \sin(\alpha + 2\pi k), \quad k \in \mathbb{Z}
$$

关键性质：在 ±180° 翻转点：

$$
\sin(+180°) = \sin(-180°) = 0
$$
$$
\cos(+180°) = \cos(-180°) = -1
$$

**函数值完全相同，没有不连续。**

### 具体例子

假设车辆从 +179° 转到 +181°（物理连续）：

| 时刻 | IMU输出 | sin(θ) | cos(θ) |
|------|---------|--------|--------|
| t₁ | +179° | sin(179°) = 0.01745 | cos(179°) = -0.99985 |
| t₂ | -179°（翻转后） | sin(-179°) = -0.01745 | cos(-179°) = -0.99985 |

从 +179° 到 -179° 的 sin/cos 变化量：
- Δsin = -0.01745 - 0.01745 = -0.0349
- Δcos = -0.99985 - (-0.99985) = 0

如果没有翻转（假设能输出 +181°）：
- sin(181°) = -0.01745，cos(181°) = -0.99985

**结果完全一致！** 因为 -179° 和 +181° 是同一个物理角度。

## 什么时候翻转会有害

1. **直接做差求角速度**：`ω = (yaw[n] - yaw[n-1]) / dt`  
   在翻转瞬间会得到 ±360° 的假跳变 → **有害**

2. **累加连续航向**：如果需要输出 "车辆总共转了几圈" → 需要 unwrap

3. **发送给上位机**：如果上位机期望连续 heading → 绘图会有跳变

## 当前固件为何安全

当前 `IM_TEST_step()` 对 `rtU.DEG` 的**唯一使用方式**：

```c
// 度 → 弧度
rtb_Sum2_mk = 0.0174532924F * rtU.DEG;

// 仅用于 sin/cos 分解轮位移到 X/Y
rtb_yraw = arm_sin_f32(rtb_Sum2_mk + 0.785398185F) * rtU.W1 
         + arm_sin_f32(rtb_Sum2_mk + 2.3561945F) * rtU.W2;

rtb_Sum2_mk = arm_cos_f32(rtb_Sum2_mk + 0.785398185F) * rtU.W1 
            + arm_cos_f32(rtb_Sum2_mk + 2.3561945F) * rtU.W2;
```

**不存在**以下操作：
- ❌ `current_yaw - prev_yaw`（求差）
- ❌ `yaw_total += delta_yaw`（累加）
- ❌ 条件判断 `if (yaw > threshold)`

模型只把 YAW 当作"当前朝向的三角函数参数"，而三角函数天然是周期性的。

## 后续需要 unwrap 的场景

当我们实现 `ODOM_POSE` 二进制协议，向上位机（ROS）发送连续 heading 时：

```c
// 需要在固件侧加 unwrap
static float yaw_continuous = 0.0f;
static float yaw_prev = 0.0f;

float delta = yaw_current - yaw_prev;
if (delta > 180.0f) delta -= 360.0f;
if (delta < -180.0f) delta += 360.0f;
yaw_continuous += delta;
yaw_prev = yaw_current;
```

这部分将在 ODOM_POSE 协议实现时加入，**当前阶段不需要**。

## IMU 验证数据总结

| 指标 | 测量值 | 判定 |
|------|--------|------|
| 静止20s漂移 | ~0.08° (0.004°/s) | ✅ 优秀 |
| 方向约定 | CW减小 / CCW增大 | ✅ 右手系Z-up |
| 90°转动精度 | 读数~91.86° | ✅ 误差~2% |
| ±180°边界 | 正→负跳变 | ✅ 对sin/cos无害 |
