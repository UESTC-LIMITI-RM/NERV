# NERV

NERV 是 NERV's Embedded Robotic Vehicle 的缩写, 一个面向嵌入式机器人底盘的 C 语言库.

NERV 负责电机控制, 底盘运动学, PID 调节与总线收发, 把硬件差异收敛到驱动描述符中. 调用者只面对统一的实例与 API, 不关心具体电机与平台.

## 特性

- 统一电机接口, 所有类型的电机共用同一套 Init, SetControl, Update, OnRx 调用方式
- 达妙 DM-J4310-2EC 支持 MIT, 位置速度, 速度三种控制模式
- DJI M3508, M2006, GM6020 电流与电压模式
- 底盘运动学支持全向轮与麦克纳姆轮两种构型
- 底盘使用串级速控, 集成里程计与安全监督
- FDCAN 总线抽象, 模块层不接触寄存器
- 返回值与错误位两层诊断, 兼顾流程控制与故障定位
- 全库 C11, 无动态内存, 无 RTOS 依赖

## 目录结构

```
public/     通用工具, 数学, PID, 单位换算, 日志, 状态码
bsp/        板级抽象, FDCAN, GPIO, PWM, 时间
module/     业务模块, 电机与底盘
config/     编译期配置入口 config.h
```

## 快速开始

### 1. 选择平台

编辑 `config/Inc/config.h`, 选择目标平台:

```c
#define NERV_PLATFORM NERV_PLATFORM_STM32H7
```

可选值为 `NERV_PLATFORM_STM32H7` 与 `NERV_PLATFORM_CUSTOM`.未选择时编译会直接报错, 避免在错误平台上静默通过.

### 2. 打开需要的模块

```c
#define NERV_USE_MOTOR   NERV_ENABLE
#define NERV_USE_CHASSIS NERV_ENABLE
#define NERV_USE_MOTOR_DM  NERV_ENABLE
#define NERV_USE_MOTOR_DJI NERV_ENABLE
```

关闭的模块不参与编译, 头文件内容为空.

### 3. 加入工程

把以下源文件加入构建:

```
public/Src/*.c
module/motor/Src/*.c
module/motor/driver/*.c
module/motor/driver/DM/*.c
module/motor/driver/DJI/*.c
module/chassis/Src/*.c
config/Src/config_check.c
```

添加头文件搜索路径:

```
public/Inc
config/Inc
bsp/Inc
module/motor/Inc
module/motor/driver
module/motor/driver/DM
module/motor/driver/DJI
module/chassis/Inc
```

以 Keil 为例, 把上述路径填入 Include Path, 把源文件加入对应分组即可.仓库不提供 CMake 或 Makefile.

### 4. 定义总线

实现一个 FDCAN 实例, 把发送, 接收与错误读取接到具体平台:

```c
static const NERV_FDCAN_OpsTypeDef fdcan_ops = {
    .init      = fdcan_init,
    .start     = fdcan_start,
    .stop      = fdcan_stop,
    .configure_filter = fdcan_filter,
    .transmit  = fdcan_transmit,
};

static NERV_FDCAN_InstanceTypeDef can2 = {
    .ops = &fdcan_ops,
    .rx_callback = my_on_rx,
    .rx_user = NULL,
};
```

`bsp/Inc/nerv_fdcan.h` 给了完整的实例定义与操作表约定.

### 5. 初始化电机

```c
#include "nerv_motor.h"

static NERV_Motor_InstanceTypeDef dm_motor;
static NERV_Motor_ControlTypeDef control;

static const NERV_Motor_ConfigTypeDef dm_config = {
    .driver      = &NERV_Motor_Driver_DmVelocity,
    .bus         = &can2,
    .device_id   = 1U,
    .gear_ratio  = 1.0f,
    .offline_ms  = 100U,
    .over_temperature_c = 100.0f,
    .control_limit = 0U,
    .invert_direction = false,
};

NERV_Motor_Init(&dm_motor, &dm_config);
NERV_Motor_Enable(&dm_motor);

control.kind     = NERV_MOTOR_CONTROL_SCALAR;
control.u.scalar = 2.0f;
NERV_Motor_SetControl(&dm_motor, &control);
```

### 6. 周期发送

```c
NERV_Motor_InstanceTypeDef *group[] = { &dm_motor };
NERV_Motor_UpdateGroup(group, 1U, HAL_GetTick());
```

`UpdateGroup` 负责刷新测量值, 执行离线与过温监督, 按总线与帧标识符分组发送指令.同一组每周期只发一帧.

### 7. 接收反馈

在 FDCAN 接收中断中把帧交给对应实例:

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags)
{
    NERV_FDCAN_FrameTypeDef frame;
    /* 从硬件取出帧并填入 frame */
    NERV_Motor_OnRx(&dm_motor, &frame);
}
```

多个电机时按 `frame.id` 路由到各自实例.

### 8. 读取反馈

```c
NERV_Motor_MeasurementTypeDef measurement;
NERV_Motor_GetMeasurement(&dm_motor, &measurement);
```

角度单位为 rad, 速度单位为 rad/s, 温度单位为摄氏度.

## 支持的硬件

| 类型 | 型号               | 控制模式            |
| ---- | ------------------ | ------------------- |
| 达妙 | DM-J4310-2EC       | MIT, 位置速度, 速度 |
| DJI  | M3508 配 C620 电调 | 电流                |
| DJI  | M2006 配 C610 电调 | 电流                |
| DJI  | GM6020             | 电流, 电压          |

达妙驱动实例为 `NERV_Motor_Driver_DmMit`, `NERV_Motor_Driver_DmPositionVelocity`, `NERV_Motor_Driver_DmVelocity`.DJI 驱动实例为 `NERV_Motor_Driver_Dji3508`, `NERV_Motor_Driver_Dji2006`, `NERV_Motor_Driver_Dji6020Current`, `NERV_Motor_Driver_Dji6020Voltage`.

## 底盘

底盘把四路电机组合成机构系速度, 负责运动学解算与安全监督.

底盘使用串级速控: 目标速度经车体速度环修正, 逆解为各轮目标转速,
再由每轮速度环输出电流.

```c
#include "nerv_chassis.h"

static NERV_Chassis_InstanceTypeDef chassis;

NERV_Chassis_Init(&chassis, &chassis_config);

NERV_Chassis_VelocityTypeDef velocity = {
    .vx_mps = 1.0f,
    .vy_mps = 0.0f,
    .wz_radps = 0.0f,
};
NERV_Chassis_SetVelocity(&chassis, &velocity);

NERV_Chassis_Update(&chassis, HAL_GetTick());
```

配置里必须提供三个速度环:

```c
static const NERV_PID_ConfigTypeDef chassis_translate_pid = { ... };
static const NERV_PID_ConfigTypeDef chassis_rotate_pid    = { ... };
static const NERV_PID_ConfigTypeDef chassis_wheel_pid     = { ... };

chassis_config.translate_pid = &chassis_translate_pid;
chassis_config.rotate_pid    = &chassis_rotate_pid;
chassis_config.wheel_pid     = &chassis_wheel_pid;
```

`translate_pid` 与 `rotate_pid` 是车体速度环, 输出速度修正量, 单位分别为 m/s 与 rad/s.
`wheel_pid` 是轮速环, 输出电流, 单位 A.

构型在 `config.h` 中选择:

```c
#define NERV_CHASSIS_TYPE NERV_CHASSIS_TYPE_MECANUM
```

可选 `NERV_CHASSIS_TYPE_OMNI` 与 `NERV_CHASSIS_TYPE_MECANUM`.

里程计开关为 `NERV_USE_CHASSIS_ODOMETRY`.

## 错误处理

两层机制, 职责分开.

第一层是返回值, 回答本次调用是否成功:

```c
if (NERV_OK != NERV_Motor_SetControl(&motor, &control))
{
    /* 本次调用失败 */
}
```

第二层是实例状态位, 回答电机当前处于什么异常状态:

```c
uint32_t error = NERV_Motor_ReadError(&motor);

if (0U != (error & NERV_ERR_MOTOR_OFFLINE)) { /* 电机离线 */ }
if (0U != (error & NERV_ERR_MOTOR_OVER_TEMP)) { /* 电机过温 */ }
if (0U != (error & NERV_ERR_MOTOR_TX_FAIL)) { /* 指令发送失败 */ }
```

底盘对应 `NERV_ERR_CHASSIS_KINEMATICS` 与 `NERV_ERR_CHASSIS_WHEEL_OFFLINE`.

状态位保持粘滞, 读取不清除, 需要显式清除:

```c
NERV_Motor_ClearError(&motor, NERV_ERR_MOTOR_OFFLINE);
```

参数错误, 空指针, 未初始化这类调用期问题不进入状态位, 由返回值表达.

## 配置项

| 宏                          | 说明         | 默认值                   |
| --------------------------- | ------------ | ------------------------ |
| `NERV_PLATFORM`             | 目标平台     | `NERV_PLATFORM_NONE`     |
| `NERV_USE_MOTOR`            | 电机模块开关 | `NERV_ENABLE`            |
| `NERV_USE_CHASSIS`          | 底盘模块开关 | `NERV_ENABLE`            |
| `NERV_USE_LOG`              | 日志模块开关 | `NERV_ENABLE`            |
| `NERV_USE_MOTOR_DM`         | 达妙驱动开关 | `NERV_ENABLE`            |
| `NERV_USE_MOTOR_DJI`        | DJI 驱动开关 | `NERV_ENABLE`            |
| `NERV_CHASSIS_TYPE`         | 底盘构型     | `NERV_CHASSIS_TYPE_OMNI` |
| `NERV_USE_CHASSIS_ODOMETRY` | 里程计开关   | `NERV_DISABLE`           |

完整列表与取值范围见 `config/Inc/config.h`, 编译期检查在 `config/Src/config_check.c`.

## 移植

移植只需要实现 `bsp/Inc` 中的四个抽象层:

- `nerv_fdcan.h`: FDCAN 实例与操作表
- `nerv_gpio.h`: GPIO 实例与操作表
- `nerv_pwm.h`: PWM 实例与操作表
- `nerv_time.h`: 毫秒计时

把 `NERV_PLATFORM` 设为 `NERV_PLATFORM_CUSTOM`, 提供上述实例即可, 模块层不需要修改.

## 许可

MIT, 见 `LICENSE`.

开发约定, 命名规范与模块设计细节见 `AGENTS.md`, 该文件面向库的维护者, 不是使用手册.
