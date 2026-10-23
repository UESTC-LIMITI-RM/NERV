# AGENTS.md - NERV: NERV's Embedded Robotic Vehicle

> 本文件定义 NERV 库的工程约束.
> NERV 是库, 不是固件工程. 使用者的实际工程负责 MCU 初始化, 时钟, 引脚, 启动文件, 链接脚本, IDE 工程, 板级配置, 应用任务, 测试与发布.

---

## 1. 定位与原则

NERV 是可复用的 C11 嵌入式机器人控制库. 它提供与 MCU 无关的控制模块, 状态码, 单位与数学工具, 以及一层很薄的板级支持接口.

NERV 不是完整固件, 也不包含任何实际工程文件. 以下内容属于使用者, 不属于本库:

- `.ioc`, `MDK-ARM`, `Core`, `Drivers`, 启动文件, 链接脚本与 IDE 配置.
- 应用任务, 任务优先级, 栈大小, 周期, 看门狗与急停硬件回路.
- 测试工程, HIL 测试, CI 配置与发布流程.

### 1.1 核心原则

按优先级排序:

1. **鲁棒**: 所有对外接口显式返回状态, 检查参数与边界, 不产生未定义行为, 不隐藏动态分配, 故障时进入安全状态.
2. **可读**: 名称表达意图, 函数短小, 控制流直接, 注释只解释设计原因与约束.
3. **可移植**: 公共头文件与模块层不包含任何 MCU 厂商头文件, 不依赖特定编译器扩展, 不依赖特定 RTOS.
4. **优雅**: API 数量少而完整, 实例化方式统一, 扩展点使用显式回调, 不依赖全局单例.
5. **简洁**: 只保留必要的抽象层, 同一件事只有一种推荐写法, 不为假设中的未来需求增加复杂度.

### 1.2 MCU 兼容性

- NERV 的通信基线是 FDCAN. 目标 MCU 只要提供 FDCAN, 就可以通过实现 `bsp/<platform>` 适配层使用本库.
- FDCAN 是唯一必需外设. GPIO, PWM, UART, 定时器等外设只在对应模块需要时通过 BSP 接口提供.
- 所有平台差异集中在 `config/Inc/config.h` 与 `bsp/<platform>` 中. 新增 MCU 不修改 `module/` 与 `public/` 的公开接口.
- 库核心不依赖 HAL. STM32 HAL 作为参考 BSP 实现, HAL 头文件只允许出现在 `bsp/stm32h7/` 及其平台门面头文件中.
- `module/` 与 `public/` 不包含 HAL 头文件, 不暴露 `HAL_` 类型.
- 只维护 STM32 BSP 也可以. 其他 MCU 通过新增 `bsp/<platform>` 接入, 不需要修改模块层.
- 库不绑定特定 RTOS, IDE 或构建系统.
- 库核心不创建任务, 不创建互斥量, 不调用延时函数. RTOS 是使用者的选择, 不是库的依赖.

### 1.3 扩展方式

- 模块扩展点统一使用注册回调函数指针, 由实例绑定.
- `module/` 与 `public/` 不使用 `__weak` 或编译器弱符号属性, 不使用全局钩子函数.
- BSP 为对接厂商 HAL 的 weak 回调, 可以在 `bsp/<platform>/` 内按厂商规则覆盖. 这不属于 NERV 模块扩展点.
- 回调的上下文, 生命周期与 ISR 安全性必须在注册接口的契约中写明.

---

## 2. 目录结构

NERV 采用"每个模块一个目录, 目录下再分 `Inc/` 与 `Src/`"的布局. 仓库只包含可复用源码, 配置, 文档与可选的仓库检查脚本.

```
NERV/
├── public/
│   ├── Inc/
│   │   ├── nerv_status.h
│   │   ├── nerv_types.h
│   │   ├── nerv_utils.h
│   │   ├── nerv_math.h
│   │   └── nerv_log.h
│   └── Src/
│       ├── nerv_math.c
│       ├── nerv_utils.c
│       └── nerv_log.c
├── config/
│   ├── Inc/
│   │   └── config.h
│   └── Src/
│       └── config_check.c
├── bsp/
│   ├── Inc/
│   │   ├── nerv_fdcan.h
│   │   ├── nerv_gpio.h
│   │   ├── nerv_pwm.h
│   │   └── nerv_time.h
│   └── <platform>/
│       ├── Inc/
│       └── Src/
├── module/
│   ├── motor/
│   │   ├── Inc/
│   │   └── Src/
│   ├── chassis/
│   │   ├── Inc/
│   │   └── Src/
│   ├── gimbal/
│   │   ├── Inc/
│   │   └── Src/
│   ├── shoot/
│   │   ├── Inc/
│   │   └── Src/
│   └── safety/
│       ├── Inc/
│       └── Src/
└── scripts/
    └── check_style.py
```

规则:

- `public/Inc/` 是唯一允许被任意层包含的基础类型层.
- `config/Inc/config.h` 是唯一配置入口, 承载平台选择, 模块裁剪与日志配置.
- `bsp/Inc/` 定义与 MCU 无关的外设接口; `bsp/<platform>/` 是实现, 可以包含 MCU 厂商头文件.
- `bsp/stm32h7/` 是 STM32 HAL 参考实现. 该目录的源文件与平台门面头可以包含 HAL; `bsp/Inc/` 保持与 MCU 无关.
- `module/<name>/` 只包含硬件无关逻辑, 不得包含 MCU 厂商头文件或 RTOS 头文件.
- `public/Inc/nerv_log.h` 与 `public/Src/nerv_log.c` 是可选日志基础设施, 通过回调输出, 不直接包含 UART 或 RTOS 头文件.
- 仓库不包含 `app/`, `tests/`, `MDK-ARM/`, `Core/`, `Drivers/`, `.ioc`, 启动文件或链接脚本.
- 应用层由使用者实现. 库不提供 `chassis_task`, `gimbal_task`, `shoot_task` 或 `app_init`.
- 新增 MCU 时只新增 `bsp/<platform>/` 并在 `config.h` 中选择, 不修改已有 `module/`.
- 库不提供构建系统. 使用者把自己的工程需要的 `public/`, `config/`, `bsp/`, `module/` 源文件加入工程.

include path 顺序:

1. `public/Inc`
2. `config/Inc`
3. `bsp/Inc`
4. `module/<name>/Inc`

`bsp/<platform>/Inc` 只能被该平台适配层自己的源文件包含, 不得进入 `module/` 的 include path. 禁止使用 `#include "../module/xxx/Inc/xxx.h"` 这类相对路径.

---

## 3. 分层与依赖

依赖方向只能向下:

```
module/       硬件无关功能单元
bsp/          外设接口与平台适配
public/       状态码, 类型, 数学, 单位与日志工具
config/       编译期平台选择与功能裁剪
```

### 3.1 public

- 除日志模块外, 只依赖 C 标准库与 `stdint.h`, `stddef.h`, `stdbool.h`.
- 日志模块的 `nerv_log.h` 可以包含 `config.h` 与 `nerv_status.h`.
- 不包含 HAL, 不包含 CMSIS, 不包含 RTOS 头文件.
- 提供 `NERV_StatusTypeDef`, 错误码, 数学函数, 单位转换与通用宏.
- 提供确实被多个模块使用的通用类型, 例如 `NERV_AttitudeTypeDef`. 只有单一模块使用的类型放在该模块的头文件中, 带模块前缀.
- 不得包含 `module/` 或 `bsp/` 的头文件.

### 3.2 config

- 只包含编译期宏, 不包含 HAL 或其他模块头文件.
- 承载平台选择, 模块裁剪, 日志级别与日志缓冲区大小.
- 不承载任务优先级, 栈大小, 周期, 看门狗超时或应用参数.
- `config_check.c` 使用 `_Static_assert` 检查宏组合.

### 3.3 bsp

- `bsp/Inc/` 只定义与 MCU 无关的接口类型, 函数指针与状态.
- `bsp/<platform>/` 是唯一允许包含 MCU 厂商头文件的层.
- `bsp` 不得包含 `module/` 或应用层头文件.
- 平台适配层通过函数指针结构体向 `module/` 暴露能力, 不把 HAL 句柄泄漏到公开接口.
- 反向通知使用注册回调, 不使用弱符号或全局钩子. HAL 的 weak 回调只用于 BSP 对接.

### 3.4 module

- 只包含 `public/`, `config/` 与 `bsp/Inc/` 的头文件.
- 不包含任何 MCU 厂商头文件, 不包含 `bsp/<platform>/` 的头文件, 不包含 RTOS 头文件.
- 通过 BSP 实例指针访问硬件, 不直接调用 HAL 或厂商库.
- 对外接口使用 SI 单位与 `NERV_StatusTypeDef`.
- 每个持有实例的模块提供带实例指针的 `GetError` 与 `ClearError`.

### 3.5 log

- `public/Inc/nerv_log.h` 与 `public/Src/nerv_log.c` 只依赖 `public/` 与 `config/`.
- 日志通过使用者的输出回调发送, 不直接操作 UART, 不创建任务, 不依赖 RTOS.
- 日志模块是全局服务, 允许一个静态输出回调; 其他模块不得使用全局单例.

### 3.6 应用层

应用层位于使用者工程中, 不属于本库. 库不规定任务划分, 不规定调度周期, 不规定看门狗策略. 应用层负责:

- MCU 时钟, 引脚, FDCAN 与中断初始化.
- 任务, 互斥量, 队列, 定时器与栈大小.
- 看门狗, 急停硬件回路与系统级故障策略.
- 测试, 发布与 IDE 工程维护.

---

## 4. 状态码与错误体系

### 4.1 基础状态码

所有对外函数默认返回 `NERV_StatusTypeDef`, 取值为四个基础状态, 不依赖 HAL 头文件.

```c
/* public/Inc/nerv_status.h */
#ifndef NERV_STATUS_H
#define NERV_STATUS_H

#include <stdint.h>

typedef enum
{
    NERV_OK      = 0x00U, /**< 操作成功 */
    NERV_ERROR   = 0x01U, /**< 通用错误 */
    NERV_BUSY    = 0x02U, /**< 资源忙, 稍后重试 */
    NERV_TIMEOUT = 0x03U, /**< 操作超时 */
} NERV_StatusTypeDef;

#endif /* NERV_STATUS_H */
```

### 4.2 扩展错误码

扩展错误码采用位掩码语义. 高 8 位为域号, 低 24 位为域内错误位. 域号使用 one-hot 编码, 多个域号可以按位或. 低 24 位按域分段, 每个子错误在整个低 24 位中占据唯一位置. 使用 `#define` 与固定宽度无符号整型, 不使用 `enum`, 因为调用者需要对错误位做按位或.

| 域 | 域号 | 低 24 位范围 | 域掩码 |
| --- | --- | --- | --- |
| 公共 | `NERV_ERR_BIT(0)` | bit 0..5 | `NERV_ERR_COMMON_BITS_MASK` |
| FDCAN | `NERV_ERR_BIT(1)` | bit 6..8 | `NERV_ERR_FDCAN_BITS_MASK` |
| 电机 | `NERV_ERR_BIT(2)` | bit 9..10 | `NERV_ERR_MOTOR_BITS_MASK` |
| 底盘 | `NERV_ERR_BIT(3)` | bit 11..12 | `NERV_ERR_CHASSIS_BITS_MASK` |
| 云台 | `NERV_ERR_BIT(4)` | bit 13..14 | `NERV_ERR_GIMBAL_BITS_MASK` |
| 发射 | `NERV_ERR_BIT(5)` | bit 15..16 | `NERV_ERR_SHOOT_BITS_MASK` |
| 安全 | `NERV_ERR_BIT(6)` | bit 17..18 | `NERV_ERR_SAFETY_BITS_MASK` |

bit 19..23 为预留位. 扩展现有域时优先从预留位分配, 把新位并入对应域的 `NERV_ERR_<DOMAIN>_BITS_MASK`, 不移动已有错误位. 预留位用尽时才重新分配并同步升级遥测协议版本.

错误码直接使用 `NERV_ERR_BIT(全局位号)`, 不引入域内偏移辅助宏. 域号同样直接使用 `NERV_ERR_BIT(域内位号)`, 显式写出置位的是哪一位, 位号即域编号字节 (高 8 位) 中的位置. 域掩码也用 `NERV_ERR_BIT` 显式组合, 使每个错误码的位号可以直接核对.

```c
#define NERV_ERR_NONE 0x00000000U
#define NERV_ERR_ALL  0xFFFFFFFFU
#define NERV_ERR_BITS_MASK 0x00FFFFFFU

#define NERV_ERR_DOMAIN(_domain) ((uint32_t)(_domain) << 24U)
#define NERV_ERR_BIT(_bit)       ((uint32_t)1U << (_bit))

/* 域号用 NERV_ERR_BIT 显式写出置位位号, 位号即域编号字节中的位置. */
#define NERV_ERR_DOMAIN_COMMON  NERV_ERR_BIT(0U)
#define NERV_ERR_DOMAIN_FDCAN   NERV_ERR_BIT(1U)
#define NERV_ERR_DOMAIN_MOTOR   NERV_ERR_BIT(2U)
#define NERV_ERR_DOMAIN_CHASSIS NERV_ERR_BIT(3U)
#define NERV_ERR_DOMAIN_GIMBAL  NERV_ERR_BIT(4U)
#define NERV_ERR_DOMAIN_SHOOT   NERV_ERR_BIT(5U)
#define NERV_ERR_DOMAIN_SAFETY  NERV_ERR_BIT(6U)

/* 域掩码用 NERV_ERR_BIT 显式组合, 例如: */
#define NERV_ERR_FDCAN_BITS_MASK \
    (NERV_ERR_BIT(6U) | NERV_ERR_BIT(7U) | NERV_ERR_BIT(8U))

#define NERV_ERR_COMMON_INVALID_PARAM \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(0U))
#define NERV_ERR_COMMON_NULL_PTR \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(1U))
#define NERV_ERR_COMMON_NOT_INITIALIZED \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(2U))
#define NERV_ERR_COMMON_NO_MEMORY \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(3U))
#define NERV_ERR_COMMON_OUT_OF_RANGE \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(4U))
#define NERV_ERR_COMMON_NOT_SUPPORTED \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_COMMON) | NERV_ERR_BIT(5U))

#define NERV_ERR_FDCAN_FILTER \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_FDCAN) | NERV_ERR_BIT(6U))
#define NERV_ERR_FDCAN_TX_MAILBOX \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_FDCAN) | NERV_ERR_BIT(7U))
#define NERV_ERR_FDCAN_RX_OVERRUN \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_FDCAN) | NERV_ERR_BIT(8U))

#define NERV_ERR_MOTOR_OFFLINE \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_MOTOR) | NERV_ERR_BIT(9U))
#define NERV_ERR_MOTOR_OVER_TEMP \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_MOTOR) | NERV_ERR_BIT(10U))

#define NERV_ERR_CHASSIS_KINEMATICS \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_CHASSIS) | NERV_ERR_BIT(11U))
#define NERV_ERR_CHASSIS_WHEEL_OFFLINE \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_CHASSIS) | NERV_ERR_BIT(12U))

#define NERV_ERR_GIMBAL_LIMIT \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_GIMBAL) | NERV_ERR_BIT(13U))
#define NERV_ERR_GIMBAL_OFFLINE \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_GIMBAL) | NERV_ERR_BIT(14U))

#define NERV_ERR_SHOOT_JAM \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SHOOT) | NERV_ERR_BIT(15U))
#define NERV_ERR_SHOOT_OFFLINE \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SHOOT) | NERV_ERR_BIT(16U))

#define NERV_ERR_SAFETY_ESTOP \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SAFETY) | NERV_ERR_BIT(17U))
#define NERV_ERR_SAFETY_LATCHED \
    (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SAFETY) | NERV_ERR_BIT(18U))
```

模块实例只保存 `code & NERV_ERR_BITS_MASK`. `GetError` 用各域的 `NERV_ERR_<DOMAIN>_BITS_MASK` 判断哪些域非零, 再用 `NERV_ERR_DOMAIN(domain_mask)` 重建高 8 位. `ClearError` 用 `mask & NERV_ERR_BITS_MASK` 只清除低 24 位, 避免清除共享的域号字节.

代码块只列名称与取值. `public/Inc/nerv_status.h` 中每个宏单独带一段文档块, 写明语义, 使任意宏都能被 `@ref` 引用. 参数的可选值列表按第 8.7 节书写.

### 4.3 规则

- 禁止自定义新的基础状态枚举. 需要新语义时使用扩展错误码.
- 持有实例的硬件模块必须提供:

```c
uint32_t NERV_<Module>_GetError(const NERV_<Module>_InstanceTypeDef *self);
void     NERV_<Module>_ClearError(NERV_<Module>_InstanceTypeDef *self, uint32_t mask);
```

- `GetError` 返回粘滞位掩码, `0U` 表示无错误. 读取本身不清除错误.
- `ClearError` 只清除 `mask` 中置位的错误. `NERV_ERR_ALL` 只用于清除, 不作为 `GetError` 的返回值.
- `ClearError` 不处理 `NERV_ERR_SAFETY_ESTOP`. 该位只由 `NERV_Safety_ClearEStop` 清除, 避免全清操作绕过急停保护.
- 错误位在中断中置位, 在任务中清除时必须使用临界区或原子位操作.
- 错误码集中定义在 `public/Inc/nerv_status.h`. 禁止在模块头文件中散落定义.
- 域号宏 `NERV_ERR_DOMAIN_<DOMAIN>` 必须 one-hot. 低 24 位的子错误位必须在全局范围内唯一, 不允许不同域复用同一低位.
- 错误码定义直接写出 `NERV_ERR_BIT(全局位号)`, 域号定义直接写出 `NERV_ERR_BIT(域内位号)`, 不引入 `NERV_ERR_<DOMAIN>_BIT_OFFSET` 或 `NERV_ERR_<DOMAIN>_BIT(_bit)` 这类隐藏偏移的辅助宏. 域掩码也使用 `NERV_ERR_BIT` 显式组合.
- 模块实例的错误寄存器只保存 `code & NERV_ERR_BITS_MASK`. 置位与清除按以下模式实现:

```c
self->error |= (code & NERV_ERR_BITS_MASK);
self->error &= ~(mask & NERV_ERR_BITS_MASK);
```

- `GetError` 根据低 24 位重建高 8 位域号:

```c
uint32_t NERV_Example_GetError(const NERV_Example_InstanceTypeDef *self)
{
    uint32_t bits;
    uint32_t domain;

    if (NULL == self)
    {
        return NERV_ERR_NONE;
    }

    bits = self->error & NERV_ERR_BITS_MASK;
    domain = 0U;

    if (0U != (bits & NERV_ERR_COMMON_BITS_MASK))
    {
        domain |= NERV_ERR_DOMAIN_COMMON;
    }

    if (0U != (bits & NERV_ERR_FDCAN_BITS_MASK))
    {
        domain |= NERV_ERR_DOMAIN_FDCAN;
    }

    /* 其他域同理. */

    return NERV_ERR_DOMAIN(domain) | bits;
}
```

- 扩展现有域时优先使用 bit 19..23, 把新位并入该域的 `NERV_ERR_<DOMAIN>_BITS_MASK`, 不移动已有错误位. 预留位用尽时才重新分配并同步升级遥测协议版本.
- 错误码名称统一为 `NERV_ERR_<DOMAIN>_<CONTENT>`, 域名与具体内容之间用下划线连接. 域号宏 `NERV_ERR_DOMAIN_<DOMAIN>` 是例外, 它表示域编号本身, 不是错误码.
- 全局服务模块, 例如日志模块, 通过返回值报告错误, 不提供实例错误寄存器.
- `config/Src/config_check.c` 使用 `_Static_assert` 检查域号 one-hot, 各域低位掩码与错误码位号.

```c
_Static_assert(NERV_ERR_COMMON_INVALID_PARAM != NERV_ERR_COMMON_NULL_PTR,
               "common error bits must be distinct.");
```

---

## 5. 单位与坐标系

### 5.1 SI 单位

所有模块内部计算与对外接口使用 SI 单位. 仅在 BSP 或电机协议边界做单位转换.

| 物理量 | 单位 | 变量后缀 | 示例 |
| --- | --- | --- | --- |
| 长度 | m | `_m` | `wheel_radius_m` |
| 角度 | rad | `_rad` | `yaw_rad` |
| 线速度 | m/s | `_mps` | `vx_mps` |
| 角速度 | rad/s | `_radps` | `wz_radps` |
| 加速度 | m/s² | `_mps2` | `ax_mps2` |
| 电流 | A | `_a` | `current_a` |
| 电压 | V | `_v` | `voltage_v` |
| 力矩 | N·m | `_n_m` | `torque_n_m` |
| 时间 | s 或 ms 或 us | `_s` 或 `_ms` 或 `_us` | `dt_s`, `timeout_ms` |
| 温度 | ℃ | `_c` | `temp_c` |
| 质量 | kg | `_kg` | `mass_kg` |

### 5.2 坐标系

- 右手系, `x` 轴指向机器人正前方, `y` 轴指向左方, `z` 轴指向上方, 与 REP-103 的机体坐标系约定一致.
- yaw 绕 `z` 轴, 逆时针为正, 从上向下俯视.
- pitch 绕 `y` 轴, 抬头为正.
- roll 绕 `x` 轴, 右侧下沉为正.
- 位置与速度分属不同参考系. 位置在参考系中累积, 速度在机体系中表达, 与 ROS `nav_msgs/Odometry` 的 `pose` 与 `twist` 处理一致. 换算到参考系需要航向角, 由调用方完成.
- 遥控器给出的底盘速度指令在云台系, 即操作手视角, 云台指向的方向为正前方. 它与底盘解算所用的机体系相差一个绕 z 轴的旋转, 角度为云台航向与底盘航向之差, 两个航向必须在同一参考系下测量.
- 底盘接口只接受机体系速度. 遥控器指令到机体系的转换由调用方完成, 因为"底盘相对云台, 相对车体还是相对场地"是机器人级策略, 库不替使用者选定. 绕 z 轴的角速度不随参考系改变, 转换时原样传递.
- 姿态用欧拉角表示, 不采用 REP-103 推荐的旋转矩阵或四元数. 嵌入式控制需要直接的角度值, 不做坐标变换.
- 姿态结构体的每个字段必须在定义处用 `/**< ... */` 注明轴向与正方向.

### 5.3 角度范围

- 角度统一位于 `[-π, π)`.
- 跨 ±π 的归一化必须调用 `NERV_Math_Wrap(angle, -NERV_MATH_PI, NERV_MATH_PI)`.
- 禁止在每个模块中重复实现角度归一化.
- `NERV_Math_Wrap` 位于 `public/`, 不依赖任何模块. 它把数值归一化到调用方指定的半开区间 `[min_value, max_value)`.

### 5.4 单位转换

```c
/* public/Inc/nerv_utils.h */
float NERV_Utils_RpmToRadps(float rpm);
float NERV_Utils_RadpsToRpm(float radps);
```

- 完整契约写在 `public/Src/nerv_utils.c` 的定义处, 包括参数单位, 返回值单位与调用约束.
- 单位转换函数属于 `utils` 模块, 使用 `NERV_Utils_` 前缀.
- 2π 使用 `NERV_MATH_TWO_PI`, 定义在 `public/Inc/nerv_math.h`, 取值为硬编码的 `6.28318530717958647692f`.
- 模块不得为了单位转换包含 `nerv_motor.h`.

---

## 6. 命名规范

所有对外符号必须携带 `NERV_` 前缀, 并按类别携带模块名, 公共域名或配置域前缀, 防止与 HAL, 中间件或其他库冲突.

| 类别 | 规则 | 示例 |
| --- | --- | --- |
| 公开函数 | `NERV_<Module>_<Verb><Noun>` | `NERV_Chassis_Init` |
| 公共工具函数 | `NERV_Math_`, `NERV_Utils_`, `NERV_Ptr_` | `NERV_Math_Wrap`, `NERV_Utils_RpmToRadps` |
| 公共工具宏 | `NERV_UTILS_<NAME>`, 仅限 `public/Inc/nerv_utils.h` | `NERV_UTILS_ARRAY_SIZE`, `NERV_UTILS_TIMEOUT_MS` |
| 公共数学常量 | `NERV_MATH_<NAME>`, 仅限 `public/Inc/nerv_math.h` | `NERV_MATH_PI`, `NERV_MATH_TWO_PI` |
| 初始化与注册 | `NERV_<Module>_Register`, 首参数为实例指针 | `NERV_Chassis_Register` |
| 实例类型 | `NERV_<Module>_InstanceTypeDef` | `NERV_Chassis_InstanceTypeDef` |
| 配置类型 | `NERV_<Module>_ConfigTypeDef` | `NERV_Chassis_ConfigTypeDef` |
| 枚举类型 | `NERV_<Module>_<Name>TypeDef` | `NERV_Chassis_KindTypeDef` |
| 枚举成员 | `NERV_<MODULE>_<NAME>`, 必须显式赋值 | `NERV_CHASSIS_TYPE_MECANUM = 1U` |
| 回调类型 | `NERV_<Module>_<Event>CallbackTypeDef` | `NERV_Chassis_FaultCallbackTypeDef` |
| 位掩码与宏常量 | `NERV_<MODULE>_<NAME>` | `NERV_ERR_COMMON_INVALID_PARAM` |
| 用户可覆盖的默认值宏 | `NERV_<MODULE>_<NAME>`, 用 `#ifndef` 包裹 | `NERV_MOTOR_MAX_CURRENT_MA` |
| 用户禁止覆盖的推导宏 | `NERV_<MODULE>_<NAME>`, 用 `#if defined(...)` 与 `#error` 保护 | `NERV_CHASSIS_WHEEL_COUNT` |
| 内部实现宏 | `__NERV_<MODULE>_<NAME>`, 仅限 `*_priv.h` 或 `.c` | `__NERV_CHASSIS_INTERNAL_MAX_WHEEL_COUNT` |
| 内部函数 | `static` 修饰, 仍使用 `NERV_` 前缀 | `static void NERV_Chassis_UpdateOdometry(...)` |
| 内部变量 | 小写下划线, 带单位后缀 | `chassis_cfg`, `speed_radps`, `dt_s` |
| 函数指针成员 | 动词开头, 语义简短 | `init`, `update`, `set_ref` |

规则:

- 公共工具宏与公共数学常量是命名规则的例外. 公共工具宏仅允许定义在 `public/Inc/nerv_utils.h`, 公共数学常量仅允许定义在 `public/Inc/nerv_math.h`.
- 禁止无前缀的全局符号.
- 禁止 `err1`, `tmp`, `data` 这类无法表达含义的命名.
- 禁止单字母变量. 不超过 10 行的循环可以使用 `i`.
- 除 `log` 这类全局服务外, 模块不得使用全局单例.
- 应用层命名由使用者负责, 不属于本库约束.

---

## 7. 代码风格

### 7.1 缩进与大括号

- 使用 4 个空格缩进, 禁止 Tab.
- 大括号独占一行, 即使代码块只有一行.
- `if`, `for`, `while` 与括号之间保留一个空格.

```c
if (NULL == chassis)
{
    return NERV_ERROR;
}

for (uint8_t i = 0U; i < motor_num; i++)
{
    /* ... */
}
```

仓库根目录的 `.clang-format` 是格式化的唯一基准. 源文件与头文件必须与该配置的输出一致, IDE 与命令行使用同一份配置. 规范中的代码样例是示意片段, 只展示结构与语义, 不要求与配置输出逐字一致.

### 7.2 字面量后缀与常量在左

- 无符号字面量加 `U` 后缀, 例如 `0U`, `1U`, `0xFFU`.
- 浮点字面量加 `f` 后缀, 例如 `0.0f`, `1.0f`.
- 长整型字面量加 `UL` 后缀, 例如 `100000UL`.
- 判断变量是否等于常量时, 常量写在 `==` 与 `!=` 左侧.

```c
/* 反例 */
if (status = NERV_OK)          { /* 赋值, 不是比较 */ }
if (hchassis == NULL)          { /* 若误写为赋值, 编译器可能不报错 */ }

/* 正例 */
if (NERV_OK == status)          { /* ... */ }
if (NULL == hchassis)          { /* ... */ }
```

本规则不适用于:

- 不等式比较 `<`, `>`, `<=`, `>=`.
- 布尔变量判断, 例如 `if (is_ready)`.
- 浮点数相等比较. 浮点比较必须使用容差函数, 例如 `NERV_Math_IsClose`.
- 位掩码测试, 例如 `if (0U != (fault & NERV_ERR_CHASSIS_KINEMATICS))`.
- `switch` 的 `case` 标签.

### 7.3 函数长度与参数

- 单个函数原则上不超过 80 行, 不含注释与空行.
- 参数不超过 6 个. 超出时封装为配置结构体并传指针.
- 对外函数不得使用可变参数或隐式类型转换.
- 日志宏与日志格式化函数是可变参数的唯一例外.

### 7.4 Magic Number

禁止裸数字字面量. 以下例外允许直接使用:

- `0U`, `1U`, `0`, `1`, `-1`, `NULL`, `true`, `false`, `'\0'`, `0.0f`, `1.0f`.
- 命名常量, 宏, `static const` 与枚举成员的初始化值.
- `0` 与 `1` 在 signed 表达式或 `snprintf` 返回值判断中的使用.

规则:

- 无符号上下文必须使用 `0U` 与 `1U`.
- 循环边界, 数组大小, 缓冲区长度, 协议常量, 寄存器值, 超时时间必须使用命名常量.
- 硬件寄存器值放在 `bsp/<platform>/Inc/*_reg.h`.
- 协议常量放在 `module/<name>/Inc/*_def.h`.
- 可调参数放在 `config/Inc/config.h` 或模块的默认参数头文件.
- 超时使用 `NERV_UTILS_TIMEOUT_MS(x)` 或 `NERV_UTILS_TIMEOUT_S_TO_MS(x)`, 禁止裸数字. 宏名必须写明输入单位与输出单位, 避免"名字是秒, 结果是毫秒"这类歧义.
- 数组长度使用 `NERV_UTILS_ARRAY_SIZE(array)`.

```c
/* 反例 */
NERV_FDCAN_Transmit(hfdcan, data, 8U, 5U);

/* 正例 */
NERV_FDCAN_Transmit(hfdcan, data, frame_len,
                    NERV_UTILS_TIMEOUT_MS(NERV_FDCAN_TX_TIMEOUT_MS));
```

### 7.5 宏与枚举的选择

判定顺序:

1. 纯常量计算使用宏, 例如超时换算, 数组长度, 位掩码.
2. 编译期条件分支使用 `#if`, 例如平台选择与模块裁剪.
3. 零开销薄封装优先使用宏. 涉及返回值, 资源管理或多步逻辑时使用函数.
4. 必须参与 `#if` 的取值集合使用 `#define`.
5. 日志级别, 断言开关与调试开关使用预处理宏.
6. 类型化有限取值集合使用 `typedef enum`.

不应使用宏的场景:

- 需要返回值且调用者依赖返回值.
- 需要局部变量或静态存储.
- 参数可能带副作用.
- 需要类型安全, 调试器可视化或 `switch` 完整性检查.
- 需要作为函数参数类型或结构体成员类型.

```c
/* 反例 */
#define NERV_MOTOR_CMD_ENABLE   0x01U
#define NERV_MOTOR_CMD_DISABLE  0x02U

/* 正例 */
typedef enum
{
    NERV_MOTOR_CMD_ENABLE  = 0x01U,  /**< 使能电机 */
    NERV_MOTOR_CMD_DISABLE = 0x02U,  /**< 失能电机 */
} NERV_Motor_CmdTypeDef;
```

### 7.6 宏与枚举镜像

同一个有限取值集合需要同时参与预处理与运行时, 采用"宏定义值, 枚举镜像值"的模式.

- 预处理期使用 `#define`, 因为 `#if` 无法引用 `enum` 成员.
- 运行时使用 `typedef enum`, 获得类型安全与调试器可视化.
- 在宏与枚举镜像模式下, 枚举成员必须带 `_RT` 后缀, 避免与宏同名.
- 两者数值必须一致, 并在 `.c` 中使用 `_Static_assert` 校验.
- 枚举成员必须显式赋值, 禁止依赖隐式递增.
- 禁止在 `#if` 中引用枚举成员或使用强制类型转换.

```c
/* config/Inc/config.h */
#define NERV_CHASSIS_TYPE_MECANUM   1U
#define NERV_CHASSIS_TYPE_OMNI      2U
#define NERV_CHASSIS_TYPE_STEERING  3U

/* module/chassis/Inc/nerv_chassis.h */
typedef enum
{
    NERV_CHASSIS_TYPE_MECANUM_RT  = 1U,  /**< 麦克纳姆轮 */
    NERV_CHASSIS_TYPE_OMNI_RT     = 2U,  /**< 全向轮 */
    NERV_CHASSIS_TYPE_STEERING_RT = 3U,  /**< 舵轮 */
} NERV_Chassis_KindTypeDef;

/* module/chassis/Src/nerv_chassis.c */
_Static_assert((int)NERV_CHASSIS_TYPE_MECANUM == (int)NERV_CHASSIS_TYPE_MECANUM_RT,
               "NERV_CHASSIS_TYPE_MECANUM and NERV_CHASSIS_TYPE_MECANUM_RT must be equal.");
_Static_assert((int)NERV_CHASSIS_TYPE_OMNI == (int)NERV_CHASSIS_TYPE_OMNI_RT,
               "NERV_CHASSIS_TYPE_OMNI and NERV_CHASSIS_TYPE_OMNI_RT must be equal.");
_Static_assert((int)NERV_CHASSIS_TYPE_STEERING == (int)NERV_CHASSIS_TYPE_STEERING_RT,
               "NERV_CHASSIS_TYPE_STEERING and NERV_CHASSIS_TYPE_STEERING_RT must be equal.");
```

编译期选择平台或构型时使用宏:

```c
#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)
    hchassis->solve = NERV_Mecanum_Solve;
#elif (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)
    hchassis->solve = NERV_Omni_Solve;
#else
#error "Unsupported NERV_CHASSIS_TYPE."
#endif
```

只有在运行期切换构型的工具中才使用函数指针分派. 固件控制路径优先使用编译期选择.

### 7.7 宏定义函数

#### 7.7.1 多语句宏使用 do-while(0)

展开体包含多条语句时, 必须使用 `do { ... } while (0)` 包裹.

```c
/* 反例 */
#define NERV_MOTOR_SET_SPEED(_motor, _speed) \
    (_motor).speed_radps = (_speed);         \
    (_motor).updated = 1U

/* 正例 */
#define NERV_MOTOR_SET_SPEED(_motor, _speed) \
    do {                                     \
        (_motor).speed_radps = (_speed);     \
        (_motor).updated = 1U;               \
    } while (0)
```

#### 7.7.2 单表达式宏加外层括号

展开体是单一表达式时, 使用外层括号包裹整个展开体.

```c
/* 反例 */
#define NERV_CLAMP_POSITIVE(_x) (_x) > 0.0f ? (_x) : 0.0f

/* 正例 */
#define NERV_CLAMP_POSITIVE(_x) (((_x) > 0.0f) ? (_x) : 0.0f)
```

#### 7.7.3 形参加括号

宏形参在展开体内每次出现都必须加括号, 防止调用方传入表达式时优先级错误.

```c
/* 反例 */
#define NERV_MAX(_a, _b) ((_a) > (_b) ? (_a) : (_b))

/* 正例 */
#define NERV_MAX(_a, _b) (((_a) > (_b)) ? (_a) : (_b))
```

字符串字面量拼接, 字符串化与记号粘贴是例外. 这些场景中的形参不得加括号, 否则会破坏拼接, 字符串化或记号粘贴. 此类形参必须在 Doxygen 中用 `@note` 约束合法实参.

#### 7.7.4 形参不得多次求值

形参在展开体内出现多次时, 调用方传入带副作用的实参会产生多次副作用. 涉及多次求值时改用 `static inline` 函数.

```c
/* 反例 */
#define NERV_MAX(_a, _b) (((_a) > (_b)) ? (_a) : (_b))
int32_t result = NERV_MAX(i++, j++);

/* 正例 */
static inline int32_t NERV_Math_MaxI32(int32_t a, int32_t b)
{
    return (a > b) ? a : b;
}
```

如果因类型泛化必须使用宏, 必须在 Doxygen 中用 `@warning` 禁止传入带副作用的实参.

#### 7.7.5 宏内局部变量加前缀

宏展开后的局部变量进入调用方作用域, 必须使用 `_nerv_<module>_<name>` 或 `_nerv_<name>` 命名.

```c
#define NERV_MOTOR_SET_SPEED(_motor, _speed)          \
    do {                                              \
        NERV_Motor_InstanceTypeDef *_nerv_motor_self; \
        _nerv_motor_self = &(_motor);                 \
        _nerv_motor_self->speed_radps = (_speed);     \
        _nerv_motor_self->updated = 1U;               \
    } while (0)
```

#### 7.7.6 宏不得改变调用方控制流

宏体内禁止 `return`, `goto` 与 `continue`. 需要提前返回时改用 `static inline` 函数.

```c
/* 反例 */
#define NERV_CHECK_NULL(_p) \
    do {                    \
        if (NULL == (_p))   \
        {                   \
            return NERV_ERROR; \
        }                   \
    } while (0)

/* 正例 */
static inline NERV_StatusTypeDef NERV_Ptr_CheckNotNull(const void *p)
{
    return (NULL == p) ? NERV_ERROR : NERV_OK;
}
```

#### 7.7.7 范例

第 11.4 节的 `NERV_LOG_FORMAT_AND_WRITE` 是满足规则的范例:

- 使用 `do { ... } while (0)`.
- 不在宏内声明缓冲区, 格式化由 `NERV_Log_Printf` 完成.
- `_level`, `_file`, `_line` 与 `__VA_ARGS__` 只求值一次.
- 宏体内没有 `return`, `goto` 或 `continue`.

`public/Inc/nerv_utils.h` 中的 `NERV_UTILS_ARRAY_SIZE`, `NERV_UTILS_TIMEOUT_MS`, `NERV_UTILS_TIMEOUT_S_TO_MS` 是单表达式宏:

- 整个展开体加外层括号.
- 形参每次出现都加括号.
- 不需要 `do { ... } while (0)`.
- `NERV_UTILS_ARRAY_SIZE` 的形参可能出现多次, 必须用 `@warning` 禁止副作用实参.

### 7.8 宏保护与命名空间

#### 7.8.1 保护范围

以下宏属于关键宏, 必须保护:

- 影响硬件行为或诊断语义: 引脚, 时序, 协议常量, 寄存器位, 错误码.
- 影响安全性: 电流, 电压, 温度阈值, 安全状态参数.
- 影响可观测性: 日志开关, 日志级别, 断言开关.
- 被多处引用的配置: 平台选择, 模块裁剪, 构型选择.
- 与编译器, 标准库或 RTOS 交互的宏.
- 内部实现细节.

#### 7.8.2 三层保护

1. 命名空间隔离: 公开宏, 用户可覆盖宏与内部宏使用不同前缀.
2. 防重定义包裹: `#ifndef` 或 `#if defined(...)` 加 `#error`.
3. 编译期检查: 非法取值使用 `#error` 拒绝.

#### 7.8.3 公开宏与内部宏

- 公开宏使用 `NERV_<MODULE>_<NAME>`.
- 内部宏使用 `__NERV_<MODULE>_<NAME>`, 仅允许出现在 `*_priv.h` 或 `.c` 中.
- 公开头文件不得暴露 `__NERV_` 宏.

```c
/* module/motor/Inc/nerv_motor_priv.h */
#define __NERV_MOTOR_INTERNAL_MAX_CURRENT_MA  60000U
```

#### 7.8.4 用户可覆盖的默认值宏

使用 `#ifndef` 与 `#endif` 包裹, 并在后面追加范围检查.

```c
#ifndef NERV_MOTOR_MAX_CURRENT_MA
#define NERV_MOTOR_MAX_CURRENT_MA  30000U
#endif

#if (NERV_MOTOR_MAX_CURRENT_MA < 1000U) || (NERV_MOTOR_MAX_CURRENT_MA > 60000U)
#error "NERV_MOTOR_MAX_CURRENT_MA must be in range 1000 to 60000."
#endif
```

#### 7.8.5 用户禁止覆盖的推导宏

使用 `#if defined(...)` 加 `#error` 保护.

```c
#if defined(NERV_CHASSIS_WHEEL_COUNT)
#error "NERV_CHASSIS_WHEEL_COUNT is derived and must not be defined by the user."
#endif

#if (NERV_CHASSIS_TYPE_MECANUM == NERV_CHASSIS_TYPE)
#define NERV_CHASSIS_WHEEL_COUNT  4U
#elif (NERV_CHASSIS_TYPE_OMNI == NERV_CHASSIS_TYPE)
#define NERV_CHASSIS_WHEEL_COUNT  3U
#else
#error "Unsupported NERV_CHASSIS_TYPE."
#endif
```

#### 7.8.6 关键宏注释包裹

关键宏定义区使用注释标明禁止修改.

```c
/* ========================================================================== */
/* 以下定义为 NERV 核心契约, 禁止修改.                                      */
/* 作用域: NERV_StatusTypeDef 与全部 NERV_ERR_ 开头的宏, 直至文件末尾.      */
/* 修改必须同步更新 AGENTS.md 与遥测协议定义.                               */
/* ========================================================================== */

#define __NERV_CHASSIS_INTERNAL_MAX_WHEEL_COUNT 4U

/* ========================================================================== */
/* 核心契约定义区结束                                                       */
/* ========================================================================== */
```

规则:

- 起始横幅必须写明作用域, 不能只写"以下宏". 作用域用符号名描述, 不写行号, 行号会随改动漂移.
- 定义区必须有结束横幅闭合, 位置在最后一条核心定义之后.
- 作用域覆盖非宏定义时, 横幅措辞用"定义"而不是"宏".
- 起始横幅紧跟文件头与包含语句, 保证读者先看到边界再看内容.

### 7.9 扩展点

NERV 模块只使用注册回调作为扩展机制.

- `module/` 与 `public/` 不使用 `__weak`.
- BSP 为对接 HAL 的 weak 回调, 可以在 `bsp/<platform>/` 内按厂商规则覆盖.
- 不使用编译器相关的弱符号属性作为模块扩展点.
- 不使用全局钩子函数.
- 回调必须与实例绑定, 通过 `NERV_<Module>_Register<Event>Callback` 注册.
- 注册函数返回 `NERV_StatusTypeDef`.
- 回调类型使用 `NERV_<Module>_<Event>CallbackTypeDef`.
- 回调的第一个参数是 `void *user`, 由注册者提供.
- 回调的上下文, 生命周期与 ISR 安全性必须写在注册函数的契约中.

```c
typedef void (*NERV_Chassis_FaultCallbackTypeDef)(void *user, uint32_t fault_mask);

NERV_StatusTypeDef NERV_Chassis_RegisterFaultCallback(
    NERV_Chassis_InstanceTypeDef *self,
    NERV_Chassis_FaultCallbackTypeDef callback,
    void *user);
```

---

## 8. 注释规范

### 8.1 注释内容

注释只回答两类问题:

1. 设计原因, 约束, 硬件限制, 协议特殊要求.
2. 调用者需要知道的信息: 合法取值, 前置条件, 副作用, 上下文安全性.

禁止用注释重复代码正在做什么. 如果代码必须靠注释才能看懂, 优先重构代码.

### 8.2 语言与标点

- 注释必须科学, 准确, 详细.
- 禁止用 `+` 表示并列关系, 使用"与"或逗号.
- 括号只承载技术符号, 例如函数名, 宏名, 数学表达式, 代码片段. 解释性内容写入主句.
- 禁止模糊表述, 例如"可能", "大概", "一般情况下".
- 物理量必须写单位, 例如 `@param rpm 转速, 单位 rpm`.
- 前置条件必须可验证, 例如 `@note 必须在 FDCAN 启动之后调用`.
- 禁止 emoji 与颜文字.
- 标点使用半角 ASCII.

### 8.3 Doxygen 位置

- 函数完整契约写在 `.c` 的定义处, 包含 `@brief`, `@param`, `@return` 或 `@retval`, `@note`, `@warning`.
- `@param` 的变量名与描述之间使用空格分隔, 不使用冒号.
- 一个函数只能有一个 Doxygen 块. 函数在 `.c` 中定义时, 唯一 Doxygen 块写在定义处, `.h` 中的声明不写 Doxygen 注释.
- 函数在 `.h` 中以 `static inline` 定义时, 唯一 Doxygen 块写在 `.h` 的定义处.
- `.h` 中的宏, 枚举, 结构体与 `typedef` 没有 `.c` 定义, 其契约写在 `.h` 的定义处.
- 结构体成员使用 `/**< ... */` 行尾注释.
- 回调参数必须说明上下文, 生命周期与 ISR 安全性.
- Doxygen 的 `INPUT` 必须同时包含 `*.h` 与 `*.c`.

```c
/* module/chassis/Src/nerv_chassis.c */

/**
  * @brief  初始化底盘模块.
  * @param  hchassis 底盘实例指针, 不得为 NULL.
  * @param  p_config 配置指针, 生命周期必须覆盖实例生命周期.
  * @retval NERV_OK 初始化成功.
  * @retval NERV_ERROR 参数非法或底层初始化失败.
  * @note   必须在 FDCAN 初始化与启动之后调用.
  * @note   NOT ISR-safe.
  */
NERV_StatusTypeDef NERV_Chassis_Init(
    NERV_Chassis_InstanceTypeDef *hchassis,
    const NERV_Chassis_ConfigTypeDef *p_config);
```

对应头文件:

```c
NERV_StatusTypeDef NERV_Chassis_Init(
    NERV_Chassis_InstanceTypeDef *hchassis,
    const NERV_Chassis_ConfigTypeDef *p_config);
```

### 8.4 返回值标签

`@return` 与 `@retval` 都描述返回值. 选择依据是返回值能否逐值枚举, 不是返回类型是否为宏.

#### 8.4.1 @return

`@return` 用于返回值无法逐值枚举的情况:

- 浮点数, 整数, 位掩码等数值结果.
- 指针, 结构体, 句柄.
- 返回值的范围, 单位或计算公式需要整体说明.

```c
/**
  * @brief  把数值归一化到指定区间.
  * @param  value 待归一化的数值.
  * @param  min_value 区间下界, 包含.
  * @param  max_value 区间上界, 不包含.
  * @return 与 value 相差整数个区间长度且落在 [min_value, max_value) 的值.
  * @note   边界非法时返回 value, 不做归一化.
  * @note   ISR-safe.
  */
float NERV_Math_Wrap(float value, float min_value, float max_value);
```

规则:

- 每个函数只写一条 `@return`.
- 数值返回值写明单位, 范围与非法输入时的结果.
- 指针返回值写明指向对象的生命周期与所有权, 失败返回 `NULL` 时写明.
- 位掩码返回值写明位布局, 不需要逐位写 `@retval`.
- 不得用 `@return` 罗列多个命名状态. 命名状态使用 `@retval`.

#### 8.4.2 @retval

`@retval` 用于返回值是有限个命名取值之一的情况:

- `NERV_StatusTypeDef` 与其他状态枚举.
- 普通枚举类型.
- 展开为离散命名取值的宏.

```c
/**
  * @brief  判断两个浮点数是否在容差内相等.
  * @param  a 第一个值.
  * @param  b 第二个值.
  * @param  tolerance 绝对容差, 单位与被比较值相同, 必须为大于等于 0 的有限值.
  * @param  is_close 比较结果输出, 不得为 NULL. 成功时写入, 失败时不修改.
  * @retval NERV_OK 比较完成, 结果已写入 is_close.
  * @retval NERV_ERROR is_close 为 NULL, 或 tolerance 小于 0 或非有限.
  * @note   ISR-safe.
  */
NERV_StatusTypeDef NERV_Math_IsClose(float a, float b, float tolerance, bool *is_close);
```

规则:

- 函数实际可能返回的每个命名取值各写一条 `@retval`. 不可能返回的值不写.
- 每条格式为 `@retval <value> <description>`. `<value>` 与描述之间保留一个空格.
- 不使用冒号分隔取值与描述. 冒号会被 Doxygen 当作普通字符处理, 可能进入取值或描述.
- `<value>` 写宏名或枚举成员名, 不写表达式, 强制类型转换或数值字面量.
- 描述写明该返回值对应的条件, 副作用与输出参数状态.
- 返回状态码并通过输出参数返回结果时, `@retval` 只描述状态码, 结果写在 `@param[out]`.
- 不得用 `@retval` 描述连续区间, 指针或结构体.

#### 8.4.3 选择规则

| 返回值 | 标签 |
| --- | --- |
| 有限个命名取值, 例如 `NERV_StatusTypeDef` 或枚举 | `@retval`, 每个取值一条 |
| 数值, 范围, 位掩码 | `@return` |
| 指针, 结构体, 句柄 | `@return` |
| 状态码与输出参数 | `@retval` 描述状态, `@param[out]` 描述结果 |

Doxygen 生成文档时, `@retval` 条目渲染为返回值表格, `@return` 渲染为一段说明. 同一函数不同时使用两者.

#### 8.4.4 其他常用标签

| 标签 | 用途 |
| --- | --- |
| `@brief` | 一句话描述 |
| `@param[in]`, `@param[out]`, `@param[in,out]` | 参数方向 |
| `@note` | 上下文, 合法取值, 线程安全性 |
| `@warning` | 未定义行为或硬件风险 |
| `@see` | 关联函数, 类型或宏 |

### 8.5 行内注释

行内注释只允许用于:

- 硬件勘误或编译器特殊语法.
- `goto` 清理边界.
- Magic Number 的物理来源.
- `do { ... } while (0)` 的结构意图.
- 关键宏定义区的禁止修改边界.

行内注释使用 `/* ... */`, 不使用 `//`.

### 8.6 文件头

每个源文件与头文件必须带文件头. 文件头位于文件第一处, 在 include guard 之前.

C 文件模板:

```c
/**
  * @file    nerv_status.h
  * @brief   NERV 状态码与扩展错误码.
  * @version 1.0.0
  * @date    2026-10-21
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */
```

Python 脚本模板:

```python
#!/usr/bin/env python3
# Copyright (c) 2026 NERV Contributors
# SPDX-License-Identifier: MIT
"""脚本职责说明."""
```

规则:

- `@file` 与文件名一致, 不写目录.
- `@brief` 一句话说明文件职责, 不重复文件内容.
- `@version` 使用语义化版本. 新建文件为 `1.0.0`, 接口或行为变化时递增.
- `@date` 使用 `YYYY-MM-DD`, 取文件首次创建日期. 后续修改不更新该日期.
- `@copyright` 与仓库根目录 `LICENSE` 一致, 统一为 `Copyright (c) 2026 NERV Contributors`. 不写个人姓名与个人邮箱.
- 许可证标识用独立一行的 `SPDX-License-Identifier: MIT` 表示, 不写 Doxygen 标签.
- 禁止使用 `@license`. Doxygen 没有内置的 `\license` 命令, 未知命令按普通文本处理并产生告警.
- 文件头只写 Doxygen 标签与版权信息. 不写 TODO, 不写临时说明, 不写生成标记.
- 脚本的 shebang 必须在第一行, 版权行紧随其后.

### 8.7 参数可选值列表

参数有有限个可选值时, 在 `@param` 中用 `@arg` 逐条列出, 值的引用用 `@ref`. 与 ST HAL 的写法一致.

```c
/**
  * @brief  由域号生成错误码的域号段.
  * @param  _domain 域号, 可取以下值.
  *                @arg @ref NERV_ERR_DOMAIN_COMMON 公共域.
  *                @arg @ref NERV_ERR_DOMAIN_FDCAN FDCAN 域.
  * @return 域号位于高 8 位, 低 24 位为 0 的掩码.
  */
#define NERV_ERR_DOMAIN(_domain) ((uint32_t)(_domain) << 24U)
```

规则:

- 只用 `@ref` 引用已登记的符号. 不用 `@group`, `@group` 用于组织文档分组, 不能引用取值.
- 被 `@ref` 引用的宏必须自己带文档块. 多个宏共用一个文档块时, Doxygen 只登记块后紧邻的那一个, 其余引用会报 `unable to resolve reference`.
- 列表项的说明写清语义, 不重复宏名.
- 取值是枚举类型时, 直接引用类型, 例如 `This parameter can be a value of @ref SPI_Mode`.
- 掩码类宏的 `@brief` 写明产出的是掩码, `@return` 描述结果. 不写"生成错误位"这类会被理解成产生具体错误的措辞.

---

## 9. 面向对象与实例

### 9.1 实例封装

数据与操作放在同一结构体中. 对外函数把实例指针作为第一个参数, 模拟 `this`.

```c
typedef struct NERV_Chassis_Instance
{
    NERV_Chassis_ConfigTypeDef config;  /**< 配置, 由 Register 绑定 */

    NERV_StatusTypeDef (*init)(struct NERV_Chassis_Instance *self);
    NERV_StatusTypeDef (*update)(
        struct NERV_Chassis_Instance *self,
        const NERV_Chassis_VelocityTypeDef *target);
    void (*get_odometry)(
        const struct NERV_Chassis_Instance *self,
        NERV_Chassis_OdometryTypeDef *odom);

    void *platform_data;  /**< 指向调用者提供的静态平台数据 */
} NERV_Chassis_InstanceTypeDef;
```

规则:

- 实例由调用者以静态存储提供.
- 禁止模块内部依赖全局单例.
- 禁止在实例中隐藏 `void *` 形式的 HAL 句柄. BSP 句柄放在 `platform_data` 指向的平台数据结构中.
- 库不创建互斥量. 线程安全由调用者通过外部同步保证.

### 9.2 继承

派生结构体的第一个成员必须是基类结构体.

```c
typedef struct
{
    NERV_Chassis_InstanceTypeDef base;  /**< 必须是第一个成员 */
    NERV_Mecanum_PrivDataTypeDef mecanum;
} NERV_Mecanum_InstanceTypeDef;
```

### 9.3 多态

- 固件控制路径优先使用 `config.h` 在编译期选择实现.
- 运行期切换构型时使用函数指针分派.
- 函数指针表必须与枚举值一一对应.

### 9.4 注册与初始化

```c
NERV_StatusTypeDef NERV_Chassis_Register(
    NERV_Chassis_InstanceTypeDef *self,
    const NERV_Chassis_ConfigTypeDef *p_config);
```

- `Register` 只绑定配置, 函数指针与软件状态, 不访问硬件, 不分配内存.
- `Init` 访问硬件, 由调用者在 MCU 与 FDCAN 初始化完成后调用.
- 两者职责不得合并.
- 库不规定 `Register` 与 `Init` 相对于 RTOS 启动的顺序.

### 9.5 扩展点选择

| 机制 | 适用场景 | 使用方式 |
| --- | --- | --- |
| 注册回调 | 运行期可变的, 多实例的, 与实例绑定的通知 | `NERV_<Module>_Register<Event>Callback` |
| 编译期函数指针 | 固件构建时确定的实现选择 | `config.h` 宏与 `#if` |
| 配置结构体 | 静态参数 | `NERV_<Module>_ConfigTypeDef` |

模块扩展点不使用弱符号. BSP 对接 HAL 的 weak 回调除外. 同一扩展点只选择一种机制.

---

## 10. 配置

### 10.1 唯一配置入口

`config/Inc/config.h` 是本库唯一的宏配置入口. 它承载:

- 平台选择.
- 模块裁剪.
- 日志开关与日志级别.
- 日志缓冲区大小.
- 协议与安全阈值中允许用户覆盖的默认值.

`NERV_USE_<MODULE>=0` 时, `config.h` 不定义该模块的专属配置宏. 该模块的源文件不参与编译.

`config.h` 不得包含 HAL, CMSIS, RTOS 或任何模块头文件. 不得在模块源文件中出现 `#ifdef STM32H7` 或 `#ifdef BOARD_X` 这类板级判断.

平台门面头文件 `bsp/<platform>/Inc/nerv_<platform>.h` 负责包含厂商 HAL, 只由 BSP 与应用层包含. `module/` 与 `public/` 只包含 `bsp/Inc/` 中的抽象接口.

### 10.2 平台选择

```c
/* config/Inc/config.h */
#ifndef NERV_CONFIG_H
#define NERV_CONFIG_H

#define NERV_PLATFORM_NONE      0U
#define NERV_PLATFORM_STM32H7   1U  /* STM32 HAL 参考 BSP, 不是默认值 */
#define NERV_PLATFORM_CUSTOM    2U

#ifndef NERV_PLATFORM
#define NERV_PLATFORM           NERV_PLATFORM_NONE
#endif

#if (NERV_PLATFORM == NERV_PLATFORM_NONE)
#error "NERV_PLATFORM must be selected in config.h."
#endif

#if (NERV_PLATFORM != NERV_PLATFORM_STM32H7) && \
    (NERV_PLATFORM != NERV_PLATFORM_CUSTOM)
#error "NERV_PLATFORM is not supported."
#endif

#ifndef NERV_USE_LOG
#define NERV_USE_LOG            1
#endif

#if (NERV_USE_LOG != 0) && (NERV_USE_LOG != 1)
#error "NERV_USE_LOG must be 0 or 1."
#endif

#ifndef NERV_USE_CHASSIS
#define NERV_USE_CHASSIS        1
#endif

#ifndef NERV_USE_GIMBAL
#define NERV_USE_GIMBAL         1
#endif

#ifndef NERV_USE_SHOOT
#define NERV_USE_SHOOT          1
#endif

#ifndef NERV_USE_SAFETY
#define NERV_USE_SAFETY         1
#endif

#endif /* NERV_CONFIG_H */
```

- `NERV_PLATFORM_NONE` 固定为 `0U`, 只表示未选择平台, 不得作为实际平台使用.
- 每个实际平台使用唯一编号, 从 `1U` 开始. 名称格式为 `NERV_PLATFORM_<NAME>`.
- `NERV_PLATFORM` 选择 `bsp/<platform>` 适配层, 在 `config.h` 中只定义一个有效值.
- FDCAN 是必需外设. 不提供关闭 FDCAN 的开关.
- 其他外设按模块需要选择.
- 不定义任务优先级, 栈大小, 周期, 看门狗超时或应用参数.

平台选择示例:

```c
/* 示例: 选择已经实现的平台. 宏名按实际 BSP 替换. */
#define NERV_PLATFORM NERV_PLATFORM_CUSTOM
```

选择语句必须放在 `#ifndef NERV_PLATFORM` 之前, 或直接替换默认值 `NERV_PLATFORM_NONE`.

### 10.3 新增 MCU

新增 MCU 的步骤:

1. 在 `config.h` 的平台宏定义区增加唯一编号, 例如:
   ```c
   #define NERV_PLATFORM_MY_MCU 3U
   ```
2. 更新 `NERV_PLATFORM` 的支持检查, 允许新平台宏.
3. 在 `bsp/my_mcu/` 实现 `bsp/Inc/` 中的接口, 至少包括 FDCAN.
4. 在 `config.h` 选择新平台, 例如:
   ```c
   #define NERV_PLATFORM NERV_PLATFORM_MY_MCU
   ```
5. 使用者把 `bsp/my_mcu/Src` 加入自己的工程.

禁止修改 `module/` 或 `public/` 的公开接口来适配单个 MCU. 平台差异只允许出现在 `bsp/<name>/` 与 `config.h`.

### 10.4 编译期检查

`config/Src/config_check.c` 使用 `_Static_assert` 检查:

- 平台宏取值合法.
- 功能裁剪宏为 0 或 1.
- 错误码域号与位号不冲突.
- 枚举镜像与宏数值一致.
- 关键结构体大小不超过预算.

```c
_Static_assert(NERV_USE_LOG == 0 || NERV_USE_LOG == 1,
               "NERV_USE_LOG must be 0 or 1.");
```

---

## 11. 日志

### 11.1 设计目标

日志模块是可选的全局服务. 它只做三件事:

1. 按级别过滤.
2. 把格式化结果写入调用者提供的缓冲区.
3. 调用调用者注册的输出回调.

日志模块不创建任务, 不创建队列, 不操作 UART, 不包含 RTOS 头文件. 输出目标由使用者在自己的工程中实现, 可以是 UART, RTT, 内存缓冲区或网络.

### 11.2 配置

日志配置集中在 `config/Inc/config.h`:

```c
#define NERV_LOG_LEVEL_ERROR    0U
#define NERV_LOG_LEVEL_WARN     1U
#define NERV_LOG_LEVEL_INFO     2U
#define NERV_LOG_LEVEL_DEBUG    3U

#ifndef NERV_LOG_LEVEL
#define NERV_LOG_LEVEL          NERV_LOG_LEVEL_INFO
#endif

#if (NERV_LOG_LEVEL > NERV_LOG_LEVEL_DEBUG)
#error "NERV_LOG_LEVEL is invalid."
#endif

#ifndef NERV_LOG_MSG_MAX
#define NERV_LOG_MSG_MAX        160U
#endif

#if (NERV_LOG_MSG_MAX < 32U) || (NERV_LOG_MSG_MAX > 512U)
#error "NERV_LOG_MSG_MAX must be in range 32 to 512."
#endif

#ifndef NERV_LOG_USE_FLOAT
#define NERV_LOG_USE_FLOAT      0
#endif

#if (NERV_LOG_USE_FLOAT != 0) && (NERV_LOG_USE_FLOAT != 1)
#error "NERV_LOG_USE_FLOAT must be 0 or 1."
#endif
```

- `NERV_USE_LOG=0` 时所有日志宏展开为空操作, 不产生格式化开销.
- `NERV_USE_LOG=1` 时日志宏按 `NERV_LOG_LEVEL` 过滤.
- `NERV_LOG_USE_FLOAT=1` 只在使用者确认标准库支持浮点格式化时启用.
- 不在 `config.h` 中定义 UART 句柄, 任务优先级, 队列长度或输出函数.

### 11.3 接口

```c
/* public/Inc/nerv_log.h */

typedef enum
{
    NERV_LOG_LEVEL_ERROR_RT = 0U,  /**< 错误 */
    NERV_LOG_LEVEL_WARN_RT  = 1U,  /**< 警告 */
    NERV_LOG_LEVEL_INFO_RT  = 2U,  /**< 信息 */
    NERV_LOG_LEVEL_DEBUG_RT = 3U,  /**< 调试 */
} NERV_Log_LevelTypeDef;

typedef void (*NERV_Log_OutputTypeDef)(const char *msg,
                                       uint16_t len,
                                       void *user);

NERV_StatusTypeDef NERV_Log_SetOutput(NERV_Log_OutputTypeDef output, void *user);
NERV_StatusTypeDef NERV_Log_Write(const char *msg, uint16_t len);

NERV_StatusTypeDef NERV_Log_Printf(NERV_Log_LevelTypeDef level,
                                   const char *file,
                                   uint32_t line,
                                   const char *fmt,
                                   ...);
```

- `NERV_Log_SetOutput` 保存或替换输出回调与上下文, 不访问硬件, 不负责同步.
- `NERV_Log_Write` 把已经格式化的字节序列交给输出回调.
- `NERV_Log_Printf` 是日志模块的格式化入口, 也是唯一的可变参数函数例外.
- 所有函数在失败时返回 `NERV_ERROR`, 不设置全局错误寄存器.

### 11.4 日志宏

```c
#if (NERV_USE_LOG != 0)

#define NERV_LOG_FORMAT_AND_WRITE(_level, _file, _line, ...) \
    do {                                                       \
        (void)NERV_Log_Printf((_level), (_file), (_line), __VA_ARGS__); \
    } while (0)

#define NERV_LOG_ERROR(...) \
    NERV_LOG_FORMAT_AND_WRITE(NERV_LOG_LEVEL_ERROR, __FILE__, __LINE__, __VA_ARGS__)

#define NERV_LOG_WARN(...) \
    NERV_LOG_FORMAT_AND_WRITE(NERV_LOG_LEVEL_WARN, __FILE__, __LINE__, __VA_ARGS__)

#define NERV_LOG_INFO(...) \
    NERV_LOG_FORMAT_AND_WRITE(NERV_LOG_LEVEL_INFO, __FILE__, __LINE__, __VA_ARGS__)

#define NERV_LOG_DEBUG(...) \
    NERV_LOG_FORMAT_AND_WRITE(NERV_LOG_LEVEL_DEBUG, __FILE__, __LINE__, __VA_ARGS__)

#else

#define NERV_LOG_FORMAT_AND_WRITE(...) ((void)0)
#define NERV_LOG_ERROR(...)            ((void)0)
#define NERV_LOG_WARN(...)             ((void)0)
#define NERV_LOG_INFO(...)             ((void)0)
#define NERV_LOG_DEBUG(...)            ((void)0)

#endif
```

- 日志宏至少包含一个格式化字符串实参.
- `_level`, `_file`, `_line` 每次只求值一次.
- `NERV_LOG_FORMAT_AND_WRITE` 使用 `do { ... } while (0)`, 不改变调用方控制流.
- 不在宏内声明缓冲区, 格式化与缓冲区管理由 `NERV_Log_Printf` 完成.
- 禁止在日志宏实参中写带副作用的表达式.

### 11.5 实现

```c
/* public/Src/nerv_log.c */

#include "nerv_log.h"

#include <stdarg.h>
#include <stdio.h>

_Static_assert((int)NERV_LOG_LEVEL_ERROR == (int)NERV_LOG_LEVEL_ERROR_RT,
               "NERV_LOG_LEVEL_ERROR and NERV_LOG_LEVEL_ERROR_RT must be equal.");
_Static_assert((int)NERV_LOG_LEVEL_WARN == (int)NERV_LOG_LEVEL_WARN_RT,
               "NERV_LOG_LEVEL_WARN and NERV_LOG_LEVEL_WARN_RT must be equal.");
_Static_assert((int)NERV_LOG_LEVEL_INFO == (int)NERV_LOG_LEVEL_INFO_RT,
               "NERV_LOG_LEVEL_INFO and NERV_LOG_LEVEL_INFO_RT must be equal.");
_Static_assert((int)NERV_LOG_LEVEL_DEBUG == (int)NERV_LOG_LEVEL_DEBUG_RT,
               "NERV_LOG_LEVEL_DEBUG and NERV_LOG_LEVEL_DEBUG_RT must be equal.");

#if (NERV_USE_LOG != 0)

static NERV_Log_OutputTypeDef nerv_log_output;
static void *nerv_log_user;

NERV_StatusTypeDef NERV_Log_SetOutput(NERV_Log_OutputTypeDef output, void *user)
{
    if (NULL == output)
    {
        return NERV_ERROR;
    }

    nerv_log_output = output;
    nerv_log_user = user;

    return NERV_OK;
}

NERV_StatusTypeDef NERV_Log_Write(const char *msg, uint16_t len)
{
    if ((NULL == msg) || (0U == len))
    {
        return NERV_ERROR;
    }

    if (NULL == nerv_log_output)
    {
        return NERV_ERROR;
    }

    nerv_log_output(msg, len, nerv_log_user);

    return NERV_OK;
}

NERV_StatusTypeDef NERV_Log_Printf(NERV_Log_LevelTypeDef level,
                                   const char *file,
                                   uint32_t line,
                                   const char *fmt,
                                   ...)
{
    char msg[NERV_LOG_MSG_MAX];
    int written;
    va_list args;
    uint16_t len;

    if ((NULL == file) || (NULL == fmt))
    {
        return NERV_ERROR;
    }

    if ((uint32_t)level > (uint32_t)NERV_LOG_LEVEL)
    {
        return NERV_OK;
    }

    va_start(args, fmt);
    written = vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (written < 0)
    {
        return NERV_ERROR;
    }

    if ((size_t)written >= sizeof(msg))
    {
        len = (uint16_t)(sizeof(msg) - 1U);
    }
    else
    {
        len = (uint16_t)written;
    }

    (void)file;
    (void)line;

    return NERV_Log_Write(msg, len);
}

#else

NERV_StatusTypeDef NERV_Log_SetOutput(NERV_Log_OutputTypeDef output, void *user)
{
    (void)output;
    (void)user;
    return NERV_OK;
}

NERV_StatusTypeDef NERV_Log_Write(const char *msg, uint16_t len)
{
    (void)msg;
    (void)len;
    return NERV_OK;
}

NERV_StatusTypeDef NERV_Log_Printf(NERV_Log_LevelTypeDef level,
                                   const char *file,
                                   uint32_t line,
                                   const char *fmt,
                                   ...)
{
    (void)level;
    (void)file;
    (void)line;
    (void)fmt;
    return NERV_OK;
}

#endif
```

说明:

- 头文件前缀可以通过 `file` 与 `line` 增加, 但本库不强制格式. 格式化格式由使用者在输出回调或 `NERV_Log_Printf` 的封装层决定.
- 日志模块不分配堆内存. `msg` 位于调用者栈上, 大小由 `NERV_LOG_MSG_MAX` 限制.
- `vsnprintf` 的返回值按标准 C 处理: 负值表示格式错误, 大于等于缓冲区大小表示截断.
- `NERV_LOG_USE_FLOAT=0` 时, 调用者不得在格式串中使用浮点格式, 否则行为取决于标准库.
- 日志模块不提供弱符号钩子. 输出函数必须通过 `NERV_Log_SetOutput` 显式注册.

### 11.6 线程与 ISR 约束

- `NERV_Log_Printf` 与 `NERV_Log_Write` 默认不是 ISR-safe.
- 输出回调必须由使用者保证线程安全. 库不创建互斥量, 不加锁.
- 禁止在中断中调用日志宏. 中断中只置错误位或计数, 由任务上下文输出.
- 输出回调不得阻塞, 不得回调库的控制路径, 不得递归调用日志.
- 若使用者需要异步日志, 在自己的工程中实现环形缓冲区与输出任务. 该缓冲区与任务不属于本库.

### 11.7 日志级别

- `NERV_LOG_ERROR`: 影响安全, 通信或控制正确性的故障.
- `NERV_LOG_WARN`: 可恢复异常, 降级运行, 参数接近边界.
- `NERV_LOG_INFO`: 初始化, 模式切换, 配置摘要.
- `NERV_LOG_DEBUG`: 周期状态与调试细节.

生产构建推荐 `NERV_LOG_LEVEL_WARN` 或 `NERV_LOG_LEVEL_ERROR`. 调试构建可使用 `NERV_LOG_LEVEL_DEBUG`.

---

## 12. 并发与 RTOS

### 12.1 核心约束

- 库核心不包含任何 RTOS 头文件.
- 库核心不创建任务, 互斥量, 信号量, 队列, 定时器或事件组.
- 库核心不调用延时函数, 不主动让出 CPU.
- 库核心不依赖调度器已经启动.
- 库不规定任务优先级, 栈大小, 周期或看门狗策略.
- 库不提供 `nerv_rtos_config.h`. 所有平台与功能选择只在 `config/Inc/config.h` 中完成.

### 12.2 线程模型

每个对外接口必须在契约中标注以下一种线程模型:

| 标注 | 含义 |
| --- | --- |
| `ISR-safe` | 可以在中断上下文调用, 不阻塞, 不分配内存, 不调用日志 |
| `NOT ISR-safe` | 只能在任务或主循环上下文调用 |
| `caller-synchronized` | 库内部不加锁, 调用者必须保证同一实例不被并发访问 |

默认模型是 `NOT ISR-safe` 与 `caller-synchronized`. 只有明确标注 `ISR-safe` 的函数才能在中断中调用.

标注 `ISR-safe` 时只写 `@note ISR-safe.` 不重复说明不阻塞, 不分配内存等含义.

### 12.3 调用者责任

使用者的应用层负责:

- MCU 时钟, 引脚, FDCAN 与中断初始化.
- RTOS 任务, 优先级, 栈大小, 周期与队列.
- 互斥量, 临界区与共享数据保护.
- 看门狗, 急停硬件回路与系统级故障策略.
- 启动顺序与掉电保存.

库只提供可重入的函数, 实例与回调. 库不替调用者决定并发策略.

### 12.4 初始化顺序

推荐的调用顺序:

1. 使用者初始化 MCU 时钟与引脚.
2. 使用者初始化 FDCAN 外设与中断.
3. 使用者调用 `NERV_FDCAN_Init` 与 `NERV_FDCAN_Start`.
4. 使用者调用各模块的 `NERV_<Module>_Init`.
5. 使用者启动 RTOS 调度器或进入主循环.

库不要求第 5 步一定存在. 裸机工程可以在主循环中周期调用模块函数.

### 12.5 中断规则

- 中断服务函数不得调用日志宏.
- 中断服务函数不得调用可能阻塞的接口.
- 中断服务函数不得分配内存.
- 中断服务函数应只做寄存器读取, 帧复制, 错误位置位与回调通知.
- 回调在中断上下文执行时, 回调实现必须满足 `ISR-safe` 约束.
- 共享错误位在中断中置位, 在任务中清除时必须由调用者使用临界区或原子位操作.

### 12.6 可选 RTOS 适配

- 库可以在使用者工程中与任意 RTOS 共存, 但 RTOS 适配代码不属于库核心.
- 若使用者需要 RTOS 互斥量, 应在自己的封装层中加锁, 不修改模块内部实现.
- 若使用者需要异步日志, 应在自己的工程中实现缓冲区与输出任务, 不修改 `public/Inc/nerv_log.h` 与 `public/Src/nerv_log.c`.
- 库不提供 FreeRTOS, CMSIS-OS, Zephyr 或 RT-Thread 的绑定.

---

## 13. FDCAN 与可移植性

### 13.1 外设基线

- FDCAN 是本库唯一必需的外设.
- 目标 MCU 只要提供 FDCAN 外设与中断, 就可以实现 `bsp/<platform>` 适配层并使用本库.
- GPIO, PWM, UART, 定时器等外设只在底盘, 云台, 发射或调试模块需要时通过可选 BSP 接口提供.
- 本库不要求目标 MCU 具备特定型号的定时器, DMA, 浮点单元或 DSP 指令.
- 只有经典 CAN 外设而没有 FDCAN 外设的 MCU 不在本库支持范围内. FDCAN 外设可以发送经典 CAN 帧, 通过 `NERV_FDCAN_FLAG_FD=0` 表示.

### 13.2 BSP 接口

`bsp/Inc/nerv_fdcan.h` 定义与 MCU 无关的帧, 过滤器, 回调与操作表. 平台适配层实现操作表, 模块层只依赖该头文件.

```c
/* bsp/Inc/nerv_fdcan.h */

typedef struct NERV_FDCAN_Instance NERV_FDCAN_InstanceTypeDef;

typedef enum
{
    NERV_FDCAN_FLAG_NONE     = 0x00U,  /**< 经典 CAN 标准帧 */
    NERV_FDCAN_FLAG_EXTENDED = 0x01U,  /**< 扩展帧 */
    NERV_FDCAN_FLAG_FD       = 0x02U,  /**< FDCAN 帧 */
    NERV_FDCAN_FLAG_BRS      = 0x04U,  /**< 位速率切换 */
} NERV_FDCAN_FlagTypeDef;

typedef struct
{
    uint32_t id;          /**< 11 位或 29 位标识符 */
    uint8_t data[64];     /**< 数据区, 长度由 len 指定 */
    uint8_t len;          /**< 数据长度, 经典帧 0 到 8, FDCAN 帧 0 到 64 */
    uint8_t flags;        /**< NERV_FDCAN_FLAG_* 位掩码 */
} NERV_FDCAN_FrameTypeDef;

typedef struct
{
    uint32_t id;          /**< 过滤器标识符 */
    uint32_t mask;        /**< 过滤器掩码 */
    uint8_t flags;        /**< NERV_FDCAN_FLAG_EXTENDED 等 */
} NERV_FDCAN_FilterTypeDef;

typedef void (*NERV_FDCAN_RxCallbackTypeDef)(
    void *user,
    const NERV_FDCAN_FrameTypeDef *frame);

typedef struct
{
    NERV_StatusTypeDef (*init)(NERV_FDCAN_InstanceTypeDef *self);
    NERV_StatusTypeDef (*start)(NERV_FDCAN_InstanceTypeDef *self);
    NERV_StatusTypeDef (*stop)(NERV_FDCAN_InstanceTypeDef *self);
    NERV_StatusTypeDef (*configure_filter)(
        NERV_FDCAN_InstanceTypeDef *self,
        const NERV_FDCAN_FilterTypeDef *filters,
        uint8_t count);
    NERV_StatusTypeDef (*transmit)(
        NERV_FDCAN_InstanceTypeDef *self,
        const NERV_FDCAN_FrameTypeDef *frame);
    uint32_t (*get_error)(const NERV_FDCAN_InstanceTypeDef *self);
    void (*clear_error)(NERV_FDCAN_InstanceTypeDef *self, uint32_t mask);
} NERV_FDCAN_OpsTypeDef;

struct NERV_FDCAN_Instance
{
    const NERV_FDCAN_OpsTypeDef *ops;       /**< 平台操作表 */
    NERV_FDCAN_RxCallbackTypeDef rx_callback;
    void *rx_user;
    void *platform_data;                     /**< 平台私有数据 */
};
```

通用接口:

```c
NERV_StatusTypeDef NERV_FDCAN_Init(NERV_FDCAN_InstanceTypeDef *self);
NERV_StatusTypeDef NERV_FDCAN_Start(NERV_FDCAN_InstanceTypeDef *self);
NERV_StatusTypeDef NERV_FDCAN_Stop(NERV_FDCAN_InstanceTypeDef *self);

NERV_StatusTypeDef NERV_FDCAN_ConfigureFilter(
    NERV_FDCAN_InstanceTypeDef *self,
    const NERV_FDCAN_FilterTypeDef *filters,
    uint8_t count);

NERV_StatusTypeDef NERV_FDCAN_Transmit(
    NERV_FDCAN_InstanceTypeDef *self,
    const NERV_FDCAN_FrameTypeDef *frame);

NERV_StatusTypeDef NERV_FDCAN_RegisterRxCallback(
    NERV_FDCAN_InstanceTypeDef *self,
    NERV_FDCAN_RxCallbackTypeDef callback,
    void *user);

uint32_t NERV_FDCAN_GetError(const NERV_FDCAN_InstanceTypeDef *self);
void NERV_FDCAN_ClearError(NERV_FDCAN_InstanceTypeDef *self, uint32_t mask);

void NERV_FDCAN_OnRx(
    NERV_FDCAN_InstanceTypeDef *self,
    const NERV_FDCAN_FrameTypeDef *frame);
```

约束:

- `NERV_FDCAN_Init`, `Start`, `Stop`, `ConfigureFilter` 默认 `NOT ISR-safe`.
- `NERV_FDCAN_ConfigureFilter` 必须在 `NERV_FDCAN_Start` 之前调用.
- `NERV_FDCAN_Transmit` 非阻塞. 没有空闲发送邮箱时返回 `NERV_BUSY`.
- `NERV_FDCAN_OnRx` 由平台适配层在接收中断中调用. 它只复制回调指针并通知, 不阻塞, 不分配内存, 不调用日志.
- `NERV_FDCAN_GetError` 返回粘滞位掩码. `NERV_FDCAN_ClearError` 只清除 `mask` 中置位的错误.
- 回调的上下文由注册函数契约写明. 若在中断中调用, 回调必须是 `ISR-safe`.

### 13.3 帧与过滤器规则

- `id` 的位宽由 `NERV_FDCAN_FLAG_EXTENDED` 决定. 标准帧只使用低 11 位, 扩展帧只使用低 29 位.
- `len` 必须与帧类型一致. 经典帧 `len <= 8`, FDCAN 帧 `len <= 64`.
- `NERV_FDCAN_FLAG_BRS` 只能与 `NERV_FDCAN_FLAG_FD` 同时使用.
- 过滤器必须与帧的标识符类型匹配. 标准帧过滤器不得匹配扩展帧.
- 电机协议常量放在 `module/<name>/Inc/*_def.h`, 不得放在 BSP 头文件.
- 平台适配层负责把 `NERV_FDCAN_FilterTypeDef` 转换为厂商过滤器配置. 模块层不接触寄存器.

### 13.4 平台适配层

`bsp/<platform>/Src/nerv_fdcan_port.c` 负责:

- 定义平台私有结构体, 第一个成员为 `NERV_FDCAN_InstanceTypeDef`.
- 实现 `NERV_FDCAN_OpsTypeDef` 中的全部函数.
- 在接收中断中填充 `NERV_FDCAN_FrameTypeDef`, 调用 `NERV_FDCAN_OnRx`.
- 在发送完成或错误中断中更新错误位.
- 提供平台初始化入口, 由使用者把 HAL 句柄或寄存器基址绑定到 `platform_data`.

平台适配层可以包含 MCU 厂商头文件, 但不得包含 `module/` 或应用层头文件.

STM32 HAL 参考实现的边界:

- `bsp/stm32h7/Inc/nerv_stm32h7.h` 可以包含 `stm32h7xx_hal.h`, 只由应用层与 BSP 包含.
- `bsp/stm32h7/Src/nerv_fdcan_stm32h7.c` 调用 `HAL_FDCAN_*`, 并实现 `NERV_FDCAN_OpsTypeDef`.
- HAL 的 `HAL_FDCAN_RxFifo0Callback` 等 weak 回调由 BSP 覆盖, 用于把接收事件转交 `NERV_FDCAN_OnRx`.
- HAL 源文件, CubeMX 生成文件与启动文件属于使用者工程, 不进入本库.
- 只维护 STM32 BSP 是可以接受的. 其他 MCU 通过新增 `bsp/<platform>/` 接入, 不作为当前必须完成的工作.

### 13.5 新增 MCU

新增 MCU 时:

1. 在 `config.h` 增加 `NERV_PLATFORM_<NAME>`.
2. 新建 `bsp/<name>/Inc` 与 `bsp/<name>/Src`.
3. 实现 `nerv_fdcan.h` 的操作表, 至少支持初始化, 启动, 过滤器, 发送, 接收与错误读取.
4. 若模块需要 GPIO, PWM 或时间接口, 再实现对应可选 BSP 接口.
5. 在 `config.h` 选择新平台.
6. 使用者把 `bsp/<name>/Src` 加入自己的工程.

禁止修改 `module/` 或 `public/` 的公开接口来适配单个 MCU. 平台差异只允许出现在 `bsp/<name>/` 与 `config.h`.

### 13.6 可移植性规则

- 使用 C11 与固定宽度整型. 只包含 `stdint.h`, `stddef.h`, `stdbool.h` 等标准头文件.
- 公共头文件与模块层不得使用编译器扩展, 例如 `__attribute__`, `__weak`, `#pragma pack`, 内联汇编.
- BSP 内对接 HAL 的 `__weak` 回调可以按厂商规则使用, 但不得作为模块扩展点.
- 平台适配层若必须使用其他编译器扩展, 必须封装在 `bsp/<platform>/` 内部, 并在平台文档中说明.
- 禁止变长数组与隐式动态分配.
- 禁止依赖主机字节序. 多字节协议字段必须显式编码与解码.
- 禁止直接对协议缓冲区做结构体覆盖, 除非使用 `_Static_assert` 校验大小, 对齐与偏移.
- 禁止假设 `int` 为 32 位. 使用 `uint32_t`, `int32_t` 等固定宽度类型.
- 禁止在公共头文件中包含 IDE 生成文件, 板级头文件或厂商 HAL 头文件.
- 库不提供构建系统, 不提交工程文件. 使用者用任意支持 C11 的工具链编译.

---

## 14. goto

`goto` 只允许用于函数末尾的统一清理与统一返回.

规则:

- 标签只允许出现在函数末尾.
- 禁止向后跳转.
- 禁止跳过资源初始化.
- 进入清理段之前, 所有资源变量必须已经初始化为安全值.
- 清理段只释放或关闭已经初始化的资源.
- 清理段之后只允许 `return`.
- 禁止用 `goto` 实现循环.
- `goto` 标签使用小写下划线, 例如 `out`, `cleanup`, `fail`.

```c
NERV_StatusTypeDef NERV_Chassis_Init(
    NERV_Chassis_InstanceTypeDef *hchassis,
    const NERV_Chassis_ConfigTypeDef *p_config)
{
    NERV_StatusTypeDef status;

    status = NERV_OK;

    if ((NULL == hchassis) || (NULL == p_config))
    {
        status = NERV_ERROR;
        goto out;
    }

    if (NULL == p_config->hfdcan)
    {
        status = NERV_ERROR;
        goto out;
    }

    status = NERV_Chassis_Register(hchassis, p_config);
    if (NERV_OK != status)
    {
        goto out;
    }

    status = hchassis->init(hchassis);
    if (NERV_OK != status)
    {
        goto out;
    }

out:
    return status;
}
```

---

## 15. 防御性编程与 C++ 兼容

### 15.1 头文件骨架

所有公共头文件采用相同结构:

```c
#ifndef NERV_MOTOR_H
#define NERV_MOTOR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "nerv_status.h"
#include "nerv_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 声明 */

#ifdef __cplusplus
}
#endif

#endif /* NERV_MOTOR_H */
```

- 一个头文件一个 include guard, 命名与文件名一致.
- 公共头文件只包含标准头文件与本库 `public/`, `config/`, `bsp/Inc/` 头文件.
- 禁止包含 MCU 厂商头文件, RTOS 头文件, IDE 生成头文件.
- 禁止在头文件中定义变量或分配存储.
- 禁止在头文件中使用 `using`, `namespace`, `class`, `template` 等 C++ 关键字作为标识符.

### 15.2 参数检查

所有对外函数必须检查:

- 指针是否为 `NULL`.
- 长度, 数量, 索引是否越界.
- 枚举值是否在合法范围.
- 实例是否已经初始化.
- 配置指针的生命周期是否满足契约.

```c
if (NULL == self)
{
    return NERV_ERROR;
}

if ((0U == count) || (count > NERV_MOTOR_MAX_COUNT))
{
    return NERV_ERROR;
}
```

规则:

- 参数错误返回 `NERV_ERROR`, 不返回未定义值.
- 计算结果与参数错误需要同时返回时, 函数返回 `NERV_StatusTypeDef`, 计算结果通过输出参数返回.
- 不得用 `assert` 替代运行时参数检查. `assert` 只用于开发期不变量.
- 失败路径不得留下部分初始化状态.
- 失败路径不得泄漏资源.
- 不得对空指针做解引用后再检查.
- 不得依赖调用者保证参数合法.

### 15.3 数值与边界

- 所有减法, 加法, 乘法在可能溢出时必须显式检查.
- 无符号回绕必须是有意设计, 并在注释中说明.
- 浮点比较使用容差函数, 禁止直接用 `==`.
- 角度归一化必须使用 `NERV_Math_Wrap(angle, -NERV_MATH_PI, NERV_MATH_PI)`.
- 数组索引必须使用 `size_t` 或足以覆盖范围的固定宽度类型.
- 不得把负数转换为无符号类型后再比较大小.
- 不得对负数做移位.
- 不得使用未定义行为, 例如有符号整数溢出, 越界访问, 未对齐访问.

### 15.4 错误处理顺序

参数检查与状态检查按以下顺序执行:

1. 检查实例指针与输出指针.
2. 检查数值范围与长度.
3. 检查实例初始化状态.
4. 检查硬件或资源状态.
5. 执行操作.
6. 更新错误位与返回值.

禁止先访问硬件再检查参数. 禁止在参数非法时修改实例状态.

### 15.5 C++ 兼容

- 公共头文件必须可以被 C++ 编译器包含.
- 使用 `extern "C"` 包裹声明.
- 不使用 C++ 引用, 异常, RTTI, 模板或 `constexpr`.
- 枚举成员不得与 C++ 关键字冲突.
- `_Static_assert` 只放在 `.c` 文件中. 若公共头文件需要编译期检查, 使用条件编译区分 C 与 C++.
- C++ 调用者负责把 `nullptr` 转换为 `NULL`, 库接口只使用 C 指针.
- 库不提供 C++ 封装类. 需要时由使用者在自己工程中实现.

### 15.6 禁止隐藏行为

- 禁止在库内部动态分配内存.
- 禁止在库内部创建线程, 任务, 定时器或信号量.
- 禁止在库内部调用 `malloc`, `free`, `calloc`, `realloc`.
- 禁止在库内部调用 `printf` 系列函数, 日志模块的 `vsnprintf` 除外.
- 禁止在库内部调用 `exit`, `abort` 或无限循环.
- 禁止在库内部修改全局状态, 日志模块的静态输出回调除外.

---

## 16. 扩展点

### 16.1 唯一机制

NERV 模块只使用注册回调作为运行期扩展机制.

- `module/` 与 `public/` 不使用 `__weak`.
- BSP 为对接 HAL 的 weak 回调, 可以在 `bsp/<platform>/` 内按厂商规则覆盖.
- 不使用编译器相关的弱符号属性作为模块扩展点.
- 不使用全局钩子函数.
- 不使用链接脚本符号覆盖.
- 不使用未文档化的函数指针强转.

### 16.2 回调契约

每个注册函数必须说明:

- 回调上下文: 任务, 中断或启动阶段.
- 回调生命周期: 注册后何时生效, 何时失效.
- 回调是否可以为 `NULL`, `NULL` 是否表示注销.
- 回调是否允许阻塞.
- 回调是否允许递归调用库接口.
- 回调的 `user` 指针由谁拥有, 何时释放.
- 回调在 ISR 中执行时, 必须标注 `ISR-safe`.

```c
typedef void (*NERV_Chassis_FaultCallbackTypeDef)(
    void *user,
    uint32_t fault_mask);

/**
  * @brief  注册底盘故障回调.
  * @param  self 底盘实例指针, 不得为 NULL.
  * @param  callback 回调函数, NULL 表示注销.
  * @param  user 回调上下文, 由调用者保证生命周期.
  * @retval NERV_OK 注册成功.
  * @retval NERV_ERROR 参数非法.
  * @note   回调可能在任务上下文执行, 不得阻塞.
  * @note   NOT ISR-safe.
  */
NERV_StatusTypeDef NERV_Chassis_RegisterFaultCallback(
    NERV_Chassis_InstanceTypeDef *self,
    NERV_Chassis_FaultCallbackTypeDef callback,
    void *user);
```

### 16.3 注册与注销

- 注册函数返回 `NERV_StatusTypeDef`.
- 注册成功前不得调用回调.
- 注销后不得再调用旧回调.
- 多实例模块的回调必须绑定实例, 不得共享全局回调.
- 单实例全局服务, 例如日志, 允许一个全局输出回调, 但必须通过显式函数注册.
- 注册函数本身默认 `NOT ISR-safe`.

### 16.4 禁止的扩展方式

- 禁止在模块公共头文件中声明可被用户覆盖的弱函数. BSP 对接 HAL 的 weak 回调除外.
- 禁止使用 `#ifdef` 在模块层嵌入板级实现.
- 禁止通过全局变量在模块之间传递回调.
- 禁止在中断中注册或注销回调.
- 禁止在回调中调用会阻塞的库函数.

---

## 17. 安全与故障

### 17.1 适用范围

`module/safety/` 是可选的库模块, 提供故障分级, 急停状态与安全状态切换的软件接口. 它不替代硬件安全回路, 不管理看门狗, 不创建监控任务.

以下内容由使用者负责:

- 硬件急停回路与电源切断.
- 看门狗喂狗与超时策略.
- 任务心跳与调度监控.
- 安全响应时间与硬件冗余.
- 故障后的现场处置与复位流程.

库只提供可测试的状态与接口, 不承诺系统级安全认证.

### 17.2 故障分级

```c
typedef enum
{
    NERV_SAFETY_LEVEL_OK       = 0U,  /**< 正常 */
    NERV_SAFETY_LEVEL_WARNING  = 1U,  /**< 警告, 可降级运行 */
    NERV_SAFETY_LEVEL_CRITICAL = 2U,  /**< 严重, 必须进入安全状态 */
} NERV_Safety_LevelTypeDef;
```

- `WARNING` 允许继续运行, 但必须记录错误并通知上层.
- `CRITICAL` 必须停止执行器输出, 进入安全状态.
- 故障级别只能由明确的安全策略提升, 不得静默降级.

### 17.3 急停与锁存

```c
NERV_StatusTypeDef NERV_Safety_Init(NERV_Safety_InstanceTypeDef *self);
void NERV_Safety_TriggerEStop(NERV_Safety_InstanceTypeDef *self, uint32_t reason);
NERV_StatusTypeDef NERV_Safety_ClearEStop(NERV_Safety_InstanceTypeDef *self);
bool NERV_Safety_IsEStopActive(const NERV_Safety_InstanceTypeDef *self);
uint32_t NERV_Safety_GetError(const NERV_Safety_InstanceTypeDef *self);
void NERV_Safety_ClearError(NERV_Safety_InstanceTypeDef *self, uint32_t mask);
```

约束:

- `TriggerEStop` 是 `ISR-safe`, 只置位与记录原因, 不阻塞.
- 急停状态必须锁存, 不能因为输入恢复而自动清除.
- `ClearEStop` 必须由任务上下文显式调用, 并满足使用者的复位条件.
- `ClearEStop` 不得清除硬件急停输入仍然有效的状态.
- 急停原因必须保留到显式清除.
- `NERV_Safety_GetError` 返回粘滞位掩码.

### 17.4 执行器安全状态

每个执行器模块必须提供:

```c
NERV_StatusTypeDef NERV_<Module>_EnterSafeState(
    NERV_<Module>_InstanceTypeDef *self);
```

安全状态至少包括:

- 电机输出置零或失能.
- PWM 占空比置零.
- 控制环积分项清零.
- 目标值清零.
- 通信超时计数复位.

安全状态不得依赖通信成功, 不得等待上位机指令, 不得动态分配内存.

### 17.5 故障传播

- 底层模块检测到 `CRITICAL` 故障时, 必须置错误位并通知已注册回调.
- 上层模块收到 `CRITICAL` 故障后, 必须停止相关执行器并进入安全状态.
- 故障回调不得阻塞, 不得递归调用触发故障的接口.
- 故障回调在中断中执行时, 只能置位与记录, 不得执行控制逻辑.
- 禁止在故障路径中调用日志宏. 日志由任务上下文输出.

### 17.6 安全相关错误码

安全域使用 `NERV_ERR_DOMAIN_SAFETY`:

```c
#define NERV_ERR_SAFETY_ESTOP          (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SAFETY) | NERV_ERR_BIT(17U))
#define NERV_ERR_SAFETY_LATCHED        (NERV_ERR_DOMAIN(NERV_ERR_DOMAIN_SAFETY) | NERV_ERR_BIT(18U))
```

- `NERV_ERR_SAFETY_ESTOP` 表示软件急停已触发并锁存, 等待显式复位.
- `NERV_ERR_SAFETY_LATCHED` 表示安全状态已锁存, 需要显式复位.

置位与清除:

- `NERV_ERR_SAFETY_ESTOP` 只由 `NERV_Safety_TriggerEStop` 置位, 只由 `NERV_Safety_ClearEStop` 清除. `ClearError` 不处理该位.
- `NERV_ERR_SAFETY_LATCHED` 在进入安全状态时置位, 显式复位时清除.
- 硬件急停回路直接切断执行器供电, 由使用者负责. 库只接收急停已触发这一事件, 不读取硬件急停输入.

---

## 18. 编码与标点

### 18.1 文件编码

- 所有文本文件使用 UTF-8 无 BOM.
- 换行使用 LF, 禁止 CRLF.
- 缩进使用 4 个空格, 禁止 Tab.
- 文件末尾保留一个换行.
- 禁止行尾空白.
- 禁止 NBSP, 零宽字符与不可见控制字符.
- 禁止 emoji 与颜文字.

### 18.2 标点

- 中文正文使用半角 ASCII 标点: `,` `.` `:` `;` `?` `!` `(` `)`.
- 代码, 宏, 类型, 函数名保持英文原文.
- 数学符号与单位可以使用 `+`, `-`, `*`, `/`, `%`, `±`, `π`, `²`, `³`, `·`, `℃`.
- 禁止使用全角逗号, 全角句号, 全角括号, 全角冒号, 全角分号与中文引号.
- 禁止用 `+` 表示并列关系. 需要并列时使用"与"或逗号.

### 18.3 仓库检查

- `.gitattributes` 统一文本文件换行.
- `.editorconfig` 统一缩进, 字符集与末尾换行.
- `scripts/check_style.py` 检查 BOM, UTF-8, CRLF, Tab, 行尾空白, 全角标点, NBSP, 零宽字符与文件末尾换行.
- 提交前运行:

```sh
python3 scripts/check_style.py .
git diff --check
```

### 18.4 仓库边界

- 库仓库只包含可复用源码, 配置, 文档与仓库检查脚本.
- 不提交 `.ioc`, `MDK-ARM`, `Core/`, `Drivers/`, 启动文件, 链接脚本, IDE 工程文件或构建产物.
- 不提交测试工程, HIL 配置, CI 配置或发布脚本.
- 使用者的实际工程负责这些文件, 本库不生成也不约束它们.

---

## 19. 提交规范

### 19.1 格式

提交信息使用 Conventional Commits:

```text
<type>(<scope>): <subject>

<body>
```

- `type` 使用小写英文: `feat`, `fix`, `refactor`, `style`, `docs`, `chore`.
- `scope` 使用小写英文: 模块名, `public`, `config`, `bsp`, `log`, `status`, `type`, `macro`, `docs`, `layout`.
- `subject` 使用英文, 不超过 72 个字符, 不以句号结尾.
- `body` 使用中文, 说明原因, 影响范围与验证方式.
- 禁止 emoji.
- 禁止使用 `app`, `mdk`, `tests`, `ci`, `rtos` 作为 scope. 这些属于使用者工程.
- 一个提交只做一件事. 禁止把重构, 格式化与功能修改混在一起.
- 禁止 `WIP`, `tmp`, `fix again` 这类无意义信息.

### 19.2 示例

```text
feat(fdcan): add MCU independent FDCAN interface

FDCAN 接口改为操作表与实例绑定, 平台适配层只依赖 bsp/Inc.
模块层不再包含厂商 HAL 头文件.

验证: 使用两个平台的 BSP 适配层完成编译.
```

```text
refactor(module): remove RTOS dependency from control path

控制路径不再引用任何 RTOS 类型. 任务, 互斥量与周期由使用者负责.
```

```text
docs(agents): define library-only repository boundary

明确本库不包含 MDK-ARM, .ioc, Core, Drivers, 应用任务与测试工程.
补充 FDCAN 作为唯一必需外设的移植约束.
```

### 19.3 提交前检查

- 运行 `python3 scripts/check_style.py .`.
- 运行 `git diff --check`.
- 改动过的源文件与头文件用 `clang-format` 检查无差异, 或先用 `clang-format -i` 格式化.
- 确认没有构建产物, IDE 文件, 测试工程或临时文件进入暂存区.
- 确认提交信息与实际修改一致.

---

## 20. 自检清单

### 20.1 库身份

- 仓库没有 `.ioc`, `MDK-ARM`, `Core/`, `Drivers/`, 启动文件, 链接脚本或 IDE 工程.
- 仓库没有 `app/`, 应用任务, `chassis_task`, `gimbal_task`, `shoot_task` 或 `app_init`.
- 仓库没有 `tests/`, HIL 配置, CI 配置或测试报告.
- 仓库没有构建系统文件. 使用者把自己的工程需要的源文件加入工程.
- 库只包含 `public/`, `config/`, `bsp/`, `module/` 与仓库检查脚本.

### 20.2 可移植性

- `public/`, `module/` 没有包含 MCU 厂商头文件.
- `module/` 没有包含 `bsp/<platform>/` 头文件.
- HAL 调用只出现在 `bsp/<platform>/`; HAL 源文件与 CubeMX 生成文件不在本库.
- `config/Inc/config.h` 是唯一配置入口, 不包含 HAL 头文件.
- 平台差异只在 `bsp/<platform>/` 与 `config.h`.
- FDCAN 是唯一必需外设. 其他外设通过可选 BSP 接口提供.
- 新增 MCU 不需要修改 `module/` 或 `public/` 的公开接口.
- 公共头文件使用 C11 与固定宽度类型, 不依赖编译器扩展.
- 不依赖主机字节序, 不对协议缓冲区做未校验的结构体覆盖.

### 20.3 鲁棒性

- 所有对外函数返回 `NERV_StatusTypeDef` 或明确的状态类型.
- 所有指针参数在使用前检查 `NULL`.
- 所有长度, 索引, 枚举值与数值范围检查边界.
- 失败路径不留下部分初始化状态, 不泄漏资源.
- 库内部不动态分配内存.
- 库内部不创建任务, 互斥量, 定时器或信号量.
- 库内部不调用阻塞函数, 不调用 `exit`, `abort` 或无限循环.
- 执行器模块提供 `EnterSafeState`.
- 严重故障进入安全状态并锁存, 需要显式复位.

### 20.4 可读性与优雅

- 名称使用 `NERV_` 前缀, 模块名与动词清晰.
- 函数不超过 80 行, 参数不超过 6 个.
- 控制流直接, 大括号独占一行, 4 个空格缩进.
- Magic Number 使用命名常量.
- 注释解释原因, 约束与前置条件, 不重复代码.
- 实例由调用者提供, 不依赖全局单例.
- 模块扩展点只使用注册回调; BSP 对接 HAL 的 weak 回调除外.
- 同一扩展点只有一种机制.

### 20.5 简洁性

- 只保留必要的抽象层.
- 同一件事只有一种推荐写法.
- 不为假设中的未来需求增加配置项.
- 不提供未使用的 API, 宏, 类型或文件.
- 日志模块不引入 RTOS 任务, 队列或 UART 句柄.
- 配置文件中不定义任务优先级, 栈大小, 周期或看门狗参数.

### 20.6 编码与提交

- 文本文件为 UTF-8 无 BOM, LF 换行, 文件末尾一个换行.
- 无 Tab, 无行尾空白, 无 NBSP, 无零宽字符.
- 中文正文使用半角标点, 无 emoji.
- 每个源文件与头文件带文件头, 版权行与 `LICENSE` 一致.
- `python3 scripts/check_style.py .` 通过.
- `git diff --check` 通过.
- 提交信息符合第 19 节.
- 提交中不包含构建产物, IDE 文件, 测试工程或临时文件.
