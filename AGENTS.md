# AGENTS.md — NERV’s Embedded Robotic Vehicle

> 本文件面向 AI 编程助手与人类开发者，定义 NERV 库的全部工程约束。
> 当本文件与代码现状冲突时，以本文件为准；修改规范须通过独立 PR 并更新本文件。

---

## 1. 项目定位

NERV 是电子科技大学 RoboMaster 战队电控组的通用嵌入式机器人库，基于 STM32 HAL、FDCAN、FreeRTOS CMSIS-OS v2 与 AC6 工具链，覆盖底盘、云台、发射机构等常见模块。

设计目标按优先级排序：

1. **可移植**：通过 `config/Inc/config.h` 宏配置适配不同机器人构型与硬件差异。
2. **可组合**：模块之间只通过显式接口与配置结构交互，避免隐式耦合。
3. **可审查**：代码本身说明意图，注释只补充设计原因与使用约束。
4. **可扩展**：新增底盘类型、电机型号或通信链路时，不修改已有模块的内部实现。
5. **可预测**：单位、坐标系、状态码、调度优先级全局统一，不依赖调用者记忆。

---

## 2. 目录结构（强制）

NERV 采用“每个模块文件夹下再分 `Inc/` 与 `Src/`”的布局。**禁止**在仓库根建立唯一的 `Inc/` 与 `Src/` 再平铺所有文件。

全局公共基础设施与编译期配置分别独立为 `public/` 与 `config/` 两个顶层目录，同样遵循 `Inc/` 与 `Src/` 规则：

```
NERV/
├── public/                      全局公共基础，跨层共享的状态码、类型、工具函数
│   ├── Inc/                     nerv_status.h、nerv_types.h、nerv_utils.h、nerv_math.h
│   └── Src/                     nerv_math.c，其余均为纯头文件
├── config/                      编译期配置，唯一配置入口
│   ├── Inc/                     config.h、nerv_rtos_config.h
│   └── Src/                     通常为空；如需编译期断言可放 config_check.c
├── app/
│   ├── Inc/                     chassis_task.h、gimbal_task.h、shoot_task.h、app_init.h
│   └── Src/                     chassis_task.c、gimbal_task.c、shoot_task.c、app_init.c
├── module/
│   ├── chassis/
│   │   ├── Inc/                 nerv_chassis.h、nerv_chassis_def.h、nerv_chassis_priv.h
│   │   └── Src/                 nerv_chassis.c
│   ├── gimbal/
│   │   ├── Inc/
│   │   └── Src/
│   ├── shoot/
│   │   ├── Inc/
│   │   └── Src/
│   └── motor/
│       ├── Inc/                 nerv_motor.h、nerv_unit.h
│       └── Src/                 nerv_motor.c、nerv_unit.c
├── bsp/
│   ├── fdcan/
│   │   ├── Inc/
│   │   └── Src/
│   ├── uart/
│   │   ├── Inc/
│   │   └── Src/
│   └── ...
├── log/
│   ├── Inc/                     nerv_log.h
│   └── Src/                     nerv_log.c
└── Core/                        CubeMX 生成，仅 USER CODE 区可改
```

**规则**：

- 每个功能模块 `public`、`config`、`app`、`module/<name>`、`bsp/<name>`、`log` 拥有独立的 `Inc/` 与 `Src/`。
- 头文件与源文件同名：`Inc/nerv_chassis.h` 对应 `Src/nerv_chassis.c`。
- 模块私有头 `*_priv.h` 与协议与宏定义头 `*_def.h` 放在该模块的 `Inc/` 下。`*_priv.h` 不得被其他模块包含；`*_def.h` 允许被其他模块包含，用于共享协议常量与推导宏。
- `public/Inc/` 是唯一允许被任意层包含的基础类型层，承载 `NERV_StatusTypeDef`、通用结构体、数学与单位工具。
- `config/Inc/` 是唯一编译期配置入口，承载 `config.h`、`nerv_rtos_config.h`、板级与构型选择宏。
- 构建系统按以下顺序加入 include path：
  1. `public/Inc`
  2. `config/Inc`
  3. `log/Inc`
  4. `bsp/<name>/Inc`，每个 bsp 模块
  5. `module/<name>/Inc`，每个 module 模块
  6. `app/Inc`
- **禁止**使用相对路径 `#include "../chassis/Inc/nerv_chassis.h"`。

---

## 3. 架构分层与依赖方向

依赖方向只能向下：

```
app/         应用层，机器人级任务编排，包括 chassis_task、gimbal_task、shoot_task
module/      模块层，硬件无关的功能单元，包括电机控制、PID、云台解算、发射状态机
bsp/         板级支持层，对 HAL 的薄封装，包括 fdcan、pwm、gpio、dwt、uart
log/         横切基础设施，宏驱动日志与 FreeRTOS 异步输出任务
public/      全局公共基础，状态码、通用类型、数学与单位工具
config/      编译期配置，构型选择、功能裁剪、RTOS 优先级、日志开关
```

- `bsp` 直接依赖 HAL，不得包含 `module` 或 `app` 的头文件。
- `module` 通过 `bsp` 提供的实例指针访问硬件，不得直接调用 `HAL_xxx`。
- `app` 通过 `module` 的公开接口组合行为，不得包含 `bsp` 内部头文件。
- `log` 与 `public` 可被任意层包含，二者不得反向包含 `module`、`app`、`bsp` 的具体实现头文件。
- `config` 是编译期配置，可被任意层包含；`config` 不得包含任何 `module`、`bsp`、`app` 头文件。
- 跨层反向通知采用注册回调函数指针，而不是在 `bsp` 中 `#include` 上层头文件。典型场景是 FDCAN 接收完成后通知电机模块。

---

## 4. NERV 状态码与错误体系

### 4.1 基础状态码，对齐 HAL

所有对外函数默认返回 `NERV_StatusTypeDef`，取值与 `HAL_StatusTypeDef` 一一对应。

```c
/* public/Inc/nerv_status.h */
#ifndef NERV_STATUS_H
#define NERV_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  NERV 通用状态码，对齐 HAL_StatusTypeDef 的四个基础值。
  * @note   对外函数默认返回本类型。需要更细原因时，通过各模块的
  *         NERV_<Module>_GetError 函数获取扩展错误码。
  */
typedef enum
{
    NERV_OK       = 0x00U,   /**< 操作成功 */
    NERV_ERROR    = 0x01U,   /**< 通用错误 */
    NERV_BUSY     = 0x02U,   /**< 资源忙，稍后重试 */
    NERV_TIMEOUT  = 0x03U,   /**< 操作超时 */
} NERV_StatusTypeDef;

#ifdef __cplusplus
}
#endif

#endif /* NERV_STATUS_H */
```

### 4.2 扩展错误码，按模块域分段

`NERV_StatusTypeDef` 只表达成功、失败、忙、超时四态。具体失败原因由 `NERV_<Module>_GetError` 返回的 `uint32_t` 位掩码表达，风格对齐 `HAL_GetError`。

扩展错误码采用位掩码语义，因此使用 `#define` 加固定宽度类型，而不使用 `enum`。原因在于 C 标准未保证枚举值做按位或运算后的类型与宽度，位掩码需要调用者执行 OR 组合，必须使用无符号整型常量。

```c
/* public/Inc/nerv_status.h 续 */

/**
  * @brief 扩展错误码，按模块域分段，可作为位掩码由 GetError 返回。
  * @note  域划分如下。0x0000_0001 至 0x0000_00FF 为通用域。
  *        0x0000_0100 至 0x0000_01FF 为 FDCAN 域。
  *        0x0000_0200 至 0x0000_02FF 为电机域。
  *        0x0000_0300 至 0x0000_03FF 为底盘域。
  *        0x0000_0400 至 0x0000_04FF 为云台域。
  *        0x0000_0500 至 0x0000_05FF 为发射域。
  * @note  同一模块内多个错误可同时置位，调用方使用位与判断。
  * @note  本组常量按位掩码语义定义，必须保持 2 的幂或 2 的幂之和，
  *        以便调用方使用按位或组合多个错误。
  */
#define NERV_ERR_NONE                 0x00000000U

/* 通用域 */
#define NERV_ERR_INVALID_PARAM        0x00000001U  /**< 参数非法 */
#define NERV_ERR_NULL_PTR             0x00000002U  /**< 空指针 */
#define NERV_ERR_NOT_INITIALIZED      0x00000004U  /**< 模块未初始化 */
#define NERV_ERR_NO_MEMORY            0x00000008U  /**< 内存不足 */
#define NERV_ERR_OUT_OF_RANGE         0x00000010U  /**< 数值越界 */
#define NERV_ERR_NOT_SUPPORTED        0x00000020U  /**< 功能不支持 */

/* FDCAN 域 */
#define NERV_FDCAN_ERR_FILTER         0x00000100U  /**< 过滤器配置失败 */
#define NERV_FDCAN_ERR_TX_MAILBOX     0x00000200U  /**< 发送邮箱满 */
#define NERV_FDCAN_ERR_RX_OVERRUN     0x00000400U  /**< 接收溢出 */

/* 电机域 */
#define NERV_MOTOR_ERR_OFFLINE        0x00010000U  /**< 电机离线 */
#define NERV_MOTOR_ERR_OVER_CURRENT   0x00020000U  /**< 过流 */
#define NERV_MOTOR_ERR_OVER_TEMP      0x00040000U  /**< 过温 */

/* 底盘域 */
#define NERV_CHASSIS_ERR_KINEMATICS    0x01000000U /**< 运动学解算失败 */
#define NERV_CHASSIS_ERR_WHEEL_OFFLINE 0x02000000U /**< 轮组离线 */
```

### 4.3 规则

- 禁止自定义新的基础状态枚举。需要表达新语义时使用扩展错误码。
- 对外函数返回 `NERV_StatusTypeDef`，详细原因通过 `NERV_<Module>_GetError` 查询。
- 每个模块必须提供 `NERV_<Module>_GetError` 与 `NERV_<Module>_ClearError`。
- 错误码定义集中放在 `public/Inc/nerv_status.h`，禁止在模块头文件中散落定义。
- 错误码与日志联动：`NERV_LOG_ERROR` 打印错误码时使用 `0x%08lX`，便于与本文档对照。

---

## 5. 单位与坐标系约定（强制）

### 5.1 内部统一 SI 单位

所有模块内部计算与对外接口统一使用 SI 单位。仅在最低层电机驱动做 `rpm` 与 `rad/s` 的转换，转换函数集中在 `module/motor/Inc/nerv_unit.h`。

| 物理量 | 单位          | 变量后缀               | 示例                 |
| ------ | ------------- | ---------------------- | -------------------- |
| 长度   | m             | `_m`                   | `wheel_radius_m`     |
| 角度   | rad           | `_rad`                 | `yaw_rad`            |
| 线速度 | m/s           | `_mps`                 | `vx_mps`             |
| 角速度 | rad/s         | `_radps`               | `wz_radps`           |
| 加速度 | m/s²          | `_mps2`                | `ax_mps2`            |
| 电流   | A             | `_a`                   | `current_a`          |
| 电压   | V             | `_v`                   | `voltage_v`          |
| 力矩   | N·m           | `_n_m`                 | `torque_n_m`         |
| 时间   | s 或 ms 或 us | `_s` 或 `_ms` 或 `_us` | `dt_s`、`timeout_ms` |
| 温度   | ℃             | `_c`                   | `temp_c`             |
| 质量   | kg            | `_kg`                  | `mass_kg`            |

### 5.2 坐标系

- 采用右手系，`x` 轴指向机器人正前方，`y` 轴指向左方，`z` 轴指向上方，与 REP-103 一致。
- yaw 绕 `z` 轴旋转，逆时针为正，观察方向为从上向下俯视。
- pitch 绕 `y` 轴旋转，抬头为正。
- roll 绕 `x` 轴旋转，右侧下沉为正。
- 所有位姿结构体 `NERV_*_PoseTypeDef` 的每个字段必须在定义处使用 `/**< ... */` 注明轴向与正方向。

### 5.3 角度范围

- 所有角度统一位于 `[-π, π)` 区间。
- 跨 ±π 的归一化必须调用 `NERV_Math_WrapPi`，禁止各模块自行实现 `if (angle > PI) angle -= 2*PI`。
- `NERV_Math_WrapPi` 与 `NERV_Math_Wrap2Pi` 集中在 `public/`，并提供单元测试。

### 5.4 转换函数

```c
/* module/motor/Inc/nerv_unit.h */

/**
  * @brief  将电机转速从 rpm 转换为 rad/s。
  * @param  rpm: 转速，单位 rpm。
  * @return 转速，单位 rad/s。
  * @note   仅允许在电机驱动层与标定工具中调用，上层一律使用 SI 单位。
  */
float NERV_Unit_RpmToRadps(float rpm);

/**
  * @brief  将角速度从 rad/s 转换为 rpm。
  * @param  radps: 角速度，单位 rad/s。
  * @return 转速，单位 rpm。
  */
float NERV_Unit_RadpsToRpm(float radps);
```

---

## 6. 命名规范

所有对外符号必须携带 `NERV_` 前缀与模块名，防止与 HAL、中间件或其他库冲突。

| 类别                   | 规则                                                                       | 示例                                           |
| ---------------------- | -------------------------------------------------------------------------- | ---------------------------------------------- |
| 公开函数               | `NERV_<Module>_<Verb><Noun>`                                               | `NERV_Chassis_Init`、`NERV_Gimbal_SetAngle`    |
| 初始化与注册           | `NERV_<Module>_Register` 返回实例指针                                      | `NERV_Chassis_Register`                        |
| 实例类型               | `NERV_<Module>_InstanceTypeDef`                                            | `NERV_Chassis_InstanceTypeDef`                 |
| 配置类型               | `NERV_<Module>_ConfigTypeDef`                                              | `NERV_Chassis_ConfigTypeDef`                   |
| 枚举类型               | `NERV_<Module>_<Name>TypeDef`                                              | `NERV_Chassis_TypeTypeDef`                     |
| 枚举成员               | `NERV_<MODULE>_<NAME>` 全大写，必须显式赋值                                | `NERV_CHASSIS_TYPE_MECANUM = 1U`               |
| 位掩码与预处理常量     | `NERV_<MODULE>_<NAME>` 全大写                                              | `NERV_ERR_INVALID_PARAM`                       |
| 用户可覆盖的默认值宏   | `NERV_<MODULE>_<NAME>`，必须用 `#ifndef` 与 `#endif` 包裹                  | `NERV_CHASSIS_UPDATE_PERIOD_MS`                |
| 用户禁止覆盖的推导宏   | `NERV_<MODULE>_<NAME>`，必须用 `#if defined(...)` 加 `#error` 保护         | `NERV_CHASSIS_WHEEL_COUNT`                     |
| 内部实现宏，不对外发布 | `__NERV_<MODULE>_<NAME>`，双下划线前缀，仅允许出现在 `*_priv.h` 或 `.c` 中 | `__NERV_CHASSIS_INTERNAL_UPDATE_PERIOD_MS`     |
| 用户可覆盖的弱函数     | `NERV_<Module>_On<Event>`，使用 `__weak` 修饰，函数名仍带 `NERV_` 前缀     | `NERV_Chassis_OnFault`                         |
| 内部函数               | `static` 修饰并采用同命名规则                                              | `static void NERV_Chassis_UpdateOdometry(...)` |
| 内部变量               | 小写下划线，带单位后缀                                                     | `chassis_cfg`、`speed_radps`、`dt_s`           |
| 函数指针成员           | 动词开头，语义简短                                                         | `init`、`update`、`set_ref`                    |

禁止事项：无前缀的全局符号；`err1`、`tmp`、`data` 这类无法表达含义的命名；单字母变量。循环计数器 `i` 在不超过 10 行的循环中可以使用。

---

## 7. 代码风格，对齐 HAL

### 7.1 缩进与大括号

- 使用 4 个空格缩进，禁止 Tab。
- 大括号独占一行，即使代码块只有一行。
- `if`、`for`、`while` 与括号之间保留一个空格。

```c
if (chassis == NULL)
{
    return NERV_ERROR;
}

for (uint8_t i = 0U; i < motor_num; i++)
{
    /* ... */
}
```

### 7.2 字面量后缀与常量在左的比较规则（强制）

**字面量后缀**：

- 无符号字面量加 `U` 后缀，例如 `0U`、`1U`、`0xFFU`。
- 浮点字面量显式加 `f` 后缀，例如 `0.0f`、`1.0f`。
- 长整型字面量加 `UL` 后缀，例如 `100000UL`。

**常量在左的相等比较（Yoda 条件，强制）**：

判断一个变量是否等于一个常量时，必须将常量写在比较运算符 `==` 与 `!=` 的左侧，将变量写在右侧。此规则的唯一目的是预防误将比较运算符 `==` 写成赋值运算符 `=`。赋值给常量在编译期即产生错误，而赋值给变量的写法会静默通过编译并改变变量值，是嵌入式系统中最难定位的缺陷之一。

```c
/* 反例：变量在左，误写为赋值时不会产生编译错误 */
if (status = NERV_OK)          { /* 永远为真，且修改了 status */ }
if (motor_id = NERV_MOTOR_ID_1) { /* 永远为真，且修改了 motor_id */ }
if (hchassis == NULL)          { /* 若误写为 = 则 NULL 是常量，编译期报错 */ }

/* 正例：常量在左，误写为赋值时编译期报错 */
if (NERV_OK == status)          { /* ... */ }
if (NERV_MOTOR_ID_1 == motor_id) { /* ... */ }
if (NULL == hchassis)          { /* ... */ }
```

**本规则不适用于以下场景**：

1. **不等式比较**：`<`、`>`、`<=`、`>=` 不适用常量在左规则。例如 `if (motor_count > NERV_CHASSIS_MAX_WHEEL_COUNT)` 保持原序，因为反转顺序为 `if (NERV_CHASSIS_MAX_WHEEL_COUNT < motor_count)` 会降低可读性，且不等式与赋值运算符在键盘上不相邻、不会混淆。

2. **布尔变量判断**：布尔变量直接使用 `if (is_ready)` 与 `if (!is_ready)`，禁止写 `if (true == is_ready)` 或 `if (false == is_ready)`。原因在于冗余比较掩盖了布尔语义。

3. **浮点数相等比较**：浮点数的 `==` 与 `!=` 本身属于编程错误，无论常量是否前置。浮点比较必须使用容差范围，调用 `NERV_Math_IsClose` 系列函数。禁用场景下常量前置规则无意义。

4. **位掩码测试**：使用 `if (0U != (fault & NERV_CHASSIS_ERR_KINEMATICS))` 形式时，被测试的位掩码常量位于表达式内部，比较对象为 `0U`。此时 `0U` 在左符合本规则，`fault & ...` 在右。

5. **`switch` 语句的 `case` 标签**：`case` 标签是编译期常量，不适用本规则。

### 7.3 函数长度与参数

- 单个函数原则上不超过 80 行，不含注释与空行。超出时拆分为 `static` 辅助函数。
- 参数不超过 6 个。超出时封装为配置结构体并传指针。
- 对外函数不得使用可变参数或隐式类型转换。日志宏为唯一例外，详见第 11 节。

### 7.4 Magic Number，强制禁止

- 禁止裸数字字面量，以下例外：`0U`、`1U`、`-1`、`NULL`、`true`、`false`、`'\0'`、`0.0f`、`1.0f`。
- 循环边界、数组大小、缓冲区长度、协议常量、寄存器值、超时时间必须使用命名常量。
- 硬件寄存器值放 `bsp/<name>/Inc/*_reg.h`。协议常量放 `module/<name>/Inc/*_def.h`。可调参数放 `config/Inc/config.h` 或 `module/<name>/Inc/*_default_param.h`。
- 超时参数统一使用显式单位宏，例如 `NERV_TIMEOUT_MS(5)`、`NERV_TIMEOUT_S(1)`，禁止裸 `5` 或 `5000`。
- 数组长度使用 `NERV_ARRAY_SIZE(array)` 宏，该宏定义于 `public/Inc/nerv_utils.h`，禁止写死数值。

```c
/* 反例 */
HAL_UART_Transmit(&huart1, buf, 128U, 5U);

/* 正例 */
HAL_UART_Transmit(NERV_LOG_UART_HANDLE, buf, NERV_LOG_MSG_MAX,
                  NERV_TIMEOUT_MS(NERV_LOG_TX_TIMEOUT_MS));
```

### 7.5 宏优先与枚举优先原则（强制）

在满足可读性与类型安全的前提下，能够通过宏或枚举在编译期解决的问题不使用函数在运行期解决。判定顺序如下：

1. **纯常量计算**：使用宏。典型场景包括超时换算 `NERV_TIMEOUT_MS(5)`、数组长度 `NERV_ARRAY_SIZE(arr)`、位掩码 `(1U << N)`。
2. **编译期条件分支**：使用 `#if` 与 `#ifdef` 在预处理阶段完成，而不是在运行期用 `if` 判断构型。典型场景是根据 `NERV_CHASSIS_TYPE` 选择运动学解算函数指针。
3. **零开销封装**：对 HAL 调用的薄封装，如果仅涉及重命名、参数校验或常量展开，优先使用宏。如果涉及返回状态码、多步资源管理或局部变量，则使用函数。
4. **预处理期常量集合**：必须能被 `#if` 引用的取值集合使用 `#define`。典型场景是 `config/Inc/config.h` 中的构型选择宏与 `NERV_LOG_LEVEL` 阈值宏，它们参与预处理分支判断，不能被 `enum` 替代。
5. **日志、断言与调试开关**：日志级别过滤、调试模式开关、编译期断言均在预处理阶段完成。
6. **类型化有限取值集合**：鼓励使用 `typedef enum` 而非一组 `#define`。适用于协议命令字、状态机状态、工作模式、故障类别等不需要参与 `#if` 判断的有限取值集合。枚举提供类型安全、调试器可视化、编译器对 `switch` 完整性检查的支持。

不应使用宏的场景：

- 需要返回值且调用者依赖返回值的逻辑。
- 需要局部变量的复杂逻辑，因为宏容易产生多次求值副作用。
- 需要 `static` 存储期或模块内部状态的逻辑。
- 参数可能包含副作用表达式，例如 `i++`、函数调用。
- 需要类型安全、需要调试器可视化、需要编译器对 `switch` 完整性给出提示的有限取值集合。
- 需要作为函数参数类型或结构体成员类型出现的有限取值集合。

### 7.6 枚举与宏的协作（强制）

同一个有限取值集合可能同时需要在预处理期使用与在运行期使用。此时采用“宏定义值，枚举镜像值”的协作模式，并保证两者数值一致。

- 预处理期必须使用 `#define`，因为 `#if` 无法引用 `enum` 成员。
- 运行期必须使用 `typedef enum`，以获得类型安全与调试器可视化。
- 两者数值必须一一对应，并在 `.c` 中使用 `#if` 或 `static_assert` 校验。
- 枚举成员必须显式赋值，禁止依赖隐式递增，保证固件版本之间数值稳定，便于遥测与日志对照。

```c
/* config/Inc/config.h */

/**
  * @brief 底盘运动学构型，供预处理期分支判断使用。
  * @note  必须且只能定义以下宏之一。
  * @note  运行期类型 NERV_Chassis_TypeTypeDef 必须与本组宏数值一致。
  */
#define NERV_CHASSIS_TYPE_MECANUM   1U
#define NERV_CHASSIS_TYPE_OMNI      2U
#define NERV_CHASSIS_TYPE_STEERING  3U

#ifndef NERV_CHASSIS_TYPE
#define NERV_CHASSIS_TYPE           NERV_CHASSIS_TYPE_MECANUM
#endif

#if (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_MECANUM) && \
    (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_OMNI)    && \
    (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_STEERING)
#error "NERV_CHASSIS_TYPE must be MECANUM, OMNI, or STEERING."
#endif
```

```c
/* module/chassis/Inc/nerv_chassis.h */

/**
  * @brief 底盘运动学构型，供运行期类型检查与调试器可视化使用。
  * @note  数值必须与 config/Inc/config.h 中的 NERV_CHASSIS_TYPE_xxx 宏一致。
  * @note  与 NERV_Chassis_Update 中注册的解算函数一一对应。
  */
typedef enum
{
    NERV_CHASSIS_TYPE_MECANUM_RT  = 1U,  /**< 麦克纳姆轮 */
    NERV_CHASSIS_TYPE_OMNI_RT     = 2U,  /**< 全向轮 */
    NERV_CHASSIS_TYPE_STEERING_RT = 3U,  /**< 舵轮 */
} NERV_Chassis_TypeTypeDef;
```

如果团队不希望维护 `_RT` 后缀的双份命名，替代方案是在 `.c` 中使用 `#if` 校验宏与枚举一致，枚举成员与宏同名，通过编译单元隔离避免冲突：

```c
/* module/chassis/Src/nerv_chassis.c */

#if (NERV_CHASSIS_TYPE_MECANUM != (int)NERV_CHASSIS_TYPE_MECANUM_RT)
#error "NERV_CHASSIS_TYPE_MECANUM and NERV_CHASSIS_TYPE_MECANUM_RT must be equal."
#endif
```

判定示例：

```c
/* 反例：运行期判断构型，浪费分支预测且强制所有解算代码进入固件 */
if (NERV_CHASSIS_TYPE_MECANUM == cfg->type)
{
    NERV_Mecanum_Solve(...);
}

/* 正例：编译期选择解算函数，未选中构型的代码不进入固件 */
#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)
    hchassis->solve = NERV_Mecanum_Solve;
#elif (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)
    hchassis->solve = NERV_Omni_Solve;
#elif (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_STEERING)
    hchassis->solve = NERV_Steering_Solve;
#else
#error "Unsupported NERV_CHASSIS_TYPE."
#endif
```

```c
/* 反例：协议命令字使用散落的宏，类型不安全且调试器不可视化 */
#define NERV_MOTOR_CMD_ENABLE   0x01U
#define NERV_MOTOR_CMD_DISABLE  0x02U
#define NERV_MOTOR_CMD_RESET    0x03U

void NERV_Motor_SendCommand(uint8_t cmd);

/* 正例：协议命令字使用枚举，类型安全且调试器可显示成员名 */
typedef enum
{
    NERV_MOTOR_CMD_ENABLE  = 0x01U,  /**< 使能电机 */
    NERV_MOTOR_CMD_DISABLE = 0x02U,  /**< 失能电机 */
    NERV_MOTOR_CMD_RESET   = 0x03U,  /**< 复位电机 */
} NERV_Motor_CmdTypeDef;

void NERV_Motor_SendCommand(NERV_Motor_CmdTypeDef cmd);
```

```c
/* 反例：函数仅返回常量，无运行时计算需求 */
uint32_t NERV_Chassis_GetMaxWheelCount(void)
{
    return 4U;
}

/* 正例：宏直接表达常量 */
#define NERV_CHASSIS_MAX_WHEEL_COUNT  4U
```

### 7.7 宏定义函数规范（强制）

宏定义函数的写法直接决定调用方代码的正确性与可维护性。NERV 对此做硬性约束。

#### 7.7.1 多语句宏必须使用 do { ... } while (0) 包裹

当宏展开体包含多条语句时，必须使用 `do { ... } while (0)` 包裹。原因在于调用方写法 `NERV_MOTOR_SET_SPEED(m, s)` 与 `NERV_MOTOR_SET_SPEED(m, s);` 语义不同。如果没有 `do { ... } while (0)` 包裹，在 `if` 分支未加括号的调用场景会产生错误，因为 `if` 只会绑定宏展开后的第一条语句，后续语句不受 `if` 控制，属于严重缺陷。

```c
/* 反例：多语句宏未使用 do { ... } while (0) 包裹 */
#define NERV_MOTOR_SET_SPEED(_motor, _speed) \
    (_motor).speed_radps = (_speed);         \
    (_motor).updated = 1U

/* 调用方场景 */
if (is_enabled)
    NERV_MOTOR_SET_SPEED(motor, 100.0f);
/* 展开后 motor.updated = 1U 不受 if 控制，is_enabled 为 false 时仍会执行 */
```

```c
/* 正例 */
#define NERV_MOTOR_SET_SPEED(_motor, _speed) \
    do {                                     \
        (_motor).speed_radps = (_speed);     \
        (_motor).updated = 1U;               \
    } while (0)
```

`do { ... } while (0)` 结构同时保证宏体内的 `break` 语义是退出宏而非退出外层循环，调用方在宏之后写分号也天然合法。

#### 7.7.2 单表达式宏使用括号包裹整个展开体

展开体是单一表达式时，使用外层括号包裹，不需要 `do { ... } while (0)`。判定条件为：展开体不含分号，不含局部变量，不改变控制流。

```c
/* 反例：单表达式宏未整体加括号，优先级错误 */
#define NERV_CLAMP_POSITIVE(_x) (_x) > 0.0f ? (_x) : 0.0f

/* 调用方 */
float y_mps = 10.0f * NERV_CLAMP_POSITIVE(v_mps);
/* 展开为 10.0f * (v_mps) > 0.0f ? (v_mps) : 0.0f，语义错误 */

/* 正例 */
#define NERV_CLAMP_POSITIVE(_x) (((_x) > 0.0f) ? (_x) : 0.0f)
```

#### 7.7.3 宏形参在展开体内每次出现都必须加括号

调用方可能传入表达式，例如 `NERV_MAX(a + b, c)`。形参不加括号时，宏体内部的运算符优先级会与调用方预期不符。

```c
/* 反例 */
#define NERV_MAX(_a, _b) ((_a) > (_b) ? (_a) : (_b))
int32_t result = NERV_MAX(x & 0x0FU, 1U);
/* 展开为 ((x & 0x0FU) > (1U) ? (x & 0x0FU) : (1U))，此处 (&) 优先级低于 (>)，
   如果形参未加括号，实际展开为 (x & 0x0FU > 1U ? ...)，语义错误 */

/* 正例：每个形参出现处都加括号 */
#define NERV_MAX(_a, _b) (((_a) > (_b)) ? (_a) : (_b))
```

#### 7.7.4 宏形参在展开体内不得多次求值

形参在展开体内出现多次时，调用方传入带副作用的实参会触发多次副作用。这是宏相对于函数最危险的缺陷。凡涉及多次求值的场景，改用 `static inline` 函数。

```c
/* 反例：形参多次求值 */
#define NERV_MAX(_a, _b) (((_a) > (_b)) ? (_a) : (_b))
int32_t result = NERV_MAX(i++, j++);  /* i 与 j 各被求值两次 */

/* 正例：改用 static inline 函数 */
static inline int32_t NERV_Math_MaxI32(int32_t a, int32_t b)
{
    return (a > b) ? a : b;
}
```

如果因类型泛化需求必须使用宏，则必须在 Doxygen 注释中以 `@warning` 明确禁止传入副作用实参，并在单元测试中覆盖该约束。

#### 7.7.5 宏内局部变量必须加 NERV 前缀与模块前缀

宏展开后局部变量进入调用方作用域，必须使用 `_nerv_<module>_<name>` 或 `_nerv_<name>` 形式的命名，避免与调用方同名变量冲突。

```c
/* 反例：宏内变量名与调用方常见变量名冲突 */
#define NERV_ABS(_x) \
    do {             \
        int32_t tmp; \
        tmp = (_x);  \
        if (tmp < 0) \
        {            \
            tmp = -tmp; \
        }            \
    } while (0)

int32_t tmp = get_sensor_value();
NERV_ABS(delta);
/* 展开后宏内 tmp 遮蔽外层 tmp，外层逻辑失效 */

/* 正例 */
#define NERV_ABS(_x)                 \
    do {                             \
        int32_t _nerv_abs_tmp;       \
        _nerv_abs_tmp = (_x);        \
        if (_nerv_abs_tmp < 0)       \
        {                            \
            _nerv_abs_tmp = -_nerv_abs_tmp; \
        }                            \
    } while (0)
```

对于极短宏，可以使用 `##__COUNTER__` 拼接生成唯一变量名，例如 `_nerv_tmp_ ## __COUNTER__`。此写法依赖编译器扩展，AC6 与 arm-none-eabi-gcc 均支持，但 NERV 优先推荐显式模块前缀命名，因为 `##__COUNTER__` 会降低宏的可读性并增加调试难度。

#### 7.7.6 宏不得改变调用方控制流

宏定义体内禁止包含 `return`、`goto`、`continue` 这类改变调用方控制流的语句。`break` 在 `do { ... } while (0)` 结构内是安全的，因为 `break` 只退出宏体。如果业务逻辑需要提前返回，改用 `static inline` 函数并由调用方显式检查返回值。

```c
/* 反例：宏内使用 return 改变调用方控制流 */
#define NERV_CHECK_NULL(_p) \
    do {                    \
        if (NULL == (_p))   \
        {                   \
            return NERV_ERROR; \
        }                   \
    } while (0)

/* 调用方：宏内 return 直接跳出调用方函数，语义与宏名称不符，
   且调用方无法在宏返回前执行清理动作 */

/* 正例：改用 static inline 函数 */
static inline NERV_StatusTypeDef NERV_Ptr_CheckNotNull(const void *p)
{
    return (NULL == p) ? NERV_ERROR : NERV_OK;
}

/* 调用方 */
if (NERV_OK != NERV_Ptr_CheckNotNull(hchassis))
{
    goto out;
}
```

#### 7.7.7 正例汇总

第 11.3 节中的 `NERV_LOG_FORMAT_AND_PUSH` 是满足全部规则的范例：使用 `do { ... } while (0)` 包裹，局部变量加 `_nerv_log_` 前缀，形参在展开体内加括号，宏体内不含 `return` 与 `break`。

`public/Inc/nerv_utils.h` 中的单表达式宏同样是范例：`NERV_ARRAY_SIZE(arr)`、`NERV_ALIGN_UP(x, align)`、`NERV_TIMEOUT_MS(x)` 均使用外层括号包裹，形参每次都加括号，判定为单表达式因此不需要 `do { ... } while (0)`。

#### 7.7.8 宏定义函数的单元测试

所有多语句宏必须在 PC 环境执行单元测试，覆盖以下两类调用场景：

1. 宏调用后加分号 `NERV_LOG_INFO("...");` 与不加分号 `NERV_LOG_INFO("...")`。
2. 宏调用置于无大括号的 `if`、`for`、`while` 语句内与置于大括号代码块内。

单元测试使用 Unity 或 Ceedling，测试用例集中在 `tests/` 目录，与 CI 集成。

### 7.8 宏的保护与命名空间隔离（强制）

NERV 中存在一类宏，一旦被无意重命名、覆盖或删除，会导致难以定位的缺陷。此类宏必须采取保护措施。

#### 7.8.1 判定标准

以下类型的宏属于“很怕被无意重命名”的范畴，必须采取保护措施：

1. 影响硬件行为的宏，包括引脚定义、时序参数、协议常量、寄存器位定义。
2. 影响安全性的宏，包括电压阈值、电流限幅、温度保护阈值、看门狗超时。
3. 影响调试可观测性的宏，包括日志开关、日志级别、断言开关。
4. 被多处代码引用的关键配置宏，包括底盘构型、模块启用开关、任务优先级。
5. 与编译器、标准库、RTOS 交互的宏，包括中断优先级、堆栈大小、内存对齐。
6. 内部实现细节宏，用户不应感知也不应依赖的宏。

此类宏被误改后的典型后果包括硬件损坏、调试能力丧失、协议失配、偶发故障。

#### 7.8.2 三层保护策略

NERV 采用三层保护策略，按强度从弱到强排列：

1. **命名空间隔离**：通过命名约定将宏分为公开、用户可覆盖、内部实现三类，避免命名冲突。
2. **防重定义包裹**：通过 `#ifndef` 与 `#define` 与 `#endif`，或 `#if defined(...)` 加 `#error`，阻止无意覆盖。
3. **编译期强制检查**：通过 `#error` 在编译期拒绝非法配置。

#### 7.8.3 公开宏与内部宏的命名空间隔离

公开宏以 `NERV_<MODULE>_<NAME>` 形式命名，允许被用户在 `config/Inc/config.h` 中覆盖。

内部实现宏以 `__NERV_<MODULE>_<NAME>` 形式命名，采用双下划线前缀。C11 标准 §7.1.3 规定，以下划线开头且后跟大写字母或另一个下划线的标识符，在所有作用域中都保留给实现。NERV 作为库作者是“实现”的一部分，可以使用此命名空间。用户代码不应定义 `__` 前缀的标识符，因此 `__NERV_` 前缀天然与用户代码隔离。此约定与 Linux 内核的实践一致。

`__NERV_` 前缀的宏仅允许出现在 `*_priv.h` 或 `.c` 文件中，禁止出现在公开头文件中。公开头文件只暴露 `NERV_<MODULE>_<NAME>` 形式的宏。

```c
/* module/chassis/Inc/nerv_chassis_priv.h */

/**
  * @brief 内部实现使用的底盘更新周期。
  * @note  本宏为内部实现细节，禁止在 NERV 库外部引用或定义。
  */
#define __NERV_CHASSIS_INTERNAL_UPDATE_PERIOD_MS  10U
```

#### 7.8.4 用户可覆盖的默认值宏

用户允许在 `config/Inc/config.h` 中覆盖默认值的宏，使用 `#ifndef` 与 `#define` 与 `#endif` 包裹，并在包裹后追加编译期范围检查。

```c
/* config/Inc/config.h */

/**
  * @brief 底盘控制任务周期，单位为毫秒。
  * @note  用户可在本文件之外定义同名宏以覆盖默认值。
  * @note  合法范围为 1 至 100，超出范围将在编译期被 #error 拒绝。
  */
#ifndef NERV_CHASSIS_UPDATE_PERIOD_MS
#define NERV_CHASSIS_UPDATE_PERIOD_MS  10U
#endif

#if (NERV_CHASSIS_UPDATE_PERIOD_MS < 1U) || (NERV_CHASSIS_UPDATE_PERIOD_MS > 100U)
#error "NERV_CHASSIS_UPDATE_PERIOD_MS must be in range 1 to 100."
#endif
```

#### 7.8.5 用户禁止覆盖的推导宏

由 NERV 内部推导得出的宏，用户禁止定义或覆盖。此类宏使用 `#if defined(...)` 加 `#error` 保护，在用户尝试定义时立即产生编译错误。

```c
/* module/chassis/Inc/nerv_chassis_def.h */

/**
  * @brief 底盘轮组数量，由 NERV_CHASSIS_TYPE 推导得出。
  * @note  本宏禁止用户定义。用户尝试定义将在编译期被 #error 拒绝。
  */
#if defined(NERV_CHASSIS_WHEEL_COUNT)
#error "NERV_CHASSIS_WHEEL_COUNT is derived from NERV_CHASSIS_TYPE and must not be defined by the user."
#endif

#if (NERV_CHASSIS_TYPE_MECANUM == NERV_CHASSIS_TYPE)
#define NERV_CHASSIS_WHEEL_COUNT  4U
#elif (NERV_CHASSIS_TYPE_OMNI == NERV_CHASSIS_TYPE)
#define NERV_CHASSIS_WHEEL_COUNT  3U
#elif (NERV_CHASSIS_TYPE_STEERING == NERV_CHASSIS_TYPE)
#define NERV_CHASSIS_WHEEL_COUNT  4U
#else
#error "Unsupported NERV_CHASSIS_TYPE."
#endif
```

#### 7.8.6 防误删与防误改的注释包裹

对关键宏定义区，使用注释块显式标注“禁止修改”，提醒 IDE 自动重构工具与协作者。

```c
/* module/chassis/Inc/nerv_chassis_def.h */

/* ========================================================================== */
/* 以下宏定义区为 NERV 库核心契约，禁止修改。                                 */
/* 修改必须同步更新本文档、对应单元测试与遥测协议定义。                       */
/* ========================================================================== */

#define __NERV_CHASSIS_INTERNAL_MAX_WHEEL_COUNT  4U
#define __NERV_CHASSIS_INTERNAL_MIN_RADIUS_M     0.01f

/* ========================================================================== */
/* 核心契约宏定义区结束                                                       */
/* ========================================================================== */
```

#### 7.8.7 防止第三方库覆盖的兼容处理

当第三方库头文件可能定义与 NERV 同名宏时，在 NERV 头文件顶部保存原值，在底部恢复。

```c
/* bsp/uart/Inc/nerv_uart.h */

#ifdef NERV_UART_BAUDRATE
#pragma push_macro("NERV_UART_BAUDRATE")
#undef NERV_UART_BAUDRATE
#endif

/* 第三方库头文件包含区 */

#ifdef NERV_UART_BAUDRATE
#pragma pop_macro("NERV_UART_BAUDRATE")
#endif
```

此写法依赖 `#pragma push_macro` 与 `#pragma pop_macro`，AC6 与 arm-none-eabi-gcc 均支持。NERV 内部头文件之间不需要使用此机制，仅在 NERV 头文件包含第三方库头文件的场景使用。

#### 7.8.8 反例与正例汇总

```c
/* 反例一：用户可覆盖的默认值宏未使用 #ifndef 包裹，用户定义同名宏后触发重定义警告 */
#define NERV_CHASSIS_UPDATE_PERIOD_MS  10U

/* 反例二：内部实现宏使用了公开命名规则，被用户无意覆盖后难以定位 */
#define NERV_CHASSIS_TEMP_CALC_COEF  0.5f

/* 反例三：推导得出的宏未使用 #if defined(...) 保护，用户定义同名宏后产生静默覆盖 */
#define NERV_CHASSIS_WHEEL_COUNT  4U
```

```c
/* 正例一：用户可覆盖的默认值宏 */
#ifndef NERV_CHASSIS_UPDATE_PERIOD_MS
#define NERV_CHASSIS_UPDATE_PERIOD_MS  10U
#endif

/* 正例二：内部实现宏使用 __NERV_ 前缀 */
#define __NERV_CHASSIS_INTERNAL_TEMP_CALC_COEF  0.5f

/* 正例三：推导得出的宏使用 #if defined(...) 保护 */
#if defined(NERV_CHASSIS_WHEEL_COUNT)
#error "NERV_CHASSIS_WHEEL_COUNT must not be defined by the user."
#endif
```

### 7.9 弱函数（`__weak`）规范（强制）

除宏之外，NERV 还通过 `__weak` 修饰符提供函数级别的可覆盖点。本节规定弱函数的适用场景、命名、默认实现与调用约束。

#### 7.9.1 `__weak` 是修饰符而非命名前缀

`__weak` 是编译器属性，由 CMSIS 在 `cmsis_compiler.h` 中定义，GCC 与 AC6 展开为 `__attribute__((weak))`。它表示该符号为弱符号，链接阶段允许用户在其他编译单元中定义同名强符号覆盖。

`__weak` 必须写在返回类型之前，与 HAL 的写法一致：

```c
__weak void NERV_Chassis_OnFault(uint32_t fault_mask);
```

弱函数的名称仍然遵循第 6 节的命名规范，以 `NERV_<Module>_` 开头。**不得**使用 `__weak_` 或 `weak_` 之类的命名前缀。命名前缀与修饰符是两件事：修饰符决定链接行为，命名决定符号归属。

#### 7.9.2 与第 7.8 节宏保护的关系

第 7.8 节解决“宏被无意重命名或覆盖”的问题。第 7.9 节解决“函数被用户扩展或覆盖”的问题。两者互补：

- 宏用于编译期常量与条件分支，通过命名空间与 `#ifndef` 保护。
- 弱函数用于运行期回调与钩子，通过 `__weak` 修饰符与默认实现保护。

#### 7.9.3 适用场景

弱函数仅用于以下场景：

1. 模块级事件通知，例如故障发生、状态迁移、周期完成。
2. 用户可选的扩展钩子，例如日志发送前的过滤、底盘解算后的修正。
3. 需要在编译期不依赖具体实现即可链接的回调点，例如用户未定义时使用空默认实现。

弱函数**不得**用于以下场景：

1. 关键资源管理，包括过滤器配置、内存分配、外设初始化。此类逻辑必须由 NERV 内部实现，用户不得覆盖。
2. 协议编解码核心路径，因为用户覆盖会破坏与其他模块的契约。
3. 中断上下文与任务上下文混用的逻辑，因为默认实现无法同时满足两种上下文的约束。
4. 返回值被 NERV 内部依赖的逻辑，因为用户覆盖可能返回未定义值。

#### 7.9.4 命名规则

弱函数名称采用 `NERV_<Module>_On<Event>` 或 `NERV_<Module>_<HookName>` 形式：

- `On` 前缀用于事件通知，例如 `NERV_Chassis_OnFault`、`NERV_Gimbal_OnLimitReached`。
- 动词前缀用于钩子，例如 `NERV_Log_BeforeTransmit`、`NERV_Chassis_AfterSolve`。

弱函数必须携带 `NERV_` 前缀与模块名，与普通公开函数一致。

#### 7.9.5 声明与默认实现的位置

- **声明**放在模块公开头文件 `Inc/nerv_<module>.h` 中，因为用户需要包含该头文件才能在自己的 `.c` 中定义同名强符号。
- **默认实现**放在模块源文件 `Src/nerv_<module>.c` 中，使用 `__weak` 修饰，提供安全、无副作用的空实现或最简行为。
- 默认实现**不得**依赖任何未初始化的资源，**不得**调用 RTOS 阻塞 API，**不得**进入死循环。

```c
/* module/chassis/Inc/nerv_chassis.h */
/** @brief 底盘故障钩子，用户可覆盖，默认实现为空。 */
void NERV_Chassis_OnFault(uint32_t fault_mask);

/* 注意：声明处不写 __weak。__weak 只出现在定义处。
   这样调用方在任何编译单元看到的都是普通函数声明，
   链接器根据定义处的弱符号属性决定是否被用户覆盖。 */
```

```c
/* module/chassis/Src/nerv_chassis.c */

/**
  * @brief  底盘故障钩子，在检测到故障时被调用。
  * @param  fault_mask: 故障位掩码，合法成员见 NERV_CHASSIS_ERR_xxx 宏。
  * @note   本函数为弱函数，用户可在自己的 .c 文件中定义同名函数覆盖默认实现。
  * @note   默认实现为空，不执行任何动作。
  * @note   NOT ISR-safe，在 chassis_task 上下文中调用。
  * @note   用户覆盖时函数签名必须完全一致，包括返回类型与参数类型。
  */
__weak void NERV_Chassis_OnFault(uint32_t fault_mask)
{
    (void)fault_mask;
}
```

#### 7.9.6 用户覆盖方法

用户在自己的源文件中定义与声明签名完全一致的同名函数，**不加** `__weak`：

```c
/* 用户代码，例如 app/Src/chassis_task.c 或独立文件 */

#include "nerv_chassis.h"

/**
  * @brief 底盘故障钩子，用户覆盖版本。
  * @param fault_mask: 故障位掩码。
  * @note  在 chassis_task 上下文中调用。
  */
void NERV_Chassis_OnFault(uint32_t fault_mask)
{
    if (0U != (fault_mask & NERV_CHASSIS_ERR_WHEEL_OFFLINE))
    {
        NERV_LOG_ERROR("chassis wheel offline, fault=0x%08lX",
                       (unsigned long)fault_mask);
    }
}
```

链接器在链接阶段选择强符号，`__weak` 默认实现被丢弃。如果用户未定义同名函数，链接器保留 `__weak` 默认实现，功能正常但无用户扩展行为。

#### 7.9.7 线程模型与 ISR 安全

弱函数必须在 Doxygen 注释中标注线程模型，使用第 12.4 节的四类之一。默认实现必须与该线程模型一致：

- 如果弱函数标注 `@note NOT ISR-safe`，默认实现不得使用任何可能在 ISR 中不安全的操作。
- 如果弱函数标注 `@note ISR-safe`，默认实现必须保持无阻塞、无动态内存、无 RTOS 调用。

NERV 内部调用弱函数的代码必须在调用点保证线程模型匹配。例如，`NERV_Chassis_OnFault` 在 `chassis_task` 中调用，因此标注 `NOT ISR-safe`。如果需要在 ISR 中通知用户，采用第 4 节的错误标志位方案，由低频任务检测后调用弱函数。

#### 7.9.8 默认实现的约束

默认实现必须遵循以下约束：

1. **空实现或最简行为**：默认实现通常为空，参数以 `(void)参数名;` 显式忽略，避免编译器产生未使用参数警告。
2. **无副作用**：默认实现不得修改模块状态、不得发送数据、不得操作外设。
3. **无返回值依赖**：弱函数**不得**返回 `NERV_StatusTypeDef` 或其他被 NERV 内部依赖的值。如果确实需要返回值，返回固定常量并由 NERV 内部忽略该返回值，在 Doxygen 注释中明确说明。
4. **无阻塞**：默认实现不得调用 `osDelay`、`HAL_Delay`、`HAL_UART_Transmit` 等阻塞函数。
5. **无动态内存**：默认实现不得调用 `malloc`、`free`。
6. **无递归**：默认实现不得递归调用自身或其他弱函数。

#### 7.9.9 弱函数与单元测试

弱函数必须在 PC 环境执行单元测试，覆盖两类场景：

1. 用户未定义同名强符号时，`__weak` 默认实现被调用，功能正常。
2. 用户定义同名强符号时，强符号被调用，默认实现被丢弃。

单元测试通过链接两个不同的测试可执行文件实现：一个不包含用户定义，一个包含用户定义。测试用例集中在 `tests/` 目录，与 CI 集成。

#### 7.9.10 反例与正例汇总

```c
/* 反例一：弱函数使用 __weak_ 命名前缀，与修饰符混淆 */
__weak void __weak_Chassis_OnFault(uint32_t fault_mask);

/* 反例二：声明处写 __weak，导致用户包含头文件后看到的声明也是弱符号，
   某些链接器配置下会产生不可预测的行为 */
/* nerv_chassis.h */
__weak void NERV_Chassis_OnFault(uint32_t fault_mask);

/* 反例三：弱函数返回 NERV_StatusTypeDef，NERV 内部依赖返回值，
   用户覆盖后返回未定义值导致内部逻辑失效 */
__weak NERV_StatusTypeDef NERV_Chassis_OnFault(uint32_t fault_mask);

/* 反例四：默认实现包含阻塞调用，用户未覆盖时在控制路径中引入不确定延迟 */
__weak void NERV_Chassis_OnFault(uint32_t fault_mask)
{
    HAL_UART_Transmit(NERV_LOG_UART_HANDLE, (uint8_t *)"fault\r\n", 7U, 100U);
}

/* 反例五：弱函数用于关键资源管理，用户覆盖后破坏过滤器配置 */
__weak NERV_StatusTypeDef NERV_FDCAN_ConfigureFilter(void);
```

```c
/* 正例一：声明处不写 __weak，定义处写 __weak */
/* nerv_chassis.h */
void NERV_Chassis_OnFault(uint32_t fault_mask);

/* nerv_chassis.c */
__weak void NERV_Chassis_OnFault(uint32_t fault_mask)
{
    (void)fault_mask;
}

/* 正例二：弱函数无返回值，默认实现为空 */
__weak void NERV_Gimbal_OnLimitReached(float angle_rad)
{
    (void)angle_rad;
}

/* 正例三：弱函数标注线程模型与用户覆盖方法 */
/**
  * @brief  日志发送前钩子，在 HAL_UART_Transmit 之前被调用。
  * @param  p_item: 待发送日志元素指针，用户可读取但不得修改。
  * @note   本函数为弱函数，用户可覆盖以在发送前执行过滤或统计。
  * @note   默认实现为空。
  * @note   NOT ISR-safe，在 nerv_log 任务中调用。
  */
__weak void NERV_Log_BeforeTransmit(const NERV_Log_ItemTypeDef *p_item)
{
    (void)p_item;
}
```

---

## 8. 注释规范

### 8.1 注释总量原则

代码应替代注释本身。如果一段逻辑需要行内注释才能看懂，优先重构代码，手段包括提取函数、重命名变量、使用查表替代分支。注释只回答两类问题：

1. 设计原因，涵盖约束、硬件限制、协议特殊要求。
2. 调用者需要知道的信息，涵盖合法取值、前置条件、副作用、线程与中断上下文安全性。

禁止使用注释解释代码正在做什么。

### 8.2 注释语言规范（强制）

注释内容必须科学、准确、详细，具体要求如下：

1. **禁止使用 `+` 表示并列关系**。并列关系使用“与”“和”“及”或分号表达。例如：
   - 反例：`宏驱动日志 + FreeRTOS 异步输出任务`
   - 正例：`宏驱动日志与 FreeRTOS 异步输出任务`

2. **禁止使用英文括号 `()` 表示解释或补充**。解释性内容融入主句，或使用“即”“也就是”“具体而言”等连接词引出。例如：
   - 反例：`任务栈大小（字节，CMSIS-OS 语义下由 osThreadAttr_t 使用）`
   - 正例：`任务栈大小，单位为字节，CMSIS-OS 语义下由 osThreadAttr_t 使用`

3. **允许保留的括号场景**：函数名引用、宏名引用、标准缩写 `ARMCLANG`、数学表达式 `(1U << N)`、代码片段引用。这些括号属于技术符号，不视为解释性括号。

4. **禁止使用模糊表述**。例如“可能”“大概”“一般情况下”在注释中均属不合格，因为注释目的是传递确定信息。若行为不确定，应改为在代码层面消除不确定性，或明确说明触发条件。

5. **单位必须显式声明**。涉及物理量的注释必须写明单位，例如 `@param rpm: 转速，单位 rpm`，禁止写 `@param rpm: 转速`。

6. **前置条件必须可验证**。例如 `@note 必须在 FDCAN 启动之后调用` 可验证；`@note 应在合适的时机调用` 不可验证，禁止使用。

7. **禁止使用 emoji 与颜文字**。注释、字符串常量、日志格式串、提交信息、文档中均不得出现 emoji 与颜文字。原因在于不同工具链、终端、版本控制系统对 Unicode 扩展字符的渲染与编码处理不一致，且 emoji 无法表达确定的技术语义。需要表达强调时使用文字描述，例如“必须”“禁止”“注意”。需要表达状态时使用枚举或状态码。

### 8.3 Doxygen 注释的位置规则（强制）

NERV 采用“完整文档随定义走，接口保持整洁”的策略：

- 完整 Doxygen 块包含 `@brief`、`@param`、`@retval`、`@note`、`@warning`、`@see`，必须写在 `.c` 文件的函数定义处。
- `.h` 文件中只放函数声明。`extern "C"` 块内默认不写 Doxygen 注释；确有必要时允许且仅允许一行 `/** @brief 一句话说明 */`，禁止在 `.h` 中书写 `@param`、`@retval`、`@note`。
- 宏、枚举、结构体成员的注释跟随其定义位置。定义在 `.h` 中的注释写在 `.h`，定义在 `.c` 中的私有类型注释写在 `.c`。
- `static` 内部函数的完整 Doxygen 块同样写在定义处；如果仅为设计原因说明，可以只保留 `@brief`。
- 宏定义函数的 Doxygen 注释中必须用 `@warning` 注明“不得传入副作用实参”，参见第 7.7.4 节。
- 属于第 7.8.1 节判定标准的“很怕被无意重命名”的宏，必须在 Doxygen 注释中明确说明“禁止用户定义或覆盖”。
- 弱函数的 Doxygen 注释必须写在 `.c` 的默认实现处，用 `@note` 说明“本函数为弱函数，用户可覆盖”，参见第 7.9.5 节。

Doxygen 配置要求：`INPUT` 必须同时包含 `*.h` 与 `*.c`。若 `WARN_IF_UNDOCUMENTED = YES`，配合 `EXTRACT_ALL = YES` 或在 `.h` 中保留一行 `@brief` 以避免告警。

### 8.4 函数注释模板，写在 `.c` 定义处

```c
/* module/chassis/Src/nerv_chassis.c */

/**
  * @brief  初始化底盘模块。
  * @param  hchassis: 底盘实例指针，不得为 NULL。
  * @param  p_config: 底盘配置指针，生命周期须覆盖实例生命周期。
  * @retval NERV_OK: 初始化成功。
  * @retval NERV_ERROR: 配置非法或底层初始化失败。
  * @retval NERV_TIMEOUT: 底层外设初始化超时。
  * @note   必须在 FDCAN 启动之后调用。
  * @note   只能在 osKernelStart 之后的任务中调用。
  * @note   chassis_type 的合法值见 NERV_Chassis_TypeTypeDef。
  */
NERV_StatusTypeDef NERV_Chassis_Init(NERV_Chassis_InstanceTypeDef *hchassis,
                                      const NERV_Chassis_ConfigTypeDef *p_config)
{
    /* ... */
}
```

对应 `.h`：

```c
/* module/chassis/Inc/nerv_chassis.h */
/** @brief 初始化底盘模块，完整契约见 nerv_chassis.c。 */
NERV_StatusTypeDef NERV_Chassis_Init(NERV_Chassis_InstanceTypeDef *hchassis,
                                      const NERV_Chassis_ConfigTypeDef *p_config);
```

常用标签：

| 标签                                              | 用途                                          |
| ------------------------------------------------- | --------------------------------------------- |
| `@brief`                                          | 一句话功能描述                                |
| `@param[in]` 与 `@param[out]` 与 `@param[in,out]` | 参数方向                                      |
| `@retval`                                         | 枚举或宏返回值的具体含义，多个 `@retval` 并列 |
| `@return`                                         | 非枚举返回值，如指针或数值                    |
| `@note`                                           | 上下文约束、合法取值、线程安全性              |
| `@warning`                                        | 可能导致未定义行为或硬件损坏的条件            |
| `@see`                                            | 关联函数、类型或宏                            |

### 8.5 宏、枚举、结构体成员的注释

宏和枚举必须用 `@brief` 说明用途。如果该宏或枚举成员是一组合法取值之一，须在 `@note` 中列出全部关联成员。结构体成员使用 `/**< ... */` 行尾注释。注释跟随定义位置。

```c
/* module/chassis/Inc/nerv_chassis.h */

/**
  * @brief 底盘运动学构型，供运行期类型检查与调试器可视化使用。
  * @note  合法成员为 NERV_CHASSIS_TYPE_MECANUM_RT、NERV_CHASSIS_TYPE_OMNI_RT、
  *        NERV_CHASSIS_TYPE_STEERING_RT。
  * @note  数值必须与 config/Inc/config.h 中的同名宏一致。
  */
typedef enum
{
    NERV_CHASSIS_TYPE_MECANUM_RT  = 1U,  /**< 麦克纳姆轮 */
    NERV_CHASSIS_TYPE_OMNI_RT     = 2U,  /**< 全向轮 */
    NERV_CHASSIS_TYPE_STEERING_RT = 3U,  /**< 舵轮 */
} NERV_Chassis_TypeTypeDef;
```

### 8.6 行内注释

行内注释仅允许出现在以下场景，且必须使用 `/* ... */` 而不是 `//`：

- 标记硬件勘误或 AC6 特殊语法。
- 标记 `goto` 清理逻辑的边界。
- 说明魔数来源。魔数本身仍须替换为命名常量，行内注释只解释该常量取值的物理来源。
- 标记 `do { ... } while (0)` 的结构意图，例如在复杂宏体旁注明 `/* 宏体使用 do-while(0) 防止 if 无括号调用错误 */`。
- 标记关键宏定义区的“禁止修改”边界，参见第 7.8.6 节。
- 标记弱函数定义处的可覆盖意图，例如在 `__weak` 定义旁注明 `/* 弱函数，用户可覆盖默认实现 */`。

---

## 9. 用 C 模拟面向对象

### 9.1 封装

数据与操作放在同一结构体中。方法用函数指针表达，调用时把实例指针作为第一个参数，模拟 `this`。

```c
typedef struct NERV_Chassis_Instance
{
    NERV_Chassis_ConfigTypeDef  config;
    NERV_StatusTypeDef        (*init)(struct NERV_Chassis_Instance *self);
    NERV_StatusTypeDef        (*update)(struct NERV_Chassis_Instance *self,
                                         const NERV_VelocityTypeDef *target);
    void                      (*get_odometry)(const struct NERV_Chassis_Instance *self,
                                              NERV_OdometryTypeDef *odom);
    void                       *priv;  /**< 指向具体构型的私有数据，不得跨模块访问 */
} NERV_Chassis_InstanceTypeDef;
```

### 9.2 继承

基类结构体必须是派生结构体的第一个成员。通过将派生结构体指针转换为基类指针实现向上转型。

```c
typedef struct
{
    NERV_Chassis_InstanceTypeDef base;  /* 必须是第一个成员 */
    NERV_Mecanum_PrivDataTypeDef mecanum;
} NERV_Mecanum_InstanceTypeDef;
```

### 9.3 多态

优先使用条件编译在编译期解析多态。底盘构型在固件构建时已经确定，运行期切换既无收益又增加开销。`config/Inc/config.h` 选择构型，`NERV_Chassis_Init` 根据宏挂载对应的函数指针。此处的选择遵循第 7.5 节的宏优先原则。

只有在需要运行期切换构型的场景，例如调试工具在单个固件中切换构型，才使用函数指针表进行动态分派。此时构型标识使用第 7.6 节定义的枚举类型。

### 9.4 实例化接口

所有模块统一采用配置、注册、实例指针三段式模式，禁止使用全局单例：

```c
NERV_Chassis_InstanceTypeDef *NERV_Chassis_Register(
        const NERV_Chassis_ConfigTypeDef *p_config);
```

返回的实例指针由调用者持有并传入所有后续操作。模块内部不得依赖静态全局实例。

### 9.5 回调、钩子与弱函数的选择

NERV 提供三种用户扩展机制，按使用场景选择：

| 机制             | 适用场景                                         | 生命周期     | 用户定义方式                          |
| ---------------- | ------------------------------------------------ | ------------ | ------------------------------------- |
| 注册回调函数指针 | 运行期可变的、多实例的、需要与特定实例绑定的回调 | 注册后可注销 | 调用 `NERV_<Module>_RegisterCallback` |
| 弱函数           | 模块级全局事件通知，用户可选覆盖                 | 链接期确定   | 定义同名强符号                        |
| 宏钩子           | 编译期可裁剪的扩展点                             | 编译期确定   | 定义配置宏                            |

回调函数指针适用于 `instance` 绑定的场景，例如某个底盘实例的接收回调。弱函数适用于模块级全局场景，例如故障通知。宏钩子适用于编译期开关场景，例如是否启用某段调试代码。三种机制不得混用：同一扩展点只能选一种，避免用户困惑。

---

## 10. `config/`，编译期配置入口

### 10.1 唯一入口

`config/Inc/config.h` 是唯一的编译期配置入口。所有硬件差异、构型选择、功能裁剪、日志开关都在此处以宏表达。禁止在模块源文件中出现 `#ifdef STM32H7`、`#ifdef BOARD_C` 等板级宏判断。

RTOS 相关的任务优先级与栈大小集中在 `config/Inc/nerv_rtos_config.h`，参见第 12 节。

`config/Inc/config.h` 中的宏分为两类，分别遵循第 7.8.4 节与第 7.8.5 节的保护规则：

- 用户可覆盖的默认值宏，使用 `#ifndef` 与 `#define` 与 `#endif` 包裹，并在包裹后追加范围检查。
- 由用户选择推导出的强制宏，使用 `#if defined(...)` 加 `#error` 保护。

### 10.2 底盘构型示例

此处的宏服务于预处理期分支判断，运行期枚举镜像值定义在 `module/chassis/Inc/nerv_chassis.h`，两者必须一致，协作规则见第 7.6 节。

```c
/* config/Inc/config.h */

/**
  * @brief 底盘运动学构型选择，供预处理期分支判断使用。
  * @note  必须且只能定义以下宏之一。
  * @note  运行期类型 NERV_Chassis_TypeTypeDef 必须与本组宏数值一致。
  */
#define NERV_CHASSIS_TYPE_MECANUM   1U
#define NERV_CHASSIS_TYPE_OMNI      2U
#define NERV_CHASSIS_TYPE_STEERING  3U

#ifndef NERV_CHASSIS_TYPE
#define NERV_CHASSIS_TYPE           NERV_CHASSIS_TYPE_MECANUM
#endif

#if (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_MECANUM) && \
    (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_OMNI)    && \
    (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_STEERING)
#error "NERV_CHASSIS_TYPE must be MECANUM, OMNI, or STEERING."
#endif
```

### 10.3 功能裁剪

可裁剪模块用 `NERV_USE_<MODULE>` 宏控制，默认值须以 `#ifndef` 方式声明：

```c
#ifndef NERV_USE_GIMBAL
#define NERV_USE_GIMBAL             1
#endif
```

---

## 11. 日志系统，宏驱动与 FreeRTOS 最低优先级任务

### 11.1 设计原则

- 日志的全部行为由宏驱动，包括是否启用、输出级别、串口句柄、缓冲区大小、队列深度，均由 `config/Inc/config.h` 中的宏决定。
- 非 Debug 模式下，所有 `NERV_LOG_xxx` 宏展开为 `((void)0)`，运行时开销为零。日志实现文件整体被 `#if` 排除，Release 构建不产生任何符号与字符串。
- 日志宏与日志实现属于 `log/` 层，`log/Inc/nerv_log.h` 与 `log/Src/nerv_log.c` 是唯一入口。任何模块不得直接包含串口 HAL 头文件来打印日志。
- 日志串口句柄通过宏 `NERV_LOG_UART_HANDLE` 注入，日志实现中禁止出现 `&huart1` 之类的具体句柄字面量。
- 日志输出运行在独立的 CMSIS-OS v2 任务中，任务优先级为全局最低。日志宏只负责格式化并入队，不阻塞调用者。队列满时静默丢弃，绝不等待。
- 日志级别阈值参与 `#if` 判断，必须使用 `#define`。日志级别同时提供枚举 `NERV_Log_LevelTypeDef`，用于队列元素字段与函数参数，协作规则见第 7.6 节。
- 日志宏内部的 `##__VA_ARGS__` 是 GNU 扩展，AC6 与 arm-none-eabi-gcc 均支持，编译标准须为 `gnu99` 或更高。
- 日志宏的宏体必须遵循第 7.7 节的全部规则，包括 `do { ... } while (0)` 包裹、局部变量加 `_nerv_log_` 前缀、形参加括号。
- 日志开关与日志级别属于第 7.8.1 节判定标准的关键宏，必须遵循第 7.8.4 节的 `#ifndef` 包裹规则。
- 日志发送前钩子 `NERV_Log_BeforeTransmit` 按第 7.9 节定义为弱函数，允许用户覆盖以执行过滤或统计。

### 11.2 `config.h` 中的日志配置

```c
/* config/Inc/config.h */

/**
  * @brief 调试模式总开关。
  * @note  0 关闭全部日志，NERV_LOG_xxx 宏展开为 ((void)0)。
  * @note  1 开启日志，具体输出级别由 NERV_LOG_LEVEL 控制。
  * @note  发布固件必须保持为 0。
  */
#ifndef NERV_DEBUG_MODE
#define NERV_DEBUG_MODE             0
#endif

/**
  * @brief 日志输出级别阈值，供预处理期过滤使用。
  * @note  数值越小越严重，ERROR 为 0，WARN 为 1，INFO 为 2，DEBUG 为 3。
  * @note  仅输出级别数值小于或等于 NERV_LOG_LEVEL 的日志。
  * @note  合法值见 NERV_LOG_LEVEL_ERROR 等宏。
  */
#ifndef NERV_LOG_LEVEL
#define NERV_LOG_LEVEL              NERV_LOG_LEVEL_INFO
#endif

/**
  * @brief 日志输出串口句柄。
  * @note  必须指向已由 MX_USARTx_UART_Init 初始化的 UART_HandleTypeDef 实例。
  * @note  使用该宏的源文件必须能看见对应 huart 的 extern 声明，
  *        通常在主头文件或 config.h 顶部声明 extern UART_HandleTypeDef huart1;
  */
#ifndef NERV_LOG_UART_HANDLE
#define NERV_LOG_UART_HANDLE        (&huart1)
#endif

/**
  * @brief 单条日志的最大字节数，包含级别、文件、行号、消息体。
  * @note  必须大于格式化前缀长度，推荐范围 128 至 256。
  */
#ifndef NERV_LOG_MSG_MAX
#define NERV_LOG_MSG_MAX            160U
#endif

/**
  * @brief 日志队列深度，单位为元素个数。
  * @note  队列满时日志静默丢弃，不影响控制任务。
  */
#ifndef NERV_LOG_QUEUE_SIZE
#define NERV_LOG_QUEUE_SIZE         32U
#endif

/**
  * @brief 日志任务栈大小，单位为字节，CMSIS-OS 语义下由 osThreadAttr_t 使用。
  */
#ifndef NERV_LOG_TASK_STACK_SIZE
#define NERV_LOG_TASK_STACK_SIZE    (512U * 4U)
#endif

/**
  * @brief HAL_UART_Transmit 阻塞超时时间，单位为毫秒。
  * @note  日志任务允许阻塞，但不得超过该值。
  */
#ifndef NERV_LOG_TX_TIMEOUT_MS
#define NERV_LOG_TX_TIMEOUT_MS      5U
#endif
```

### 11.3 `nerv_log.h`，日志宏与声明

```c
/* log/Inc/nerv_log.h */
#ifndef NERV_LOG_H
#define NERV_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "nerv_status.h"

/**
  * @brief 日志级别阈值，供预处理期过滤使用。
  * @note  合法成员为 NERV_LOG_LEVEL_ERROR、NERV_LOG_LEVEL_WARN、
  *        NERV_LOG_LEVEL_INFO、NERV_LOG_LEVEL_DEBUG。
  * @note  本组宏属于关键宏，禁止用户定义或覆盖，仅允许在 config/Inc/config.h 中引用。
  */
#define NERV_LOG_LEVEL_ERROR        0U
#define NERV_LOG_LEVEL_WARN         1U
#define NERV_LOG_LEVEL_INFO         2U
#define NERV_LOG_LEVEL_DEBUG        3U

#if (NERV_DEBUG_MODE != 0U)

#include "cmsis_os2.h"

/**
  * @brief 日志级别，供运行期字段与函数参数使用。
  * @note  数值必须与 NERV_LOG_LEVEL_xxx 宏一致。
  * @note  合法成员为 NERV_LOG_LEVEL_ERROR_RT、NERV_LOG_LEVEL_WARN_RT、
  *        NERV_LOG_LEVEL_INFO_RT、NERV_LOG_LEVEL_DEBUG_RT。
  */
typedef enum
{
    NERV_LOG_LEVEL_ERROR_RT = 0U,  /**< 错误，影响功能 */
    NERV_LOG_LEVEL_WARN_RT  = 1U,  /**< 警告，可能异常 */
    NERV_LOG_LEVEL_INFO_RT  = 2U,  /**< 关键流程信息 */
    NERV_LOG_LEVEL_DEBUG_RT = 3U,  /**< 调试细节，量最大 */
} NERV_Log_LevelTypeDef;

/**
  * @brief 单条日志队列元素。
  */
typedef struct
{
    NERV_Log_LevelTypeDef level;        /**< 日志级别 */
    uint16_t              len;          /**< msg 有效字节数，不含结束符 */
    char                  msg[NERV_LOG_MSG_MAX];  /**< 已格式化好的完整消息 */
} NERV_Log_ItemTypeDef;

/** @brief 初始化日志队列与日志任务，须在 osKernelStart 之后调用。 */
NERV_StatusTypeDef NERV_Log_Init(void);

/**
  * @brief  将一条已格式化的日志入队，不阻塞调用者。
  * @param  p_item: 日志元素指针，函数内部拷贝。
  * @return osOK 表示入队成功。队列满时返回 osErrorResource。
  * @note   NOT ISR-safe，ISR 中禁止调用。
  */
osStatus_t NERV_Log_Push(const NERV_Log_ItemTypeDef *p_item);

/**
  * @brief 日志发送前钩子，弱函数，用户可覆盖。
  * @param p_item: 待发送日志元素指针。
  */
void NERV_Log_BeforeTransmit(const NERV_Log_ItemTypeDef *p_item);

/**
  * @brief 格式化一条日志并入队，宏体使用 do { ... } while (0) 防止 if 无括号调用错误。
  * @note  仅允许通过 NERV_LOG_ERROR、NERV_LOG_WARN、NERV_LOG_INFO、NERV_LOG_DEBUG 调用。
  * @note  不得在 ISR 中调用，参见第 11.5 节。
  */
#define NERV_LOG_FORMAT_AND_PUSH(_level, _file, _line, _fmt, ...)             \
    do {                                                                       \
        NERV_Log_ItemTypeDef _nerv_log_item;                                   \
        int _nerv_log_ret;                                                     \
        _nerv_log_item.level = (NERV_Log_LevelTypeDef)(_level);                \
        _nerv_log_ret = snprintf(_nerv_log_item.msg,                           \
                                 sizeof(_nerv_log_item.msg),                   \
                                 "[%u] %s:%lu " _fmt,                          \
                                 (unsigned)(_level), (const char *)(_file),    \
                                 (unsigned long)(_line), ##__VA_ARGS__);       \
        if (_nerv_log_ret > 0)                                                 \
        {                                                                      \
            _nerv_log_item.len = (uint16_t)((_nerv_log_ret <                   \
                (int)sizeof(_nerv_log_item.msg)) ? _nerv_log_ret               \
                : (int)sizeof(_nerv_log_item.msg) - 1);                        \
            (void)NERV_Log_Push(&_nerv_log_item);                              \
        }                                                                      \
    } while (0)

#if (NERV_LOG_LEVEL >= NERV_LOG_LEVEL_ERROR)
#define NERV_LOG_ERROR(_fmt, ...)                                              \
    NERV_LOG_FORMAT_AND_PUSH(NERV_LOG_LEVEL_ERROR, __FILE__, __LINE__, (_fmt), ##__VA_ARGS__)
#else
#define NERV_LOG_ERROR(_fmt, ...)    ((void)0)
#endif

#if (NERV_LOG_LEVEL >= NERV_LOG_LEVEL_WARN)
#define NERV_LOG_WARN(_fmt, ...)                                               \
    NERV_LOG_FORMAT_AND_PUSH(NERV_LOG_LEVEL_WARN, __FILE__, __LINE__, (_fmt), ##__VA_ARGS__)
#else
#define NERV_LOG_WARN(_fmt, ...)     ((void)0)
#endif

#if (NERV_LOG_LEVEL >= NERV_LOG_LEVEL_INFO)
#define NERV_LOG_INFO(_fmt, ...)                                               \
    NERV_LOG_FORMAT_AND_PUSH(NERV_LOG_LEVEL_INFO, __FILE__, __LINE__, (_fmt), ##__VA_ARGS__)
#else
#define NERV_LOG_INFO(_fmt, ...)     ((void)0)
#endif

#if (NERV_LOG_LEVEL >= NERV_LOG_LEVEL_DEBUG)
#define NERV_LOG_DEBUG(_fmt, ...)                                              \
    NERV_LOG_FORMAT_AND_PUSH(NERV_LOG_LEVEL_DEBUG, __FILE__, __LINE__, (_fmt), ##__VA_ARGS__)
#else
#define NERV_LOG_DEBUG(_fmt, ...)    ((void)0)
#endif

#else  /* NERV_DEBUG_MODE == 0 */

#define NERV_LOG_ERROR(_fmt, ...)    ((void)0)
#define NERV_LOG_WARN(_fmt, ...)     ((void)0)
#define NERV_LOG_INFO(_fmt, ...)     ((void)0)
#define NERV_LOG_DEBUG(_fmt, ...)    ((void)0)

#endif /* NERV_DEBUG_MODE */

#ifdef __cplusplus
}
#endif

#endif /* NERV_LOG_H */
```

### 11.4 `nerv_log.c`，最低优先级日志任务

```c
/* log/Src/nerv_log.c */
#include "config.h"

#if (NERV_DEBUG_MODE != 0U)

#include "nerv_log.h"
#include "nerv_rtos_config.h"
#include "stm32h7xx_hal.h"  /* 替换为目标系列的 HAL 头文件 */
#include <stdio.h>
#include <string.h>

#if (NERV_LOG_LEVEL_ERROR != (int)NERV_LOG_LEVEL_ERROR_RT)
#error "NERV_LOG_LEVEL_ERROR and NERV_LOG_LEVEL_ERROR_RT must be equal."
#endif

static osMessageQueueId_t nerv_log_queue = NULL;

/**
  * @brief  日志发送前钩子，弱函数，用户可覆盖默认实现。
  * @param  p_item: 待发送日志元素指针，用户可读取但不得修改。
  * @note   默认实现为空。
  * @note   NOT ISR-safe，在 nerv_log 任务中调用。
  */
__weak void NERV_Log_BeforeTransmit(const NERV_Log_ItemTypeDef *p_item)
{
    (void)p_item;
}

/**
  * @brief  日志任务主循环，从队列取出消息并阻塞发送到串口。
  * @note   任务优先级固定为 NERV_TASK_PRIO_LOG，取值为 osPriorityLow，
  *         是系统中优先级最低的任务。
  * @note   本任务允许阻塞，但每条发送不超过 NERV_LOG_TX_TIMEOUT_MS。
  */
static void NERV_Log_Task(void *argument)
{
    NERV_Log_ItemTypeDef item;

    (void)argument;

    for (;;)
    {
        if (osOK == osMessageQueueGet(nerv_log_queue, &item, NULL, osWaitForever))
        {
            if (NULL != NERV_LOG_UART_HANDLE)
            {
                NERV_Log_BeforeTransmit(&item);
                (void)HAL_UART_Transmit(NERV_LOG_UART_HANDLE,
                                        (uint8_t *)item.msg,
                                        item.len,
                                        NERV_LOG_TX_TIMEOUT_MS);
            }
        }
    }
}

NERV_StatusTypeDef NERV_Log_Init(void)
{
    const osThreadAttr_t log_task_attr =
    {
        .name       = "nerv_log",
        .stack_size = NERV_LOG_TASK_STACK_SIZE,
        .priority   = NERV_TASK_PRIO_LOG,
    };

    nerv_log_queue = osMessageQueueNew(NERV_LOG_QUEUE_SIZE,
                                       sizeof(NERV_Log_ItemTypeDef),
                                       NULL);
    if (NULL == nerv_log_queue)
    {
        return NERV_ERROR;
    }

    if (NULL == osThreadNew(NERV_Log_Task, NULL, &log_task_attr))
    {
        return NERV_ERROR;
    }

    return NERV_OK;
}

osStatus_t NERV_Log_Push(const NERV_Log_ItemTypeDef *p_item)
{
    if (NULL == nerv_log_queue || NULL == p_item)
    {
        return osErrorParameter;
    }

    /* 队列满时立即返回，绝不阻塞调用者 */
    return osMessageQueuePut(nerv_log_queue, p_item, 0U, 0U);
}

#endif /* NERV_DEBUG_MODE */
```

### 11.5 使用约束

- 模块中打印日志只能通过 `NERV_LOG_xxx` 宏，不得直接调用 `NERV_Log_Push`。
- 日志宏展开为 `do { ... } while (0)` 块，内部做栈上 `snprintf` 与入队，耗时约数十微秒。即便如此，仍不得放在频率超过 1kHz 的中断或每帧 FDCAN 回调中。
- 确需在中断中记录异常时，置位模块错误标志，参见第 4 节，由低频任务检测后打印。不得在 ISR 中调用 `NERV_LOG_xxx`。
- 日志消息中禁止出现动态内存、浮点格式串、超长字符串、emoji 与颜文字。浮点格式串为默认禁止项，除非 `config/Inc/config.h` 显式启用 `NERV_LOG_USE_FLOAT`。
- `NERV_DEBUG_MODE` 为 0 时，`NERV_LOG_xxx` 展开为 `((void)0)`，调用方不得依赖其返回值或副作用。
- 用户覆盖 `NERV_Log_BeforeTransmit` 时，不得修改 `p_item` 指向的内容，不得调用 `NERV_Log_Push` 产生递归入队，不得执行阻塞操作超过 `NERV_LOG_TX_TIMEOUT_MS`。

---

## 12. FreeRTOS 与 CMSIS-OS v2，强制

### 12.1 统一使用 CMSIS-OS v2 API

NERV 统一使用 CMSIS-OS v2 接口，所有 RTOS 调用以 `os` 开头。禁止直接调用原生 FreeRTOS API 与裸 HAL 延时函数。

| 功能       | 允许                                                          | 禁止                         |
| ---------- | ------------------------------------------------------------- | ---------------------------- |
| 创建任务   | `osThreadNew`                                                 | `xTaskCreate`                |
| 延时       | `osDelay` 与 `osDelayUntil`                                   | `vTaskDelay` 与 `HAL_Delay`  |
| 消息队列   | `osMessageQueueNew`、`osMessageQueueGet`、`osMessageQueuePut` | `xQueueCreate`、`xQueueSend` |
| 信号量     | `osSemaphoreNew`、`osSemaphoreAcquire`、`osSemaphoreRelease`  | `xSemaphoreCreateBinary`     |
| 互斥量     | `osMutexNew`、`osMutexAcquire`、`osMutexRelease`              | `xSemaphoreCreateMutex`      |
| 软件定时器 | `osTimerNew`、`osTimerStart`                                  | `xTimerCreate`               |
| 内核启动   | `osKernelInitialize`、`osKernelStart`                         | `vTaskStartScheduler`        |
| 时间戳     | `osKernelGetTickCount`                                        | `xTaskGetTickCount`          |

仅当 CMSIS-OS v2 未覆盖所需功能时，才允许调用原生宏，典型场景包括 `taskENTER_CRITICAL` 与 `portYIELD_FROM_ISR`。使用前必须在文件顶部注释说明原因。中断中只允许调用 `osXxxFromISR` 结尾的函数。

### 12.2 优先级与栈大小集中配置

所有任务优先级与栈大小集中在 `config/Inc/nerv_rtos_config.h`，禁止在 `osThreadNew` 调用处写裸数字。

```c
/* config/Inc/nerv_rtos_config.h */
#ifndef NERV_RTOS_CONFIG_H
#define NERV_RTOS_CONFIG_H

#include "cmsis_os2.h"

/* 任务优先级，数值越大越高，与 osPriorityXxx 枚举对齐 */
#define NERV_TASK_PRIO_LOG          osPriorityLow          /**< 日志任务，全局最低 */
#define NERV_TASK_PRIO_APP          osPriorityHigh         /**< 应用调度任务 */
#define NERV_TASK_PRIO_CHASSIS      osPriorityAboveNormal  /**< 底盘任务 */
#define NERV_TASK_PRIO_GIMBAL       osPriorityAboveNormal  /**< 云台任务 */
#define NERV_TASK_PRIO_SHOOT        osPriorityAboveNormal  /**< 发射任务 */

/* 任务栈大小，单位为字节 */
#define NERV_TASK_STACK_APP         (1024U * 4U)
#define NERV_TASK_STACK_CHASSIS     (1024U * 4U)
#define NERV_TASK_STACK_GIMBAL      (1024U * 4U)
#define NERV_TASK_STACK_SHOOT       (512U  * 4U)

/* 常用周期，单位为毫秒 */
#define NERV_PERIOD_MS(_ms)         ((uint32_t)(_ms))

#endif /* NERV_RTOS_CONFIG_H */
```

任务优先级此处保留 `#define` 的原因在于 CMSIS-OS v2 的 `osPriorityXxx` 本身是枚举，而 `osThreadAttr_t` 的 `priority` 字段类型为 `osPriority_t`。直接使用 `osPriorityXxx` 枚举成员即可，无需定义新的枚举类型。`NERV_TASK_PRIO_xxx` 宏仅用于集中管理与语义命名，符合第 7.5 节第 1 条。

任务优先级与栈大小属于第 7.8.1 节判定标准的关键宏。需要按第 7.8.4 节使用 `#ifndef` 与 `#define` 与 `#endif` 包裹的，是允许用户在 `config.h` 之外覆盖的值，例如任务周期。任务优先级与栈大小由 NERV 统一管理，不在用户覆盖范围内，保持直接 `#define` 形式。

### 12.3 中断优先级

- NVIC 优先级分组固定为 `NVIC_PRIORITYGROUP_4`，采用 4 位抢占优先级与 0 位子优先级。
- 可调用 `osXxxFromISR` 的中断，其抢占优先级数值必须大于或等于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，即逻辑优先级不得高于该阈值。
- 优先级分配建议如下：

| 中断源                     | 抢占优先级                           | 说明                                      |
| -------------------------- | ------------------------------------ | ----------------------------------------- |
| FreeRTOS SysTick 与 PendSV | 最低                                 | 由 `configKERNEL_INTERRUPT_PRIORITY` 决定 |
| FDCAN RX                   | 高于 SysTick，且不小于 `MAX_SYSCALL` | 允许调用 `FromISR`                        |
| UART RX                    | 同上                                 | 允许调用 `FromISR`                        |
| DWT 与高精度定时           | 最高                                 | 不得调用任何 `FromISR` 与 RTOS API        |

禁止在高于 `MAX_SYSCALL` 的中断中调用任何 `osXxx` 或 `osXxxFromISR` 函数。

### 12.4 临界区与线程安全

- 临界区只保护极短代码，长度不超过 10 行。禁止在 `taskENTER_CRITICAL` 或 `osKernelLock` 内调用 HAL 阻塞函数或 `osDelay`。
- 模块级共享数据优先使用 `osMutexNew`、`osMutexAcquire`、`osMutexRelease` 保护，而不是关中断。
- 每个模块在 `.c` 的公开函数 Doxygen 中标注线程模型，使用以下四类之一：
  - `@note 仅可在 chassis_task 中调用`
  - `@note 可从任意任务调用，内部持锁`
  - `@note NOT ISR-safe`
  - `@note ISR-safe`

### 12.5 初始化顺序

`main` 函数中顺序固定如下：

1. `HAL_Init`
2. `SystemClock_Config`
3. 外设 `MX_xxx_Init`
4. `osKernelInitialize`
5. `NERV_Log_Init`
6. 各模块 `NERV_<Module>_Register`
7. `osKernelStart`

依赖关系必须在 `app/Src/app_init.c` 中以显式顺序调用并注释原因，禁止依赖链接顺序或构造函数。

---

## 13. FDCAN 与 AC6 的特定约束

### 13.1 FDCAN

- `bsp/fdcan/` 负责过滤器配置、中断注册、发送邮箱管理。电机协议解析与反馈解码属于 `module/motor/`。
- FDCAN 接收中断中只做拷贝与回调通知，不做浮点运算、不调用 `osDelay`、不操作动态内存、不得直接调用 `NERV_LOG_xxx`。
- 过滤器配置必须在 `NERV_FDCAN_Start` 之前完成。启动后的变更须通过显式 API。
- 发送接口不得阻塞等待邮箱空出，应返回 `NERV_BUSY` 由调用者决定重试策略。
- FDCAN 的 ID 分配表、帧格式、超时参数属于第 7.8.1 节判定标准的关键宏，须遵循第 7.8.4 节与第 7.8.5 节的保护规则。
- FDCAN 的接收完成通知使用注册回调函数指针，不使用弱函数，因为回调与具体实例绑定，参见第 9.5 节。

### 13.2 AC6

- AC6 中 `__attribute__((at(...)))` 无效，固定地址变量须使用 `__attribute__((section(".ARM.__AT_0xXXXXXXXX")))`。
- 结构体紧凑布局使用 `__attribute__((packed))`，对齐使用 `__attribute__((aligned(N)))`。
- 中断服务函数命名遵循启动文件中的向量名，禁止使用 AC5 的 `__irq` 关键字。
- 开启 `-Werror=implicit-function-declaration`。`-Wall -Wextra` 不允许新增警告。建议额外开启 `-Wparentheses` 与 `-Wmisleading-indentation`，用于捕获 `if (a = b)` 与无括号 `if` 语句的缩进错误。这两个警告与第 7.2 节、第 7.7 节的规则形成互补，编译器层面的保障优先于人工审查。
- `__weak` 由 CMSIS 在 `cmsis_compiler.h` 中定义，AC6 展开为 `__attribute__((weak))`，GCC 展开为 `__attribute__((weak))`。NERV 直接使用 `__weak`，不得自行定义同名宏，不得使用 AC5 的 `__weak` 与其他非标准属性写法。
- 弱函数的声明处不写 `__weak`，定义处写 `__weak`，参见第 7.9.5 节。
- 使用 `-Wmissing-prototypes` 时，弱函数必须在头文件中声明原型，否则会产生警告。NERV 强制弱函数在公开头文件中声明原型。

---

## 14. `goto`，严格对齐 Linux 内核风格

`goto` 只用于集中清理与统一返回。无清理需求时直接 `return`。

### 14.1 标签命名

标签名必须说明 `goto` 要做什么，使用 `out_<资源>` 或 `<动作>_<资源>` 形式。禁止使用 `err1:`、`err2:`、`fail:` 这类名字。

```c
NERV_StatusTypeDef NERV_Chassis_Init(NERV_Chassis_InstanceTypeDef *hchassis,
                                      const NERV_Chassis_ConfigTypeDef *p_config)
{
    NERV_StatusTypeDef status = NERV_OK;

    if (NULL == hchassis || NULL == p_config)
    {
        status = NERV_ERROR;
        goto out;
    }

    hchassis->priv = NERV_Malloc(sizeof(NERV_Chassis_PrivDataTypeDef));
    if (NULL == hchassis->priv)
    {
        status = NERV_ERROR;
        goto out;
    }

    status = NERV_FDCAN_RegisterCallback(p_config->hfdcan,
                                          NERV_Chassis_FdcanRxCallback,
                                          hchassis);
    if (NERV_OK != status)
    {
        goto out_free_priv;
    }

    goto out;

out_free_priv:
    NERV_Free(hchassis->priv);
    hchassis->priv = NULL;
out:
    return status;
}
```

### 14.2 禁忌

- 不允许向后跳转。
- 不允许跳过变量初始化后使用该变量。
- 不允许在 `goto` 链中重复释放同一资源。每个 `out_` 标签只负责释放该标签之后分配的资源。
- 不允许从 `goto` 标签之前落穿到清理代码。

---

## 15. 防御性编程与 C++ 兼容

### 15.1 头文件骨架，每个 `.h` 强制

`extern "C"` 块内只放声明与宏。Doxygen 完整注释不在头文件展开，参见第 8.3 节。

```c
#ifndef NERV_CHASSIS_H
#define NERV_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes */

/* types, macros, declarations */

#ifdef __cplusplus
}
#endif

#endif /* NERV_CHASSIS_H */
```

`extern "C"` 块必须包裹所有产生链接符号的声明，包括函数与全局变量。`#include` 应放在块外。弱函数的声明同样放在 `extern "C"` 块内。

### 15.2 参数检查

所有对外函数在入口检查指针与枚举合法性。检查失败返回 `NERV_ERROR`，并通过 `NERV_<Module>_GetError` 置位 `NERV_ERR_INVALID_PARAM` 等扩展码。不得解引用空指针或进入 `assert` 死循环。

### 15.3 编译时防御

在 `config/Inc/config.h` 或模块头文件中使用 `#if` 与 `#error` 拦截非法宏组合。对可能被 C++ 包含的头文件，在 C++ 侧检查 `sizeof` 与对齐：

```c
#ifdef __cplusplus
static_assert(sizeof(NERV_Chassis_InstanceTypeDef) <= 256,
              "NERV_Chassis_InstanceTypeDef exceeds expected size.");
#endif
```

### 15.4 枚举规范

- 对枚举类型的 `switch` 必须包含 `default` 分支，返回 `NERV_ERROR` 或执行安全停机。
- 枚举成员必须显式赋值，禁止依赖隐式递增。
- 枚举类型名以 `TypeDef` 结尾，与 HAL 风格一致。
- 枚举成员的数值一经发布不得变更，新增成员只能在末尾追加，避免破坏协议与遥测兼容性。
- 枚举成员若用于持久化或通信，必须在注释中说明数值来源。
- 枚举类型作为函数参数时，调用者传入非法值的检查在函数入口完成，参见第 15.2 节。
- 枚举成员与变量做相等比较时，同样遵循第 7.2 节的常量在左规则，例如 `if (NERV_OK == status)`。

### 15.5 弱函数规范

- 弱函数仅用于第 7.9.3 节列出的适用场景。关键资源管理、协议编解码核心路径、混合上下文逻辑、返回值被内部依赖的逻辑禁止使用弱函数。
- 弱函数的声明写在公开头文件中，默认实现写在 `.c` 中，声明处不写 `__weak`，定义处写 `__weak`。
- 弱函数的默认实现必须为空或最简行为，遵循第 7.9.8 节的全部约束。
- 弱函数的 Doxygen 注释必须说明“本函数为弱函数，用户可覆盖”，并标注线程模型。
- 弱函数必须与注册回调函数指针、宏钩子三种扩展机制区分使用，参见第 9.5 节。

---

## 16. Git Commit Message

采用 Conventional Commits 1.0.0 规范，中文团队适配规则如下：

- `type` 保留英文关键字，取值限定为 `feat`、`fix`、`docs`、`style`、`refactor`、`perf`、`test`、`chore`、`ci`、`revert`。
- `scope` 使用英文模块名，采用小写，与源码目录名或模块名保持一致。典型取值包括 `chassis`、`gimbal`、`shoot`、`motor`、`fdcan`、`uart`、`log`、`rtos`、`status`、`config`、`public`、`layout`、`type`、`macro`、`def`、`weak`、`bsp`、`app`。新增 scope 必须与已有目录或模块命名保持一致，不得使用拼音、中文或自造缩写。
- `description` 使用中文动宾短语，不超过 50 字，不以句号结尾。
- `body` 说明为什么改、怎么改、影响范围，每行不超过 72 字符。
- 提交信息中禁止使用 emoji 与颜文字，理由同第 8.2 节第 7 条。

判定规则：`type` 与 `scope` 均为英文，`description` 与 `body` 使用中文。禁止在 `scope` 中使用中文、拼音或自造缩写。

### 16.1 格式

```
<type>(<scope>): <中文动宾描述>

<中文 body>

<footer>
```

### 16.2 示例

```
feat(chassis): 增加舵轮构型的运动学解算

通过 config/Inc/config.h 中的 NERV_CHASSIS_TYPE 选择舵轮构型时，
NERV_Chassis_Init 挂载舵轮专用解算函数指针。

影响范围：module/chassis、config/Inc/config.h
测试确认：已通过舵轮台架联调
```

```
refactor(weak): 引入弱函数扩展点规范

新增 NERV_Chassis_OnFault 与 NERV_Log_BeforeTransmit 两个弱函数，
声明写在公开头文件，默认实现为空且标注 __weak。
明确弱函数仅适用于模块级事件通知，不适用于关键资源管理。

影响范围：module/chassis、log/
测试确认：新增单元测试覆盖用户覆盖与用户不覆盖两类场景
```

```
refactor(def): 引入宏保护与命名空间隔离规则

用户可覆盖的默认值宏统一使用 #ifndef 与 #define 与 #endif 包裹。
用户禁止覆盖的推导宏统一使用 #if defined(...) 加 #error 保护。
内部实现宏统一使用 __NERV_ 双下划线前缀，并仅允许出现在 *_priv.h 与 .c 中。
关键宏定义区增加禁止修改注释包裹。

影响范围：config/Inc/config.h、module/*/Inc/*_def.h、module/*/Inc/*_priv.h
测试确认：新增单元测试覆盖用户定义禁止宏与超出范围默认值两类场景
```

```
refactor(layout): 将公共基础与配置从根 Inc 拆出

新增 public/Inc 目录承载 nerv_status.h、nerv_types.h、nerv_utils.h、nerv_math.h。
新增 config/Inc 目录承载 config.h 与 nerv_rtos_config.h。二者均沿用 Inc 与 Src 结构。

影响范围：全仓库 include 路径、CMakeLists、所有模块头文件
```

```
refactor(macro): 统一宏定义函数使用 do-while(0) 包裹

所有多语句宏改为 do { ... } while (0) 结构。
宏内局部变量统一加 _nerv_<module>_ 前缀。
宏形参在展开体内每次出现均加括号。

影响范围：log/Inc/nerv_log.h、public/Inc/nerv_utils.h、module/*/Inc/*.h
测试确认：新增单元测试覆盖 if 无括号调用与宏后加分号两类场景
```

```
refactor(type): 将协议命令字由宏改为枚举

NERV_Motor_SendCommand 参数类型由 uint8_t 改为 NERV_Motor_CmdTypeDef。
枚举成员显式赋值以保持协议数值稳定。
config/Inc/config.h 中参与预处理判断的宏保持不变。

影响范围：module/motor、所有调用 NERV_Motor_SendCommand 的位置
```

```
refactor(status): 引入 NERV_StatusTypeDef 与分段扩展错误码

基础四态对齐 HAL_StatusTypeDef。扩展码按模块域分段并使用位掩码语义。
各模块新增 NERV_<Module>_GetError 与 ClearError。
所有相等比较改写为常量在左形式。

影响范围：public/Inc/nerv_status.h、所有模块公开头
```

```
fix(fdcan): 修复过滤器未在启动前配置导致的丢帧

将过滤器配置移至 NERV_FDCAN_Start 之前执行。
补充 AC6 下 section 属性的兼容写法。

影响范围：bsp/fdcan
Closes #42
```

### 16.3 禁止的写法

以下写法因 scope 使用中文、拼音或自造缩写而不合格：

```
fix: 修了一个bug
feat: 更新代码
chore: 改了点东西
feat(底盘): 增加舵轮构型的运动学解算
refactor(状态码): 引入分段扩展错误码
fix(cd): 修复过滤器配置
```

---

## 17. 补充原则，必守

### 17.1 SOLID 在嵌入式语境下的含义

- **单一职责**：`bsp/fdcan` 只负责帧收发与过滤器，电机协议解码属于 `module/motor`。`log/` 只负责格式化、入队与串口输出，不感知业务语义。`public/` 只提供跨层基础类型与纯函数。`config/` 只提供编译期宏。
- **开闭**：新增底盘构型或电机型号时，通过 `config/Inc/config.h` 宏与新增文件实现，不修改 `NERV_Chassis_Update` 主流程。用户扩展通过弱函数与注册回调实现，不修改 NERV 内部实现。
- **里氏替换**：派生构型实例必须能安全地以基类指针传入 `NERV_Chassis_Update`。
- **接口隔离**：模块头文件只暴露调用者需要的类型与函数。私有类型定义放 `.c` 或 `_priv.h`，不进入公开头文件。头文件不承载契约文档，契约文档写在 `.c`。
- **依赖倒置**：`module` 依赖 `bsp` 提供的抽象接口，形式为函数指针与实例指针，不依赖具体 HAL 函数。

### 17.2 中断上下文规则

- 在 `module` 层明确标注 `@note ISR-safe` 或 `@note NOT ISR-safe`。未标注默认为不可在中断中调用。
- `bsp` 层 ISR 只允许调用 `_ISR` 后缀的模块接口。
- `NERV_LOG_xxx` 默认 NOT ISR-safe。ISR 中记录异常采用置位错误标志与低频任务打印的组合方案。
- 弱函数必须在 Doxygen 注释中标注线程模型，默认实现必须与该线程模型一致，参见第 7.9.7 节。

### 17.3 编译目标

- 必须同时通过 AC6 与 arm-none-eabi-gcc 编译。
- 开启 `-Wall -Wextra`，不允许新增警告。
- 建议开启 `-Wparentheses` 与 `-Wmisleading-indentation`，用于捕获 `if (a = b)` 与无括号 `if` 语句的缩进错误，与第 7.2 节、第 7.7 节形成互补。
- 建议开启 `-Wmissing-prototypes`。此选项要求所有非 `static` 函数都有原型声明，弱函数必须在公开头文件中声明，与第 7.9.5 节一致。
- 不使用 VLA。运行期禁止 `malloc` 与 `free`，除非模块配置显式启用 `NERV_USE_HEAP` 并在该模块文档中声明。
- `NERV_DEBUG_MODE` 为 0 的 Release 构建须开启 `-ffunction-sections -fdata-sections -Wl,--gc-sections`，确保日志实现的符号与字符串被链接器裁剪。
- include path 中 `public/Inc` 与 `config/Inc` 必须排在所有模块目录之前，保证 `#include "nerv_status.h"` 与 `#include "config.h"` 解析到唯一入口。

### 17.4 文档同步

修改任何函数定义时，必须在同一提交内更新该函数上方 `.c` 中的 Doxygen 注释。PR 审查时，注释与代码不一致视为阻塞问题。

---

## 18. 快速自检清单，提交前

- [ ] 所有新增公开符号是否以 `NERV_<Module>_` 开头？
- [ ] 新模块是否在 `<module>/Inc/` 与 `<module>/Src/` 下成对创建 `.h` 与 `.c`？
- [ ] 全局公共类型是否放在 `public/Inc/`？编译期配置是否放在 `config/Inc/`？是否避免了在根 `Inc/` 平铺业务头？
- [ ] 头文件是否有 Include Guard、`extern "C"` 块，且 `#include` 在块外？
- [ ] `.h` 中是否没有完整 Doxygen 块，除一行 `@brief` 外？完整契约注释是否位于 `.c` 定义处？
- [ ] 对外函数是否返回 `NERV_StatusTypeDef`，并通过 `NERV_<Module>_GetError` 暴露扩展码？
- [ ] 所有物理量是否使用 SI 单位并带单位后缀，例如 `_rad`、`_mps`、`_ms`？
- [ ] 坐标系是否为右手系，x 轴前、y 轴左、z 轴上，yaw 逆时针为正？角度是否在 `[-π, π)` 并用 `NERV_Math_WrapPi` 归一化？
- [ ] 是否存在裸数字字面量，0、1、-1、NULL、true、false 除外？超时是否使用 `NERV_TIMEOUT_MS(x)`？
- [ ] 所有相等比较与不等比较是否将常量写在运算符左侧，例如 `if (NERV_OK == status)`、`if (NULL == hchassis)`？不等式比较与布尔判断是否保持原序？
- [ ] 多语句宏是否使用 `do { ... } while (0)` 包裹？单表达式宏是否整体加外层括号？
- [ ] 宏形参在展开体内每次出现是否都加括号？是否存在形参在宏体内多次求值的场景？
- [ ] 宏内局部变量是否使用 `_nerv_<module>_` 前缀，或使用 `##__COUNTER__` 拼接唯一名称？
- [ ] 宏体内是否存在 `return`、`goto`、`continue` 等改变调用方控制流的语句？
- [ ] 属于第 7.8.1 节判定标准的关键宏，是否按第 7.8.4 节使用 `#ifndef` 与 `#define` 与 `#endif` 包裹？
- [ ] 用户禁止覆盖的推导宏是否按第 7.8.5 节使用 `#if defined(...)` 加 `#error` 保护？
- [ ] 内部实现宏是否使用 `__NERV_` 双下划线前缀，且仅出现在 `*_priv.h` 或 `.c` 中，未进入公开头文件？
- [ ] 关键宏定义区是否按第 7.8.6 节使用“禁止修改”注释包裹？
- [ ] 与第三方库头文件交互的 NERV 头文件是否按第 7.8.7 节使用 `#pragma push_macro` 与 `#pragma pop_macro` 保护同名宏？
- [ ] 弱函数是否仅用于第 7.9.3 节列出的适用场景？是否避免了关键资源管理、协议编解码核心路径、混合上下文逻辑、返回值被内部依赖的逻辑？
- [ ] 弱函数的声明是否写在公开头文件中，且声明处不写 `__weak`？定义处是否使用 `__weak`？
- [ ] 弱函数是否使用 `NERV_<Module>_On<Event>` 或 `NERV_<Module>_<HookName>` 命名？是否未使用 `__weak_` 或 `weak_` 之类的命名前缀？
- [ ] 弱函数的默认实现是否为空或最简行为，是否符合第 7.9.8 节的全部约束？
- [ ] 弱函数的 Doxygen 注释是否说明了“本函数为弱函数，用户可覆盖”，并标注线程模型？
- [ ] 是否存在本可用宏或枚举在编译期解决却改用函数在运行期解决的逻辑？参见第 7.5 节。
- [ ] 类型化有限取值集合是否使用 `typedef enum`，而不是一组 `#define`？仅参与 `#if` 判断的常量是否保留为 `#define`？
- [ ] 参与 `#if` 的宏与运行期枚举是否数值一致，并有 `#if` 或 `static_assert` 校验？
- [ ] 枚举成员是否显式赋值，且没有依赖隐式递增？
- [ ] 注释中是否出现 `+` 表示并列或解释性 `()`？语言是否科学、准确、详细？
- [ ] 注释、日志格式串、提交信息、文档中是否没有 emoji 与颜文字？
- [ ] `config/Inc/config.h` 中的宏是否都有 `#ifndef` 默认值与合法性 `#error` 检查？
- [ ] `goto` 标签是否描述行为，例如 `out_free_xxx`，而非编号，例如 `err1`？
- [ ] 是否使用了函数指针与实例指针，而不是全局单例？用户扩展点是否按第 9.5 节在回调、弱函数、宏钩子之间选择了正确的机制？
- [ ] 是否在中断安全边界内标注了 `ISR-safe` 与 `NOT ISR-safe`？
- [ ] RTOS 对象是否全部使用 CMSIS-OS v2 的 `osXxx` API，而非原生 FreeRTOS API？
- [ ] 任务优先级与栈大小是否引用 `config/Inc/nerv_rtos_config.h` 中的宏，而非裸数字？
- [ ] 中断中是否只调用 `osXxxFromISR`，且中断优先级不小于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`？
- [ ] 日志是否仅通过 `NERV_LOG_xxx` 宏输出，日志任务优先级是否为 `NERV_TASK_PRIO_LOG`，即最低优先级？
- [ ] 日志中是否没有硬编码串口句柄，只能通过 `NERV_LOG_UART_HANDLE` 宏注入？
- [ ] `NERV_DEBUG_MODE` 是否在 Release 配置中保持为 0，且未残留运行时 debug 判断？
- [ ] 高频与中断路径中是否避免了日志输出，或改用错误标志位方案？
- [ ] Doxygen 配置是否同时扫描 `.h` 与 `.c`，并保证 `WARN_IF_UNDOCUMENTED` 与现有注释策略不冲突？
- [ ] commit message 中 `type` 与 `scope` 是否为英文，且 `scope` 与源码目录名或模块名一致？`description` 与 `body` 是否为中文且不含 emoji？
