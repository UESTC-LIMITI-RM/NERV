# NERV 开发计划

> 本文件规划 NERV 库的实现顺序与做法.
> 规则以 `AGENTS.md` 为准. 命名, 风格, 提交规范与自检清单不在这里重复.

---

## 1. 目标与边界

NERV 面向 RoboMaster 机器人. 主要服务英雄, 哨兵与步兵三类机型, 覆盖底盘, 云台, 发射机构与安全状态四类控制对象.

要做的事: 按 `AGENTS.md` 实现一个与 MCU 无关的 C11 机器人控制库, 通过 BSP 适配到具体 MCU.

库覆盖的范围:

- 电机抽象: 指令下发, 反馈解析, 在线判定, 输出限幅与过温保护.
- 底盘: 运动学解算, 速度控制, 里程计.
- 云台: 角度控制, 限位, 目标平滑.
- 发射: 摩擦轮与进弹机构状态机, 卡弹判定.
- 安全: 故障分级, 急停锁存, 执行器安全状态.
- 基础设施: 状态码, 单位与数学工具, 日志, FDCAN 抽象.

库不覆盖, 属于使用者工程:

- 应用任务, 调度周期, 任务优先级, 栈大小, 看门狗与急停硬件回路.
- 遥控器与键鼠解析, 裁判系统协议解析, 超级电容管理, 功率限制策略.
- 弹速标定与摩擦轮调参, 视觉与自瞄, 上层决策.
- 构建系统, IDE 工程, 测试工程, CI 配置.

硬件选型不写死在库里. 电机型号, 减速比, 编码比例, 轮径, 轮距等由使用者按实际硬件填写, 库只提供结构与算法.

---

## 2. 路线

| 阶段 | 目标           | 依赖   | 主要交付物                                                  |
| ---- | -------------- | ------ | ----------------------------------------------------------- |
| M0   | 文档收口       | 无     | `README.md`                                                 |
| M1   | 基础层         | M0     | `public/Inc`, `public/Src`                                  |
| M2   | 配置与日志     | M1     | `config/`, `public/Inc/nerv_log.h`, `public/Src/nerv_log.c` |
| M3   | BSP 抽象接口   | M1     | `bsp/Inc`                                                   |
| M4   | 电机模块       | M3     | `module/motor/`                                             |
| M5   | 底盘模块       | M4     | `module/chassis/`                                           |
| M6   | 云台与发射     | M4     | `module/gimbal/`, `module/shoot/`                           |
| M7   | 安全模块       | M4, M5 | `module/safety/`                                            |
| M8   | STM32 参考实现 | M3     | `bsp/stm32h7/`                                              |
| M9   | 发布收尾       | 全部   | 文档与首个 tag                                              |

M4 到 M7 之间只通过公开接口与实例交互, 可以并行推进. M8 只依赖 `bsp/Inc/` 的接口, 可以与模块层并行.

---

## 3. 编译期配置机制

RoboMaster 的底盘构型, 云台轴数在同一次编译中只有一个是需要的. 电机型号可以在同一次编译中混用, 按型号裁剪编译. 库按编译期选择实现, 使用者通过 `config.h` 决定.

原则:

- 选什么由使用者决定, 写在 `config/Inc/config.h`.
- 未选中的实现不参与编译, 不产生目标文件里的代码.
- 不提供运行期切换构型的接口. 需要运行期切换的工具另说, 不进入固件控制路径.
- 宏值参与 `#if`, 枚举值参与运行时类型检查, 两者数值必须一致.
- 是否启用类配置统一取 `NERV_ENABLE` 或 `NERV_DISABLE`, 不写裸 `0` 与 `1`.

三层结构:

1. 宏, 定义在 `config.h` 或模块的 `*_def.h`, 参与 `#if` 与 `#error`.
2. 枚举镜像, 定义在模块头文件, 成员带 `_RT` 后缀, 用于类型安全与调试器可视化.
3. 静态断言, 写在模块 `.c` 中, 校验宏与枚举数值一致.

`config.h` 中的构型选择模板:

```c
/* 底盘构型: 使用者只保留一个选择, 值为 0 时表示未选择. */
#define NERV_CHASSIS_TYPE_MECANUM   1U
#define NERV_CHASSIS_TYPE_OMNI      2U
#define NERV_CHASSIS_TYPE_STEERING  3U

#ifndef NERV_CHASSIS_TYPE
#define NERV_CHASSIS_TYPE           NERV_CHASSIS_TYPE_MECANUM
#endif

#if (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_MECANUM) && \
    (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_OMNI) && \
    (NERV_CHASSIS_TYPE != NERV_CHASSIS_TYPE_STEERING)
#error "NERV_CHASSIS_TYPE is not supported."
#endif
```

模块中的派生宏模板, 推导结果不允许使用者覆盖:

```c
/* module/chassis/Inc/nerv_chassis.h */
#if defined(NERV_CHASSIS_WHEEL_COUNT)
#error "NERV_CHASSIS_WHEEL_COUNT is derived and must not be defined by the user."
#endif

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)
#define NERV_CHASSIS_WHEEL_COUNT  4U
#elif (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)
#define NERV_CHASSIS_WHEEL_COUNT  4U
#else
#define NERV_CHASSIS_WHEEL_COUNT  4U
#endif
```

模块头文件中的枚举镜像:

```c
/* module/chassis/Inc/nerv_chassis.h */
typedef enum
{
    NERV_CHASSIS_TYPE_MECANUM_RT  = 1U,
    NERV_CHASSIS_TYPE_OMNI_RT     = 2U,
    NERV_CHASSIS_TYPE_STEERING_RT = 3U,
} NERV_Chassis_KindTypeDef;
```

模块源文件中的数值校验与分支:

```c
/* module/chassis/Src/nerv_chassis.c */
_Static_assert((int)NERV_CHASSIS_TYPE_MECANUM == (int)NERV_CHASSIS_TYPE_MECANUM_RT,
               "NERV_CHASSIS_TYPE_MECANUM and NERV_CHASSIS_TYPE_MECANUM_RT must be equal.");

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)
static void NERV_Chassis_SolveMecanum(NERV_Chassis_InstanceTypeDef *self,
                                       const NERV_Chassis_VelocityTypeDef *target);
#elif (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)
static void NERV_Chassis_SolveOmni(NERV_Chassis_InstanceTypeDef *self,
                                    const NERV_Chassis_VelocityTypeDef *target);
#else
static void NERV_Chassis_SolveSteering(NERV_Chassis_InstanceTypeDef *self,
                                        const NERV_Chassis_VelocityTypeDef *target);
#endif
```

需要用编译期选择的地方:

- 底盘构型, 麦克纳姆, 全向, 舵轮.
- 云台轴数, 两轴或三轴.
- 板间通信链路, FDCAN 或 UART.
- 模块裁剪, `NERV_USE_*`, 取 `NERV_ENABLE` 或 `NERV_DISABLE`.
- 电机型号裁剪, `NERV_USE_MOTOR_<VENDOR>_<MODEL>`, 同一次编译可以启用多个.
- 日志级别, `NERV_LOG_LEVEL`.

---

## 4. 任务清单

### M0 文档收口

- [ ] 新建 `README.md`, 覆盖项目定位, 适用机型, 目录结构, 接入方式, 构型选择, 许可证与开发规范命令.
- [ ] 在 `README.md` 中给出 `config.h` 的构型选择示例, 说明使用者需要改哪些项.
- [ ] 在 `README.md` 中说明实现代码的合入状态, 避免读者误以为已有可用代码.
- [ ] 确认 `LICENSE` 版权行的权利人口径, 与 `README.md` 表述一致.
- [ ] 修正 `AGENTS.md` 第 20.1 节, 明确允许根目录文档 (`README.md`, `LICENSE`, `PLAN.md`) 与 `scripts/`.
- [ ] 提交 `README.md` 与 `PLAN.md`.

### M1 `public/` 基础层

交付物:

```
public/Inc/nerv_status.h
public/Inc/nerv_types.h
public/Inc/nerv_utils.h
public/Inc/nerv_math.h
public/Src/nerv_math.c
public/Src/nerv_utils.c
```

- [x] `nerv_status.h`: 定义 `NERV_StatusTypeDef` 与 `AGENTS.md` 第 4.2 节的全部扩展错误码, 错误码只在这里定义.
- [x] `nerv_types.h`: 定义 `NERV_AttitudeTypeDef`, 供底盘航向与装有 IMU 的模块共用. 只含三个角度, 字段注明轴向与正方向, 使用 SI 单位后缀.
- [x] `nerv_utils.h`: 提供 `NERV_UTILS_ARRAY_SIZE`, `NERV_UTILS_TIMEOUT_MS`, `NERV_UTILS_TIMEOUT_S_TO_MS`, 以及 `NERV_Utils_*` 单位转换声明. 宏名写明输入与输出单位.
- [x] `nerv_math.h`: 定义 `NERV_MATH_PI` 与 `NERV_MATH_TWO_PI`, 供角度与转速换算复用.
- [x] `nerv_math.c`: 实现通用区间归一化 `NERV_Math_Wrap`, `NERV_Math_IsClose` 与限幅, 死区函数, 以及三阶线性方程组求解 `NERV_Math_SolveLinear3` (Cramer 法则, 供舵轮正解与力分解使用). 角度归一化使用 `NERV_Math_Wrap(angle, -NERV_MATH_PI, NERV_MATH_PI)`.
- [x] `nerv_math.c`: 实现绕 z 轴的二维旋转, 把云台系的底盘速度指令转到机体系. 角速度分量原样传递.
- [x] `nerv_utils.c`: 实现 `NERV_Utils_RpmToRadps` 与 `NERV_Utils_RadpsToRpm`.
- [x] 头文件自包含, 可被 C 与 C++ 编译器包含.
- [x] 每个对外函数写完整 Doxygen, 头文件声明不写 Doxygen 块.

### M2 `config/` 与日志

交付物:

```
config/Inc/config.h
config/Src/config_check.c
public/Inc/nerv_log.h
public/Src/nerv_log.c
```

- [x] `config.h`: 按 `AGENTS.md` 第 10.2 节落平台选择, `NERV_PLATFORM_NONE` 表示未选择并触发 `#error`.
- [x] `config.h`: 落底盘构型, 云台轴数, 板间链路的宏定义, 全部支持编译期 `#if` 分支; 电机型号改为按型号裁剪, 不再做全局协议单选.
- [x] `config.h`: 定义 `NERV_ENABLE` 与 `NERV_DISABLE`, 全部是否启用类配置统一取这两个值.
- [x] `config.h`: 保留 `NERV_USE_LOG`, `NERV_USE_CHASSIS`, `NERV_USE_GIMBAL`, `NERV_USE_SHOOT`, `NERV_USE_SAFETY`, `NERV_USE_MOTOR` 裁剪开关.
- [x] `config.h`: 增加 `NERV_USE_MOTOR_DJI` 协议族裁剪开关, 可同时启用多个协议族; 同族型号用描述符数据表达, 不按型号条件编译.
- [x] `config.h`: 模块裁剪为 `NERV_DISABLE` 时, 不定义该模块的专属配置宏.
- [x] `config.h`: 关键宏按第 7.8 节分层保护, 用户可覆盖项用 `#ifndef`, 推导项用 `#if defined(...)` 加 `#error`.
- [x] `config_check.c`: 校验平台, 开关取值, 裁剪宏, 电机协议族裁剪, 底盘, 云台, 板间链路与日志配置, 错误码域号与位号, 公共结构体大小.
- [ ] `config_check.c`: 在模块枚举镜像定义后补充宏与枚举镜像数值校验.
- [x] `nerv_log.h`: 级别枚举使用 `_RT` 后缀, 宏使用 `NERV_LOG_LEVEL_*`, 日志宏用 `do { ... } while (0)`.
- [x] `nerv_log.c`: 按 `AGENTS.md` 第 11.5 节实现, `NERV_USE_LOG` 为 `NERV_DISABLE` 时保留同签名空实现.
- [x] 日志模块不操作 UART, 不创建任务, 不包含 RTOS 头文件.

### M3 `bsp/Inc/` 抽象接口

交付物:

```
bsp/Inc/nerv_fdcan.h
bsp/Inc/nerv_gpio.h
bsp/Inc/nerv_pwm.h
bsp/Inc/nerv_time.h
```

- [x] `nerv_fdcan.h`: 按 `AGENTS.md` 第 13.2 节实现帧类型, 过滤器类型, 接收回调, 操作表, 实例结构体.
- [x] `nerv_fdcan.h`: 声明 `Init`, `Start`, `Stop`, `ConfigureFilter`, `Transmit`, `RegisterRxCallback`, `GetError`, `ClearError`, `OnRx`.
- [x] `nerv_fdcan.h`: 帧结构满足经典帧 8 字节与 FDCAN 帧 64 字节两种长度, 过滤项区分标准帧与扩展帧.
- [x] 平台句柄只通过 `platform_data` 传递, 公开接口不出现 HAL 句柄类型.
- [x] `nerv_gpio.h`: 定方向, 上下拉, 电平枚举, 配置结构体, 操作表与 `Init`, `DeInit`, `Write`, `Read` 通用接口, 未含边沿中断.
- [x] `nerv_pwm.h`: 定操作表与 `Init`, `DeInit`, `SetDuty` 通用接口, 占空比取值 0.0 到 1.0, 载波属性归平台配置.
- [x] `nerv_time.h`: 定操作表与 `Init`, `GetTickMs` 通用接口, 毫秒计数按无符号回绕, 间隔计算用无符号减法.

### M4 `module/motor/`

交付物:

```
module/motor/Inc/nerv_motor.h
module/motor/Src/nerv_motor.c
module/motor/driver/nerv_motor_driver.h
module/motor/driver/DJI/nerv_motor_driver_dji.c
```

- [x] 默认参数全部下沉为实例配置字段: 减速比, 离线阈值, 过温阈值与电流限幅是逐实例的硬件事实或机构策略, 同一次编译混用型号时一个宏无法服务多个实例, 因此不设模块级默认宏, `nerv_motor_def.h` 删除.
- [x] 驱动分层: driver 接口在 `nerv_motor_driver.h`, 每个协议族一个 `module/motor/driver/<vendor>/nerv_motor_driver_<vendor>.c`; 族内型号差异全部放进 `NERV_Motor_ModelTypeDef` 描述符, 新增型号只增加一个描述符常量与一个 `NERV_Motor_DriverTypeDef` 实例, 不增加函数与宏; 驱动在实例配置中绑定, 同一次编译允许混用多个协议族的多个型号, 主逻辑不含型号分支, 不再做全局编译期单选.
- [x] 模块公开头文件由 `NERV_USE_MOTOR` 条件编译, 模块关闭时 `nerv_motor_driver.h` 内容为空.
- [x] 协议族驱动整体由 `NERV_USE_MOTOR` 与 `NERV_USE_MOTOR_<VENDOR>` 条件编译, 协议族关闭时驱动源文件不产生代码, `nerv_motor_driver.h` 中对应的驱动符号也不可见.
- [x] `driver/DJI/nerv_motor_driver_dji.c`: DJI CAN 电调协议族实现与 M3508 (C620 电调), M2006 (C610 电调), GM6020 (电流与电压两种控制模式各一个驱动实例) 型号描述符, 帧 ID 分组, 显式大端编解码, 驱动层满量程限幅, 温度有符号透传与参数防御; 控制指令与反馈中控制量字段的编码在描述符中分开表达; 无温度字段的型号 (C610) 以 NERV_MOTOR_FB_OFF_NONE 表达, 解码置 INT8_MIN; 临时数值测试覆盖帧分组, 编解码字节布局与限幅.
- [x] `nerv_motor.h`: 实例结构体, 配置结构体, 机构系测量值类型与对外函数声明. 配置字段含 `driver` 驱动绑定, `bus` 与 `device_id` 传输与标识, `gear_ratio` 减速比 (必须大于 0), `offline_ms` 离线判定阈值 (0 关闭), `over_temperature_c` 过温阈值 (0 关闭), `control_limit` 控制量编码值上限 (量纲同型号的 `control_cmd_limit`, 0 不额外限幅), `invert_direction` 安装方向取反; 控制量的单位与语义由驱动绑定的型号描述符约定, 不限定为电流. 接口: Init (带 config, 电机无硬件初始化阶段, 不设 Register), SetTarget, Update, UpdateGroup, OnRx, GetMeasurement, GetError, ClearError, EnterSafeState. 指令按总线与帧标识符分组发送, 同一总线上的同一标识符每周期只发一帧, 避免同组电调互相清零.
- [x] 反馈解析显式按字节偏移编码与解码, 不把协议缓冲区直接覆盖成结构体.
- [x] 转速换算链路: 原始反馈值, 电机转速, 输出轴转速, 三级量纲分清, 单位换算只调用 `NERV_Utils_*`.
- [x] 每个电机实例维护离线与过温两个错误位, 提供 `GetError` 与 `ClearError`. 过温检查以 `over_temperature_c` 大于 0 为前提; 反馈温度为电机温度, 由电调上报, 可用性取决于温度传感器与电调固件, 数据不可靠的机构关闭该检查, 仅剩电调自身的温度保护兜底.
- [x] 控制量指令按可配置上限限幅, 上限取实例配置的 `control_limit`, 以编码值表达 (量纲同型号描述符的 `control_cmd_limit`, 直接填 16384, 25000 这类数字, 全型号统一, 不按物理单位), 0 表示不额外限幅. 发送时按型号的满量程比例换算为物理量限幅. 过流由电调自身保护兜底, 库不设过流错误位.
- [x] 指令帧按组独立发送: 某组首轮失败当场重发一次, 仍失败置该组全体成员的 `NERV_ERR_MOTOR_TX_FAIL` 错误位并继续发送后续组, 单总线邮箱压力不饿死其他总线, 发送成功后自动清除该位.
- [x] 提供 `NERV_Motor_EnterSafeState`, 清零目标值与输出.
- [x] 通信超时判定使用 `NERV_UTILS_TIMEOUT_MS`, 阈值取实例配置的 `offline_ms`, 0 表示关闭离线判定.
- [x] 模块层只通过 `NERV_FDCAN_InstanceTypeDef` 发送, 不接触寄存器.

### M5 `module/chassis/`

交付物:

```
module/chassis/Inc/nerv_chassis.h
module/chassis/Src/nerv_chassis.c
```

- [x] `nerv_chassis.h`: 定义机体系速度与里程计两个公开类型, 字段注明轴向与正方向, 使用 SI 单位后缀. 姿态复用 `public/Inc/nerv_types.h` 的 `NERV_AttitudeTypeDef`.
- [x] 底盘接口只接受机体系速度. 遥控器的云台系指令由调用方转换, 转换用 `NERV_Math_` 的绕 z 轴旋转.
- [x] 按第 3 节模板落 `NERV_CHASSIS_TYPE` 宏, 枚举镜像与 `_Static_assert` 校验.
- [x] 轮数等派生宏按 `#if` 链推导, 并用 `#if defined(...)` 加 `#error` 禁止使用者覆盖. 三种构型均为 4 轮; 默认构型为全向轮.
- [x] 轮径, 投影距, 扭矩常数, 限幅与速度环增益等可调项按 M4 先例全部下沉为实例配置字段, 由 `NERV_Chassis_Init` 校验范围, 不设模块级默认宏.
- [x] 实现麦克纳姆, 全向, 舵轮三种解算, 每次编译只保留选中的一种. 三种构型均已实现: 全向轮与麦克纳姆轮为固定驱动方向的力分解 (麦轮分解系数 sqrt(2)/4, 杠杆为两投影距之和), 舵轮为动态舵角下的三阶正规方程求解 (正解与力分解共用矩阵, 求解用 public 层 `NERV_Math_SolveLinear3`) 加舵向角度环; 配置提供逐轮指令方向符号位 `wheel_inverted`; 构型间经 `#if` 编译期选择, 无运行期分派.
- [x] 解算函数不使用运行期函数指针分派, 由 `#if` 直接调用.
- [x] 全向轮解算按 4 轮布局: 轮号 0 到 3 依次为左前, 左后, 右后, 右前, 编号方向为绕底盘一周, 左前起始, 符号约定写入头文件契约; 控制框架采用速度环加力分解的三层串级 (运动学正解算, 平移与旋转速度环输出牵引力与转矩, 力分解叠加前馈后换算电流下发轮向电机), 四个固定方向的轮子合成任意牵引力与转矩, 等效实现舵轮框架的合力效果, 不设舵向电机.
- [x] 里程计先推进角度再更新位置, 角度用 `NERV_Math_Wrap` 归一化到 [-π, π). 提供 `NERV_Chassis_SetOdometry` 供上层把视觉或惯性融合结果回写校正; 里程计功能可用编译期宏 `NERV_USE_CHASSIS_ODOMETRY` 整体裁剪.
- [x] 斜坡规划: 目标速度按配置的平移与旋转加速度限幅平滑趋近 (分量独立限变化率, 反向必过零, 0 限幅直通), 规划值由速度环与舵角解算使用.
- [x] 错误位覆盖运动学异常与轮离线.
- [x] 严重故障置错误位并通知已注册回调, 故障路径不调用日志宏.

### M6 `module/gimbal/` 与 `module/shoot/`

交付物:

```
module/gimbal/Inc/nerv_gimbal.h
module/gimbal/Inc/nerv_gimbal_def.h
module/gimbal/Src/nerv_gimbal.c
module/shoot/Inc/nerv_shoot.h
module/shoot/Inc/nerv_shoot_def.h
module/shoot/Src/nerv_shoot.c
```

- [ ] 云台轴数用宏区分, 两轴与三轴按编译期分支实现, 角度数组长度由轴数宏推导.
- [ ] 云台目标角与当前角位于 `[-π, π)`, 越限置 `NERV_ERR_GIMBAL_LIMIT`.
- [ ] 发射按摩擦轮与进弹机构拆分, 摩擦轮目标转速, 进弹速度与弹仓状态各自独立.
- [ ] 发射实现卡弹与离线检测, 状态迁移规则写在头文件契约中.
- [ ] 两个模块都提供 `EnterSafeState`, 满足 `AGENTS.md` 第 17.4 节的安全状态清单.
- [ ] 若引入 PID, 积分项放在模块私有状态, 安全状态与误差超限时清零.

### M7 `module/safety/`

交付物:

```
module/safety/Inc/nerv_safety.h
module/safety/Src/nerv_safety.c
```

- [ ] 定义 `NERV_SAFETY_LEVEL_OK`, `WARNING`, `CRITICAL`, 只能提升不能静默降级.
- [ ] `NERV_Safety_TriggerEStop` 标注 `ISR-safe`, 只置位与记录原因.
- [ ] 急停状态锁存, 只有任务上下文的 `ClearEStop` 能清除.
- [ ] 错误位使用 `NERV_ERR_SAFETY_ESTOP` 与 `NERV_ERR_SAFETY_LATCHED`.
- [ ] 明确本模块不替代硬件急停回路, 不管理看门狗, 不创建监控任务.

### M8 `bsp/stm32h7/` 参考实现

交付物:

```
bsp/stm32h7/Inc/nerv_stm32h7.h
bsp/stm32h7/Src/nerv_fdcan_stm32h7.c
```

- [ ] `nerv_stm32h7.h` 包含 `stm32h7xx_hal.h`, 只由 BSP 与应用层包含, 提供平台初始化入口并把 HAL 句柄绑定到 `platform_data`.
- [ ] `nerv_fdcan_stm32h7.c` 实现 `NERV_FDCAN_OpsTypeDef` 全部函数.
- [ ] 接收中断中填充 `NERV_FDCAN_FrameTypeDef` 并调用 `NERV_FDCAN_OnRx`.
- [ ] 发送无空闲邮箱时返回 `NERV_BUSY`.
- [ ] 覆盖 `HAL_FDCAN_RxFifo0Callback` 等 HAL weak 回调, 只用于 BSP 对接.
- [ ] 不引入 HAL 源文件, CubeMX 生成文件与启动文件.

### M9 发布收尾

- [ ] 按实际实现更新 `README.md` 的模块清单, 支持的构型与平台, 接入步骤.
- [ ] 实现过程中形成的新约定同步补进 `AGENTS.md`.
- [ ] 打首个语义化版本 tag.

---

## 5. 每个模块的做法

- [ ] 读 `AGENTS.md` 中与该模块相关的节.
- [ ] 先写头文件: 结构体, 配置结构体, 对外函数声明.
- [ ] 确认需要的错误位已在 `public/Inc/nerv_status.h` 中定义, 缺少时先补错误码.
- [ ] 实现 `.c`, Doxygen 写在定义处, 参数标注 `[in]`, `[out]`, `[in,out]`.
- [ ] 参数检查顺序按第 15.4 节: 指针, 数值范围, 初始化状态, 硬件状态, 执行, 更新错误位.
- [ ] 标注线程模型, 默认 `NOT ISR-safe` 与 `caller-synchronized`.
- [ ] 错误位在中断置位, 任务清除时使用临界区或原子操作.
- [ ] 需要编译期选择的部分按第 3 节的三层结构落, 不留运行期开关.
- [ ] 自检与提交.

---

## 6. 验证方式

仓库不提供构建系统, 验证用临时命令完成, 不提交构建产物.

语法检查, 在仓库根目录执行:

```sh
gcc -std=c11 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -fsyntax-only -Ipublic/Inc -Iconfig/Inc -Ibsp/Inc -Imodule/motor/Inc public/Src/nerv_math.c
```

- [ ] 每新增一个源文件, 至少用它自己的 include path 做一次 `-fsyntax-only`.
- [ ] 头文件单独用 `gcc -x c -fsyntax-only` 编译一次, 确认自包含.
- [ ] 每种编译期构型至少编译一次, 确认未选中的实现不产生符号, 例如切换 `NERV_CHASSIS_TYPE` 后检查目标文件中的符号表.
- [ ] STM32 BSP 需要 HAL 头文件, 属于使用者工程. 本地检查用临时 HAL 声明桩, 放在仓库外的临时目录.
- [ ] 功能验证在仓库外的临时工程中进行, 验证完删除.

文本与差异检查:

```sh
python3 scripts/check_style.py .
git diff --check
```

格式检查:

```sh
clang-format --dry-run --Werror public/Inc/nerv_status.h
```

提交前的完整检查项见 `AGENTS.md` 第 19.3 节.

---

## 7. 待确认的问题

- [ ] 电机型号与协议: 目标电机列表, 电调型号, 帧 ID 分配, 反馈字段比例与量程, 各机型减速比. 减速比等硬件参数由使用者在实例配置中逐台填写, 不设库级默认.
- [ ] 底盘构型: 已定为全向轮 (`config.h` 默认 `NERV_CHASSIS_TYPE_OMNI`), 4 个轮向电机, 轮号 0 到 3 为左前, 左后, 右后, 右前; 轮径, 轮距, 电机型号, 减速比与扭矩常数待定. 控制框架为三层串级: 运动学正解算, 平移与旋转速度环 (输出牵引力与转矩), 力分解与前馈叠加, 不设舵向电机.
- [ ] 云台形态: 两轴还是三轴, 限位来源, 是否有双板.
- [ ] 发射机构形态: 摩擦轮与进弹机构的组合, 卡弹判定依据.
- [ ] 板间通信: 底盘, 云台, 发射之间的链路拓扑与报文走向.
- [ ] 时间源: 周期更新的时间间隔由使用者传入, 还是由 BSP 时间接口提供.
- [ ] 权利人口径: `LICENSE` 版权行写 `NERV Contributors`, 还是战队的正式英文名.
- [ ] 桌面端自检 BSP: 是否提供一个宿主平台的模拟 BSP 用于本地编译与回归, 需确认是否与库的定位冲突.
- [ ] 可选外设: 除 FDCAN 外是否真的需要 GPIO, PWM, 时间接口.
- [ ] 版本与发布方式: 语义化版本号规则, 仓库是否公开, 首个 tag 时间.
