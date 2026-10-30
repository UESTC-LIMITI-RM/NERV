/**
  * @file    nerv_motor_driver.h
  * @brief   NERV 电机驱动接口.
  * @version 1.1.0
  * @date    2026-10-23
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_MOTOR_DRIVER_H
#define NERV_MOTOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

#if (NERV_USE_MOTOR == NERV_ENABLE)

#include "nerv_fdcan.h"
#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  电机反馈.
  * @note   只承载电机转子量的物理值. 反馈帧原始编码不进入本结构, 输出轴量按
  *         减速比由模块内部另行换算.
  * @note   temperature_c 在型号无温度字段时为 INT8_MIN, 见 NERV_MOTOR_FB_OFF_NONE.
  */
typedef struct NERV_Motor_Fb
{
    float angle_rad;   /**< 转子机械角度, 单位 rad, 逆时针为正, 范围 [0, 2π). */
    float speed_radps; /**< 转子转速, 单位 rad/s, 逆时针为正. */
    float current_a; /**< 实际输出电流, 即转矩电流, 单位 A, 正值对应转速正方向. 型号不上报时置 0. */
    float torque_n_m;     /**< 转子输出转矩, 单位 N·m, 型号不上报时置 0. */
    int8_t temperature_c; /**< 电机温度, 单位 ℃. 由电调上报, 可用性取决于温度传感器与电调固件. */
    int8_t temperature_mos_c; /**< 驱动器 MOS 温度, 单位 ℃, 型号不上报时置 INT8_MIN. */
    uint8_t error_code;       /**< 驱动器故障码原值, 型号不上报时置 0. */
} NERV_Motor_FbTypeDef;

/**
  * @brief  控制量种类.
  * @note   与 NERV_Motor_ControlTypeDef 的联合体成员一一对应, 驱动按本值选择解码分支.
  */
typedef enum
{
    NERV_MOTOR_CONTROL_SCALAR = 0U, /**< 单一控制量, 单位由型号描述符约定 (如电流, 电压或速度) */
    NERV_MOTOR_CONTROL_MIT,         /**< MIT 模式, 位置/速度/刚度/阻尼/前馈转矩 */
    NERV_MOTOR_CONTROL_POSITION_VELOCITY, /**< 位置速度模式, 目标位置与限速 */
} NERV_Motor_ControlKindTypeDef;

/**
  * @brief  电机控制目标.
  * @note   联合体只保留一份有效成员, 由 kind 标识. 各驱动只读取自己支持的分支,
  *         kind 不受支持时返回 NERV_ERROR.
  * @note   SCALAR 分支用于只接受单一控制量的协议, 物理量由型号描述符的
  *         control_full_scale 约定, 例如 M3508 是电流, 6020 电压版是电压;
  *         MIT 分支用于达妙等支持位置, 速度, 刚度, 阻尼与前馈转矩的协议.
  * @note   MIT 的 position_rad 与 velocity_radps 为转子量, kp 与 kd 为协议约定单位,
  *         torque_n_m 为前馈转矩, 单位 N·m.
  */
typedef struct
{
    NERV_Motor_ControlKindTypeDef kind; /**< 有效分支标识 */
    union
    {
        float scalar; /**< SCALAR 分支, 单位由型号描述符 control_full_scale 约定 */
        struct
        {
            float position_rad;   /**< 目标位置, 单位 rad */
            float velocity_radps; /**< 目标速度, 单位 rad/s */
            float kp;             /**< 位置刚度, 单位由协议约定 */
            float kd;             /**< 速度阻尼, 单位由协议约定 */
            float torque_n_m;     /**< 前馈转矩, 单位 N·m */
        } mit;
        struct
        {
            float position_rad;   /**< 目标位置, 单位 rad */
            float velocity_radps; /**< 限速, 单位 rad/s */
        } position_velocity;
    } u;
} NERV_Motor_ControlTypeDef;

/**
  * @brief  描述符中表示反馈帧无此字段的偏移值.
  * @note   帧数据长度不超过 64, 0xFF 不会是有效偏移.
  */
#define NERV_MOTOR_FB_OFF_NONE 0xFFU

/**
  * @brief  SCALAR 控制量的物理量纲.
  * @note   仅 SCALAR 控制模式有效, 用于区分 control.scalar.full_scale 的单位.
  */
typedef enum
{
    NERV_MOTOR_SCALAR_CURRENT = 0U, /**< 电流, 单位 A */
    NERV_MOTOR_SCALAR_SPEED,        /**< 角速度, 单位 rad/s */
    NERV_MOTOR_SCALAR_VOLTAGE,      /**< 电压, 单位由协议约定 */
} NERV_Motor_ScalarQuantityTypeDef;

/**
  * @brief  电机型号描述符.
  * @note   承载同一协议族内不同型号的全部差异: 标识符映射, 帧长, 反馈字段布局与
  *         刻度. 协议族代码只读描述符, 不包含型号分支.
  * @note   描述符由协议族驱动以常量给出, 实例只读, 不得修改.
  * @note   控制帧与反馈帧标识符均为 11 位标准帧标识符, 高位必须为 0.
  * @note   控制量的单位与语义由型号描述符决定, 同一协议族的不同控制模式可以不同.
  *         控制指令与反馈中的控制量字段分别由 control_full_scale 与
  *         fb_control_full_scale 给出编码, 两者相互独立.
  */
typedef struct
{
    uint32_t tx_id_low;           /**< 控制帧标识符, 设备 id 较小的一组 */
    uint32_t tx_id_high;          /**< 控制帧标识符, 设备 id 较大的一组 */
    uint32_t rx_id_base;          /**< 反馈帧标识符基值, 反馈标识符等于本值加设备 id */
    uint8_t id_min;               /**< 设备 id 最小值 */
    uint8_t id_max;               /**< 设备 id 最大值 */
    uint8_t motors_per_frame;     /**< 单条控制帧控制的电机数量 */
    uint8_t frame_len;            /**< 控制帧与反馈帧数据长度, 单位 byte */
    uint8_t fb_angle_off;         /**< 反馈帧转子角度字段偏移, uint16 大端 */
    uint8_t fb_speed_off;         /**< 反馈帧转速字段偏移, int16 大端 */
    uint8_t fb_current_off;       /**< 反馈帧实际输出电流字段偏移, int16 大端 */
    uint8_t fb_temperature_off;   /**< 反馈帧温度字段偏移, 无字段时取 NERV_MOTOR_FB_OFF_NONE */
    uint16_t angle_count;         /**< 转子每圈编码数, 角度字段值域 0 到本值减 1 */
    float fb_control_full_scale;  /**< 反馈中控制量字段的满量程, 单位由协议族约定 */
    int16_t fb_control_cmd_limit; /**< 反馈中控制量字段的编码满量程 */
    bool fb_bit_packed;           /**< true 表示反馈为协议自定义位打包, fb_*_off 不适用 */
    bool
        tx_id_offset_by_device; /**< true 表示控制帧标识符等于 tx_id_low 加设备 id (不减去 id_min), false 表示按低组/高组选帧 */
    bool
        rx_id_offset_by_device; /**< true 表示反馈帧标识符等于 rx_id_base 加设备 id, false 表示固定为 rx_id_base */
    float torque_constant_n_m_per_a; /**< 手册给出的扭矩常数, 单位 N·m/A, 0 表示未提供 */
    float rated_gear_ratio;          /**< 手册标称减速比, 无减速箱填 1, 必须大于 0 */
    bool fb_torque_present; /**< true 表示反馈直接给出转矩, false 表示由电流乘扭矩常数换算 */
    uint8_t
        control_mode; /**< 控制模式, 取 NERV_Motor_ControlKindTypeDef, 决定 control 联合体的有效成员 */
    NERV_Motor_ScalarQuantityTypeDef
        scalar_quantity; /**< SCALAR 控制量量纲, 仅 SCALAR 模式有效, 决定 full_scale 的单位 */
    union
    {
        struct
        {
            float full_scale;  /**< 单一控制量满量程, 单位由协议族约定 */
            int16_t cmd_limit; /**< 单一控制量编码满量程 */
        } scalar;
        struct
        {
            float p_max_rad;   /**< 位置满量程, 单位 rad */
            float v_max_radps; /**< 速度满量程, 单位 rad/s */
            float t_max_n_m;   /**< 转矩满量程, 单位 N·m */
            float kp_max;      /**< 位置刚度满量程 */
            float kd_max;      /**< 速度阻尼满量程 */
        } profile;
    } control;
} NERV_Motor_ModelTypeDef;

/**
  * @brief  把控制量物理量换算为电机控制编码.
  * @note   线性比例 control.scalar.cmd_limit / control.scalar.full_scale. 物理量的单位
  *         必须与 control.scalar.full_scale 一致 (电流模式为 A, 电压模式为协议给定值).
  *         结果为有符号控制编码, 范围约 ±control.scalar.cmd_limit. 只适用于 SCALAR 控制
  *         模式, 多字段模式由驱动自行换算.
  */
#define NERV_Motor_ControlFromPhysical(_model, _physical)             \
    ((float)(_physical) * (float)(_model)->control.scalar.cmd_limit / \
     (_model)->control.scalar.full_scale)

/**
  * @brief  电机帧几何信息.
  * @note   由 get_frame_geo 一次给出, 调用方无需分多次查询标识符与长度.
  */
typedef struct
{
    uint32_t tx_id; /**< 控制帧标识符 */
    uint32_t rx_id; /**< 反馈帧标识符 */
    uint8_t len;    /**< 帧数据长度, 单位 byte */
} NERV_Motor_FrameGeoTypeDef;

/**
  * @brief  驱动可选能力, 每位一项能力, 可组合成位掩码.
  * @note   驱动在描述符的 capabilities 中声明支持的能力, 核心层按位判断.
  *         声明与操作表函数指针必须一致, NERV_Motor_Init 会交叉校验.
  */
typedef enum
{
    NERV_MOTOR_CAP_ENABLE      = 1U << 0, /**< 使能 */
    NERV_MOTOR_CAP_DISABLE     = 1U << 1, /**< 失能 */
    NERV_MOTOR_CAP_CLEAR_FAULT = 1U << 2, /**< 清除故障 */
    NERV_MOTOR_CAP_SET_MODE    = 1U << 3, /**< 运行期切换控制模式 */
    NERV_MOTOR_CAP_SAVE_ZERO   = 1U << 4, /**< 保存当前位置为零点 */
} NERV_Motor_CapabilityTypeDef;

/**
  * @brief  电机驱动操作表.
  * @note   每个协议族一个操作表常量, 由该族的驱动源文件给出. 同一次编译允许混用
  *         多个协议族的多个型号, 主逻辑不包含型号分支.
  * @note   成员契约:
  *         - get_frame_geo: 由设备 id 给出控制帧标识符, 反馈帧标识符与帧长.
  *                          model 或 geo 为 NULL, device_id 越界时返回 NERV_ERROR
  *                          且不修改 geo. ISR-safe.
  *         - encode_tx: 把一个设备的控制编码编码到控制帧内该设备对应的位置, 帧缓冲
  *                       由调用方清零后传入, 驱动只写入该设备占用的字节, 失败时不
  *                       修改帧缓冲. 本函数只按 control_cmd_limit 把控制编码限幅到
  *                       协议满量程, 实例级限幅由调用方在调用前完成. ISR-safe.
  *         - decode_fb: 把反馈帧解码为电机反馈, len 必须等于 model 的 frame_len,
  *                      失败时不修改输出结构. 温度按有符号 int8 透传, 型号无温度
  *                      字段时置 INT8_MIN. 型号不上报的电流, 转矩, MOS 温度与故障码
  *                      必须显式置 0 或 INT8_MIN, 不得保留未定义值. ISR-safe.
  */
typedef struct
{
    NERV_StatusTypeDef (*get_frame_geo)(const NERV_Motor_ModelTypeDef *model, uint8_t device_id,
                                        NERV_Motor_FrameGeoTypeDef *geo);
    NERV_StatusTypeDef (*encode_tx)(const NERV_Motor_ModelTypeDef *model, uint8_t *frame,
                                    uint8_t device_id, const NERV_Motor_ControlTypeDef *control);
    NERV_StatusTypeDef (*decode_fb)(const NERV_Motor_ModelTypeDef *model, const uint8_t *frame,
                                    uint8_t len, NERV_Motor_FbTypeDef *fb);

    /* 可选能力, 驱动不支持时置 NULL, 核心层按空操作处理. */
    NERV_StatusTypeDef (*enable)(const NERV_FDCAN_InstanceTypeDef *bus,
                                 const NERV_Motor_ModelTypeDef *model, uint8_t device_id);
    NERV_StatusTypeDef (*disable)(const NERV_FDCAN_InstanceTypeDef *bus,
                                  const NERV_Motor_ModelTypeDef *model, uint8_t device_id);
    NERV_StatusTypeDef (*clear_fault)(const NERV_FDCAN_InstanceTypeDef *bus,
                                      const NERV_Motor_ModelTypeDef *model, uint8_t device_id);
    NERV_StatusTypeDef (*save_zero)(const NERV_FDCAN_InstanceTypeDef *bus,
                                    const NERV_Motor_ModelTypeDef *model, uint8_t device_id);
    NERV_StatusTypeDef (*set_mode)(const NERV_FDCAN_InstanceTypeDef *bus,
                                   const NERV_Motor_ModelTypeDef *model, uint8_t device_id,
                                   NERV_Motor_ControlKindTypeDef mode);
} NERV_Motor_Driver_OpsTypeDef;

/**
  * @brief  电机驱动, 型号绑定单元.
  * @note   每个型号一个常量实例, 把协议族操作表与型号描述符打包. 实例配置绑定一个
  *         指针即可使用, 绑定后不再更改.
  */
typedef struct
{
    uint32_t capabilities;                   /**< NERV_MOTOR_CAP_* 位掩码, 声明支持的可选能力 */
    const NERV_Motor_Driver_OpsTypeDef *ops; /**< 协议族操作表 */
    const NERV_Motor_ModelTypeDef *model;    /**< 型号描述符 */
} NERV_Motor_DriverTypeDef;

NERV_StatusTypeDef NERV_Motor_Driver_NotSupported(const NERV_FDCAN_InstanceTypeDef *bus,
                                                  const NERV_Motor_ModelTypeDef *model,
                                                  uint8_t device_id);

NERV_StatusTypeDef NERV_Motor_Driver_NotSupportedSetMode(const NERV_FDCAN_InstanceTypeDef *bus,
                                                         const NERV_Motor_ModelTypeDef *model,
                                                         uint8_t device_id,
                                                         NERV_Motor_ControlKindTypeDef mode);

/* 具体驱动声明, 由驱动层统一汇总, 使用者包含本文件即可拿到全部驱动常量. */
#if (NERV_USE_MOTOR_DJI == NERV_ENABLE)
#include "nerv_motor_driver_dji.h"
#endif

#if (NERV_USE_MOTOR_DM == NERV_ENABLE)
#include "nerv_motor_driver_dm.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_MOTOR */

#endif /* NERV_MOTOR_DRIVER_H */
