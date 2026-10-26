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
    float angle_rad;      /**< 转子机械角度, 单位 rad, 逆时针为正, 范围 [0, 2π). */
    float speed_radps;    /**< 转子转速, 单位 rad/s, 逆时针为正. */
    float current_a;      /**< 实际输出电流, 即转矩电流, 单位 A, 正值对应转速正方向. */
    float torque_n_m;     /**< 转子输出转矩, 单位 N·m, 型号不上报时置 0. */
    int8_t temperature_c; /**< 电机温度, 单位 ℃. 由电调上报, 可用性取决于温度传感器与电调固件. */
    int8_t temperature_mos_c; /**< 驱动器 MOS 温度, 单位 ℃, 型号不上报时置 INT8_MIN. */
    uint8_t error_code;       /**< 驱动器故障码原值, 型号不上报时置 0. */
} NERV_Motor_FbTypeDef;

/**
  * @brief  控制量种类.
  * @note   与 NERV_Motor_CmdTypeDef 的联合体成员一一对应, 驱动按本值选择解码分支.
  */
typedef enum
{
    NERV_MOTOR_CMD_SCALAR = 0U,       /**< 单一控制量, 单位由型号描述符约定 (如电流, 电压或速度) */
    NERV_MOTOR_CMD_MIT,               /**< MIT 模式, 位置/速度/刚度/阻尼/前馈转矩 */
    NERV_MOTOR_CMD_POSITION_VELOCITY, /**< 位置速度模式, 目标位置与限速 */
} NERV_Motor_CmdKindTypeDef;

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
    NERV_Motor_CmdKindTypeDef kind; /**< 有效分支标识 */
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
} NERV_Motor_CmdTypeDef;

/**
  * @brief  描述符中表示反馈帧无此字段的偏移值.
  * @note   帧数据长度不超过 64, 0xFF 不会是有效偏移.
  */
#define NERV_MOTOR_FB_OFF_NONE 0xFFU

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
        tx_id_offset_by_device; /**< true 表示控制帧标识符等于 tx_id_low 加设备 id 偏移, false 表示按低组/高组选帧 */
    bool
        rx_id_offset_by_device; /**< true 表示反馈帧标识符等于 rx_id_base 加设备 id, false 表示固定为 rx_id_base */
    uint8_t
        control_mode; /**< 控制模式, 取 NERV_Motor_CmdKindTypeDef, 决定 control 联合体的有效成员 */
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
#define NERV_Motor_CmdFromPhysical(_model, _physical)                 \
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
  *                      字段时置 INT8_MIN. ISR-safe.
  */
typedef struct
{
    NERV_StatusTypeDef (*get_frame_geo)(const NERV_Motor_ModelTypeDef *model, uint8_t device_id,
                                        NERV_Motor_FrameGeoTypeDef *geo);
    NERV_StatusTypeDef (*encode_tx)(const NERV_Motor_ModelTypeDef *model, uint8_t *frame,
                                    uint8_t device_id, const NERV_Motor_CmdTypeDef *cmd);
    NERV_StatusTypeDef (*decode_fb)(const NERV_Motor_ModelTypeDef *model, const uint8_t *frame,
                                    uint8_t len, NERV_Motor_FbTypeDef *fb);
} NERV_Motor_Driver_OpsTypeDef;

/**
  * @brief  电机驱动, 型号绑定单元.
  * @note   每个型号一个常量实例, 把协议族操作表与型号描述符打包. 实例配置绑定一个
  *         指针即可使用, 绑定后不再更改.
  */
typedef struct
{
    const NERV_Motor_Driver_OpsTypeDef *ops; /**< 协议族操作表 */
    const NERV_Motor_ModelTypeDef *model;    /**< 型号描述符 */
} NERV_Motor_DriverTypeDef;

#if (NERV_USE_MOTOR_DJI == NERV_ENABLE)

/**
  * @brief DJI M3508 电机 (C620 电调) 驱动.
  * @note  由 module/motor/driver/DJI/nerv_motor_driver_dji.c 提供.
  * @note  只在 NERV_USE_MOTOR_DJI 为 NERV_ENABLE 时声明.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji3508;

/**
  * @brief DJI M2006 电机 (C610 电调) 驱动.
  * @note  由 module/motor/driver/DJI/nerv_motor_driver_dji.c 提供.
  * @note  只在 NERV_USE_MOTOR_DJI 为 NERV_ENABLE 时声明.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji2006;

/**
  * @brief DJI GM6020 电机电流控制模式驱动.
  * @note  由 module/motor/driver/DJI/nerv_motor_driver_dji.c 提供.
  * @note  只在 NERV_USE_MOTOR_DJI 为 NERV_ENABLE 时声明.
  * @warning 反馈帧标识符区间 0x205 到 0x20B 与 C620, C610 电调 id 5 到 8 的反馈区间
  *           0x205 到 0x208 重叠, 同总线混编时会互相误收, 使用时必须错开设备 id
  *           或避开混编.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji6020Current;

/**
  * @brief DJI GM6020 电机电压控制模式驱动.
  * @note  由 module/motor/driver/DJI/nerv_motor_driver_dji.c 提供.
  * @note  只在 NERV_USE_MOTOR_DJI 为 NERV_ENABLE 时声明.
  * @warning 电压模式控制帧标识符 0x1FF 与 C620 电调高组指令标识符相同, 混编
  *           总线上会冲突, 使用时必须错开设备 id 或避开电压模式.
  * @warning 反馈帧标识符区间 0x205 到 0x20B 与 C620, C610 电调 id 5 到 8 的反馈区间
  *           0x205 到 0x208 重叠, 同总线混编时会互相误收, 使用时必须错开设备 id
  *           或避开混编.
  * @warning 本型号的控制量单位是协议给定值 (满量程 25000, 无量纲), 不是电流. 电机实例
  *           配置的 control_limit 按同单位解释, 按电流安培数配置会得到近乎为零的输出.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji6020Voltage;

#endif /* NERV_USE_MOTOR_DJI */

#if (NERV_USE_MOTOR_DM == NERV_ENABLE)

/**
  * @brief 达妙 DM-J4310-2EC MIT 模式驱动.
  * @note  由 module/motor/driver/DM/nerv_motor_driver_dm.c 提供.
  * @note  控制帧标识符为 0x000 加设备 id, 反馈帧标识符由 Master ID 配置.
  * @note  控制量为 NERV_MOTOR_CMD_MIT, 位置/速度/刚度/阻尼/前馈转矩位打包进 8 字节.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmMit;

/**
  * @brief 达妙 DM-J4310-2EC 速度模式驱动.
  * @note  由 module/motor/driver/DM/nerv_motor_driver_dm.c 提供.
  * @note  控制帧标识符为 0x200 加设备 id, 控制量为 NERV_MOTOR_CMD_SCALAR, 单位 rad/s.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmVelocity;

/**
  * @brief 达妙 DM-J4310-2EC 位置速度模式驱动.
  * @note  由 module/motor/driver/DM/nerv_motor_driver_dm.c 提供.
  * @note  控制帧标识符为 0x100 加设备 id, 控制量为 NERV_MOTOR_CMD_POSITION_VELOCITY.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmPositionVelocity;

/** @brief 达妙特殊命令码, 占用 8 字节帧的最后一字节, 前 7 字节为 0xFF. */
#define NERV_MOTOR_DM_CMD_CLEAR_ERROR 0xFBU /**< 清除故障 */
#define NERV_MOTOR_DM_CMD_ENABLE      0xFCU /**< 使能电机 */
#define NERV_MOTOR_DM_CMD_DISABLE     0xFDU /**< 失能电机 */
#define NERV_MOTOR_DM_CMD_SAVE_ZERO   0xFEU /**< 当前位置保存为零点 */

/**
  * @brief  发送达妙特殊命令帧.
  * @param  bus 电机所在总线, 不得为 NULL.
  * @param  model 型号描述符, 不得为 NULL.
  * @param  device_id 电机 CAN ID, 范围 model 的 id_min 到 id_max.
  * @param  command 命令码, 取 NERV_MOTOR_DM_CMD_*.
  * @retval NERV_OK 帧已交给总线.
  * @retval NERV_ERROR 参数为 NULL, 设备 id 越界, 或总线发送失败.
  * @note   命令帧的 CAN ID 等于当前控制模式的控制帧 ID, 数据为 7 字节 0xFF 加命令码.
  * @note   电机必须使能后才能执行控制帧; 掉线后需要重新使能.
  * @note   NOT ISR-safe. 经平台适配层发送.
  */
NERV_StatusTypeDef NERV_Motor_Dm_SendCommand(const NERV_FDCAN_InstanceTypeDef *bus,
                                             const NERV_Motor_ModelTypeDef *model, uint8_t device_id,
                                             uint8_t command);

#endif /* NERV_USE_MOTOR_DM */

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_MOTOR */

#endif /* NERV_MOTOR_DRIVER_H */
