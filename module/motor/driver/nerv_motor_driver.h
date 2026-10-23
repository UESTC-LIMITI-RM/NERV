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

#include <stdint.h>

#include "config.h"

#if (NERV_USE_MOTOR == NERV_ENABLE)

#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  电机反馈.
  * @note   只承载电机转子量的物理值. 反馈帧原始编码不进入本结构, 输出轴量按
  *         减速比由模块内部另行换算.
  */
typedef struct NERV_Motor_Fb
{
    float angle_rad;      /**< 转子机械角度, 单位 rad, 逆时针为正, 范围 [0, 2π). */
    float speed_radps;    /**< 转子转速, 单位 rad/s, 逆时针为正. */
    float current_a;      /**< 实际输出转矩, 电调以电流值上报, 单位 A, 正值对应转速正方向. */
    int8_t temperature_c; /**< 电调温度, 单位 ℃. 来自电调上报, 可用性取决于电调固件与机构. */
} NERV_Motor_FbTypeDef;

/**
  * @brief  电机型号描述符.
  * @note   承载同一协议族内不同型号的全部差异: 标识符映射, 帧长, 反馈字段布局与
  *         刻度. 协议族代码只读描述符, 不包含型号分支.
  * @note   描述符由协议族驱动以常量给出, 实例只读, 不得修改.
  * @note   控制帧与反馈帧标识符均为 11 位标准帧标识符, 高位必须为 0.
  * @note   控制量的单位与语义由协议族约定. 控制指令与反馈中的控制量字段分别由
  *         control_full_scale 与 fb_control_full_scale 给出编码, 两者相互独立.
  */
typedef struct
{
    uint32_t tx_id_low;          /**< 控制帧标识符, 设备 id 较小的一组 */
    uint32_t tx_id_high;         /**< 控制帧标识符, 设备 id 较大的一组 */
    uint32_t rx_id_base;         /**< 反馈帧标识符基值, 反馈标识符等于本值加设备 id */
    uint8_t id_min;              /**< 设备 id 最小值 */
    uint8_t id_max;              /**< 设备 id 最大值 */
    uint8_t motors_per_frame;    /**< 单条控制帧控制的电机数量 */
    uint8_t frame_len;           /**< 控制帧与反馈帧数据长度, 单位 byte */
    uint8_t fb_angle_off;        /**< 反馈帧转子角度字段偏移, uint16 大端 */
    uint8_t fb_speed_off;        /**< 反馈帧转速字段偏移, int16 大端, 单位 rpm */
    uint8_t fb_torque_off;       /**< 反馈帧实际输出转矩字段偏移, int16 大端 */
    uint8_t fb_temperature_off;  /**< 反馈帧温度字段偏移, uint8, 单位 ℃ */
    uint16_t angle_count;        /**< 转子每圈编码数, 角度字段值域 0 到本值减 1 */
    float control_full_scale;    /**< 控制指令满量程, 单位由协议族约定, 编码满量程对应本值 */
    int16_t control_cmd_limit;   /**< 控制指令编码满量程 */
    float fb_control_full_scale; /**< 反馈中控制量字段的满量程, 单位由协议族约定 */
    int16_t fb_control_cmd_limit; /**< 反馈中控制量字段的编码满量程 */
} NERV_Motor_ModelTypeDef;

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
  *         - encode_tx: 把一个设备的控制量编码到控制帧内该设备对应的位置, 帧缓冲
  *                       由调用方清零后传入, 驱动只写入该设备占用的字节, 失败时不
  *                       修改帧缓冲. 本函数只按 control_full_scale 限幅到协议满
  *                       量程, 实例级限幅由调用方在调用前完成. ISR-safe.
  *         - decode_fb: 把反馈帧解码为电机反馈, len 必须等于 model 的 frame_len,
  *                      失败时不修改输出结构. ISR-safe.
  */
typedef struct
{
    NERV_StatusTypeDef (*get_frame_geo)(const NERV_Motor_ModelTypeDef *model,
                                        uint8_t device_id,
                                        NERV_Motor_FrameGeoTypeDef *geo);
    NERV_StatusTypeDef (*encode_tx)(const NERV_Motor_ModelTypeDef *model, uint8_t *frame,
                                    uint8_t device_id, float control);
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
    const NERV_Motor_Driver_OpsTypeDef *ops;  /**< 协议族操作表 */
    const NERV_Motor_ModelTypeDef *model;     /**< 型号描述符 */
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
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji6020Current;

/**
  * @brief DJI GM6020 电机电压控制模式驱动.
  * @note  由 module/motor/driver/DJI/nerv_motor_driver_dji.c 提供.
  * @note  只在 NERV_USE_MOTOR_DJI 为 NERV_ENABLE 时声明.
  * @warning 电压模式控制帧标识符 0x1FF 与 C620 电调高组指令标识符相同, 混编
  *           总线上会冲突, 使用时必须错开设备 id 或避开电压模式.
  */
extern const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji6020Voltage;

#endif /* NERV_USE_MOTOR_DJI */

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_MOTOR */

#endif /* NERV_MOTOR_DRIVER_H */
