/**
  * @file    nerv_motor.h
  * @brief   NERV 电机模块公开接口.
  * @version 1.0.0
  * @date    2026-10-23
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_MOTOR_H
#define NERV_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

#if (NERV_USE_MOTOR == NERV_ENABLE)

#include "nerv_fdcan.h"
#include "nerv_motor_driver.h"
#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  电机实例配置.
  * @note   由调用方以静态存储提供, 生命周期必须覆盖实例生命周期.
  * @note   driver 与 bus 指向的对象由调用方提供, 绑定后只读.
  * @note   控制量的单位与语义由 driver 绑定的型号描述符约定, 本结构体不限定为电流.
  * @note   一个电机实例只能由一个消费者持有. 多个消费者共用同一实例会互相覆盖控制
  *         目标与监督计时, 导致控制不可预期.
  */
typedef struct
{
    const NERV_Motor_DriverTypeDef *driver; /**< 驱动绑定, 决定协议族与控制模式 */
    const NERV_FDCAN_InstanceTypeDef *bus;  /**< FDCAN 实例, 指令发送与反馈接收的通道 */
    uint8_t device_id;  /**< 电调 id, 合法范围由驱动描述符的 id_min 与 id_max 决定 */
    uint32_t master_id; /**< 电调侧配置的反馈帧标识符, 由驱动写入帧几何信息.
                             电调可独立配置反馈标识符的协议族按本值匹配反馈帧, 同一总线上的
                             多个实例必须填写互不相同的值, 否则同一帧会被多个实例解码.
                             电调不可配置反馈标识符的协议族忽略本字段, 由型号与 device_id
                             推导, 例如 DJI. 取值可以为 0, 与达妙电调的出厂默认一致 */
    float gear_ratio;   /**< 减速比, 必须大于 0, 输出轴角速度等于转子角速度除以本值 */
    uint32_t
        offline_ms; /**< 离线判定阈值, 单位 ms, 取大于 0 的有限值启用, NERV_DISABLE 表示关闭离线判定 */
    float
        over_temperature_c; /**< 电机过温阈值, 单位 ℃, 取大于 0 的有限值启用, NERV_DISABLE 表示关闭过温检查 */
    float control_limit; /**< 控制量物理量上限, 单位与型号描述符的 scalar_quantity 一致
                              (电流为 A, 电压为 V, 转速为 rad/s), 取 0 表示不额外限幅 */
    bool invert_direction; /**< 安装方向取反, 指令与反馈统一乘符号 */
} NERV_Motor_ConfigTypeDef;

/**
  * @brief  电机测量值, 机构系.
  * @note   角度与角速度已按减速比换算到输出轴, 角度, 角速度与电流按安装方向
  *         取反, 温度原值透传, 正方向为机构正方向.
  * @note   角度不做归一化, 原样保留驱动解码出的量并只做减速比与方向换算. 达妙反馈
  *         落在 ±P_MAX, DJI 编码器量本身落在 [0, 2π).
  * @note   temperature_c 为 INT8_MIN 表示该型号无温度字段, 见 NERV_MOTOR_FB_OFF_NONE.
  */
typedef struct NERV_Motor_Measurement
{
    float angle_rad;   /**< 输出轴机械角度, 单位 rad, 机构正方向为正, 未归一化 */
    float speed_radps; /**< 输出轴角速度, 单位 rad/s, 机构正方向为正 */
    float torque_n_m; /**< 输出轴转矩, 单位 N·m, 机构正方向为正. 型号不直接上报时由
                               电流乘扭矩常数换算, 型号未提供扭矩常数时为 0 */
    int8_t temperature_c; /**< 电机温度, 单位 ℃ */
} NERV_Motor_MeasurementTypeDef;

/**
  * @brief  电机实例.
  * @note   由调用方以静态存储提供并清零, 生命周期覆盖全部调用.
  * @note   fb 与 measurement 是流水线两级: fb 为驱动解码的原始反馈, 转子量, 由
  *         OnRx 在中断上下文写入; measurement 为按减速比与安装方向换算后的机构系
  *         测量值, 由 Update 在任务上下文整体刷新.
  * @note   fb 的单个字段为 32 位对齐读写, 不会撕裂, 字段之间可能来自相邻两帧,
  *         混合窗口为一个帧间隔. measurement 由 Update 整体写入, 同一任务上下文
  *         的读者读到完整快照.
  * @note   config 之后的字段由模块维护, 调用方只读, 不得写入.
  */
typedef struct NERV_Motor_Instance
{
    NERV_Motor_ConfigTypeDef config; /**< 配置, 由 NERV_Motor_Init 绑定, 之后只读 */

    NERV_Motor_FbTypeDef fb;                   /**< 原始反馈, 转子量, 由 NERV_Motor_OnRx 写入 */
    NERV_Motor_MeasurementTypeDef measurement; /**< 机构系测量值, 由 NERV_Motor_UpdateGroup 刷新 */
    uint32_t error;                            /**< 粘滞错误位, 经 NERV_Motor_ReadError 读取 */
    NERV_Motor_FrameGeoTypeDef geo;            /**< 帧几何信息, 由 NERV_Motor_Init 填充 */
    NERV_Motor_ControlTypeDef target_control;  /**< 控制目标, 由 NERV_Motor_SetControl 写入 */
    uint32_t last_rx_ms;                       /**< 最近一次有效反馈的时刻, 单位 ms */
    bool offline_armed;                        /**< 离线监督已建立计时起点, 由模块维护 */
    bool initialized;                          /**< 初始化完成标志, 由 NERV_Motor_Init 置位 */
    bool fb_fresh;                             /**< 有新反馈待结算, 中断置位, UpdateGroup 清除 */
} NERV_Motor_InstanceTypeDef;

/* 调用顺序: NERV_Motor_Init 用配置建立实例, 之后周期调用 NERV_Motor_UpdateGroup
 * 完成测量值刷新, 监督与指令发送, 这是唯一发送指令的入口. 反馈经 NERV_Motor_OnRx
 * 进入. 完整契约见各函数定义处. */

NERV_StatusTypeDef NERV_Motor_Init(NERV_Motor_InstanceTypeDef *self,
                                   const NERV_Motor_ConfigTypeDef *config);

NERV_StatusTypeDef NERV_Motor_SetControl(NERV_Motor_InstanceTypeDef *self,
                                         const NERV_Motor_ControlTypeDef *control);

bool NERV_Motor_Supports(const NERV_Motor_InstanceTypeDef *self,
                         NERV_Motor_CapabilityTypeDef capability);

NERV_StatusTypeDef NERV_Motor_UpdateGroup(NERV_Motor_InstanceTypeDef *const *members, uint8_t count,
                                          uint32_t now_ms);

void NERV_Motor_OnRx(NERV_Motor_InstanceTypeDef *self, const NERV_FDCAN_FrameTypeDef *frame);

NERV_StatusTypeDef NERV_Motor_GetMeasurement(const NERV_Motor_InstanceTypeDef *self,
                                             NERV_Motor_MeasurementTypeDef *measurement);

uint32_t NERV_Motor_ReadError(const NERV_Motor_InstanceTypeDef *self);

void NERV_Motor_ClearError(NERV_Motor_InstanceTypeDef *self, uint32_t mask);

NERV_StatusTypeDef NERV_Motor_EnterSafeState(NERV_Motor_InstanceTypeDef *self);

NERV_StatusTypeDef NERV_Motor_Enable(NERV_Motor_InstanceTypeDef *self);

NERV_StatusTypeDef NERV_Motor_Disable(NERV_Motor_InstanceTypeDef *self);

NERV_StatusTypeDef NERV_Motor_ClearFault(NERV_Motor_InstanceTypeDef *self);

NERV_StatusTypeDef NERV_Motor_SetMode(NERV_Motor_InstanceTypeDef *self,
                                      NERV_Motor_ControlKindTypeDef mode);

NERV_StatusTypeDef NERV_Motor_SaveZero(NERV_Motor_InstanceTypeDef *self);

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_MOTOR */

#endif /* NERV_MOTOR_H */
