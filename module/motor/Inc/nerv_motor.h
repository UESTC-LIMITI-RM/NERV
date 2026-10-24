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
  * @note   控制量的单位与语义由 driver 的协议族约定, 本结构体不限定为电流.
  */
typedef struct
{
    const NERV_Motor_DriverTypeDef *driver; /**< 驱动绑定, 决定协议族与控制模式 */
    const NERV_FDCAN_InstanceTypeDef *bus;  /**< FDCAN 实例, 指令发送与反馈接收的通道 */
    uint8_t device_id;        /**< 电调 id, 合法范围由驱动描述符的 id_min 与 id_max 决定 */
    float gear_ratio;         /**< 减速比, 必须大于 0, 输出轴角速度等于转子角速度除以本值 */
    uint32_t offline_ms;      /**< 离线判定阈值, 单位 ms, 0 表示关闭离线判定 */
    float over_temperature_c; /**< 电机过温阈值, 单位 ℃, 0 表示关闭过温检查 */
    float control_limit;      /**< 控制量限幅, 单位由协议族约定, 0 表示不额外限幅 */
    bool invert_direction;    /**< 安装方向取反, 指令与反馈统一乘符号 */
} NERV_Motor_ConfigTypeDef;

/**
  * @brief  电机测量值, 机构系.
  * @note   角度与角速度已按减速比换算到输出轴, 角度, 角速度与控制量按安装方向
  *         取反, 温度原值透传, 正方向为机构正方向.
  * @note   角度范围为 [0, 2π), 不做多圈累积. 需要连续位置时由调用方用 NERV_Math_Wrap
  *         计算增量后自行累积.
  * @note   temperature_c 为 INT8_MIN 表示该型号无温度字段, 见 NERV_MOTOR_FB_OFF_NONE.
  */
typedef struct NERV_Motor_Measurement
{
    float angle_rad;      /**< 输出轴机械角度, 单位 rad, 机构正方向为正, 范围 [0, 2π) */
    float speed_radps;    /**< 输出轴角速度, 单位 rad/s, 机构正方向为正 */
    float control;        /**< 控制量反馈, 单位由协议族约定, 机构正方向为正 */
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
    NERV_Motor_MeasurementTypeDef measurement; /**< 机构系测量值, 由 NERV_Motor_Update 刷新 */
    uint32_t error;                            /**< 粘滞错误位, 经 NERV_Motor_GetError 读取 */
    NERV_Motor_FrameGeoTypeDef geo;            /**< 帧几何信息, 由 NERV_Motor_Init 填充 */
    float target;                              /**< 控制量目标, 由 NERV_Motor_SetTarget 写入 */
    uint32_t last_rx_ms;                       /**< 最近一次有效反馈的时刻, 单位 ms */
    bool initialized;                          /**< 初始化完成标志, 由 NERV_Motor_Init 置位 */
    bool fb_fresh;                             /**< 有新反馈待结算, 中断置位, Update 清除 */
} NERV_Motor_InstanceTypeDef;

/* 调用顺序: NERV_Motor_Init 用配置建立实例, 之后周期调用 NERV_Motor_UpdateGroup
 * 完成测量值刷新, 监督与指令发送. 单个实例的测量值刷新与监督可单独调用
 * NERV_Motor_Update. 反馈经 NERV_Motor_OnRx 进入. 完整契约见各函数定义处. */

NERV_StatusTypeDef NERV_Motor_Init(NERV_Motor_InstanceTypeDef *self,
                                   const NERV_Motor_ConfigTypeDef *config);

NERV_StatusTypeDef NERV_Motor_SetTarget(NERV_Motor_InstanceTypeDef *self, float target);

NERV_StatusTypeDef NERV_Motor_Update(NERV_Motor_InstanceTypeDef *self, uint32_t now_ms);

NERV_StatusTypeDef NERV_Motor_UpdateGroup(NERV_Motor_InstanceTypeDef *const *members, uint8_t count,
                                          uint32_t now_ms);

void NERV_Motor_OnRx(NERV_Motor_InstanceTypeDef *self, const NERV_FDCAN_FrameTypeDef *frame);

NERV_StatusTypeDef NERV_Motor_GetMeasurement(const NERV_Motor_InstanceTypeDef *self,
                                             NERV_Motor_MeasurementTypeDef *measurement);

uint32_t NERV_Motor_GetError(const NERV_Motor_InstanceTypeDef *self);

void NERV_Motor_ClearError(NERV_Motor_InstanceTypeDef *self, uint32_t mask);

NERV_StatusTypeDef NERV_Motor_EnterSafeState(NERV_Motor_InstanceTypeDef *self);

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_MOTOR */

#endif /* NERV_MOTOR_H */
