/**
  * @file    nerv_pid.h
  * @brief   NERV 通用 PID 控制器.
  * @version 1.0.0
  * @date    2026-10-24
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_PID_H
#define NERV_PID_H

#include <stdbool.h>

#include "nerv_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  PID 配置.
  * @note   配置由调用方以静态存储提供, NERV_PID_Init 复制内容, 调用方保留所有权.
  * @note   kp, ki 与 kd 必须为大于等于 0 的有限值. 反向机构通过被控对象的
  *         方向配置处理, 不使用负增益.
  * @note   output_min 与 output_max 必须为有限值, 满足 output_min 小于 output_max,
  *         且区间包含 0, 保证安全状态可以输出零.
  * @note   integral_min 与 integral_max 限制积分项的贡献, 与 output 同单位, 必须为
  *         有限值, 满足 integral_min 小于 integral_max, 且区间包含 0.
  * @note   integral_error_threshold 为变速积分阈值, 与被控量同单位, 必须为大于等于 0
  *         的有限值. 误差绝对值达到该阈值时停止累加新积分量, 误差为 0 时全速累加,
  *         中间线性过渡. 取 0 表示不启用变速积分.
  * @note   derivative_tau_s 为一阶低通时间常数, 取 0 表示不对微分项滤波.
  */
typedef struct
{
    float kp;                       /**< 比例系数, 量纲由被控量与输出量决定 */
    float ki;                       /**< 积分系数, 输出单位除以被控量单位与秒 */
    float kd;                       /**< 微分系数, 输出单位乘以秒除以被控量单位 */
    float output_min;               /**< 输出下限, 与 output 同单位 */
    float output_max;               /**< 输出上限, 与 output 同单位 */
    float integral_min;             /**< 积分项输出下限, 与 output 同单位 */
    float integral_max;             /**< 积分项输出上限, 与 output 同单位 */
    float integral_error_threshold; /**< 变速积分误差阈值, 与被控量同单位, 0 表示不启用 */
    float derivative_tau_s;         /**< 微分项一阶低通时间常数, 单位 s, 0 表示不滤波 */
} NERV_PID_ConfigTypeDef;

/**
  * @brief  PID 实例.
  * @note   由调用方以静态存储提供并清零, 生命周期覆盖全部调用.
  * @note   config 之后的字段由模块维护, 调用方只读, 不得写入.
  * @note   微分作用在测量值上, 不作用在设定值上, 设定值阶跃不会产生微分冲击.
  */
typedef struct
{
    NERV_PID_ConfigTypeDef config; /**< 配置, 由 NERV_PID_Init 绑定, 之后只读 */
    float integral;                /**< 积分项输出值, 与 output 同单位 */
    float derivative;              /**< 滤波后的微分项输出值, 与 output 同单位 */
    float prev_measurement;        /**< 上一次测量值, 与 measurement 同单位 */
    float output;                  /**< 最近一次输出值, 与 output 同单位 */
    bool has_prev_measurement;     /**< prev_measurement 有效标志 */
    bool initialized;              /**< 初始化完成标志 */
} NERV_PID_InstanceTypeDef;

/**
  * @brief  串级 PID 输入.
  * @note   外环输出作为内环设定值, 外环的输出限幅同时定义内环设定值范围.
  * @note   setpoint 与 outer_measurement 使用外环被控量单位; inner_measurement
  *         使用内环被控量单位; outer_feedforward 与内环设定值同单位;
  *         inner_feedforward 与最终输出同单位.
  * @note   两个环使用同一个 dt_s, 串级更新要求两个环在同一周期内完成.
  */
typedef struct
{
    float setpoint;          /**< 外环设定值, 单位由外环被控量决定 */
    float outer_measurement; /**< 外环测量值, 单位与外环设定值相同 */
    float inner_measurement; /**< 内环测量值, 单位由内环被控量决定 */
    float outer_feedforward; /**< 外环前馈, 与内环设定值同单位 */
    float inner_feedforward; /**< 内环前馈, 与最终输出同单位 */
    float dt_s;              /**< 控制周期, 单位 s, 必须大于 0 */
} NERV_PID_CascadeInputTypeDef;

NERV_StatusTypeDef NERV_PID_Init(NERV_PID_InstanceTypeDef *self,
                                 const NERV_PID_ConfigTypeDef *config);

NERV_StatusTypeDef NERV_PID_Reset(NERV_PID_InstanceTypeDef *self);

/**
  * @brief  PID 可调参数标识.
  * @note   用于 NERV_PID_SetParam 定位单个参数, 与 NERV_PID_ConfigTypeDef 字段一一对应.
  */
typedef enum
{
    NERV_PID_PARAM_KP                       = 0U,
    NERV_PID_PARAM_KI                       = 1U,
    NERV_PID_PARAM_KD                       = 2U,
    NERV_PID_PARAM_OUTPUT_MIN               = 3U,
    NERV_PID_PARAM_OUTPUT_MAX               = 4U,
    NERV_PID_PARAM_INTEGRAL_MIN             = 5U,
    NERV_PID_PARAM_INTEGRAL_MAX             = 6U,
    NERV_PID_PARAM_INTEGRAL_ERROR_THRESHOLD = 7U,
    NERV_PID_PARAM_DERIVATIVE_TAU_S         = 8U,
    NERV_PID_PARAM_COUNT
} NERV_PID_ParamTypeDef;

NERV_StatusTypeDef NERV_PID_SetParam(NERV_PID_InstanceTypeDef *self, NERV_PID_ParamTypeDef param,
                                     float value);

NERV_StatusTypeDef NERV_PID_Update(NERV_PID_InstanceTypeDef *self, float setpoint,
                                   float measurement, float feedforward, float dt_s, float *output);

NERV_StatusTypeDef NERV_PID_UpdateCascade(NERV_PID_InstanceTypeDef *outer,
                                          NERV_PID_InstanceTypeDef *inner,
                                          const NERV_PID_CascadeInputTypeDef *input, float *output);

#ifdef __cplusplus
}
#endif

#endif /* NERV_PID_H */
