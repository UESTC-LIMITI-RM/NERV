/**
  * @file    nerv_pid.c
  * @brief   NERV 通用 PID 控制器实现.
  * @version 1.0.0
  * @date    2026-10-24
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "nerv_math.h"
#include "nerv_pid.h"

/**
  * @brief  清除 PID 的运行期状态, 保留配置.
  * @param  self PID 实例指针, 由调用方保证不为 NULL 且配置已绑定.
  */
static void NERV_PID_ClearState(NERV_PID_InstanceTypeDef *self)
{
    self->integral             = 0.0f;
    self->derivative           = 0.0f;
    self->prev_measurement     = 0.0f;
    self->output               = 0.0f;
    self->has_prev_measurement = false;
}

/**
  * @brief  校验 PID 配置.
  * @param  config 配置指针, 可以为 NULL.
  * @retval NERV_OK 配置合法.
  * @retval NERV_ERROR config 为 NULL, 增益为负或非有限值, 输出或积分区间非法,
  *         变速积分阈值为负或非有限值, 或微分时间常数为负或非有限值.
  * @note   输出区间与积分区间必须包含 0, 保证安全状态可以输出零.
  */
static NERV_StatusTypeDef NERV_PID_ValidateConfig(const NERV_PID_ConfigTypeDef *config)
{
    if (NULL == config)
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->kp) || (config->kp < 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->ki) || (config->ki < 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->kd) || (config->kd < 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->output_min) || !isfinite(config->output_max) ||
        (config->output_min >= config->output_max))
    {
        return NERV_ERROR;
    }

    if ((config->output_min > 0.0f) || (config->output_max < 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->integral_min) || !isfinite(config->integral_max) ||
        (config->integral_min >= config->integral_max))
    {
        return NERV_ERROR;
    }

    if ((config->integral_min > 0.0f) || (config->integral_max < 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->integral_error_threshold) || (config->integral_error_threshold < 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->derivative_tau_s) || (config->derivative_tau_s < 0.0f))
    {
        return NERV_ERROR;
    }

    return NERV_OK;
}

/**
  * @brief  初始化 PID 控制器.
  * @param  self PID 实例指针, 不得为 NULL.
  * @param  config 配置指针, 不得为 NULL. 内容被复制, 调用方保留所有权.
  * @retval NERV_OK 配置已绑定, 运行期状态已清零.
  * @retval NERV_ERROR self 或 config 为 NULL, 或配置字段超出合法范围.
  * @note   校验与复制在修改 self 之前完成, 失败时 self 保持原状.
  * @note   重复调用以新配置重建实例, 已有积分, 微分历史与输出全部丢弃.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_PID_Init(NERV_PID_InstanceTypeDef *self,
                                 const NERV_PID_ConfigTypeDef *config)
{
    if ((NULL == self) || (NULL == config))
    {
        return NERV_ERROR;
    }

    if (NERV_OK != NERV_PID_ValidateConfig(config))
    {
        return NERV_ERROR;
    }

    self->config      = *config;
    self->initialized = true;

    NERV_PID_ClearState(self);

    return NERV_OK;
}

/**
  * @brief  复位 PID 运行期状态, 保留配置.
  * @param  self PID 实例指针, 不得为 NULL.
  * @retval NERV_OK 积分, 微分历史与输出已清零.
  * @retval NERV_ERROR self 为 NULL 或实例未初始化.
  * @note   复位后的第一次更新不计算微分, 只记录测量值.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_PID_Reset(NERV_PID_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    NERV_PID_ClearState(self);

    return NERV_OK;
}

/**
  * @brief  执行一次 PID 计算.
  * @param  self PID 实例指针, 不得为 NULL 且已初始化.
  * @param  setpoint 设定值, 单位由被控量决定, 必须为有限值.
  * @param  measurement 测量值, 单位与 setpoint 相同, 必须为有限值.
  * @param  feedforward 前馈量, 与输出同单位, 必须为有限值.
  * @param  dt_s 控制周期, 单位 s, 必须为大于 0 的有限值.
  * @param  output 输出指针, 不得为 NULL. 成功时写入限幅后的输出.
  * @retval NERV_OK 计算完成, 状态与 output 已更新.
  * @retval NERV_ERROR 参数非法, 实例未初始化, 或中间计算出现非有限值.
  * @note   输出为 P, I, D 与前馈之和, 再按 output_min 与 output_max 限幅.
  * @note   积分项按 integral_min 与 integral_max 限幅, 两个限幅值都是积分项的
  *         输出贡献, 与 output 同单位. 输出饱和且误差继续推向饱和方向时停止积分,
  *         误差反向时允许积分回退.
  * @note   变速积分按 integral_error_threshold 对新积分量线性衰减: 误差绝对值达到
  *         阈值时积分增益为 0, 误差为 0 时积分增益为 1. 阈值取 0 表示不启用.
  * @note   微分作用在测量值上, 不作用在设定值上. derivative_tau_s 为 0 时不滤波,
  *         大于 0 时使用一阶低通.
  * @note   复位后第一次更新不计算微分, 只记录 measurement, 微分项为 0.
  * @note   失败时不修改实例状态与 output.
  * @warning output 不得指向 self 内部的任何字段.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_PID_Update(NERV_PID_InstanceTypeDef *self, float setpoint,
                                   float measurement, float feedforward, float dt_s, float *output)
{
    float error;              /* 设定值与测量值之差, 与被控量同单位 */
    float p_term;             /* 比例项输出, 与 output 同单位 */
    float raw_derivative;     /* 未滤波的微分项输出, 与 output 同单位 */
    float derivative_term;    /* 本周期实际使用的微分项输出, 与 output 同单位 */
    float integral_candidate; /* 积分项候选值, 计算后按积分限幅, 与 output 同单位 */
    float integral_delta;     /* 本周期新增的积分量, 与 output 同单位 */
    float integral_scale;     /* 变速积分增益, 无量纲, 范围 [0, 1] */
    float output_unsat;       /* 限幅前的输出, 与 output 同单位 */
    float output_final;       /* 限幅后的输出, 与 output 同单位 */
    float alpha;              /* 微分项一阶低通系数, 无量纲, 范围 [0, 1] */
    bool saturated_high;      /* 限幅前输出高于 output_max */
    bool saturated_low;       /* 限幅前输出低于 output_min */

    if ((NULL == self) || (NULL == output))
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if (!isfinite(setpoint) || !isfinite(measurement) || !isfinite(feedforward) ||
        !isfinite(dt_s) || (dt_s <= 0.0f))
    {
        return NERV_ERROR;
    }

    error = setpoint - measurement;
    if (!isfinite(error))
    {
        return NERV_ERROR;
    }

    p_term = self->config.kp * error;
    if (!isfinite(p_term))
    {
        return NERV_ERROR;
    }

    derivative_term = 0.0f;

    if (self->has_prev_measurement)
    {
        raw_derivative = -self->config.kd * (measurement - self->prev_measurement) / dt_s;
        if (!isfinite(raw_derivative))
        {
            return NERV_ERROR;
        }

        if (self->config.derivative_tau_s > 0.0f)
        {
            alpha           = 1.0f / (1.0f + (dt_s / self->config.derivative_tau_s));
            derivative_term = (alpha * self->derivative) + ((1.0f - alpha) * raw_derivative);
        }
        else
        {
            derivative_term = raw_derivative;
        }

        if (!isfinite(derivative_term))
        {
            return NERV_ERROR;
        }
    }

    integral_scale = 1.0f;

    if (self->config.integral_error_threshold > 0.0f)
    {
        integral_scale = 1.0f - (fabsf(error) / self->config.integral_error_threshold);
        integral_scale = NERV_Math_Clamp(integral_scale, 0.0f, 1.0f);
    }

    integral_delta = self->config.ki * error * dt_s * integral_scale;
    if (!isfinite(integral_delta))
    {
        return NERV_ERROR;
    }

    integral_candidate = self->integral + integral_delta;
    if (!isfinite(integral_candidate))
    {
        return NERV_ERROR;
    }

    integral_candidate =
        NERV_Math_Clamp(integral_candidate, self->config.integral_min, self->config.integral_max);

    output_unsat = p_term + integral_candidate + derivative_term + feedforward;
    if (!isfinite(output_unsat))
    {
        return NERV_ERROR;
    }

    saturated_high = (output_unsat > self->config.output_max);
    saturated_low  = (output_unsat < self->config.output_min);
    output_final = NERV_Math_Clamp(output_unsat, self->config.output_min, self->config.output_max);

    if ((saturated_high && (error > 0.0f)) || (saturated_low && (error < 0.0f)))
    {
        integral_candidate = self->integral;
    }

    self->integral             = integral_candidate;
    self->derivative           = derivative_term;
    self->prev_measurement     = measurement;
    self->has_prev_measurement = true;
    self->output               = output_final;

    *output = output_final;

    return NERV_OK;
}

/**
  * @brief  执行一次两环串级 PID 计算.
  * @param  outer 外环实例指针, 不得为 NULL 且已初始化.
  * @param  inner 内环实例指针, 不得为 NULL 且已初始化. 不得与 outer 相同.
  * @param  input 串级输入指针, 不得为 NULL, 各字段必须为有限值, dt_s 必须大于 0.
  * @param  output 最终输出指针, 不得为 NULL. 成功时写入内环限幅后的输出.
  * @retval NERV_OK 两个环均计算完成, 状态与 output 已更新.
  * @retval NERV_ERROR 参数非法, 实例未初始化, 或任一环的计算出现非有限值.
  * @note   外环输出作为内环设定值, 外环的 output 限幅决定内环设定值范围.
  * @note   外环前馈与内环设定值同单位, 内环前馈与最终输出同单位.
  * @note   两个环使用同一个 dt_s. 调用前两个实例必须已由 NERV_PID_Init 初始化.
  * @note   失败时不修改两个实例的状态与 output. 成功时先提交两个实例状态,
  *         再写 output.
  * @warning output 不得指向 outer 或 inner 实例所占用的内存.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_PID_UpdateCascade(NERV_PID_InstanceTypeDef *outer,
                                          NERV_PID_InstanceTypeDef *inner,
                                          const NERV_PID_CascadeInputTypeDef *input, float *output)
{
    NERV_PID_InstanceTypeDef outer_next;
    NERV_PID_InstanceTypeDef inner_next;
    NERV_StatusTypeDef status;
    float inner_setpoint;
    float result;

    if ((NULL == outer) || (NULL == inner) || (NULL == input) || (NULL == output))
    {
        return NERV_ERROR;
    }

    if ((outer == inner) || (!outer->initialized) || (!inner->initialized))
    {
        return NERV_ERROR;
    }

    outer_next = *outer;
    inner_next = *inner;

    status = NERV_PID_Update(&outer_next, input->setpoint, input->outer_measurement,
                             input->outer_feedforward, input->dt_s, &inner_setpoint);
    if (NERV_OK != status)
    {
        return status;
    }

    status = NERV_PID_Update(&inner_next, inner_setpoint, input->inner_measurement,
                             input->inner_feedforward, input->dt_s, &result);
    if (NERV_OK != status)
    {
        return status;
    }

    *outer  = outer_next;
    *inner  = inner_next;
    *output = result;

    return NERV_OK;
}
