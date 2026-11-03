/**
  * @file    nerv_gimbal.c
  * @brief   NERV 云台模块实现.
  * @version 1.0.0
  * @date    2026-11-01
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include "nerv_gimbal.h"

#if (NERV_USE_GIMBAL == NERV_ENABLE)

#include <math.h>
#include <stddef.h>

#include "nerv_math.h"
#include "nerv_motor.h"

#define NERV_INNER_GIMBAL_MAX_DT_S 0.1f /**< 控制周期上限, 单位 s, 防止时钟跳变使积分瞬间饱和 */

static NERV_StatusTypeDef NERV_Gimbal_ValidateConfig(const NERV_Gimbal_ConfigTypeDef *config);
static void NERV_Gimbal_RefreshAttitude(NERV_Gimbal_InstanceTypeDef *self);
static NERV_StatusTypeDef NERV_Gimbal_ApplySpeed(NERV_Gimbal_InstanceTypeDef *self, uint8_t axis,
                                                 float speed_radps);
static NERV_StatusTypeDef NERV_Gimbal_ApplyPosition(NERV_Gimbal_InstanceTypeDef *self, uint8_t axis,
                                                    float position_rad, float speed_limit_radps);
static NERV_StatusTypeDef NERV_Gimbal_CheckLimit(const NERV_Gimbal_InstanceTypeDef *self,
                                                 uint8_t axis, float position_rad);
static NERV_StatusTypeDef NERV_Gimbal_ApplyRoll(NERV_Gimbal_InstanceTypeDef *self, float dt_s);
static void NERV_Gimbal_UpdateRollStall(NERV_Gimbal_InstanceTypeDef *self, uint32_t now_ms);
static void NERV_Gimbal_ApplySafeState(NERV_Gimbal_InstanceTypeDef *self, uint32_t now_ms);

/**
  * @brief  校验云台配置.
  * @param  config 配置指针, 可以为 NULL.
  * @retval NERV_OK 配置合法.
  * @retval NERV_ERROR config 为 NULL, 任一电机为空或未初始化, 控制模式与轴用途不符,
  *         限位区间非法, 或限速为负.
  * @note   限位仅在 limit_enabled 为 true 的轴生效; 未启用的轴不检查限位区间, 可填写任意值.
  * @note   启用的限位必须满足下限小于上限, 且都落在 [-π, π] 内.
  * @note   大 yaw 必须为标量角速度模式, 小 yaw 与 pitch 必须为位置速度模式,
  *         roll 必须为标量电流模式.
  */
static NERV_StatusTypeDef NERV_Gimbal_ValidateConfig(const NERV_Gimbal_ConfigTypeDef *config)
{
    if (NULL == config)
    {
        return NERV_ERROR;
    }

    for (uint8_t axis = 0U; axis < NERV_GIMBAL_AXIS_COUNT; axis++)
    {
        const NERV_Motor_InstanceTypeDef *motor = config->motors[axis];

        if ((NULL == motor) || (!motor->initialized) || (NULL == motor->config.driver) ||
            (NULL == motor->config.driver->model))
        {
            return NERV_ERROR;
        }

        const NERV_Motor_ModelTypeDef *model = motor->config.driver->model;

        /* 控制模式必须与模块对该轴的下发方式一致, 否则物理量会被当成另一种量纲,
         * 例如把角速度当作电流或电压下发. */
        switch (axis)
        {
        case NERV_GIMBAL_AXIS_YAW_BIG:
            if ((NERV_MOTOR_CONTROL_SCALAR != model->control_mode) ||
                (NERV_MOTOR_SCALAR_SPEED != model->scalar_quantity))
            {
                return NERV_ERROR;
            }
            break;

        case NERV_GIMBAL_AXIS_YAW_SMALL:
        case NERV_GIMBAL_AXIS_PITCH:
            if (NERV_MOTOR_CONTROL_POSITION_VELOCITY != model->control_mode)
            {
                return NERV_ERROR;
            }
            break;

        case NERV_GIMBAL_AXIS_ROLL:
            if ((NERV_MOTOR_CONTROL_SCALAR != model->control_mode) ||
                (NERV_MOTOR_SCALAR_CURRENT != model->scalar_quantity))
            {
                return NERV_ERROR;
            }
            break;

        default:
            return NERV_ERROR;
        }

        /* roll 轴靠堵转判定, 不参与角度限位校验. */
        if (NERV_GIMBAL_AXIS_ROLL == axis)
        {
            continue;
        }

        if (!config->limit_enabled[axis])
        {
            continue;
        }

        if (!isfinite(config->limit_min_rad[axis]) || !isfinite(config->limit_max_rad[axis]) ||
            (config->limit_min_rad[axis] >= config->limit_max_rad[axis]) ||
            (config->limit_min_rad[axis] < -NERV_MATH_PI) ||
            (config->limit_max_rad[axis] >= NERV_MATH_PI))
        {
            return NERV_ERROR;
        }
    }

    if (!isfinite(config->yaw_small_speed_limit_radps) ||
        !isfinite(config->pitch_speed_limit_radps) ||
        (config->yaw_small_speed_limit_radps < 0.0f) || (config->pitch_speed_limit_radps < 0.0f))
    {
        return NERV_ERROR;
    }

    const NERV_Motor_ModelTypeDef *roll_model =
        config->motors[NERV_GIMBAL_AXIS_ROLL]->config.driver->model;

    /* 电流反馈型号需要扭矩常数把电流换算为输出轴转矩, 转矩反馈型号不使用该字段;
     * 无论哪种来源都拒绝非有限与非正值. */
    if (!isfinite(roll_model->torque_constant_n_m_per_a) ||
        (roll_model->torque_constant_n_m_per_a <= 0.0f))
    {
        return NERV_ERROR;
    }

    if ((NULL == config->roll_speed_pid) || (NULL == config->roll_position_pid) ||
        (!isfinite(config->roll_stall_torque_n_m)) || (!isfinite(config->roll_stall_speed_radps)) ||
        (!isfinite(config->roll_seek_speed_radps)) || (config->roll_stall_torque_n_m <= 0.0f) ||
        (config->roll_stall_speed_radps <= 0.0f) || (config->roll_seek_speed_radps <= 0.0f) ||
        (0U == config->roll_stall_ms))
    {
        return NERV_ERROR;
    }

    /* 一个电机实例只能由一个消费者持有, 四个轴不得共用同一实例. */
    for (uint8_t first = 0U; first < NERV_GIMBAL_AXIS_COUNT; first++)
    {
        for (uint8_t second = (uint8_t)(first + 1U); second < NERV_GIMBAL_AXIS_COUNT; second++)
        {
            if (config->motors[first] == config->motors[second])
            {
                return NERV_ERROR;
            }
        }
    }

    return NERV_OK;
}

/**
  * @brief  刷新当前姿态, 各轴角度从电机测量值取.
  * @param  self 实例指针, 由调用方保证已初始化且不为 NULL.
  * @note   电机测量值已是输出轴量, 与本模块对外接口同量纲, 本函数不做 feedback_ratio 换算.
  */
static void NERV_Gimbal_RefreshAttitude(NERV_Gimbal_InstanceTypeDef *self)
{
    self->measured_attitude.yaw_big_rad =
        self->config.motors[NERV_GIMBAL_AXIS_YAW_BIG]->measurement.angle_rad;
    self->measured_attitude.yaw_small_rad =
        NERV_Math_Wrap(self->config.motors[NERV_GIMBAL_AXIS_YAW_SMALL]->measurement.angle_rad,
                       -NERV_MATH_PI, NERV_MATH_PI);
    self->measured_attitude.pitch_rad =
        NERV_Math_Wrap(self->config.motors[NERV_GIMBAL_AXIS_PITCH]->measurement.angle_rad,
                       -NERV_MATH_PI, NERV_MATH_PI);
    self->measured_attitude.roll_rad =
        self->config.motors[NERV_GIMBAL_AXIS_ROLL]->measurement.angle_rad;

    self->measured_attitude.yaw_total_rad =
        NERV_Math_Wrap(self->measured_attitude.yaw_big_rad + self->measured_attitude.yaw_small_rad,
                       -NERV_MATH_PI, NERV_MATH_PI);
}

/**
  * @brief  检查目标角是否在限位内.
  * @param  self 实例指针.
  * @param  axis 轴索引.
  * @param  position_rad 目标角, 单位 rad.
  * @retval NERV_OK 在限位内.
  * @retval NERV_ERROR 越限.
  */
static NERV_StatusTypeDef NERV_Gimbal_CheckLimit(const NERV_Gimbal_InstanceTypeDef *self,
                                                 uint8_t axis, float position_rad)
{
    if (!isfinite(position_rad))
    {
        return NERV_ERROR;
    }

    /* 位置轴目标必须落在 [-π, π) 定义域内, 与是否启用机械限位无关. */
    if ((position_rad < -NERV_MATH_PI) || (position_rad >= NERV_MATH_PI))
    {
        return NERV_ERROR;
    }

    /* 该轴无机械限位, 目标角放行. */
    if (!self->config.limit_enabled[axis])
    {
        return NERV_OK;
    }

    if ((position_rad < self->config.limit_min_rad[axis]) ||
        (position_rad > self->config.limit_max_rad[axis]))
    {
        return NERV_ERROR;
    }

    return NERV_OK;
}

/**
  * @brief  以角速度目标驱动一个轴, 用于大 yaw.
  * @param  self 实例指针.
  * @param  axis 轴索引.
  * @param  speed_radps 目标角速度, 单位 rad/s, 输出轴.
  * @retval NERV_OK 目标已写入电机.
  * @retval NERV_ERROR 参数非法或型号不支持 SCALAR 控制.
  * @note   型号必须绑定速度模式驱动, control_mode 为 NERV_MOTOR_CONTROL_SCALAR 且
  *         scalar_quantity 为 NERV_MOTOR_SCALAR_SPEED. 电流模式型号不能走本函数.
  */
static NERV_StatusTypeDef NERV_Gimbal_ApplySpeed(NERV_Gimbal_InstanceTypeDef *self, uint8_t axis,
                                                 float speed_radps)
{
    NERV_Motor_InstanceTypeDef *motor = self->config.motors[axis];
    NERV_Motor_ControlTypeDef control;

    if ((NULL == motor) || (!isfinite(speed_radps)))
    {
        return NERV_ERROR;
    }

    control.kind     = NERV_MOTOR_CONTROL_SCALAR;
    control.u.scalar = speed_radps * motor->config.feedback_ratio;

    return NERV_Motor_SetControl(motor, &control);
}

/**
  * @brief  以位置目标驱动一个轴, 用于小 yaw, pitch 与 roll.
  * @param  self 实例指针.
  * @param  axis 轴索引.
  * @param  position_rad 目标角, 单位 rad, 输出轴.
  * @param  speed_limit_radps 限速, 单位 rad/s, 输出轴, 0 表示使用型号最大速度.
  * @retval NERV_OK 目标已写入电机.
  * @retval NERV_ERROR 参数非法或型号不支持位置速度控制.
  * @note   型号必须绑定位置速度驱动, control_mode 为
  *         NERV_MOTOR_CONTROL_POSITION_VELOCITY. 位置与限速都乘 feedback_ratio 转为反馈侧.
  */
static NERV_StatusTypeDef NERV_Gimbal_ApplyPosition(NERV_Gimbal_InstanceTypeDef *self, uint8_t axis,
                                                    float position_rad, float speed_limit_radps)
{
    NERV_Motor_InstanceTypeDef *motor = self->config.motors[axis];
    NERV_Motor_ControlTypeDef control;

    if ((NULL == motor) || (!isfinite(position_rad)) || (!isfinite(speed_limit_radps)))
    {
        return NERV_ERROR;
    }

    const NERV_Motor_ModelTypeDef *model = motor->config.driver->model;

    /* 目标角在反馈侧展开到离当前位置最近的一圈, 避免位置环绕远路. */
    const float current_feedback = motor->measurement.angle_rad * motor->config.feedback_ratio;
    float position_feedback      = position_rad * motor->config.feedback_ratio;
    const float delta            = position_feedback - current_feedback;

    if (delta > NERV_MATH_PI)
    {
        position_feedback -= NERV_MATH_TWO_PI;
    }
    else if (delta < -NERV_MATH_PI)
    {
        position_feedback += NERV_MATH_TWO_PI;
    }

    /* 限速 0 表示使用型号允许的最大速度, 其余按 feedback_ratio 换算. */
    float velocity_feedback = (0.0f == speed_limit_radps)
                                  ? model->control.profile.v_max_radps
                                  : speed_limit_radps * motor->config.feedback_ratio;

    /* 反馈侧目标不得超过型号位置与速度满量程. */
    if (position_feedback > model->control.profile.p_max_rad)
    {
        position_feedback = model->control.profile.p_max_rad;
    }
    else if (position_feedback < -model->control.profile.p_max_rad)
    {
        position_feedback = -model->control.profile.p_max_rad;
    }

    if (velocity_feedback > model->control.profile.v_max_radps)
    {
        velocity_feedback = model->control.profile.v_max_radps;
    }

    control.kind                               = NERV_MOTOR_CONTROL_POSITION_VELOCITY;
    control.u.position_velocity.position_rad   = position_feedback;
    control.u.position_velocity.velocity_radps = velocity_feedback;

    return NERV_Motor_SetControl(motor, &control);
}

/**
  * @brief  以角速度目标驱动 roll 轴, 内部经速度环转成电流.
  * @param  self 实例指针.
  * @param  speed_radps 目标角速度, 单位 rad/s, 输出轴.
  * @param  dt_s 控制周期, 单位 s, 必须大于 0.
  * @retval NERV_OK 目标已写入电机.
  * @retval NERV_ERROR 参数非法, 速度环失败, 或型号不支持 SCALAR 控制.
  * @note   型号必须为标量电流模式. 速度环与位置环的输出都是标量控制量, 单位由型号
  *         描述符的 scalar_quantity 与 full_scale 决定, 本函数不做编码换算.
  */
static NERV_StatusTypeDef NERV_Gimbal_ApplyRoll(NERV_Gimbal_InstanceTypeDef *self, float dt_s)
{
    NERV_Motor_InstanceTypeDef *motor = self->config.motors[NERV_GIMBAL_AXIS_ROLL];
    NERV_Motor_ControlTypeDef control;
    float current_a;

    if ((NULL == motor) || (!isfinite(dt_s)) || (dt_s <= 0.0f))
    {
        return NERV_ERROR;
    }

    switch (self->roll_state)
    {
    case NERV_GIMBAL_ROLL_SEEKING:
        if (NERV_OK != NERV_PID_Update(&self->roll_speed_pid, self->roll_seek_active_radps,
                                       motor->measurement.speed_radps, 0.0f, dt_s, &current_a))
        {
            return NERV_ERROR;
        }
        break;

    case NERV_GIMBAL_ROLL_HOLDING:
        if (NERV_OK != NERV_PID_Update(&self->roll_position_pid, self->roll_hold_angle_rad,
                                       motor->measurement.angle_rad, 0.0f, dt_s, &current_a))
        {
            return NERV_ERROR;
        }
        break;

    case NERV_GIMBAL_ROLL_IDLE:
        current_a = 0.0f;
        break;

    default:
        return NERV_ERROR;
    }

    control.kind     = NERV_MOTOR_CONTROL_SCALAR;
    control.u.scalar = current_a;

    return NERV_Motor_SetControl(motor, &control);
}

/**
  * @brief  更新 roll 堵转判定.
  * @param  self 实例指针.
  * @param  now_ms 当前时刻, 单位 ms.
  * @note   measurement.torque_n_m 已是输出轴转矩, 直接与转矩阈值比较. 转矩反馈型号不依赖
  *         扭矩常数, 电流反馈型号的扭矩常数在测量换算时已经使用.
  * @note   转矩绝对值达到 roll_stall_torque_n_m 且角速度绝对值低于 roll_stall_speed_radps
  *         时开始计时, 持续 roll_stall_ms 后切换到 NERV_GIMBAL_ROLL_HOLDING, 并以切换
  *         时刻的测量角度作为位置环目标.
  * @note   仅在 SEEKING 状态判定, 条件中断则重新计时.
  */
static void NERV_Gimbal_UpdateRollStall(NERV_Gimbal_InstanceTypeDef *self, uint32_t now_ms)
{
    const NERV_Motor_InstanceTypeDef *motor = self->config.motors[NERV_GIMBAL_AXIS_ROLL];
    float torque_n_m;
    float speed_radps;

    if ((NULL == motor) || (NERV_GIMBAL_ROLL_SEEKING != self->roll_state))
    {
        self->roll_stall_timing = false;
        return;
    }

    /* 测量值已是输出轴转矩, 两种反馈来源统一, 直接与转矩阈值比较. */
    torque_n_m  = fabsf(motor->measurement.torque_n_m);
    speed_radps = fabsf(motor->measurement.speed_radps);

    if ((torque_n_m >= self->config.roll_stall_torque_n_m) &&
        (speed_radps <= self->config.roll_stall_speed_radps))
    {
        if (!self->roll_stall_timing)
        {
            self->roll_stall_since_ms = now_ms;
            self->roll_stall_timing   = true;
        }
        else if ((now_ms - self->roll_stall_since_ms) >= self->config.roll_stall_ms)
        {
            /* 进入保持前复位位置环, 避免带入上一次保持积累的积分. */
            (void)NERV_PID_Reset(&self->roll_position_pid);

            self->roll_state          = NERV_GIMBAL_ROLL_HOLDING;
            self->roll_hold_angle_rad = motor->measurement.angle_rad;
            self->roll_stall_timing   = false;
        }
    }
    else
    {
        self->roll_stall_timing = false;
    }
}

/* 公开接口 -------------------------------------------------------------------*/

/**
  * @brief  用配置建立云台实例.
  * @param  self 实例指针, 不得为 NULL.
  * @param  config 配置指针, 不得为 NULL. 内容被复制, 调用方保留所有权.
  * @retval NERV_OK 配置已绑定, 实例进入就绪状态.
  * @retval NERV_ERROR self 或 config 为 NULL, 或配置字段非法.
  * @note   校验与复制在修改 self 之前完成, 失败时 self 保持原状.
  * @note   目标与测量姿态全部清零, 错误位复位.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_Init(NERV_Gimbal_InstanceTypeDef *self,
                                    const NERV_Gimbal_ConfigTypeDef *config)
{
    if ((NULL == self) || (NULL == config))
    {
        return NERV_ERROR;
    }

    if (NERV_OK != NERV_Gimbal_ValidateConfig(config))
    {
        return NERV_ERROR;
    }

    NERV_PID_InstanceTypeDef roll_speed_pid;
    NERV_PID_InstanceTypeDef roll_position_pid;

    /* 两个 PID 先初始化到局部变量, 全部成功后再写实例, 失败时 self 保持原状. */
    if ((NERV_OK != NERV_PID_Init(&roll_speed_pid, config->roll_speed_pid)) ||
        (NERV_OK != NERV_PID_Init(&roll_position_pid, config->roll_position_pid)))
    {
        return NERV_ERROR;
    }

    self->config            = *config;
    self->roll_speed_pid    = roll_speed_pid;
    self->roll_position_pid = roll_position_pid;

    self->target_attitude.yaw_big_speed_radps = 0.0f;
    self->target_attitude.yaw_small_rad       = 0.0f;
    self->target_attitude.pitch_rad           = 0.0f;

    self->measured_attitude.yaw_big_rad   = 0.0f;
    self->measured_attitude.yaw_small_rad = 0.0f;
    self->measured_attitude.pitch_rad     = 0.0f;
    self->measured_attitude.roll_rad      = 0.0f;
    self->measured_attitude.yaw_total_rad = 0.0f;

    self->roll_state             = NERV_GIMBAL_ROLL_IDLE;
    self->roll_stall_timing      = false;
    self->roll_seek_active_radps = 0.0f;
    self->roll_stall_since_ms    = 0U;
    self->target_valid           = false;

    self->error          = NERV_ERR_NONE;
    self->fault_callback = NULL;
    self->fault_user     = NULL;
    self->last_update_ms = 0U;
    self->initialized    = true;

    return NERV_OK;
}

/**
  * @brief  设置云台目标.
  * @param  self 实例指针, 不得为 NULL.
  * @param  target 目标指针, 不得为 NULL. 内容被复制.
  * @retval NERV_OK 目标已保存.
  * @retval NERV_ERROR self 或 target 为 NULL, 实例未初始化, 字段非有限值, 或角度越限.
  * @note   本结构体不含 roll 轴, roll 的目标由 NERV_Gimbal_StartRollSeek 单独设置.
  * @note   小 yaw 与 pitch 的目标角必须落在配置的限位内, 越限返回错误且不保存.
  * @note   大 yaw 是角速度目标, 不限角度.
  * @note   首次成功后 target_valid 置位, 在此之前 UpdateGroup 不下发任何指令.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_SetTarget(NERV_Gimbal_InstanceTypeDef *self,
                                         const NERV_Gimbal_TargetTypeDef *target)
{
    if ((NULL == self) || (NULL == target))
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if (!isfinite(target->yaw_big_speed_radps) || !isfinite(target->yaw_small_rad) ||
        !isfinite(target->pitch_rad))
    {
        return NERV_ERROR;
    }

    if (NERV_OK != NERV_Gimbal_CheckLimit(self, NERV_GIMBAL_AXIS_YAW_SMALL, target->yaw_small_rad))
    {
        return NERV_ERROR;
    }

    if (NERV_OK != NERV_Gimbal_CheckLimit(self, NERV_GIMBAL_AXIS_PITCH, target->pitch_rad))
    {
        return NERV_ERROR;
    }

    self->target_attitude = *target;
    self->target_valid    = true;

    return NERV_OK;
}

/**
  * @brief  周期更新: 刷新姿态, 下发四个轴的目标并发送.
  * @param  self 实例指针, 不得为 NULL.
  * @param  now_ms 当前时刻, 单位 ms.
  * @retval NERV_OK 本周期已完成.
  * @retval NERV_BUSY 至少一组指令因发送邮箱不可用而失败, 其余组照常发送.
  * @retval NERV_ERROR 实例为 NULL 或未初始化, 通道不支持所需控制分支, 或组发送失败.
  * @note   下发顺序为 大 yaw 速度, 小 yaw 位置, pitch 位置, roll 位置, 然后按组发送.
  * @note   某一轴目标角度超出限位时置 NERV_ERR_GIMBAL_LIMIT, 本周期不下发目标, 但仍
  *         执行电机状态刷新与故障回调.
  * @note   任一电机报离线或发送失败时置 NERV_ERR_GIMBAL_OFFLINE, 过温时置
  *         NERV_ERR_MOTOR_OVER_TEMP. 错误位为粘滞, 只由 NERV_Gimbal_ClearError 清除.
  * @note   错误位非零时进入安全状态, 只执行安全动作与测量刷新, 不下发正常目标.
  * @note   尚未调用 NERV_Gimbal_SetTarget 时不下发任何指令, 上电保持不动.
  * @note   时钟未推进时跳过本周期, 不推进控制状态.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_UpdateGroup(NERV_Gimbal_InstanceTypeDef *self, uint32_t now_ms)
{
    NERV_StatusTypeDef status;
    float dt_s;
    uint32_t error_before;

    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    error_before = self->error;
    status       = NERV_OK;

    /* 错误位非零表示已进入安全状态: 执行安全动作, 不下发正常目标. */
    if (0U != self->error)
    {
        NERV_Gimbal_ApplySafeState(self, now_ms);
        NERV_Gimbal_RefreshAttitude(self);
        self->last_update_ms = now_ms;

        if ((NULL != self->fault_callback) && (0U != (self->error & ~error_before)))
        {
            self->fault_callback(self->fault_user, self->error);
        }

        return NERV_ERROR;
    }

    /* 尚未设置目标: 上电保持不动, 不下发任何指令. */
    if (!self->target_valid)
    {
        self->last_update_ms = now_ms;

        return NERV_OK;
    }

    /* 限位失败只置位并记录状态, 不提前返回, 保证健康检查与故障回调仍会执行. */
    if ((NERV_OK != NERV_Gimbal_CheckLimit(self, NERV_GIMBAL_AXIS_YAW_SMALL,
                                           self->target_attitude.yaw_small_rad)) ||
        (NERV_OK !=
         NERV_Gimbal_CheckLimit(self, NERV_GIMBAL_AXIS_PITCH, self->target_attitude.pitch_rad)))
    {
        self->error |= NERV_ERR_GIMBAL_LIMIT;
        status = NERV_ERROR;
    }

    dt_s = (float)(now_ms - self->last_update_ms) / NERV_MATH_MS_PER_S;

    if (dt_s <= 0.0f)
    {
        /* 时钟未推进或回退, 本帧没有新的时间信息, 不推进控制状态, 对齐时间基准后跳过. */
        self->last_update_ms = now_ms;

        return NERV_OK;
    }

    if (dt_s > NERV_INNER_GIMBAL_MAX_DT_S)
    {
        dt_s = NERV_INNER_GIMBAL_MAX_DT_S;
    }

    /* 限位通过时才下发各轴目标; 任一轴失败只记录状态, 继续执行后续健康检查. */
    if (NERV_OK == status)
    {
        if (NERV_OK != NERV_Gimbal_ApplySpeed(self, NERV_GIMBAL_AXIS_YAW_BIG,
                                              self->target_attitude.yaw_big_speed_radps))
        {
            status = NERV_ERROR;
        }

        if (NERV_OK != NERV_Gimbal_ApplyPosition(self, NERV_GIMBAL_AXIS_YAW_SMALL,
                                                 self->target_attitude.yaw_small_rad,
                                                 self->config.yaw_small_speed_limit_radps))
        {
            status = NERV_ERROR;
        }

        if (NERV_OK != NERV_Gimbal_ApplyPosition(self, NERV_GIMBAL_AXIS_PITCH,
                                                 self->target_attitude.pitch_rad,
                                                 self->config.pitch_speed_limit_radps))
        {
            status = NERV_ERROR;
        }

        if (NERV_OK != NERV_Gimbal_ApplyRoll(self, dt_s))
        {
            status = NERV_ERROR;
        }
    }

    {
        const NERV_StatusTypeDef motor_status =
            NERV_Motor_UpdateGroup(self->config.motors, NERV_GIMBAL_AXIS_COUNT, now_ms);
        if ((NERV_OK != motor_status) && (NERV_OK == status))
        {
            status = motor_status;
        }
    }

    /* 姿态在下发之后刷新, 保证读到的测量值对应本周期收到的反馈. */
    NERV_Gimbal_RefreshAttitude(self);

    NERV_Gimbal_UpdateRollStall(self, now_ms);

    for (uint8_t axis = 0U; axis < NERV_GIMBAL_AXIS_COUNT; axis++)
    {
        const uint32_t motor_error = NERV_Motor_ReadError(self->config.motors[axis]);

        /* 发送失败与离线都等效于该轴不可用; 过温位直接沿用电机域的错误位. */
        if (0U != (motor_error & (NERV_ERR_MOTOR_OFFLINE | NERV_ERR_MOTOR_TX_FAIL)))
        {
            self->error |= NERV_ERR_GIMBAL_OFFLINE;
        }

        if (0U != (motor_error & NERV_ERR_MOTOR_OVER_TEMP))
        {
            self->error |= NERV_ERR_MOTOR_OVER_TEMP;
        }
    }

    self->last_update_ms = now_ms;

    /* 只在本次新增错误位时通知, 同一组错误持续存在不会重复回调. */
    if ((NULL != self->fault_callback) && (0U != (self->error & ~error_before)))
    {
        self->fault_callback(self->fault_user, self->error);
    }

    return status;
}

/**
  * @brief  读取当前姿态.
  * @param  self 实例指针, 不得为 NULL.
  * @param  attitude 输出姿态, 不得为 NULL.
  * @retval NERV_OK 已写入.
  * @retval NERV_ERROR self 或 attitude 为 NULL, 或实例未初始化.
  * @note   姿态由最近一次 NERV_Gimbal_UpdateGroup 刷新.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_GetAttitude(const NERV_Gimbal_InstanceTypeDef *self,
                                           NERV_Gimbal_AttitudeTypeDef *attitude)
{
    if ((NULL == self) || (NULL == attitude))
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    *attitude = self->measured_attitude;

    return NERV_OK;
}

/**
  * @brief  读取 roll 轴寻位状态.
  * @param  self 实例指针, 不得为 NULL.
  * @param  state 输出状态, 不得为 NULL.
  * @retval NERV_OK 已写入.
  * @retval NERV_ERROR self 或 state 为 NULL, 或实例未初始化.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_GetRollState(const NERV_Gimbal_InstanceTypeDef *self,
                                            NERV_Gimbal_RollStateTypeDef *state)
{
    if ((NULL == self) || (NULL == state))
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    *state = self->roll_state;

    return NERV_OK;
}

/**
  * @brief  启动 roll 寻位.
  * @param  self 实例指针, 不得为 NULL.
  * @param  speed_radps 寻位角速度, 单位 rad/s, 正负决定转向, 0 表示使用配置的默认值.
  * @retval NERV_OK 已进入寻位状态.
  * @retval NERV_ERROR self 为 NULL 或实例未初始化.
  * @note   寻位时速度环按 speed_radps 驱动, 检测到堵转后自动切换到位置环, 并把到位
  *         时刻的角度作为保持目标. 两侧限位各调用一次, 传入相反的符号即可.
  * @note   重复调用会以新的速度重新开始寻位, 并清除上一次的保持目标.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_StartRollSeek(NERV_Gimbal_InstanceTypeDef *self, float speed_radps)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if (!isfinite(speed_radps))
    {
        return NERV_ERROR;
    }

    if (0.0f == speed_radps)
    {
        self->roll_seek_active_radps = self->config.roll_seek_speed_radps;
    }
    else
    {
        self->roll_seek_active_radps = speed_radps;
    }

    self->roll_state          = NERV_GIMBAL_ROLL_SEEKING;
    self->roll_stall_timing   = false;
    self->roll_stall_since_ms = 0U;
    self->roll_hold_angle_rad = 0.0f;

    (void)NERV_PID_Reset(&self->roll_position_pid);

    return NERV_PID_Reset(&self->roll_speed_pid);
}

/**
  * @brief  读取云台错误位.
  * @param  self 实例指针, 可以为 NULL.
  * @return 粘滞错误位掩码, self 为 NULL 时返回 0.
  * @note   错误位只由 NERV_Gimbal_ClearError 清除.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
uint32_t NERV_Gimbal_ReadError(const NERV_Gimbal_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERR_NONE;
    }

    return self->error;
}

/**
  * @brief  注册故障回调.
  * @param  self 实例指针, 不得为 NULL.
  * @param  callback 回调, 可以为 NULL 表示注销.
  * @param  user 回调上下文, 可以为 NULL. 由调用者保证生命周期不短于注册期.
  * @retval NERV_OK 已记录.
  * @retval NERV_ERROR self 为 NULL 或实例未初始化.
  * @note   回调在 NERV_Gimbal_UpdateGroup 末尾执行, 属于任务上下文, 不得阻塞, 不得递归
  *         调用云台接口. 只在本次新增错误位时触发, 同一组错误持续存在不会重复通知.
  * @note   注册后于下一次 NERV_Gimbal_UpdateGroup 起生效, 注销后立即失效.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_RegisterFaultCallback(NERV_Gimbal_InstanceTypeDef *self,
                                                     NERV_Gimbal_FaultCallbackTypeDef callback,
                                                     void *user)
{
    if ((NULL == self) || (!self->initialized))
    {
        return NERV_ERROR;
    }

    self->fault_callback = callback;
    self->fault_user     = user;

    return NERV_OK;
}

/**
  * @brief  清除云台错误位.
  * @param  self 实例指针, 可以为 NULL.
  * @param  mask 要清除的位掩码.
  * @note   self 为 NULL 时为空操作.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
void NERV_Gimbal_ClearError(NERV_Gimbal_InstanceTypeDef *self, uint32_t mask)
{
    if (NULL == self)
    {
        return;
    }

    self->error &= ~(mask & NERV_ERR_BITS_MASK);
}

/**
  * @brief  执行安全动作: 停止各轴输出并立即发送.
  * @param  self 实例指针, 由调用方保证已初始化且不为 NULL.
  * @param  now_ms 当前时刻, 单位 ms.
  * @note   支持失能的型号(达妙)直接失能, 其余型号下发全零目标; 随后立即组发送一次,
  *         不等待下一个周期.
  */
static void NERV_Gimbal_ApplySafeState(NERV_Gimbal_InstanceTypeDef *self, uint32_t now_ms)
{
    self->roll_state             = NERV_GIMBAL_ROLL_IDLE;
    self->roll_stall_timing      = false;
    self->roll_hold_angle_rad    = 0.0f;
    self->roll_seek_active_radps = 0.0f;

    (void)NERV_PID_Reset(&self->roll_speed_pid);
    (void)NERV_PID_Reset(&self->roll_position_pid);

    for (uint8_t axis = 0U; axis < NERV_GIMBAL_AXIS_COUNT; axis++)
    {
        NERV_Motor_InstanceTypeDef *motor = self->config.motors[axis];

        (void)NERV_Motor_EnterSafeState(motor);

        if (NERV_Motor_Supports(motor, NERV_MOTOR_CAP_DISABLE))
        {
            (void)NERV_Motor_Disable(motor);
        }
    }

    (void)NERV_Motor_UpdateGroup(self->config.motors, NERV_GIMBAL_AXIS_COUNT, now_ms);
}

/**
  * @brief  进入安全状态: 停止各轴输出并置安全锁存位.
  * @param  self 实例指针, 不得为 NULL.
  * @param  now_ms 当前时刻, 单位 ms.
  * @retval NERV_OK 已进入安全状态.
  * @retval NERV_ERROR 实例为 NULL 或未初始化.
  * @note   位置轴目标保持当前测量角, 不归零, 避免安全状态后把轴驱向零点; 大 yaw
  *         速度目标清零, roll 退回 NERV_GIMBAL_ROLL_IDLE, 两个 roll PID 复位.
  * @note   置 NERV_ERR_SAFETY_LATCHED, 后续 NERV_Gimbal_UpdateGroup 看到错误位即保持
  *         安全状态, 直到 NERV_Gimbal_ClearError 清除.
  * @note   支持失能的型号(达妙)直接下发失能帧; 其余型号下发全零目标并立即组发送.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_EnterSafeState(NERV_Gimbal_InstanceTypeDef *self, uint32_t now_ms)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    /* 位置目标保持当前测量角, 不能归零, 否则后续周期会把轴驱向零点. */
    self->target_attitude.yaw_big_speed_radps = 0.0f;
    self->target_attitude.yaw_small_rad       = self->measured_attitude.yaw_small_rad;
    self->target_attitude.pitch_rad           = self->measured_attitude.pitch_rad;

    /* 置安全锁存位, 后续 UpdateGroup 依据错误位保持安全状态. */
    self->error |= (NERV_ERR_SAFETY_LATCHED & NERV_ERR_BITS_MASK);

    NERV_Gimbal_ApplySafeState(self, now_ms);
    NERV_Gimbal_RefreshAttitude(self);
    self->last_update_ms = now_ms;

    return NERV_OK;
}

/**
  * @brief  使能全部支持软件使能的电机.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 全部使能成功, 或型号无软件使能而跳过.
  * @retval NERV_ERROR 实例为 NULL 或未初始化, 或任一电机使能失败.
  * @note   不支持软件使能的型号 (例如 DJI) 跳过, 其电调上电即工作.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_Enable(NERV_Gimbal_InstanceTypeDef *self)
{
    if ((NULL == self) || (!self->initialized))
    {
        return NERV_ERROR;
    }

    NERV_StatusTypeDef status = NERV_OK;

    for (uint8_t axis = 0U; axis < NERV_GIMBAL_AXIS_COUNT; axis++)
    {
        NERV_Motor_InstanceTypeDef *motor = self->config.motors[axis];

        if (!NERV_Motor_Supports(motor, NERV_MOTOR_CAP_ENABLE))
        {
            continue;
        }

        if (NERV_OK != NERV_Motor_Enable(motor))
        {
            status = NERV_ERROR;
        }
    }

    return status;
}

/**
  * @brief  失能全部支持软件失能的电机.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 全部失能成功, 或型号无软件失能而跳过.
  * @retval NERV_ERROR 实例为 NULL 或未初始化, 或任一电机失能失败.
  * @note   不支持软件失能的型号 (例如 DJI) 跳过, 只能靠零电流停止.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Gimbal_Disable(NERV_Gimbal_InstanceTypeDef *self)
{
    if ((NULL == self) || (!self->initialized))
    {
        return NERV_ERROR;
    }

    NERV_StatusTypeDef status = NERV_OK;

    for (uint8_t axis = 0U; axis < NERV_GIMBAL_AXIS_COUNT; axis++)
    {
        NERV_Motor_InstanceTypeDef *motor = self->config.motors[axis];

        if (!NERV_Motor_Supports(motor, NERV_MOTOR_CAP_DISABLE))
        {
            continue;
        }

        if (NERV_OK != NERV_Motor_Disable(motor))
        {
            status = NERV_ERROR;
        }
    }

    return status;
}

#endif /* NERV_USE_GIMBAL */
