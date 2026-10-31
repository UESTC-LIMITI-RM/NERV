/**
  * @file    nerv_chassis.c
  * @brief   NERV 底盘模块实现.
  * @version 2.0.0
  * @date    2026-10-30
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include <math.h>
#include <stddef.h>

#include "config.h"
#include "nerv_chassis.h"
#include "nerv_math.h"
#include "nerv_motor.h"
#include "nerv_status.h"

#if (NERV_USE_CHASSIS == NERV_ENABLE)

/** @brief 控制周期的上限, 单位 s.
  * @note  两次调用间隔超过本值时按本值计算. 长时间停更后, 斜坡, PID 积分与里程计
  *        会一次性推进一大步, 产生可见跳变. */
#define __NERV_CHASSIS_MAX_DT_S 0.1f

/** @brief 控制周期的下限, 单位 s.
  * @note  now_ms 未推进时差值为 0, PID 的微分项会退化为 0/0, 里程计也不前进.
  *        把间隔钳到本值, 保证每个周期都有可用的正间隔. */
#define __NERV_CHASSIS_MIN_DT_S 0.001f

/** @brief 麦克纳姆轮正解的方向系数和, 取 2.
  * @note  四轮布局下 vx 与 vy 的正解分母为 2 * DIRECTION_SUM, 即 4;
  *        wz 的分母再乘等效力臂 (center_offset_x_m + center_offset_y_m). */
#define __NERV_CHASSIS_DIRECTION_SUM 2.0f

/** @brief 全向轮正解系数, 等于 1/(2 * sqrt(2)), 约 0.35355339.
  * @note  各轮驱动方向为周向切向, 分量是 1/sqrt(2). 四轮求和后乘以本系数得到车体
  *        平移速度. 与轮径和布局尺寸无关, 只由驱动方向决定. */
#define __NERV_CHASSIS_OMNI_FORCE_FACTOR 0.35355339f

/** @brief 电机量程有效性下限.
  * @note  control 是 union, 型号描述符填错字段时, 原字段的位模式可能被解释成一个
  *        极小的正浮点数. 仅判断大于 0 会让这种脏数据通过校验, 因此用本值过滤. */
#define __NERV_CHASSIS_MIN_SCALE 1.0e-3f

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)

/* 轮号 0 到 3 依次为左前, 左后, 右后, 右前. */
static const float wheel_sign_x[NERV_CHASSIS_WHEEL_COUNT] = {1.0f, 1.0f, -1.0f, -1.0f};
static const float wheel_sign_y[NERV_CHASSIS_WHEEL_COUNT] = {1.0f, -1.0f, -1.0f, 1.0f};

#endif /* NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI */

static NERV_StatusTypeDef NERV_Chassis_ValidateMotor(const NERV_Motor_InstanceTypeDef *motor);
static void NERV_Chassis_ResetState(NERV_Chassis_InstanceTypeDef *self);
static void NERV_Chassis_RampVelocity(NERV_Chassis_InstanceTypeDef *self, float dt_s);
static NERV_StatusTypeDef NERV_Chassis_SolveForward(const NERV_Chassis_InstanceTypeDef *self,
                                                    NERV_Chassis_VelocityTypeDef *measured);
static NERV_StatusTypeDef NERV_Chassis_SolveWheelSpeed(const NERV_Chassis_InstanceTypeDef *self,
                                                       const NERV_Chassis_VelocityTypeDef *command,
                                                       float *wheel_speed_radps);
static NERV_StatusTypeDef
NERV_Chassis_ComputeVelocityCommand(NERV_Chassis_InstanceTypeDef *self,
                                    const NERV_Chassis_VelocityTypeDef *measured, float dt_s,
                                    NERV_Chassis_VelocityTypeDef *command);
static void NERV_Chassis_UpdateHealth(NERV_Chassis_InstanceTypeDef *self);

#if (NERV_USE_CHASSIS_ODOMETRY == NERV_ENABLE)
static void NERV_Chassis_UpdateOdometry(NERV_Chassis_InstanceTypeDef *self,
                                        const NERV_Chassis_VelocityTypeDef *measured, float dt_s);
#endif

_Static_assert((int)NERV_CHASSIS_TYPE_MECANUM == (int)NERV_CHASSIS_TYPE_MECANUM_RT,
               "NERV_CHASSIS_TYPE_MECANUM and NERV_CHASSIS_TYPE_MECANUM_RT must be equal.");
_Static_assert((int)NERV_CHASSIS_TYPE_OMNI == (int)NERV_CHASSIS_TYPE_OMNI_RT,
               "NERV_CHASSIS_TYPE_OMNI and NERV_CHASSIS_TYPE_OMNI_RT must be equal.");

/**
  * @brief  校验电机实例可用于底盘的标量电流控制.
  * @param  motor 电机实例, 可以为 NULL.
  * @retval NERV_OK 实例已初始化, 驱动完整, 型号支持 SCALAR 且量程有效.
  * @retval NERV_ERROR 任一条件不满足.
  * @note   底盘按标量电流下发, 因此只接受 SCALAR 模式; 电压模式同属 SCALAR,
  *         当前无法从描述符区分, 需调用方保证填入的是电流模式型号.
  */
static NERV_StatusTypeDef NERV_Chassis_ValidateMotor(const NERV_Motor_InstanceTypeDef *motor)
{
    const NERV_Motor_DriverTypeDef *driver;
    const NERV_Motor_ModelTypeDef *model;

    if ((NULL == motor) || (!motor->initialized))
    {
        return NERV_ERROR;
    }

    driver = motor->config.driver;

    if ((NULL == driver) || (NULL == driver->ops) || (NULL == driver->model))
    {
        return NERV_ERROR;
    }

    model = driver->model;

    if ((NERV_MOTOR_CONTROL_SCALAR != (NERV_Motor_ControlKindTypeDef)model->control_mode) ||
        !(model->control.scalar.full_scale > __NERV_CHASSIS_MIN_SCALE) ||
        (model->control.scalar.cmd_limit <= 0))
    {
        return NERV_ERROR;
    }

    return NERV_OK;
}

/**
  * @brief  复位实例的软件状态, 不清除配置.
  * @param  self 实例指针, 由调用方保证不为 NULL.
  */
static void NERV_Chassis_ResetState(NERV_Chassis_InstanceTypeDef *self)
{
    self->target_velocity.vx_mps    = 0.0f;
    self->target_velocity.vy_mps    = 0.0f;
    self->target_velocity.wz_radps  = 0.0f;
    self->planned_velocity.vx_mps   = 0.0f;
    self->planned_velocity.vy_mps   = 0.0f;
    self->planned_velocity.wz_radps = 0.0f;
    self->error                     = NERV_ERR_NONE;
    self->last_reported_error       = 0U;
    self->last_update_ms            = 0U;

    (void)NERV_PID_Reset(&self->translate_pid[0]);
    (void)NERV_PID_Reset(&self->translate_pid[1]);
    (void)NERV_PID_Reset(&self->rotate_pid);

    for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
    {
        (void)NERV_PID_Reset(&self->wheel_speed_pid[index]);
    }
}

/**
  * @brief  按加速度限幅把目标速度逼近到规划速度.
  * @param  self 实例指针, 由调用方保证不为 NULL.
  * @param  dt_s 控制间隔, 单位 s.
  * @note   限幅项为 0 时对应通道直接跟随目标.
  */
static void NERV_Chassis_RampVelocity(NERV_Chassis_InstanceTypeDef *self, float dt_s)
{
    float step;

    if (self->config.max_translate_accel_mps2 > 0.0f)
    {
        step = self->config.max_translate_accel_mps2 * dt_s;

        self->planned_velocity.vx_mps =
            NERV_Math_Clamp(self->target_velocity.vx_mps, self->planned_velocity.vx_mps - step,
                            self->planned_velocity.vx_mps + step);
        self->planned_velocity.vy_mps =
            NERV_Math_Clamp(self->target_velocity.vy_mps, self->planned_velocity.vy_mps - step,
                            self->planned_velocity.vy_mps + step);
    }
    else
    {
        self->planned_velocity.vx_mps = self->target_velocity.vx_mps;
        self->planned_velocity.vy_mps = self->target_velocity.vy_mps;
    }

    if (self->config.max_rotate_accel_radps2 > 0.0f)
    {
        step = self->config.max_rotate_accel_radps2 * dt_s;

        self->planned_velocity.wz_radps =
            NERV_Math_Clamp(self->target_velocity.wz_radps, self->planned_velocity.wz_radps - step,
                            self->planned_velocity.wz_radps + step);
    }
    else
    {
        self->planned_velocity.wz_radps = self->target_velocity.wz_radps;
    }
}

/**
  * @brief  由各轮测量角速度正解机体系速度.
  * @param  self 实例指针, 不得为 NULL.
  * @param  measured 输出机体系速度, 不得为 NULL.
  * @retval NERV_OK 已写入.
  * @retval NERV_ERROR 参数为 NULL, 输入非有限值或几何非法.
  * @note   全向轮使用 NERV_Chassis_Init 预计算的等效力臂, 麦克纳姆轮只用地心距之和.
  */
static NERV_StatusTypeDef NERV_Chassis_SolveForward(const NERV_Chassis_InstanceTypeDef *self,
                                                    NERV_Chassis_VelocityTypeDef *measured)
{
    const NERV_Chassis_ConfigTypeDef *config;
    float wheel_speed_radps[NERV_CHASSIS_WHEEL_COUNT];
    float center_distance_m;

    if ((NULL == self) || (NULL == measured))
    {
        return NERV_ERROR;
    }

    for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
    {
        wheel_speed_radps[index] = self->config.drive_motors[index]->measurement.speed_radps;
    }

    config = &self->config;
    for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
    {
        if (!isfinite(wheel_speed_radps[index]))
        {
            return NERV_ERROR;
        }
    }

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)

    (void)center_distance_m;

    {
        float lever_sum_m;

        lever_sum_m = config->center_offset_x_m + config->center_offset_y_m;

        if (!(lever_sum_m > 0.0f))
        {
            return NERV_ERROR;
        }

        measured->vx_mps   = (-wheel_speed_radps[0] - wheel_speed_radps[1] + wheel_speed_radps[2] +
                              wheel_speed_radps[3]) *
                             config->wheel_radius_m / (2.0f * __NERV_CHASSIS_DIRECTION_SUM);
        measured->vy_mps   = (wheel_speed_radps[0] - wheel_speed_radps[1] - wheel_speed_radps[2] +
                              wheel_speed_radps[3]) *
                             config->wheel_radius_m / (2.0f * __NERV_CHASSIS_DIRECTION_SUM);
        measured->wz_radps = (wheel_speed_radps[0] + wheel_speed_radps[1] + wheel_speed_radps[2] +
                              wheel_speed_radps[3]) *
                             config->wheel_radius_m /
                             (2.0f * __NERV_CHASSIS_DIRECTION_SUM * lever_sum_m);
    }

#else

    {
        float sum_direction_x;
        float sum_direction_y;
        float sum_lever;
        float speed_mps;

        if (!(center_distance_m > 0.0f))
        {
            return NERV_ERROR;
        }

        sum_direction_x = 0.0f;
        sum_direction_y = 0.0f;
        sum_lever       = 0.0f;

        for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
        {
            speed_mps = wheel_speed_radps[index] * config->wheel_radius_m;
            sum_direction_x += speed_mps * (-wheel_sign_y[index]);
            sum_direction_y += speed_mps * wheel_sign_x[index];
            sum_lever += speed_mps * center_distance_m;
        }

        measured->vx_mps = sum_direction_x * __NERV_CHASSIS_OMNI_FORCE_FACTOR;
        measured->vy_mps = sum_direction_y * __NERV_CHASSIS_OMNI_FORCE_FACTOR;
        measured->wz_radps =
            sum_lever / ((float)NERV_CHASSIS_WHEEL_COUNT * center_distance_m * center_distance_m);
    }

#endif

    return NERV_OK;
}

/**
  * @brief  把机体系速度逆解为各轮目标角速度.
  * @param  self 实例指针, 不得为 NULL.
  * @param  command 目标机体系速度, 不得为 NULL.
  * @param  wheel_speed_radps 输出各轮目标角速度, 单位 rad/s, 不得为 NULL.
  * @retval NERV_OK 已写入.
  * @retval NERV_ERROR 参数为 NULL 或几何非法.
  * @note   全向轮使用 NERV_Chassis_Init 预计算的等效力臂, 麦克纳姆轮只用地心距之和.
  */
static NERV_StatusTypeDef NERV_Chassis_SolveWheelSpeed(const NERV_Chassis_InstanceTypeDef *self,
                                                       const NERV_Chassis_VelocityTypeDef *command,
                                                       float *wheel_speed_radps)
{
    const NERV_Chassis_ConfigTypeDef *config;
    float center_distance_m;

    if ((NULL == self) || (NULL == command) || (NULL == wheel_speed_radps))
    {
        return NERV_ERROR;
    }

    config            = &self->config;
    center_distance_m = 0.0f;

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)

    center_distance_m = self->center_distance_m;

#endif

    if (!(config->wheel_radius_m > 0.0f))
    {
        return NERV_ERROR;
    }

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)

    (void)center_distance_m;

    {
        float lever_sum_m;

        lever_sum_m = config->center_offset_x_m + config->center_offset_y_m;

        if (!(lever_sum_m > 0.0f))
        {
            return NERV_ERROR;
        }

        wheel_speed_radps[0] =
            (-command->vx_mps + command->vy_mps + command->wz_radps * lever_sum_m) /
            config->wheel_radius_m;
        wheel_speed_radps[1] =
            (-command->vx_mps - command->vy_mps + command->wz_radps * lever_sum_m) /
            config->wheel_radius_m;
        wheel_speed_radps[2] =
            (command->vx_mps - command->vy_mps + command->wz_radps * lever_sum_m) /
            config->wheel_radius_m;
        wheel_speed_radps[3] =
            (command->vx_mps + command->vy_mps + command->wz_radps * lever_sum_m) /
            config->wheel_radius_m;
    }

#else

    if (!(center_distance_m > 0.0f))
    {
        return NERV_ERROR;
    }

    for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
    {
        wheel_speed_radps[index] =
            ((command->vx_mps * (-wheel_sign_y[index]) + command->vy_mps * wheel_sign_x[index]) /
                 sqrtf(2.0f) +
             command->wz_radps * center_distance_m) /
            config->wheel_radius_m;
    }

#endif

    return NERV_OK;
}

/**
  * @brief  运行车体速度环, 输出带修正的目标速度.
  * @param  self 实例指针, 必须已初始化, 由调用方保证不为 NULL.
  * @param  measured 当前机体系速度, 由正解算得出, 不得为 NULL.
  * @param  dt_s 控制间隔, 单位 s.
  * @param  command 输出修正后的目标速度, 不得为 NULL.
  * @retval NERV_OK 控制量已写入 command.
  * @retval NERV_ERROR 参数为 NULL 或 PID 更新失败.
  */
static NERV_StatusTypeDef
NERV_Chassis_ComputeVelocityCommand(NERV_Chassis_InstanceTypeDef *self,
                                    const NERV_Chassis_VelocityTypeDef *measured, float dt_s,
                                    NERV_Chassis_VelocityTypeDef *command)
{
    float adjust;

    if ((NULL == self) || (NULL == measured) || (NULL == command))
    {
        return NERV_ERROR;
    }

    if (NERV_OK != NERV_PID_Update(&self->translate_pid[0], self->planned_velocity.vx_mps,
                                   measured->vx_mps, 0.0f, dt_s, &adjust))
    {
        return NERV_ERROR;
    }
    command->vx_mps = self->planned_velocity.vx_mps + adjust;

    if (NERV_OK != NERV_PID_Update(&self->translate_pid[1], self->planned_velocity.vy_mps,
                                   measured->vy_mps, 0.0f, dt_s, &adjust))
    {
        return NERV_ERROR;
    }
    command->vy_mps = self->planned_velocity.vy_mps + adjust;

    if (NERV_OK != NERV_PID_Update(&self->rotate_pid, self->planned_velocity.wz_radps,
                                   measured->wz_radps, 0.0f, dt_s, &adjust))
    {
        return NERV_ERROR;
    }
    command->wz_radps = self->planned_velocity.wz_radps + adjust;

    return NERV_OK;
}

/**
  * @brief  汇总电机状态并触发故障回调.
  * @param  self 实例指针, 由调用方保证不为 NULL.
  * @note   离线与发送失败置底盘错误位, 过温沿用电机错误位.
  */
static void NERV_Chassis_UpdateHealth(NERV_Chassis_InstanceTypeDef *self)
{
    uint32_t motor_error;

    for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
    {
        motor_error = NERV_Motor_ReadError(self->config.drive_motors[index]);

        if (0U != (motor_error & NERV_ERR_MOTOR_OFFLINE))
        {
            self->error |= NERV_ERR_CHASSIS_WHEEL_OFFLINE;
        }

        if (0U != (motor_error & NERV_ERR_MOTOR_TX_FAIL))
        {
            self->error |= NERV_ERR_CHASSIS_WHEEL_OFFLINE;
        }

        if (0U != (motor_error & NERV_ERR_MOTOR_OVER_TEMP))
        {
            self->error |= NERV_ERR_MOTOR_OVER_TEMP;
        }
    }

    if ((NULL != self->fault_callback) && (self->error != self->last_reported_error))
    {
        self->last_reported_error = self->error;
        self->fault_callback(self->fault_user, self->error);
    }
}

#if (NERV_USE_CHASSIS_ODOMETRY == NERV_ENABLE)

/**
  * @brief  按机体系速度积分里程计.
  * @param  self 实例指针, 由调用方保证不为 NULL.
  * @param  measured 机体系速度, 不得为 NULL.
  * @param  dt_s 控制间隔, 单位 s.
  */
static void NERV_Chassis_UpdateOdometry(NERV_Chassis_InstanceTypeDef *self,
                                        const NERV_Chassis_VelocityTypeDef *measured, float dt_s)
{
    float yaw_rad;

    yaw_rad = self->odom.attitude.yaw_rad;

    self->odom.x_m += (measured->vx_mps * cosf(yaw_rad) - measured->vy_mps * sinf(yaw_rad)) * dt_s;
    self->odom.y_m += (measured->vx_mps * sinf(yaw_rad) + measured->vy_mps * cosf(yaw_rad)) * dt_s;
    self->odom.attitude.yaw_rad =
        NERV_Math_Wrap(yaw_rad + measured->wz_radps * dt_s, -NERV_MATH_PI, NERV_MATH_PI);
    self->odom.attitude.pitch_rad = 0.0f;
    self->odom.attitude.roll_rad  = 0.0f;
}

#endif /* NERV_USE_CHASSIS_ODOMETRY */

/**
  * @brief  用配置建立底盘实例: 校验配置与电机实例指针, 建立运行期状态.
  * @param  self 实例指针, 不得为 NULL.
  * @param  config 配置指针, 不得为 NULL. 内容被复制, 调用方保留所有权.
  * @retval NERV_OK 配置已绑定, 实例进入就绪状态.
  * @retval NERV_ERROR 参数为 NULL, 任一电机指针为 NULL, 电机未初始化或不是标量电流模式,
  *         几何参数不为正有限值, 限幅项为负或非有限值, 或 PID 配置非法.
  * @note   校验全部在修改 self 之前完成, 失败时 self 保持原状.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Chassis_Init(NERV_Chassis_InstanceTypeDef *self,
                                     const NERV_Chassis_ConfigTypeDef *config)
{
    NERV_PID_InstanceTypeDef pid_probe;

    if ((NULL == self) || (NULL == config))
    {
        return NERV_ERROR;
    }

    for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
    {
        if (NERV_OK != NERV_Chassis_ValidateMotor(config->drive_motors[index]))
        {
            return NERV_ERROR;
        }
    }

    if (!isfinite(config->wheel_radius_m) || (config->wheel_radius_m <= 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->center_offset_x_m) || (config->center_offset_x_m <= 0.0f) ||
        !isfinite(config->center_offset_y_m) || (config->center_offset_y_m <= 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->max_translate_accel_mps2) || (config->max_translate_accel_mps2 < 0.0f) ||
        !isfinite(config->max_rotate_accel_radps2) || (config->max_rotate_accel_radps2 < 0.0f))
    {
        return NERV_ERROR;
    }

    if ((NULL == config->translate_pid) || (NULL == config->rotate_pid) ||
        (NULL == config->wheel_pid) ||
        (NERV_OK != NERV_PID_Init(&pid_probe, config->translate_pid)) ||
        (NERV_OK != NERV_PID_Init(&pid_probe, config->rotate_pid)) ||
        (NERV_OK != NERV_PID_Init(&pid_probe, config->wheel_pid)))
    {
        return NERV_ERROR;
    }

    self->config = *config;

    NERV_Chassis_ResetState(self);

    (void)NERV_PID_Init(&self->translate_pid[0], config->translate_pid);
    (void)NERV_PID_Init(&self->translate_pid[1], config->translate_pid);
    (void)NERV_PID_Init(&self->rotate_pid, config->rotate_pid);

    for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
    {
        (void)NERV_PID_Init(&self->wheel_speed_pid[index], config->wheel_pid);
    }

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)

    self->center_distance_m = (config->center_offset_x_m + config->center_offset_y_m) / sqrtf(2.0f);

#endif

#if (NERV_USE_CHASSIS_ODOMETRY == NERV_ENABLE)

    self->odom.x_m                = 0.0f;
    self->odom.y_m                = 0.0f;
    self->odom.attitude.yaw_rad   = 0.0f;
    self->odom.attitude.pitch_rad = 0.0f;
    self->odom.attitude.roll_rad  = 0.0f;

#endif

    self->initialized = true;

    return NERV_OK;
}

/**
  * @brief  注册故障回调.
  * @param  self 实例指针, 不得为 NULL.
  * @param  callback 回调函数, NULL 表示取消注册.
  * @param  user 回调上下文, 可以为 NULL.
  * @retval NERV_OK 已注册.
  * @retval NERV_ERROR 实例为 NULL 或未初始化.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Chassis_RegisterFaultCallback(NERV_Chassis_InstanceTypeDef *self,
                                                      NERV_Chassis_FaultCallbackTypeDef callback,
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
  * @brief  设置目标机体系速度.
  * @param  self 实例指针, 不得为 NULL.
  * @param  target 目标速度, 不得为 NULL, 三个分量必须为有限值.
  * @retval NERV_OK 目标已保存.
  * @retval NERV_ERROR 参数为 NULL, 实例未初始化, 或分量非有限值.
  * @note   目标速度经加速度限幅后由 NERV_Chassis_Update 跟踪.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Chassis_SetVelocity(NERV_Chassis_InstanceTypeDef *self,
                                            const NERV_Chassis_VelocityTypeDef *target)
{
    if ((NULL == self) || (NULL == target))
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if (!isfinite(target->vx_mps) || !isfinite(target->vy_mps) || !isfinite(target->wz_radps))
    {
        return NERV_ERROR;
    }

    self->target_velocity = *target;

    return NERV_OK;
}

/**
  * @brief  周期更新: 斜坡, 正解, 车体速度环, 逆解, 轮速环, 下发与监督.
  * @param  self 实例指针, 不得为 NULL.
  * @param  now_ms 当前时刻, 单位 ms. 由调用方提供, 库不读取硬件时钟.
  * @retval NERV_OK 本周期控制链已完成.
  * @retval NERV_ERROR 实例为 NULL 或未初始化, 或正解, 速度环, 逆解, 轮速环任一失败.
  * @note   轮速环输出电流, 经 NERV_Motor_SetTarget 下发; 指令发送由
  *         NERV_Motor_UpdateGroup 完成.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Chassis_Update(NERV_Chassis_InstanceTypeDef *self, uint32_t now_ms)
{
    NERV_Motor_MeasurementTypeDef measurement;
    NERV_Chassis_VelocityTypeDef measured;
    NERV_Chassis_VelocityTypeDef velocity_command;
    float wheel_speed_radps[NERV_CHASSIS_WHEEL_COUNT];
    float wheel_speed_target[NERV_CHASSIS_WHEEL_COUNT];
    float drive_current_a[NERV_CHASSIS_WHEEL_COUNT];
    float dt_s;
    bool ok;

    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    dt_s = (float)(now_ms - self->last_update_ms) / 1000.0f;

    if (dt_s > __NERV_CHASSIS_MAX_DT_S)
    {
        dt_s = __NERV_CHASSIS_MAX_DT_S;
    }
    else if (dt_s <= 0.0f)
    {
        dt_s = __NERV_CHASSIS_MIN_DT_S;
    }

    NERV_Chassis_RampVelocity(self, dt_s);

    for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
    {
        if (NERV_OK == NERV_Motor_GetMeasurement(self->config.drive_motors[index], &measurement))
        {
            wheel_speed_radps[index] = measurement.speed_radps;
        }
        else
        {
            wheel_speed_radps[index] = 0.0f;
        }
    }

    ok = (NERV_OK == NERV_Chassis_SolveForward(self, &measured));

    if (ok)
    {
        ok = (NERV_OK ==
              NERV_Chassis_ComputeVelocityCommand(self, &measured, dt_s, &velocity_command));
    }

    if (ok)
    {
        ok = (NERV_OK == NERV_Chassis_SolveWheelSpeed(self, &velocity_command, wheel_speed_target));
    }

    if (ok)
    {
        for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
        {
            if (NERV_OK != NERV_PID_Update(&self->wheel_speed_pid[index], wheel_speed_target[index],
                                           wheel_speed_radps[index], 0.0f, dt_s,
                                           &drive_current_a[index]))
            {
                ok = false;
                break;
            }
        }
    }

    if (ok)
    {
        for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
        {
            if (NERV_OK !=
                NERV_Motor_SetTarget(self->config.drive_motors[index], drive_current_a[index]))
            {
                self->error |= NERV_ERR_CHASSIS_WHEEL_OFFLINE;
                ok = false;
            }
        }
    }
    else
    {
        self->error |= NERV_ERR_CHASSIS_KINEMATICS;
    }

    (void)NERV_Motor_UpdateGroup(self->config.drive_motors, NERV_CHASSIS_WHEEL_COUNT, now_ms);

#if (NERV_USE_CHASSIS_ODOMETRY == NERV_ENABLE)

    if (ok)
    {
        NERV_Chassis_UpdateOdometry(self, &measured, dt_s);
    }

#endif

    self->last_update_ms = now_ms;

    NERV_Chassis_UpdateHealth(self);

    return ok ? NERV_OK : NERV_ERROR;
}

#if (NERV_USE_CHASSIS_ODOMETRY == NERV_ENABLE)

/**
  * @brief  读取里程计.
  * @param  self 实例指针, 不得为 NULL.
  * @param  odom 输出里程计, 不得为 NULL.
  * @retval NERV_OK 已写入.
  * @retval NERV_ERROR 参数为 NULL 或实例未初始化.
  * @note   ISR-safe. 只读实例状态.
  */
NERV_StatusTypeDef NERV_Chassis_GetOdometry(const NERV_Chassis_InstanceTypeDef *self,
                                            NERV_Chassis_OdomTypeDef *odom)
{
    if ((NULL == self) || (NULL == odom) || (!self->initialized))
    {
        return NERV_ERROR;
    }

    *odom = self->odom;

    return NERV_OK;
}

/**
  * @brief  覆盖里程计.
  * @param  self 实例指针, 不得为 NULL.
  * @param  odom 目标里程计, 不得为 NULL, 位置与航向必须为有限值.
  * @retval NERV_OK 已写入.
  * @retval NERV_ERROR 参数为 NULL, 实例未初始化, 或数值非有限.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Chassis_SetOdometry(NERV_Chassis_InstanceTypeDef *self,
                                            const NERV_Chassis_OdomTypeDef *odom)
{
    if ((NULL == self) || (NULL == odom))
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if (!isfinite(odom->x_m) || !isfinite(odom->y_m) || !isfinite(odom->attitude.yaw_rad))
    {
        return NERV_ERROR;
    }

    self->odom = *odom;

    return NERV_OK;
}

#endif /* NERV_USE_CHASSIS_ODOMETRY */

/**
  * @brief  读取底盘粘滞错误位.
  * @param  self 实例指针, 可以为 NULL.
  * @return 错误位掩码, 可取 NERV_ERR_CHASSIS_KINEMATICS, NERV_ERR_CHASSIS_WHEEL_OFFLINE
  *         与 NERV_ERR_MOTOR_OVER_TEMP. self 为 NULL 时返回 NERV_ERR_NONE.
  * @note   读取不清除错误位.
  * @note   ISR-safe. 只读实例状态.
  */
uint32_t NERV_Chassis_ReadError(const NERV_Chassis_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERR_NONE;
    }

    return self->error & NERV_ERR_BITS_MASK;
}

/**
  * @brief  清除底盘错误位.
  * @param  self 实例指针, 可以为 NULL.
  * @param  mask 待清除的错误位掩码, 只使用低 24 位.
  * @note   只清除 mask 中置位的错误位. self 为 NULL 时为空操作.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
void NERV_Chassis_ClearError(NERV_Chassis_InstanceTypeDef *self, uint32_t mask)
{
    if (NULL == self)
    {
        return;
    }

    self->error &= ~(mask & NERV_ERR_BITS_MASK);
}

/**
  * @brief  进入安全状态: 目标速度清零, 速度环与轮速环复位, 下发零电流.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 已进入安全状态.
  * @retval NERV_ERROR 实例为 NULL 或未初始化.
  * @note   不清除配置与错误位.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Chassis_EnterSafeState(NERV_Chassis_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    self->target_velocity.vx_mps   = 0.0f;
    self->target_velocity.vy_mps   = 0.0f;
    self->target_velocity.wz_radps = 0.0f;
    self->planned_velocity         = self->target_velocity;

    (void)NERV_PID_Reset(&self->translate_pid[0]);
    (void)NERV_PID_Reset(&self->translate_pid[1]);
    (void)NERV_PID_Reset(&self->rotate_pid);

    for (uint8_t index = 0U; index < NERV_CHASSIS_WHEEL_COUNT; index++)
    {
        (void)NERV_PID_Reset(&self->wheel_speed_pid[index]);
        (void)NERV_Motor_SetTarget(self->config.drive_motors[index], 0.0f);
    }

    return NERV_OK;
}

#endif /* NERV_USE_CHASSIS_ENABLE */
