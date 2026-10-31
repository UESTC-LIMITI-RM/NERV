/**
  * @file    nerv_motor_driver_dm.c
  * @brief   达妙 DM-J4310-2EC 电机驱动.
  * @version 1.0.0
  * @date    2026-10-25
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "nerv_fdcan.h"
#include "nerv_math.h"
#include "nerv_motor_driver.h"
#include "nerv_status.h"

/** @brief 达妙特殊命令码, 占用 8 字节帧的最后一字节, 前 7 字节为 0xFF. */
#define __NERV_DM_CMD_CLEAR_ERROR 0xFBU
#define __NERV_DM_CMD_ENABLE      0xFCU
#define __NERV_DM_CMD_DISABLE     0xFDU
#define __NERV_DM_CMD_SAVE_ZERO   0xFEU

/** @brief 0x7FF 模式切换帧 D[4] 取值. */
#define __NERV_DM_MODE_MIT               1U
#define __NERV_DM_MODE_POSITION_VELOCITY 2U
#define __NERV_DM_MODE_VELOCITY          3U

static NERV_StatusTypeDef NERV_Motor_Dm_SendSpecial(const NERV_FDCAN_InstanceTypeDef *bus,
                                                    const NERV_Motor_ModelTypeDef *model,
                                                    uint8_t device_id, uint8_t command);

static NERV_StatusTypeDef NERV_Motor_Dm_SetMode(const NERV_FDCAN_InstanceTypeDef *bus,
                                                const NERV_Motor_ModelTypeDef *model,
                                                uint8_t device_id,
                                                NERV_Motor_ControlKindTypeDef mode);

#if (NERV_USE_MOTOR == NERV_ENABLE) && (NERV_USE_MOTOR_DM == NERV_ENABLE)

#define __NERV_DM_FRAME_LEN 8U
#define __NERV_DM_POS_MAX   65535.0f
#define __NERV_DM_AUX_MAX   4095.0f

/** @brief DM-J4310-2EC MIT 模式型号描述符. */
static const NERV_Motor_ModelTypeDef dm_model_mit = {
    .tx_id_low                   = 0U,
    .tx_id_high                  = 0U,
    .rx_id_base                  = 0U,
    .id_min                      = 1U,
    .id_max                      = 0x7FU,
    .motors_per_frame            = 1U,
    .frame_len                   = __NERV_DM_FRAME_LEN,
    .fb_angle_off                = NERV_MOTOR_FB_OFF_NONE,
    .fb_speed_off                = NERV_MOTOR_FB_OFF_NONE,
    .fb_current_off              = NERV_MOTOR_FB_OFF_NONE,
    .fb_temperature_off          = NERV_MOTOR_FB_OFF_NONE,
    .fb_bit_packed               = true,
    .angle_count                 = 65535U,
    .fb_control_full_scale       = 30.0f,
    .fb_control_cmd_limit        = 4095,
    .tx_id_offset_by_device      = true,
    .rx_id_offset_by_device      = false,
    .control_mode                = NERV_MOTOR_CONTROL_MIT,
    .torque_constant_n_m_per_a   = 1.2f,
    .rated_gear_ratio            = 10.0f,
    .fb_torque_present           = true,
    .control.profile.p_max_rad   = 12.5f,
    .control.profile.v_max_radps = 30.0f,
    .control.profile.t_max_n_m   = 10.0f,
    .control.profile.kp_max      = 500.0f,
    .control.profile.kd_max      = 5.0f,
};

/** @brief DM-J4310-2EC 速度模式型号描述符. */
static const NERV_Motor_ModelTypeDef dm_model_velocity = {
    .tx_id_low                   = 0x200U,
    .tx_id_high                  = 0U,
    .rx_id_base                  = 0U,
    .id_min                      = 1U,
    .id_max                      = 0x7FU,
    .motors_per_frame            = 1U,
    .frame_len                   = __NERV_DM_FRAME_LEN,
    .fb_angle_off                = NERV_MOTOR_FB_OFF_NONE,
    .fb_speed_off                = NERV_MOTOR_FB_OFF_NONE,
    .fb_current_off              = NERV_MOTOR_FB_OFF_NONE,
    .fb_temperature_off          = NERV_MOTOR_FB_OFF_NONE,
    .fb_bit_packed               = true,
    .angle_count                 = 65535U,
    .fb_control_full_scale       = 30.0f,
    .fb_control_cmd_limit        = 4095,
    .tx_id_offset_by_device      = true,
    .rx_id_offset_by_device      = false,
    .control_mode                = NERV_MOTOR_CONTROL_SCALAR,
    .torque_constant_n_m_per_a   = 1.2f,
    .rated_gear_ratio            = 10.0f,
    .fb_torque_present           = true,
    .scalar_quantity             = NERV_MOTOR_SCALAR_SPEED,
    .control.profile.p_max_rad   = 12.5f,
    .control.profile.v_max_radps = 30.0f,
    .control.profile.t_max_n_m   = 10.0f,
    .control.profile.kp_max      = 500.0f,
    .control.profile.kd_max      = 5.0f,
};

/** @brief DM-J4310-2EC 位置速度模式型号描述符. */
static const NERV_Motor_ModelTypeDef dm_model_position_velocity = {
    .tx_id_low                   = 0x100U,
    .tx_id_high                  = 0U,
    .rx_id_base                  = 0U,
    .id_min                      = 1U,
    .id_max                      = 0x7FU,
    .motors_per_frame            = 1U,
    .frame_len                   = __NERV_DM_FRAME_LEN,
    .fb_angle_off                = NERV_MOTOR_FB_OFF_NONE,
    .fb_speed_off                = NERV_MOTOR_FB_OFF_NONE,
    .fb_current_off              = NERV_MOTOR_FB_OFF_NONE,
    .fb_temperature_off          = NERV_MOTOR_FB_OFF_NONE,
    .fb_bit_packed               = true,
    .angle_count                 = 65535U,
    .fb_control_full_scale       = 30.0f,
    .fb_control_cmd_limit        = 4095,
    .tx_id_offset_by_device      = true,
    .rx_id_offset_by_device      = false,
    .control_mode                = NERV_MOTOR_CONTROL_POSITION_VELOCITY,
    .torque_constant_n_m_per_a   = 1.2f,
    .rated_gear_ratio            = 10.0f,
    .fb_torque_present           = true,
    .control.profile.p_max_rad   = 12.5f,
    .control.profile.v_max_radps = 30.0f,
    .control.profile.t_max_n_m   = 10.0f,
    .control.profile.kp_max      = 500.0f,
    .control.profile.kd_max      = 5.0f,
};

/**
  * @brief  极性量映射到无符号定点, 范围 [-max, +max] 对应 [0, bit_max].
  * @param  value 物理量.
  * @param  max_value 满量程绝对值.
  * @param  bit_max 编码满量程.
  * @return 无符号编码.
  */
static uint32_t NERV_Motor_Dm_EncodeBipolar(float value, float max_value, uint32_t bit_max)
{
    float normalized;

    if (!(max_value > 0.0f))
    {
        return 0U;
    }

    normalized = NERV_Math_Clamp(value / max_value, -1.0f, 1.0f);

    return (uint32_t)(((normalized * 0.5f) + 0.5f) * (float)bit_max + 0.5f);
}

/**
  * @brief  单极性量映射到无符号定点, 范围 [0, max] 对应 [0, bit_max].
  * @param  value 物理量.
  * @param  max_value 满量程.
  * @param  bit_max 编码满量程.
  * @return 无符号编码.
  */
static uint32_t NERV_Motor_Dm_EncodeUnipolar(float value, float max_value, uint32_t bit_max)
{
    float normalized;

    if (!(max_value > 0.0f))
    {
        return 0U;
    }

    normalized = NERV_Math_Clamp(value / max_value, 0.0f, 1.0f);

    return (uint32_t)(normalized * (float)bit_max + 0.5f);
}

/**
  * @brief  由设备 id 给出控制帧标识符, 反馈帧标识符与帧长.
  * @param  model 型号描述符, 不得为 NULL.
  * @param  device_id 电机 CAN ID, 范围 model 的 id_min 到 id_max.
  * @param  geo 输出帧几何, 不得为 NULL.
  * @retval NERV_OK 已写入.
  * @retval NERV_ERROR 参数为 NULL 或设备 id 越界.
  * @note   控制帧标识符等于 tx_id_low 加设备 id (MIT 模式基值 0x000, 速度模式 0x200).
  *         反馈帧标识符等于 rx_id_base 加设备 id, 要求每个电机的 Master ID 配置为
  *         0x000 加自身 CAN ID, 否则多电机共用同一 Master ID 会互相误收.
  * @note   ISR-safe.
  */
static NERV_StatusTypeDef NERV_Motor_Dm_GetFrameGeo(const NERV_Motor_ModelTypeDef *model,
                                                    uint8_t device_id,
                                                    NERV_Motor_FrameGeoTypeDef *geo)
{
    if ((NULL == model) || (NULL == geo))
    {
        return NERV_ERROR;
    }

    if ((device_id < model->id_min) || (device_id > model->id_max))
    {
        return NERV_ERROR;
    }

    geo->tx_id = model->tx_id_low + (uint32_t)device_id;
    geo->rx_id = model->rx_id_base + ((model->rx_id_offset_by_device) ? (uint32_t)device_id : 0U);
    geo->len   = model->frame_len;

    return NERV_OK;
}

/**
  * @brief  把控制目标编码到达妙控制帧.
  * @param  model 型号描述符, 不得为 NULL.
  * @param  frame 控制帧缓冲, 长度不小于 frame_len, 不得为 NULL.
  * @param  device_id 电机 CAN ID, 范围 model 的 id_min 到 id_max.
  * @param  control 控制目标, 不得为 NULL.
  * @retval NERV_OK 编码成功.
  * @retval NERV_ERROR 参数为 NULL, 设备 id 越界, 控制种类与型号模式不符, 或字段非法.
  * @note   MIT 模式按 p_des(16), v_des(12), Kp(12), Kd(12), t_ff(12) 位打包.
  *         速度模式按 v_des 四字节浮点小端写入.
  * @note   ISR-safe.
  */
static NERV_StatusTypeDef NERV_Motor_Dm_EncodeTx(const NERV_Motor_ModelTypeDef *model,
                                                 uint8_t *frame, uint8_t device_id,
                                                 const NERV_Motor_ControlTypeDef *control)
{
    uint32_t p_raw;
    uint32_t v_raw;
    uint32_t kp_raw;
    uint32_t kd_raw;
    uint32_t t_raw;
    union
    {
        float f;
        uint8_t b[4];
    } velocity;

    if ((NULL == model) || (NULL == frame) || (NULL == control))
    {
        return NERV_ERROR;
    }

    if ((device_id < model->id_min) || (device_id > model->id_max))
    {
        return NERV_ERROR;
    }

    if (NERV_MOTOR_CONTROL_MIT == (NERV_Motor_ControlKindTypeDef)model->control_mode)
    {
        if (NERV_MOTOR_CONTROL_MIT != control->kind)
        {
            return NERV_ERROR;
        }

        p_raw = NERV_Motor_Dm_EncodeBipolar(control->u.mit.position_rad,
                                            model->control.profile.p_max_rad, 65535U);
        v_raw = NERV_Motor_Dm_EncodeBipolar(control->u.mit.velocity_radps,
                                            model->control.profile.v_max_radps, 4095U);
        kp_raw =
            NERV_Motor_Dm_EncodeUnipolar(control->u.mit.kp, model->control.profile.kp_max, 4095U);
        kd_raw =
            NERV_Motor_Dm_EncodeUnipolar(control->u.mit.kd, model->control.profile.kd_max, 4095U);
        t_raw = NERV_Motor_Dm_EncodeBipolar(control->u.mit.torque_n_m,
                                            model->control.profile.t_max_n_m, 4095U);

        frame[0] = (uint8_t)((p_raw >> 8) & 0xFFU);
        frame[1] = (uint8_t)(p_raw & 0xFFU);
        frame[2] = (uint8_t)((v_raw >> 4) & 0xFFU);
        frame[3] = (uint8_t)(((v_raw & 0x0FU) << 4) | ((kp_raw >> 8) & 0x0FU));
        frame[4] = (uint8_t)(kp_raw & 0xFFU);
        frame[5] = (uint8_t)((kd_raw >> 4) & 0xFFU);
        frame[6] = (uint8_t)(((kd_raw & 0x0FU) << 4) | ((t_raw >> 8) & 0x0FU));
        frame[7] = (uint8_t)(t_raw & 0xFFU);

        return NERV_OK;
    }

    if (NERV_MOTOR_CONTROL_SCALAR == (NERV_Motor_ControlKindTypeDef)model->control_mode)
    {
        if (NERV_MOTOR_CONTROL_SCALAR != control->kind)
        {
            return NERV_ERROR;
        }

        velocity.f = control->u.scalar;

        frame[0] = velocity.b[0];
        frame[1] = velocity.b[1];
        frame[2] = velocity.b[2];
        frame[3] = velocity.b[3];
        frame[4] = 0U;
        frame[5] = 0U;
        frame[6] = 0U;
        frame[7] = 0U;

        return NERV_OK;
    }

    if (NERV_MOTOR_CONTROL_POSITION_VELOCITY == model->control_mode)
    {
        union
        {
            float f;
            uint8_t b[4];
        } position;
        union
        {
            float f;
            uint8_t b[4];
        } speed;

        if (NERV_MOTOR_CONTROL_POSITION_VELOCITY != control->kind)
        {
            return NERV_ERROR;
        }

        position.f = control->u.position_velocity.position_rad;
        speed.f    = control->u.position_velocity.velocity_radps;

        frame[0] = position.b[0];
        frame[1] = position.b[1];
        frame[2] = position.b[2];
        frame[3] = position.b[3];
        frame[4] = speed.b[0];
        frame[5] = speed.b[1];
        frame[6] = speed.b[2];
        frame[7] = speed.b[3];

        return NERV_OK;
    }

    return NERV_ERROR;
}

/**
  * @brief  把达妙反馈帧解码为电机反馈.
  * @param  model 型号描述符, 不得为 NULL.
  * @param  frame 反馈帧数据, 不得为 NULL.
  * @param  len 反馈帧长度, 必须等于 frame_len.
  * @param  fb 输出反馈, 不得为 NULL. 失败时不修改.
  * @retval NERV_OK 解码成功.
  * @retval NERV_ERROR 参数为 NULL 或长度不符.
  * @note   反馈帧布局: D[0] 为 ID 与故障码, D[1..2] 位置 16 位, D[3..4] 速度 12 位,
  *         D[4..5] 转矩 12 位, D[6] MOS 温度, D[7] 线圈温度.
  * @note   达妙反馈不含电流字段, current_a 置 0; 转矩由 torque_n_m 给出.
  * @note   ISR-safe.
  */
static NERV_StatusTypeDef NERV_Motor_Dm_DecodeFb(const NERV_Motor_ModelTypeDef *model,
                                                 const uint8_t *frame, uint8_t len,
                                                 NERV_Motor_FbTypeDef *fb)
{
    uint32_t pos_raw;
    uint32_t vel_raw;
    uint32_t torque_raw;

    if ((NULL == model) || (NULL == frame) || (NULL == fb))
    {
        return NERV_ERROR;
    }

    if (len != model->frame_len)
    {
        return NERV_ERROR;
    }

    pos_raw    = ((uint32_t)frame[1] << 8) | (uint32_t)frame[2];
    vel_raw    = ((uint32_t)frame[3] << 4) | ((uint32_t)frame[4] >> 4);
    torque_raw = (((uint32_t)frame[4] & 0x0FU) << 8) | (uint32_t)frame[5];

    fb->angle_rad = ((float)pos_raw / 65535.0f) * (2.0f * model->control.profile.p_max_rad) -
                    model->control.profile.p_max_rad;
    fb->speed_radps =
        (((float)vel_raw / 4095.0f) * 2.0f - 1.0f) * model->control.profile.v_max_radps;
    fb->torque_n_m =
        (((float)torque_raw / 4095.0f) * 2.0f - 1.0f) * model->control.profile.t_max_n_m;
    fb->current_a         = 0.0f;
    fb->temperature_c     = (int8_t)frame[7];
    fb->temperature_mos_c = (int8_t)frame[6];
    fb->error_code        = (uint8_t)(frame[0] >> 4);

    return NERV_OK;
}

static NERV_StatusTypeDef NERV_Motor_Dm_Enable(const NERV_FDCAN_InstanceTypeDef *bus,
                                               const NERV_Motor_ModelTypeDef *model,
                                               uint8_t device_id)
{
    return NERV_Motor_Dm_SendSpecial(bus, model, device_id, __NERV_DM_CMD_ENABLE);
}

static NERV_StatusTypeDef NERV_Motor_Dm_Disable(const NERV_FDCAN_InstanceTypeDef *bus,
                                                const NERV_Motor_ModelTypeDef *model,
                                                uint8_t device_id)
{
    return NERV_Motor_Dm_SendSpecial(bus, model, device_id, __NERV_DM_CMD_DISABLE);
}

static NERV_StatusTypeDef NERV_Motor_Dm_ClearFault(const NERV_FDCAN_InstanceTypeDef *bus,
                                                   const NERV_Motor_ModelTypeDef *model,
                                                   uint8_t device_id)
{
    return NERV_Motor_Dm_SendSpecial(bus, model, device_id, __NERV_DM_CMD_CLEAR_ERROR);
}

static NERV_StatusTypeDef NERV_Motor_Dm_SaveZero(const NERV_FDCAN_InstanceTypeDef *bus,
                                                 const NERV_Motor_ModelTypeDef *model,
                                                 uint8_t device_id)
{
    return NERV_Motor_Dm_SendSpecial(bus, model, device_id, __NERV_DM_CMD_SAVE_ZERO);
}

/** @brief 按能力位选择实现函数, 未声明时落回不支持实现. */
#define __NERV_OPS_PICK(_caps, _bit, _impl, _fallback) (((_caps) & (_bit)) ? (_impl) : (_fallback))

/** @brief 达妙驱动支持的全部可选能力. */
#define __NERV_DM_CAPS                                                             \
    (NERV_MOTOR_CAP_ENABLE | NERV_MOTOR_CAP_DISABLE | NERV_MOTOR_CAP_CLEAR_FAULT | \
     NERV_MOTOR_CAP_SET_MODE | NERV_MOTOR_CAP_SAVE_ZERO)

/** @brief 按能力位生成操作表, 未声明的能力自动落回不支持实现. */
#define __NERV_DM_OPS(_caps)                                                                        \
    {                                                                                               \
        .get_frame_geo = NERV_Motor_Dm_GetFrameGeo,                                                 \
        .encode_tx     = NERV_Motor_Dm_EncodeTx,                                                    \
        .decode_fb     = NERV_Motor_Dm_DecodeFb,                                                    \
        .enable        = __NERV_OPS_PICK(_caps, NERV_MOTOR_CAP_ENABLE, NERV_Motor_Dm_Enable,        \
                                         NERV_Motor_Driver_NotSupported),                           \
        .disable       = __NERV_OPS_PICK(_caps, NERV_MOTOR_CAP_DISABLE, NERV_Motor_Dm_Disable,      \
                                         NERV_Motor_Driver_NotSupported),                           \
        .clear_fault   = __NERV_OPS_PICK(_caps, NERV_MOTOR_CAP_CLEAR_FAULT,                         \
                                         NERV_Motor_Dm_ClearFault, NERV_Motor_Driver_NotSupported), \
        .set_mode      = __NERV_OPS_PICK(_caps, NERV_MOTOR_CAP_SET_MODE, NERV_Motor_Dm_SetMode,     \
                                         NERV_Motor_Driver_NotSupportedSetMode),                    \
        .save_zero     = __NERV_OPS_PICK(_caps, NERV_MOTOR_CAP_SAVE_ZERO, NERV_Motor_Dm_SaveZero,   \
                                         NERV_Motor_Driver_NotSupported),                           \
    }

/** @brief 达妙 CAN 电机协议族操作表. */
static const NERV_Motor_Driver_OpsTypeDef dm_ops = __NERV_DM_OPS(__NERV_DM_CAPS);

const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmMit = {
    .capabilities = NERV_MOTOR_CAP_ENABLE | NERV_MOTOR_CAP_DISABLE | NERV_MOTOR_CAP_CLEAR_FAULT |
                    NERV_MOTOR_CAP_SET_MODE | NERV_MOTOR_CAP_SAVE_ZERO,
    .ops          = &dm_ops,
    .model        = &dm_model_mit,
};

const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmVelocity = {
    .capabilities = NERV_MOTOR_CAP_ENABLE | NERV_MOTOR_CAP_DISABLE | NERV_MOTOR_CAP_CLEAR_FAULT |
                    NERV_MOTOR_CAP_SET_MODE | NERV_MOTOR_CAP_SAVE_ZERO,
    .ops          = &dm_ops,
    .model        = &dm_model_velocity,
};

const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmPositionVelocity = {
    .capabilities = NERV_MOTOR_CAP_ENABLE | NERV_MOTOR_CAP_DISABLE | NERV_MOTOR_CAP_CLEAR_FAULT |
                    NERV_MOTOR_CAP_SET_MODE | NERV_MOTOR_CAP_SAVE_ZERO,
    .ops          = &dm_ops,
    .model        = &dm_model_position_velocity,
};

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
static NERV_StatusTypeDef NERV_Motor_Dm_SendSpecial(const NERV_FDCAN_InstanceTypeDef *bus,
                                                    const NERV_Motor_ModelTypeDef *model,
                                                    uint8_t device_id, uint8_t command)
{
    NERV_FDCAN_FrameTypeDef frame;
    NERV_Motor_FrameGeoTypeDef geo;

    if ((NULL == bus) || (NULL == model))
    {
        return NERV_ERROR;
    }

    if (NERV_OK != NERV_Motor_Dm_GetFrameGeo(model, device_id, &geo))
    {
        return NERV_ERROR;
    }

    for (uint8_t index = 0U; index < __NERV_DM_FRAME_LEN; index++)
    {
        frame.data[index] = 0xFFU;
    }

    frame.data[__NERV_DM_FRAME_LEN - 1U] = command;
    frame.id                             = geo.tx_id;
    frame.len                            = __NERV_DM_FRAME_LEN;
    frame.flags                          = 0U;

    return NERV_FDCAN_Transmit(bus, &frame);
}

/**
  * @brief  写达妙控制模式寄存器.
  * @param  bus 电机所在总线, 不得为 NULL.
  * @param  device_id 电机 CAN ID.
  * @param  mode 模式, 取 NERV_MOTOR_DM_MODE_*.
  * @retval NERV_OK 模式帧已交给总线.
  * @retval NERV_ERROR 参数为 NULL, 模式非法, 或总线发送失败.
  * @note   帧为 8 字节标准帧, 标识符固定 0x7FF, 数据为
  *         device_id, 0x00, 0x55, 0x0A, mode, 0x00, 0x00, 0x00.
  * @note   切换流程建议: 先失能, 再写模式, 等待电机确认, 最后重新使能.
  * @warning 模式写入参数区并掉电保存. 参数区擦写寿命有限, 每个控制周期都调用会在
  *          数分钟内耗尽寿命并损坏电机. 只能在初始化或模式变更时调用一次.
  */
static NERV_StatusTypeDef NERV_Motor_Dm_SetMode(const NERV_FDCAN_InstanceTypeDef *bus,
                                                const NERV_Motor_ModelTypeDef *model,
                                                uint8_t device_id,
                                                NERV_Motor_ControlKindTypeDef mode)
{
    NERV_FDCAN_FrameTypeDef frame;
    uint8_t register_value;

    if ((NULL == bus) || (NULL == model))
    {
        return NERV_ERROR;
    }

    switch (mode)
    {
    case NERV_MOTOR_CONTROL_MIT:
        register_value = __NERV_DM_MODE_MIT;
        break;
    case NERV_MOTOR_CONTROL_POSITION_VELOCITY:
        register_value = __NERV_DM_MODE_POSITION_VELOCITY;
        break;
    case NERV_MOTOR_CONTROL_SCALAR:
        register_value = __NERV_DM_MODE_VELOCITY;
        break;
    default:
        return NERV_ERROR;
    }

    frame.id    = 0x7FFU;
    frame.len   = 8U;
    frame.flags = 0U;

    frame.data[0] = device_id;
    frame.data[1] = 0x00U;
    frame.data[2] = 0x55U;
    frame.data[3] = 0x0AU;
    frame.data[4] = register_value;
    frame.data[5] = 0x00U;
    frame.data[6] = 0x00U;
    frame.data[7] = 0x00U;

    return NERV_FDCAN_Transmit(bus, &frame);
}

#endif /* NERV_USE_MOTOR && NERV_USE_MOTOR_DM */
