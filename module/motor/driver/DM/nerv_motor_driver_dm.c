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
#include "nerv_math.h"
#include "nerv_motor_driver.h"
#include "nerv_status.h"

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
    .control_mode                = (uint8_t)NERV_MOTOR_CMD_MIT,
    .control.profile.p_max_rad   = 12.5f,
    .control.profile.v_max_radps = 30.0f,
    .control.profile.t_max_n_m   = 10.0f,
    .control.profile.kp_max      = 500.0f,
    .control.profile.kd_max      = 5.0f,
};

/** @brief DM-J4310-2EC 速度模式型号描述符. */
static const NERV_Motor_ModelTypeDef dm_model_velocity = {
    .tx_id_low                 = 0x200U,
    .tx_id_high                = 0U,
    .rx_id_base                = 0U,
    .id_min                    = 1U,
    .id_max                    = 0x7FU,
    .motors_per_frame          = 1U,
    .frame_len                 = __NERV_DM_FRAME_LEN,
    .fb_angle_off              = NERV_MOTOR_FB_OFF_NONE,
    .fb_speed_off              = NERV_MOTOR_FB_OFF_NONE,
    .fb_current_off            = NERV_MOTOR_FB_OFF_NONE,
    .fb_temperature_off        = NERV_MOTOR_FB_OFF_NONE,
    .fb_bit_packed             = true,
    .angle_count               = 65535U,
    .fb_control_full_scale     = 30.0f,
    .fb_control_cmd_limit      = 4095,
    .tx_id_offset_by_device    = true,
    .rx_id_offset_by_device    = false,
    .control_mode              = (uint8_t)NERV_MOTOR_CMD_SCALAR,
    .control.scalar.full_scale = 30.0f,
    .control.scalar.cmd_limit  = 4095,
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
    .control_mode                = (uint8_t)NERV_MOTOR_CMD_POSITION_VELOCITY,
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
  * @note   控制帧标识符等于 tx_id_base 加设备 id (MIT 模式基值 0x000, 速度模式 0x200).
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
  * @param  cmd 控制目标, 不得为 NULL.
  * @retval NERV_OK 编码成功.
  * @retval NERV_ERROR 参数为 NULL, 设备 id 越界, 控制种类与型号模式不符, 或字段非法.
  * @note   MIT 模式按 p_des(16), v_des(12), Kp(12), Kd(12), t_ff(12) 位打包.
  *         速度模式按 v_des 四字节浮点小端写入.
  * @note   ISR-safe.
  */
static NERV_StatusTypeDef NERV_Motor_Dm_EncodeTx(const NERV_Motor_ModelTypeDef *model,
                                                 uint8_t *frame, uint8_t device_id,
                                                 const NERV_Motor_CmdTypeDef *cmd)
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

    if ((NULL == model) || (NULL == frame) || (NULL == cmd))
    {
        return NERV_ERROR;
    }

    if ((device_id < model->id_min) || (device_id > model->id_max))
    {
        return NERV_ERROR;
    }

    if (NERV_MOTOR_CMD_MIT == (NERV_Motor_CmdKindTypeDef)model->control_mode)
    {
        if (NERV_MOTOR_CMD_MIT != cmd->kind)
        {
            return NERV_ERROR;
        }

        p_raw  = NERV_Motor_Dm_EncodeBipolar(cmd->u.mit.position_rad,
                                             model->control.profile.p_max_rad, 65535U);
        v_raw  = NERV_Motor_Dm_EncodeBipolar(cmd->u.mit.velocity_radps,
                                             model->control.profile.v_max_radps, 4095U);
        kp_raw = NERV_Motor_Dm_EncodeUnipolar(cmd->u.mit.kp, model->control.profile.kp_max, 4095U);
        kd_raw = NERV_Motor_Dm_EncodeUnipolar(cmd->u.mit.kd, model->control.profile.kd_max, 4095U);
        t_raw = NERV_Motor_Dm_EncodeBipolar(cmd->u.mit.torque_n_m, model->control.profile.t_max_n_m,
                                            4095U);

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

    if (NERV_MOTOR_CMD_SCALAR == (NERV_Motor_CmdKindTypeDef)model->control_mode)
    {
        if (NERV_MOTOR_CMD_SCALAR != cmd->kind)
        {
            return NERV_ERROR;
        }

        velocity.f = cmd->u.scalar;

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

    if (NERV_MOTOR_CMD_POSITION_VELOCITY == (NERV_Motor_CmdKindTypeDef)model->control_mode)
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

        if (NERV_MOTOR_CMD_POSITION_VELOCITY != cmd->kind)
        {
            return NERV_ERROR;
        }

        position.f = cmd->u.position_velocity.position_rad;
        speed.f    = cmd->u.position_velocity.velocity_radps;

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

/** @brief 达妙 CAN 电机协议族操作表. */
static const NERV_Motor_Driver_OpsTypeDef dm_ops = {
    .get_frame_geo = NERV_Motor_Dm_GetFrameGeo,
    .encode_tx     = NERV_Motor_Dm_EncodeTx,
    .decode_fb     = NERV_Motor_Dm_DecodeFb,
};

const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmMit = {
    .ops   = &dm_ops,
    .model = &dm_model_mit,
};

const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmVelocity = {
    .ops   = &dm_ops,
    .model = &dm_model_velocity,
};

const NERV_Motor_DriverTypeDef NERV_Motor_Driver_DmPositionVelocity = {
    .ops   = &dm_ops,
    .model = &dm_model_position_velocity,
};

#endif /* NERV_USE_MOTOR && NERV_USE_MOTOR_DM */
