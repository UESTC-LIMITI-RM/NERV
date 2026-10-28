/**
  * @file    nerv_motor_driver_dji.c
  * @brief   DJI CAN 电调协议族电机驱动实现.
  * @version 1.0.0
  * @date    2026-10-23
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include "nerv_motor_driver.h"

#include "config.h"

#if (NERV_USE_MOTOR == NERV_ENABLE)

#if (NERV_USE_MOTOR_DJI == NERV_ENABLE)

#include "nerv_math.h"
#include "nerv_status.h"
#include "nerv_utils.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/* ========================================================================== */
/* 以下定义为 DJI 协议族驱动内部实现常数, 禁止修改.                           */
/* 作用域: 所有 __NERV_MOTOR_DJI_ 开头的宏, 直至本定义区结束.                  */
/* ========================================================================== */

/** @brief 单条控制帧中每个电机指令占用的字节数. */
#define __NERV_MOTOR_DJI_MOTOR_BYTES 2U

/** @brief 单字节的位数, 用于显式大端编解码. */
#define __NERV_MOTOR_DJI_BYTE_BITS 8U

/* ========================================================================== */
/* DJI 协议族驱动内部常数定义区结束                                           */
/* ========================================================================== */

/* 本文件实现 DJI CAN 电调协议族. 同族型号之间只有常量差异, 全部放在型号描述符中.
 * 新增型号只需增加一个描述符常量与一个 NERV_Motor_DriverTypeDef 实例, 不增加函数. */

/**
  * @brief  DJI M3508 电机 (C620 电调) 型号描述符.
  * @note   控制帧按 4 个电调共用一条: 设备 id 1 到 4 用 tx_id_low, id 5 到 8 用
  *         tx_id_high. 反馈帧标识符等于 rx_id_base 加设备 id.
  * @note   本型号控制量为电流, 单位 A. C620 量程 20 A, 编码满量程 16384.
  * @note   数值依据 RoboMaster C620 无刷电机调速器使用说明 V1.01.
  */
static const NERV_Motor_ModelTypeDef dji_model_3508 = {
    .tx_id_low                 = 0x200U,
    .tx_id_high                = 0x1FFU,
    .rx_id_base                = 0x200U,
    .id_min                    = 1U,
    .id_max                    = 8U,
    .motors_per_frame          = 4U,
    .frame_len                 = 8U,
    .fb_angle_off              = 0U,
    .fb_speed_off              = 2U,
    .fb_current_off            = 4U,
    .fb_temperature_off        = 6U,
    .angle_count               = 8192U,
    .control.scalar.full_scale = 20.0f,
    .control.scalar.cmd_limit  = 16384,
    .fb_control_full_scale     = 20.0f,
    .fb_control_cmd_limit      = 16384,
};

/**
  * @brief  DJI M2006 电机 (C610 电调) 型号描述符.
  * @note   标识符规则同 M3508. 本型号控制量为电流, 单位 A. C610 量程 10 A,
  *         编码满量程 10000, 与 C620 的 20 A, 16384 不同, 这是两型号的主要差异.
  * @note   C610 反馈帧无温度字段, fb_temperature_off 取 NERV_MOTOR_FB_OFF_NONE,
  *         解码后 temperature_c 为 INT8_MIN.
  */
static const NERV_Motor_ModelTypeDef dji_model_2006 = {
    .tx_id_low                 = 0x200U,
    .tx_id_high                = 0x1FFU,
    .rx_id_base                = 0x200U,
    .id_min                    = 1U,
    .id_max                    = 8U,
    .motors_per_frame          = 4U,
    .frame_len                 = 8U,
    .fb_angle_off              = 0U,
    .fb_speed_off              = 2U,
    .fb_current_off            = 4U,
    .fb_temperature_off        = NERV_MOTOR_FB_OFF_NONE,
    .angle_count               = 8192U,
    .control.scalar.full_scale = 10.0f,
    .control.scalar.cmd_limit  = 10000,
    .fb_control_full_scale     = 10.0f,
    .fb_control_cmd_limit      = 10000,
};

/**
  * @brief  DJI GM6020 电机电流控制模式型号描述符.
  * @note   控制帧按 4 个电机共用一条: 设备 id 1 到 4 用 tx_id_low, id 5 到 7 用
  *         tx_id_high. 反馈帧标识符等于 rx_id_base 加设备 id.
  * @note   本模式控制量为电流, 单位 A, 编码 [-16384, 16384] 对应 [-3, 3] A.
  *         反馈中的控制量字段为实际输出电流, 编码相同.
  */
static const NERV_Motor_ModelTypeDef dji_model_6020_current = {
    .tx_id_low                 = 0x1FEU,
    .tx_id_high                = 0x2FEU,
    .rx_id_base                = 0x204U,
    .id_min                    = 1U,
    .id_max                    = 7U,
    .motors_per_frame          = 4U,
    .frame_len                 = 8U,
    .fb_angle_off              = 0U,
    .fb_speed_off              = 2U,
    .fb_current_off            = 4U,
    .fb_temperature_off        = 6U,
    .angle_count               = 8192U,
    .control.scalar.full_scale = 3.0f,
    .control.scalar.cmd_limit  = 16384,
    .fb_control_full_scale     = 3.0f,
    .fb_control_cmd_limit      = 16384,
};

/**
  * @brief  DJI GM6020 电机电压控制模式型号描述符.
  * @note   标识符规则同电流模式, 设备 id 1 到 4 用 tx_id_low, id 5 到 7 用
  *         tx_id_high. 反馈帧与电流模式相同.
  * @note   本模式控制量为电压给定值, 范围 [-25000, 25000], 协议未定义给定值与
  *         实际输出电压的对应关系, 调用方按协议给定值使用. 反馈中的控制量字段
  *         仍是实际输出电流, 编码与电流模式相同, 与指令编码不同.
  */
static const NERV_Motor_ModelTypeDef dji_model_6020_voltage = {
    .tx_id_low                 = 0x1FFU,
    .tx_id_high                = 0x2FFU,
    .rx_id_base                = 0x204U,
    .id_min                    = 1U,
    .id_max                    = 7U,
    .motors_per_frame          = 4U,
    .frame_len                 = 8U,
    .fb_angle_off              = 0U,
    .fb_speed_off              = 2U,
    .fb_current_off            = 4U,
    .fb_temperature_off        = 6U,
    .angle_count               = 8192U,
    .control.scalar.full_scale = 25000.0f,
    .control.scalar.cmd_limit  = 25000,
    .fb_control_full_scale     = 3.0f,
    .fb_control_cmd_limit      = 16384,
};

/**
  * @brief  按大端顺序读取 16 位无符号字段.
  * @param  data 帧数据起始地址, 不得为 NULL.
  * @param  offset 字段字节偏移, 读取 data[offset] 与 data[offset + 1].
  * @return 大端解码后的 16 位值.
  * @note   调用方必须保证 offset 加 1 在帧数据范围内.
  * @note   ISR-safe.
  */
static uint16_t NERV_Motor_Dji_ReadBE16(const uint8_t *data, uint8_t offset)
{
    return (uint16_t)(((uint16_t)data[offset] << __NERV_MOTOR_DJI_BYTE_BITS) |
                      (uint16_t)data[offset + 1U]);
}

/**
  * @brief  按大端顺序写入 16 位无符号字段.
  * @param  data 帧数据起始地址, 不得为 NULL.
  * @param  offset 字段字节偏移, 写入 data[offset] 与 data[offset + 1].
  * @param  value 待写入的 16 位值.
  * @note   调用方必须保证 offset 加 1 在帧数据范围内.
  * @note   ISR-safe.
  */
static void NERV_Motor_Dji_WriteBE16(uint8_t *data, uint8_t offset, uint16_t value)
{
    data[offset]      = (uint8_t)(value >> __NERV_MOTOR_DJI_BYTE_BITS);
    data[offset + 1U] = (uint8_t)(value & 0xFFU);
}

/**
  * @brief  由设备 id 给出帧几何信息.
  * @param  model 型号描述符, 不得为 NULL.
  * @param  device_id 电调 id, 范围 model 的 id_min 到 id_max.
  * @param  geo 输出帧几何信息, 不得为 NULL. 失败时不修改.
  * @retval NERV_OK 几何信息已写入 geo.
  * @retval NERV_ERROR model 或 geo 为 NULL, 或 device_id 越界.
  * @note   设备 id 减 id_min 之后小于 motors_per_frame 时控制帧用 tx_id_low,
  *         否则用 tx_id_high. 反馈帧标识符等于 rx_id_base 加 device_id.
  * @note   ISR-safe.
  */
static NERV_StatusTypeDef NERV_Motor_Dji_GetFrameGeo(const NERV_Motor_ModelTypeDef *model,
                                                     uint8_t device_id,
                                                     NERV_Motor_FrameGeoTypeDef *geo)
{
    uint8_t index;

    if ((NULL == model) || (NULL == geo))
    {
        return NERV_ERROR;
    }

    if ((device_id < model->id_min) || (device_id > model->id_max))
    {
        return NERV_ERROR;
    }

    index = (uint8_t)(device_id - model->id_min);

    geo->tx_id = (index < model->motors_per_frame) ? model->tx_id_low : model->tx_id_high;
    geo->rx_id = model->rx_id_base + device_id;
    geo->len   = model->frame_len;

    return NERV_OK;
}

/**
  * @brief  把控制编码编码到控制帧中该电调对应的位置.
  * @param  model 型号描述符, 不得为 NULL.
  * @param  frame 控制帧数据缓冲, 长度不小于 model 的 frame_len, 不得为 NULL.
  * @param  device_id 电调 id, 范围 model 的 id_min 到 id_max.
  * @param  control 控制编码, 与型号描述符 control.scalar.cmd_limit 同量纲 (如 16384, 25000).
  * @retval NERV_OK 编码成功, frame 中该电调对应位置已写入大端 int16 编码.
  * @retval NERV_ERROR model 或 frame 为 NULL, device_id 越界, 或 control 为非有限值.
  * @note   本函数只按 control.scalar.cmd_limit 把控制编码限幅到协议满量程. 实例级限幅由电机
  *         主逻辑在调用本函数之前完成.
  * @note   ISR-safe.
  */
static NERV_StatusTypeDef NERV_Motor_Dji_EncodeTx(const NERV_Motor_ModelTypeDef *model,
                                                  uint8_t *frame, uint8_t device_id,
                                                  const NERV_Motor_ControlTypeDef *control)
{
    float control_value;
    float control_clamped;
    int32_t raw;
    uint8_t offset;

    if ((NULL == model) || (NULL == frame) || (NULL == control) ||
        (control->kind != NERV_MOTOR_CONTROL_SCALAR) || !isfinite(control->u.scalar))
    {
        return NERV_ERROR;
    }

    if ((device_id < model->id_min) || (device_id > model->id_max))
    {
        return NERV_ERROR;
    }

    control_value = control->u.scalar;

    control_clamped = NERV_Math_Clamp(control_value, -(float)model->control.scalar.cmd_limit,
                                      (float)model->control.scalar.cmd_limit);

    raw = (int32_t)control_clamped;

    offset = (uint8_t)(((uint32_t)(device_id - model->id_min) % (uint32_t)model->motors_per_frame) *
                       (uint32_t)__NERV_MOTOR_DJI_MOTOR_BYTES);

    NERV_Motor_Dji_WriteBE16(frame, offset, (uint16_t)(int16_t)raw);

    return NERV_OK;
}

/**
  * @brief  把反馈帧解码为电机反馈.
  * @param  model 型号描述符, 不得为 NULL.
  * @param  frame 反馈帧数据, 不得为 NULL.
  * @param  len 反馈帧数据长度, 单位 byte, 必须等于 model 的 frame_len.
  * @param  fb 输出反馈, 不得为 NULL. 失败时不修改.
  * @retval NERV_OK 解码成功, fb 已写入.
  * @retval NERV_ERROR model, frame 或 fb 为 NULL, len 与 frame_len 不一致,
  *         或转子机械角度编码超出协议范围.
  * @note   多字节字段按大端顺序显式拼接, 不依赖主机字节序.
  * @note   温度字段为有符号 int8, 原值透传, 协议不表达 127 ℃ 以上的温度; 型号无
  *         温度字段时 temperature_c 置 INT8_MIN, 不读取帧数据.
  * @note   ISR-safe.
  */
static NERV_StatusTypeDef NERV_Motor_Dji_DecodeFb(const NERV_Motor_ModelTypeDef *model,
                                                  const uint8_t *frame, uint8_t len,
                                                  NERV_Motor_FbTypeDef *fb)
{
    uint16_t angle_raw;
    uint16_t speed_raw;
    uint16_t current_raw;

    if ((NULL == model) || (NULL == frame) || (NULL == fb))
    {
        return NERV_ERROR;
    }

    if (len != model->frame_len)
    {
        return NERV_ERROR;
    }

    angle_raw = NERV_Motor_Dji_ReadBE16(frame, model->fb_angle_off);

    if (angle_raw >= model->angle_count)
    {
        return NERV_ERROR;
    }

    speed_raw   = NERV_Motor_Dji_ReadBE16(frame, model->fb_speed_off);
    current_raw = NERV_Motor_Dji_ReadBE16(frame, model->fb_current_off);

    fb->angle_rad   = (float)angle_raw * NERV_MATH_TWO_PI / (float)model->angle_count;
    fb->speed_radps = NERV_Utils_RpmToRadps((float)(int16_t)speed_raw);
    fb->current_a   = (float)(int16_t)current_raw / (float)model->fb_control_cmd_limit *
                      model->fb_control_full_scale;

    if (model->fb_temperature_off == NERV_MOTOR_FB_OFF_NONE)
    {
        fb->temperature_c = INT8_MIN;
    }
    else
    {
        fb->temperature_c = (int8_t)frame[model->fb_temperature_off];
    }

    fb->torque_n_m        = 0.0f;
    fb->temperature_mos_c = INT8_MIN;
    fb->error_code        = 0U;

    return NERV_OK;
}

/** @brief DJI CAN 电调协议族操作表, 不提供使能与模式切换等可选能力. */
static const NERV_Motor_Driver_OpsTypeDef dji_ops = {
    .get_frame_geo = NERV_Motor_Dji_GetFrameGeo,
    .encode_tx     = NERV_Motor_Dji_EncodeTx,
    .decode_fb     = NERV_Motor_Dji_DecodeFb,
    .enable        = NERV_Motor_Driver_NotSupported,
    .disable       = NERV_Motor_Driver_NotSupported,
    .clear_fault   = NERV_Motor_Driver_NotSupported,
    .set_mode      = NERV_Motor_Driver_NotSupportedSetMode,
    .save_zero     = NERV_Motor_Driver_NotSupported,
};

/** @brief DJI M3508 电机 (C620 电调) 驱动实例. */
const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji3508 = {
    .capabilities = 0U,
    .ops          = &dji_ops,
    .model        = &dji_model_3508,
};

/** @brief DJI M2006 电机 (C610 电调) 驱动实例. */
const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji2006 = {
    .capabilities = 0U,
    .ops          = &dji_ops,
    .model        = &dji_model_2006,
};

/** @brief DJI GM6020 电机电流控制模式驱动实例. */
const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji6020Current = {
    .capabilities = 0U,
    .ops          = &dji_ops,
    .model        = &dji_model_6020_current,
};

/** @brief DJI GM6020 电机电压控制模式驱动实例. */
const NERV_Motor_DriverTypeDef NERV_Motor_Driver_Dji6020Voltage = {
    .capabilities = 0U,
    .ops          = &dji_ops,
    .model        = &dji_model_6020_voltage,
};

#endif /* NERV_USE_MOTOR_DJI */

#endif /* NERV_USE_MOTOR */
