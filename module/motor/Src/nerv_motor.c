/**
  * @file    nerv_motor.c
  * @brief   NERV 电机模块实现.
  * @version 1.0.0
  * @date    2026-10-23
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "nerv_math.h"
#include "nerv_motor.h"
#include "nerv_status.h"
#include "nerv_utils.h"

#if (NERV_USE_MOTOR == NERV_ENABLE)

/* 内部实现常数. */
#define __NERV_MOTOR_DIRECTION_SIGN_INVERTED (-1.0f)
#define __NERV_MOTOR_CONFIG_ACTIVE_F(_value) ((_value) > 0.0f)
#define __NERV_MOTOR_CONFIG_ACTIVE_U(_value) ((_value) > 0U)
#define __NERV_MOTOR_CONTROL_LIMIT_MAX       32767U

static NERV_StatusTypeDef NERV_Motor_ValidateModel(const NERV_Motor_ModelTypeDef *model);
static void NERV_Motor_ResetState(NERV_Motor_InstanceTypeDef *self);
static float NERV_Motor_InvertSign(const NERV_Motor_InstanceTypeDef *self);
static NERV_StatusTypeDef NERV_Motor_ConvertFb(NERV_Motor_InstanceTypeDef *self);
static NERV_Motor_ControlTypeDef NERV_Motor_ActiveControl(const NERV_Motor_InstanceTypeDef *self);
static NERV_StatusTypeDef NERV_Motor_EncodeInto(const NERV_Motor_InstanceTypeDef *self,
                                                uint8_t *frame);
static void NERV_Motor_UpdateHealth(NERV_Motor_InstanceTypeDef *self, uint32_t now_ms);
static bool NERV_Motor_SameGroup(const NERV_Motor_InstanceTypeDef *lhs,
                                 const NERV_Motor_InstanceTypeDef *rhs);
static void NERV_Motor_SetGroupTxFail(NERV_Motor_InstanceTypeDef *const *members, uint8_t count,
                                      uint8_t first);
static void NERV_Motor_ClearGroupTxFail(NERV_Motor_InstanceTypeDef *const *members, uint8_t count,
                                        uint8_t first);
static NERV_StatusTypeDef NERV_Motor_ValidateGroups(NERV_Motor_InstanceTypeDef *const *members,
                                                    uint8_t count);
static NERV_StatusTypeDef NERV_Motor_TransmitGroupFrame(NERV_Motor_InstanceTypeDef *const *members,
                                                        uint8_t count, uint8_t first);

/**
  * @brief  用配置建立电机实例: 校验配置, 核对设备 id, 建立运行期状态.
  * @param  self 实例指针, 不得为 NULL.
  * @param  config 配置指针, 不得为 NULL. 内容被复制, 调用方保留所有权.
  * @retval NERV_OK 配置已绑定, 帧几何信息已缓存, 实例进入就绪状态.
  * @retval NERV_ERROR self 或 config 为 NULL, config 的 driver, driver 的 model,
  *         driver 的 ops 或 bus 为 NULL, ops 的任一操作指针为空,
  *         gear_ratio 不为大于 0 的有限值, over_temperature_c 为负或非有限值,
  *         control_limit 超过 32767, device_id 超出描述符的 id_min 到 id_max 范围,
  *         或帧几何信息的帧长为 0 或超过 NERV_FDCAN_DATA_MAX, 标识符超过 11 位.
  * @note   不直接访问外设: 电调无需初始化指令, 帧几何信息只经驱动操作表派生.
  *         调用点仍应按 FDCAN 启动之后安排.
  * @note   校验与几何派生全部在修改 self 之前完成, 失败时 self 保持原状.
  * @note   重复调用以新配置重建实例, 复位错误位, 接收计时, 反馈与目标.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_Init(NERV_Motor_InstanceTypeDef *self,
                                   const NERV_Motor_ConfigTypeDef *config)
{
    NERV_Motor_FrameGeoTypeDef geo;

    if ((NULL == self) || (NULL == config))
    {
        return NERV_ERROR;
    }

    if ((NULL == config->driver) || (NULL == config->driver->model) ||
        (NULL == config->driver->ops) || (NULL == config->driver->ops->get_frame_geo) ||
        (NULL == config->driver->ops->encode_tx) || (NULL == config->driver->ops->decode_fb) ||
        (NULL == config->driver->ops->enable) || (NULL == config->driver->ops->disable) ||
        (NULL == config->driver->ops->clear_fault) || (NULL == config->driver->ops->set_mode) ||
        (NULL == config->driver->ops->save_zero) || (NULL == config->bus))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->gear_ratio) || (config->gear_ratio <= 0.0f))
    {
        return NERV_ERROR;
    }

    if (!isfinite(config->over_temperature_c) || (config->over_temperature_c < 0.0f))
    {
        return NERV_ERROR;
    }

    if (config->control_limit > __NERV_MOTOR_CONTROL_LIMIT_MAX)
    {
        return NERV_ERROR;
    }

    if (NERV_OK != NERV_Motor_ValidateModel(config->driver->model))
    {
        return NERV_ERROR;
    }

    if (NERV_OK !=
        config->driver->ops->get_frame_geo(config->driver->model, config->device_id, &geo))
    {
        return NERV_ERROR;
    }

    if ((0U == geo.len) || (geo.len > NERV_FDCAN_DATA_MAX) ||
        (geo.tx_id > NERV_FDCAN_ID_STANDARD_MAX) || (geo.rx_id > NERV_FDCAN_ID_STANDARD_MAX))
    {
        return NERV_ERROR;
    }

    self->config = *config;

    NERV_Motor_ResetState(self);

    self->geo         = geo;
    self->initialized = true;

    return NERV_OK;
}

/**
  * @brief  设置多字段控制目标.
  * @param  self 实例指针, 不得为 NULL.
  * @param  control 控制目标, 不得为 NULL. 按 kind 校验对应分支.
  * @retval NERV_OK 目标已保存.
  * @retval NERV_ERROR self 或 control 为 NULL, 实例未初始化, kind 不受支持, 或字段非法.
  * @note   MIT 分支要求 position, velocity, kp, kd, torque 均为有限值, 且 kp, kd 非负.
  * @note   本函数不限幅. 实例限幅与协议满量程限幅在发送时进行.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_SetControl(NERV_Motor_InstanceTypeDef *self,
                                         const NERV_Motor_ControlTypeDef *control)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (NULL == control)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    switch (control->kind)
    {
    case NERV_MOTOR_CONTROL_SCALAR:
        if (!isfinite(control->u.scalar))
        {
            return NERV_ERROR;
        }
        break;

    case NERV_MOTOR_CONTROL_POSITION_VELOCITY:
        if (!isfinite(control->u.position_velocity.position_rad) ||
            !isfinite(control->u.position_velocity.velocity_radps))
        {
            return NERV_ERROR;
        }
        break;

    case NERV_MOTOR_CONTROL_MIT:
        if (!isfinite(control->u.mit.position_rad) || !isfinite(control->u.mit.velocity_radps) ||
            !isfinite(control->u.mit.kp) || !isfinite(control->u.mit.kd) ||
            !isfinite(control->u.mit.torque_n_m) || (control->u.mit.kp < 0.0f) ||
            (control->u.mit.kd < 0.0f))
        {
            return NERV_ERROR;
        }
        break;

    default:
        return NERV_ERROR;
    }

    self->target_control = *control;

    return NERV_OK;
}

/**
  * @brief  设置单一控制量目标, 等价于 SCALAR 分支.
  * @param  self 实例指针, 不得为 NULL.
  * @param  target 控制量目标, 单位与语义由型号描述符约定, 必须为有限值.
  * @retval NERV_OK 目标已保存.
  * @retval NERV_ERROR self 为 NULL, 实例未初始化, 或 target 为非有限值.
  * @note   本函数不限幅. 实例限幅与协议满量程限幅在发送时进行.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_SetTarget(NERV_Motor_InstanceTypeDef *self, float target)
{
    NERV_Motor_ControlTypeDef control;

    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!isfinite(target))
    {
        return NERV_ERROR;
    }

    control.kind     = NERV_MOTOR_CONTROL_SCALAR;
    control.u.scalar = target;

    return NERV_Motor_SetControl(self, &control);
}

/**
  * @brief  刷新机构系测量值并执行离线与过温监督, 不发送指令.
  * @param  self 实例指针, 不得为 NULL.
  * @param  now_ms 当前时刻, 单位 ms. 由调用方提供, 库不读取硬件时钟.
  * @retval NERV_OK 状态已刷新, 监督已执行.
  * @retval NERV_ERROR self 为 NULL 或实例未初始化.
  * @note   监督结果写入错误位, 不通过返回值报告. 错误位为粘滞, 只由
  *         NERV_Motor_ClearError 清除.
  * @note   NERV_Motor_ConvertFb 失败分支为防御性保留: 已初始化实例的输入恒有效,
  *         该分支实际不可达. 若未来引入会失败的换算校验, 失败时本周期不执行监督.
  * @note   指令发送由 NERV_Motor_UpdateGroup 完成.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_Update(NERV_Motor_InstanceTypeDef *self, uint32_t now_ms)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if (NERV_OK != NERV_Motor_ConvertFb(self))
    {
        return NERV_ERROR;
    }

    NERV_Motor_UpdateHealth(self, now_ms);

    return NERV_OK;
}

/**
  * @brief  更新一组电机: 逐成员刷新状态与监督, 按总线与指令帧标识符分组发送.
  * @param  members 实例指针数组, 不得为 NULL. 每个元素不得为 NULL 且必须已初始化.
  * @param  count 数组元素个数, 必须大于 0.
  * @param  now_ms 当前时刻, 单位 ms.
  * @retval NERV_OK 全部指令帧已交给 FDCAN 发送.
  * @retval NERV_BUSY 至少一个组的指令帧在重发后仍因邮箱不可用而失败. 该组全部成员置
  *         NERV_ERR_MOTOR_TX_FAIL 错误位, 其余组照常发送, 下周期按当前目标重建.
  * @retval NERV_ERROR members 为 NULL, count 为 0, 数组含 NULL 元素, 成员未初始化,
  *         同一总线与指令帧标识符的成员帧长不一致, 同一总线与指令帧标识符内设备 id
  *         重复, 或编码失败. 发送阶段的编码失败同样置失败组的 TX_FAIL 位.
  * @note   参数与分组一致性错误在刷新与发送之前返回, 不会产生部分发送. 发送阶段的
  *         错误不回滚已处理成员的测量值刷新与已发送的指令帧.
  * @note   分组键为总线实例与指令帧标识符. 同一组每周期只发送一次, 帧内只包含该组
  *         成员的槽位, 其余槽位为 0. 同一指令帧标识符位于不同总线时分别发送.
  * @note   帧长取该组首个成员的值, 同一组的成员必须帧长一致.
  * @note   发送按组独立进行: 某组首轮失败时当场重发一次, 仍失败则置该组 TX_FAIL 错误位
  *         并继续发送后续组, 返回值汇总为首个失败状态. 单条总线的邮箱压力不会饿死
  *         其他总线上的组. 发送成功的组自动清除 TX_FAIL 位.
  * @warning 同一组内设备 id 必须唯一, 否则两个成员编码到同一槽位互相覆盖.
  * @warning 同一指令帧标识符的电调必须全部纳入同一次调用的 members. 未纳入的槽位
  *          发 0, 等于给同组其他电调发零指令, 可能清零由其他任务控制的电调.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_UpdateGroup(NERV_Motor_InstanceTypeDef *const *members, uint8_t count,
                                          uint32_t now_ms)
{
    NERV_StatusTypeDef overall;
    NERV_StatusTypeDef status;

    if ((NULL == members) || (0U == count))
    {
        return NERV_ERROR;
    }

    for (uint8_t index = 0U; index < count; index++)
    {
        if ((NULL == members[index]) || (!members[index]->initialized))
        {
            return NERV_ERROR;
        }
    }

    if (NERV_OK != NERV_Motor_ValidateGroups(members, count))
    {
        return NERV_ERROR;
    }

    for (uint8_t index = 0U; index < count; index++)
    {
        if (NERV_OK != NERV_Motor_Update(members[index], now_ms))
        {
            return NERV_ERROR;
        }
    }

    overall = NERV_OK;

    for (uint8_t first = 0U; first < count; first++)
    {
        bool first_of_group = true;

        for (uint8_t index = 0U; index < first; index++)
        {
            if (NERV_Motor_SameGroup(members[index], members[first]))
            {
                first_of_group = false;
                break;
            }
        }

        if (!first_of_group)
        {
            continue;
        }

        status = NERV_Motor_TransmitGroupFrame(members, count, first);

        if (NERV_OK != status)
        {
            /* 首轮失败后当场重发一次, 发送邮箱通常在下一拍即可用. */
            status = NERV_Motor_TransmitGroupFrame(members, count, first);
        }

        if (NERV_OK == status)
        {
            NERV_Motor_ClearGroupTxFail(members, count, first);
        }
        else
        {
            NERV_Motor_SetGroupTxFail(members, count, first);

            if (NERV_OK == overall)
            {
                overall = status;
            }
        }
    }

    return overall;
}

/**
  * @brief  接收反馈帧入口, 由应用的总线接收回调调用.
  * @param  self 实例指针, 可以为 NULL.
  * @param  frame 接收到的帧, 可以为 NULL.
  * @note   帧标识符与实例缓存的反馈标识符不一致时忽略该帧. 扩展帧与标准帧的标识符
  *         位宽不同, 即使低位相同也不匹配, 扩展帧一律忽略. 解码失败时丢弃, 不更新
  *         反馈, 不置位 fb_fresh.
  * @note   只做解码, 存储与置位 fb_fresh, 不读取时钟, 不维护错误位. 解码含浮点
  *         换算, 在 ISR 上下文中执行, 属于 O(1) 轻量操作; 调用方不得在本函数内追加
  *         耗时工作, 回调时长直接增加总线接收延迟.
  * @note   ISR-safe.
  */
void NERV_Motor_OnRx(NERV_Motor_InstanceTypeDef *self, const NERV_FDCAN_FrameTypeDef *frame)
{
    if ((NULL == self) || (NULL == frame))
    {
        return;
    }

    if (!self->initialized)
    {
        return;
    }

    if (0U != (((uint32_t)frame->flags) & (uint32_t)NERV_FDCAN_FLAG_EXTENDED))
    {
        return;
    }

    if (frame->id != self->geo.rx_id)
    {
        return;
    }

    if (NERV_OK != self->config.driver->ops->decode_fb(self->config.driver->model, frame->data,
                                                       frame->len, &self->fb))
    {
        return;
    }

    self->fb_fresh = true;
}

/**
  * @brief  读取机构系测量值.
  * @param  self 实例指针, 不得为 NULL.
  * @param  measurement 输出测量值, 不得为 NULL. 失败时不修改.
  * @retval NERV_OK 测量值已写入 measurement.
  * @retval NERV_ERROR self 或 measurement 为 NULL, 或实例未初始化.
  * @note   测量值由 NERV_Motor_Update 或 NERV_Motor_UpdateGroup 刷新, 读取发生在
  *         任务上下文. 收到第一次反馈之前为初始零值.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_GetMeasurement(const NERV_Motor_InstanceTypeDef *self,
                                             NERV_Motor_MeasurementTypeDef *measurement)
{
    if ((NULL == self) || (NULL == measurement))
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    *measurement = self->measurement;

    return NERV_OK;
}

/**
  * @brief  读取电机错误位.
  * @param  self 实例指针, 可以为 NULL.
  * @return 粘滞错误位掩码, 可取 NERV_ERR_MOTOR_OFFLINE,
  *         NERV_ERR_MOTOR_OVER_TEMP 与 NERV_ERR_MOTOR_TX_FAIL. self 为 NULL 时返回
  *         NERV_ERR_NONE.
  * @note   读取不清除错误位. 错误位只在任务上下文维护, 无需临界区.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
uint32_t NERV_Motor_ReadError(const NERV_Motor_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERR_NONE;
    }

    return self->error & NERV_ERR_BITS_MASK;
}

/**
  * @brief  清除电机错误位.
  * @param  self 实例指针, 可以为 NULL.
  * @param  mask 待清除的错误位掩码, 只使用低 24 位.
  * @note   只清除 mask 中置位的错误位. self 为 NULL 时为空操作.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
void NERV_Motor_ClearError(NERV_Motor_InstanceTypeDef *self, uint32_t mask)
{
    if (NULL == self)
    {
        return;
    }

    self->error &= ~(mask & NERV_ERR_BITS_MASK);
}

/**
  * @brief  进入安全状态: 清零控制量目标并复位离线监督计时.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 目标已清零.
  * @retval NERV_ERROR self 为 NULL 或实例未初始化.
  * @note   控制输出由目标派生, 本函数不保存独立输出值. 零指令随下一个
  *         NERV_Motor_UpdateGroup 发送, 之间电调保持最后收到的指令. 需要立即输出时,
  *         调用本函数后立即执行一次 NERV_Motor_UpdateGroup.
  * @note   本函数不直接切断电调供电, 也不依赖总线发送成功. 离线监督在下一个
  *         NERV_Motor_Update 或 NERV_Motor_UpdateGroup 时重新建立计时起点.
  * @note   NOT ISR-safe.
  * @note   caller-synchronized.
  */
NERV_StatusTypeDef NERV_Motor_EnterSafeState(NERV_Motor_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    self->target_control.kind     = NERV_MOTOR_CONTROL_SCALAR;
    self->target_control.u.scalar = 0.0f;
    self->offline_armed           = false;

    return NERV_OK;
}

/**
  * @brief  使能电机.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 已下发.
  * @retval NERV_ERROR 实例为 NULL, 未初始化, 或驱动不支持该能力.
  * @note   驱动不支持该能力时返回 NERV_ERROR, 上层可统一调用并检查返回值.
  */
NERV_StatusTypeDef NERV_Motor_Enable(NERV_Motor_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->config.driver) || (NULL == self->config.driver->ops) ||
        (NULL == self->config.driver->ops->enable))
    {
        return NERV_ERROR;
    }

    return self->config.driver->ops->enable(self->config.bus, self->config.driver->model,
                                            self->config.device_id);
}

/**
  * @brief  失能电机.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 已下发.
  * @retval NERV_ERROR 实例为 NULL, 未初始化, 或驱动不支持该能力.
  */
NERV_StatusTypeDef NERV_Motor_Disable(NERV_Motor_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->config.driver) || (NULL == self->config.driver->ops) ||
        (NULL == self->config.driver->ops->disable))
    {
        return NERV_ERROR;
    }

    return self->config.driver->ops->disable(self->config.bus, self->config.driver->model,
                                             self->config.device_id);
}

/**
  * @brief  清除电机故障.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 已下发.
  * @retval NERV_ERROR 实例为 NULL, 未初始化, 或驱动不支持该能力.
  */
NERV_StatusTypeDef NERV_Motor_ClearFault(NERV_Motor_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->config.driver) || (NULL == self->config.driver->ops) ||
        (NULL == self->config.driver->ops->clear_fault))
    {
        return NERV_ERROR;
    }

    return self->config.driver->ops->clear_fault(self->config.bus, self->config.driver->model,
                                                 self->config.device_id);
}

/**
  * @brief  运行期切换控制模式.
  * @param  self 实例指针, 不得为 NULL.
  * @param  mode 目标控制模式, 取 NERV_Motor_ControlKindTypeDef.
  * @retval NERV_OK 已下发.
  * @retval NERV_ERROR 实例为 NULL, 未初始化, 或驱动不支持该能力.
  * @warning 达妙等驱动会把模式写入参数区并掉电保存. 参数区擦写寿命有限, 每个
  *          控制周期都调用会在数分钟内耗尽寿命并损坏电机. 只能在初始化或模式变更
  *          时调用一次.
  */
NERV_StatusTypeDef NERV_Motor_SetMode(NERV_Motor_InstanceTypeDef *self,
                                      NERV_Motor_ControlKindTypeDef mode)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->config.driver) || (NULL == self->config.driver->ops) ||
        (NULL == self->config.driver->ops->set_mode))
    {
        return NERV_ERROR;
    }

    return self->config.driver->ops->set_mode(self->config.bus, self->config.driver->model,
                                              self->config.device_id, mode);
}

/**
  * @brief  保存当前位置为零点.
  * @param  self 实例指针, 不得为 NULL.
  * @retval NERV_OK 已下发.
  * @retval NERV_ERROR 实例为 NULL, 未初始化, 或驱动不支持该能力.
  * @warning 写入参数区并掉电保存. 参数区擦写寿命有限, 每个控制周期都调用会在数分钟内
  *          耗尽寿命并损坏电机. 只能在标定零点时调用一次.
  */
NERV_StatusTypeDef NERV_Motor_SaveZero(NERV_Motor_InstanceTypeDef *self)
{
    if (NULL == self)
    {
        return NERV_ERROR;
    }

    if (!self->initialized)
    {
        return NERV_ERROR;
    }

    if ((NULL == self->config.driver) || (NULL == self->config.driver->ops) ||
        (NULL == self->config.driver->ops->save_zero))
    {
        return NERV_ERROR;
    }

    return self->config.driver->ops->save_zero(self->config.bus, self->config.driver->model,
                                               self->config.device_id);
}

/**
  * @brief  复位实例的软件状态, 不清除配置.
  * @param  self 实例指针, 由调用方保证不为 NULL.
  */
static void NERV_Motor_ResetState(NERV_Motor_InstanceTypeDef *self)
{
    self->fb.angle_rad              = 0.0f;
    self->fb.speed_radps            = 0.0f;
    self->fb.current_a              = 0.0f;
    self->fb.torque_n_m             = 0.0f;
    self->fb.temperature_c          = 0;
    self->fb.temperature_mos_c      = INT8_MIN;
    self->fb.error_code             = 0U;
    self->measurement.angle_rad     = 0.0f;
    self->measurement.speed_radps   = 0.0f;
    self->measurement.torque_n_m    = 0.0f;
    self->measurement.temperature_c = 0;
    self->geo.tx_id                 = 0U;
    self->geo.rx_id                 = 0U;
    self->geo.len                   = 0U;
    self->error                     = 0U;
    self->target_control.kind       = NERV_MOTOR_CONTROL_SCALAR;
    self->target_control.u.scalar   = 0.0f;
    self->last_rx_ms                = 0U;
    self->offline_armed             = false;
    self->initialized               = false;
    self->fb_fresh                  = false;
}

/**
  * @brief  校验型号描述符内部一致性.
  * @param  model 型号描述符, 不得为 NULL.
  * @retval NERV_OK 描述符自洽.
  * @retval NERV_ERROR 设备 id 范围与分组容量矛盾, 帧长非法, 反馈字段偏移越界,
  *         角度编码数为 0, 或控制量与反馈量的量程非正或非有限.
  * @note   描述符由驱动提供. 数据错误时 get_frame_geo 与编解码的行为不可预期
  *         (设备 id 落到同一槽位互相覆盖, 或反馈字段在帧有效数据外读取), 因此在
  *         Init 阶段集中拦截, 不把错误描述符带入运行期.
  * @note   设备 id 范围不得超过两组的槽位总数, 即 id_max 减 id_min 必须小于
  *         两倍的 motors_per_frame.
  */
static NERV_StatusTypeDef NERV_Motor_ValidateModel(const NERV_Motor_ModelTypeDef *model)
{
    if (NULL == model)
    {
        return NERV_ERROR;
    }

    if ((model->id_min < 1U) || (model->id_max < model->id_min) || (model->motors_per_frame < 1U))
    {
        return NERV_ERROR;
    }

    if (model->tx_id_offset_by_device)
    {
        /* 控制帧 ID 直接等于基值加设备 id, 必须落在 11 位标准帧范围内. */
        if ((model->tx_id_low + (uint32_t)model->id_max) > NERV_FDCAN_ID_STANDARD_MAX)
        {
            return NERV_ERROR;
        }
    }
    else if (((uint32_t)model->id_max - (uint32_t)model->id_min) >=
             (2U * (uint32_t)model->motors_per_frame))
    {
        return NERV_ERROR;
    }

    if ((model->frame_len < 2U) || (model->frame_len > NERV_FDCAN_DATA_MAX))
    {
        return NERV_ERROR;
    }

    if (!model->fb_bit_packed)
    {
        if ((((uint16_t)model->fb_angle_off + 2U) > (uint16_t)model->frame_len) ||
            (((uint16_t)model->fb_speed_off + 2U) > (uint16_t)model->frame_len) ||
            (((uint16_t)model->fb_current_off + 2U) > (uint16_t)model->frame_len))
        {
            return NERV_ERROR;
        }

        if ((model->fb_temperature_off != NERV_MOTOR_FB_OFF_NONE) &&
            (((uint16_t)model->fb_temperature_off + 1U) > (uint16_t)model->frame_len))
        {
            return NERV_ERROR;
        }
    }

    if (0U == model->angle_count)
    {
        return NERV_ERROR;
    }

    if (!isfinite(model->rated_gear_ratio) || (model->rated_gear_ratio <= 0.0f))
    {
        return NERV_ERROR;
    }

    if (!model->fb_bit_packed)
    {
        if (!isfinite(model->fb_control_full_scale) || (model->fb_control_full_scale <= 0.0f) ||
            (model->fb_control_cmd_limit <= 0))
        {
            return NERV_ERROR;
        }
    }

    if (model->control_mode > NERV_MOTOR_CONTROL_POSITION_VELOCITY)
    {
        return NERV_ERROR;
    }

    if (NERV_MOTOR_CONTROL_SCALAR == model->control_mode)
    {
        /* 标量模式驱动的量程字段位于 control.scalar 或 control.profile,
           两类驱动各自使用其中一套. */
        const bool scalar_ok  = isfinite(model->control.scalar.full_scale) &&
                                (model->control.scalar.full_scale > 1.0e-3f) &&
                                (model->control.scalar.cmd_limit > 0);
        const bool profile_ok = isfinite(model->control.profile.v_max_radps) &&
                                (model->control.profile.v_max_radps > 1.0e-3f);
        if (!scalar_ok && !profile_ok)
        {
            return NERV_ERROR;
        }
    }
    else
    {
        if (!isfinite(model->control.profile.p_max_rad) ||
            (model->control.profile.p_max_rad <= 0.0f) ||
            !isfinite(model->control.profile.v_max_radps) ||
            (model->control.profile.v_max_radps <= 0.0f))
        {
            return NERV_ERROR;
        }
    }

    if (NERV_MOTOR_CONTROL_MIT == model->control_mode)
    {
        if (!isfinite(model->control.profile.t_max_n_m) ||
            (model->control.profile.t_max_n_m <= 0.0f) ||
            !isfinite(model->control.profile.kp_max) || (model->control.profile.kp_max <= 0.0f) ||
            !isfinite(model->control.profile.kd_max) || (model->control.profile.kd_max <= 0.0f))
        {
            return NERV_ERROR;
        }
    }

    return NERV_OK;
}

/**
  * @brief  给出安装方向对应的符号.
  * @param  self 实例指针, 由调用方保证不为 NULL.
  * @return self->config.invert_direction 为 true 时返回 -1.0f, 否则返回 1.0f.
  */
static float NERV_Motor_InvertSign(const NERV_Motor_InstanceTypeDef *self)
{
    return self->config.invert_direction ? __NERV_MOTOR_DIRECTION_SIGN_INVERTED : 1.0f;
}

/**
  * @brief  把转子量反馈换算为机构系测量值.
  * @param  self 实例指针, 必须已通过 NERV_Motor_Init 校验, 不得为 NULL.
  * @retval NERV_OK 换算完成, self->measurement 已写入.
  * @retval NERV_ERROR self 为 NULL.
  * @note   角度与角速度除以 gear_ratio, 转矩按型号换算, 全部乘安装方向符号.
  *         角度结果归回 [0, 2π).
  */
static NERV_StatusTypeDef NERV_Motor_ConvertFb(NERV_Motor_InstanceTypeDef *self)
{
    if ((NULL == self) || (!self->initialized))
    {
        return NERV_ERROR;
    }

    const NERV_Motor_ModelTypeDef *model = self->config.driver->model;
    const float gear_ratio               = self->config.gear_ratio;
    const float sign                     = NERV_Motor_InvertSign(self);

    self->measurement.angle_rad =
        NERV_Math_Wrap(self->fb.angle_rad / gear_ratio * sign, 0.0f, NERV_MATH_TWO_PI);
    self->measurement.speed_radps = self->fb.speed_radps / gear_ratio * sign;

    if (model->fb_torque_present)
    {
        self->measurement.torque_n_m = self->fb.torque_n_m * gear_ratio * sign;
    }
    else
    {
        self->measurement.torque_n_m = self->fb.current_a * model->torque_constant_n_m_per_a *
                                       gear_ratio / model->rated_gear_ratio * sign;
    }

    self->measurement.temperature_c = self->fb.temperature_c;

    return NERV_OK;
}

/**
  * @brief  给出本周期应发送的控制目标.
  * @param  self 实例指针, 由调用方保证已初始化且不为 NULL.
  * @return 已按安装方向取反并按 control_limit 限幅的控制目标.
  * @note   方向取反只作用于物理量分支: SCALAR 的 scalar, MIT 的 position,
  *         velocity 与 torque, VELOCITY 的 velocity_radps; kp 与 kd 不取反.
  * @note   control_limit 只约束 SCALAR 分支的编码, 与型号描述符的
  *         control.scalar.cmd_limit 同量纲 (如 16384, 25000). MIT 的范围由
  *         control.profile 的 p_max_rad, v_max_radps, t_max_n_m 约束.
  * @note   control_limit 为 NERV_DISABLE 时跳过实例限幅, 协议满量程限幅由驱动完成.
  */
static NERV_Motor_ControlTypeDef NERV_Motor_ActiveControl(const NERV_Motor_InstanceTypeDef *self)
{
    NERV_Motor_ControlTypeDef control;
    float sign;

    control = self->target_control;
    sign    = NERV_Motor_InvertSign(self);

    switch (control.kind)
    {
    case NERV_MOTOR_CONTROL_SCALAR:
        control.u.scalar *= sign;

        if (__NERV_MOTOR_CONFIG_ACTIVE_U(self->config.control_limit))
        {
            control.u.scalar = NERV_Math_Clamp(control.u.scalar, -(float)self->config.control_limit,
                                               (float)self->config.control_limit);
        }
        break;

    case NERV_MOTOR_CONTROL_POSITION_VELOCITY:
        control.u.position_velocity.position_rad *= sign;
        control.u.position_velocity.velocity_radps *= sign;
        break;

    case NERV_MOTOR_CONTROL_MIT:
        control.u.mit.position_rad *= sign;
        control.u.mit.velocity_radps *= sign;
        control.u.mit.torque_n_m *= sign;
        break;

    default:
        control.kind     = NERV_MOTOR_CONTROL_SCALAR;
        control.u.scalar = 0.0f;
        break;
    }

    return control;
}

/**
  * @brief  把本实例的控制量编码到控制帧内该实例对应的位置.
  * @param  self 实例指针, 必须已初始化, 由调用方保证不为 NULL.
  * @param  frame 控制帧数据缓冲, 长度不小于实例的 frame_len, 调用方负责清零.
  * @retval NERV_OK 编码成功.
  * @retval NERV_ERROR 驱动编码失败.
  */
static NERV_StatusTypeDef NERV_Motor_EncodeInto(const NERV_Motor_InstanceTypeDef *self,
                                                uint8_t *frame)
{
    NERV_Motor_ControlTypeDef control;

    control = NERV_Motor_ActiveControl(self);

    return self->config.driver->ops->encode_tx(self->config.driver->model, frame,
                                               self->config.device_id, &control);
}

/**
  * @brief  维护离线与过温错误位.
  * @param  self 实例指针, 必须已初始化, 由调用方保证不为 NULL.
  * @param  now_ms 当前时刻, 单位 ms.
  * @note   fb_fresh 置位时刷新 last_rx_ms 并清除该标志. 首次监督以 now_ms 建立计时
  *         起点, 避免实例初始化后尚未收到反馈就立即判定离线. 之后按无符号回绕减法
  *         计算间隔, 达到 NERV_UTILS_TIMEOUT_MS(offline_ms) 时置离线位. offline_ms
  *         为 NERV_DISABLE 时跳过离线判定.
  * @note   now_ms 小于 last_rx_ms (调用方时钟回跳) 时同样重建计时基准, 不用回跳产生
  *         的巨大无符号差判定离线.
  * @note   过温判定以 over_temperature_c 大于 0 为前提, 读 measurement 的温度字段,
  *         因此本函数必须在 NERV_Motor_ConvertFb 之后调用.
  * @note   错误位为粘滞, 只由 NERV_Motor_ClearError 清除.
  */
static void NERV_Motor_UpdateHealth(NERV_Motor_InstanceTypeDef *self, uint32_t now_ms)
{
    if (self->fb_fresh)
    {
        self->fb_fresh      = false;
        self->last_rx_ms    = now_ms;
        self->offline_armed = true;
    }
    else if ((!self->offline_armed) || (now_ms < self->last_rx_ms))
    {
        /* 首次监督或调用方时钟回跳时重建计时基准, 不用回跳后的巨大无符号差判定离线. */
        self->last_rx_ms    = now_ms;
        self->offline_armed = true;
    }
    else if (__NERV_MOTOR_CONFIG_ACTIVE_U(self->config.offline_ms) &&
             ((now_ms - self->last_rx_ms) >= NERV_UTILS_TIMEOUT_MS(self->config.offline_ms)))
    {
        self->error |= (NERV_ERR_MOTOR_OFFLINE & NERV_ERR_BITS_MASK);
    }

    if (__NERV_MOTOR_CONFIG_ACTIVE_F(self->config.over_temperature_c) &&
        ((float)self->measurement.temperature_c >= self->config.over_temperature_c))
    {
        self->error |= (NERV_ERR_MOTOR_OVER_TEMP & NERV_ERR_BITS_MASK);
    }
}

/**
  * @brief  判断两个实例是否属于同一发送组.
  * @param  lhs 第一个实例, 不得为 NULL.
  * @param  rhs 第二个实例, 不得为 NULL.
  * @return 总线实例与指令帧标识符均相同时返回 true, 否则返回 false.
  * @note   同一标识符的控制帧只能由一条总线发出, 因此总线也是分组键.
  */
static bool NERV_Motor_SameGroup(const NERV_Motor_InstanceTypeDef *lhs,
                                 const NERV_Motor_InstanceTypeDef *rhs)
{
    return (lhs->config.bus == rhs->config.bus) && (lhs->geo.tx_id == rhs->geo.tx_id);
}

/**
  * @brief  把指令发送失败标志置位到某组全部成员.
  * @param  members 实例指针数组, 元素由调用方保证不为 NULL.
  * @param  count 数组元素个数.
  * @param  first 该组首个成员在数组中的下标.
  * @note   同一组的成员共享一条指令帧, 帧发不出去时整组都有责任, 一起置位以便调用方
  *         经 NERV_Motor_ReadError 定位到具体是哪些电机的组发送失败.
  */
static void NERV_Motor_SetGroupTxFail(NERV_Motor_InstanceTypeDef *const *members, uint8_t count,
                                      uint8_t first)
{
    for (uint8_t index = 0U; index < count; index++)
    {
        if (NERV_Motor_SameGroup(members[index], members[first]))
        {
            members[index]->error |= (NERV_ERR_MOTOR_TX_FAIL & NERV_ERR_BITS_MASK);
        }
    }
}

/**
  * @brief  清除某组全部成员的指令发送失败标志.
  * @param  members 实例指针数组, 元素由调用方保证不为 NULL.
  * @param  count 数组元素个数.
  * @param  first 该组首个成员在数组中的下标.
  */
static void NERV_Motor_ClearGroupTxFail(NERV_Motor_InstanceTypeDef *const *members, uint8_t count,
                                        uint8_t first)
{
    for (uint8_t index = 0U; index < count; index++)
    {
        if (NERV_Motor_SameGroup(members[index], members[first]))
        {
            members[index]->error &= ~(NERV_ERR_MOTOR_TX_FAIL & NERV_ERR_BITS_MASK);
        }
    }
}

/**
  * @brief  校验同一发送组内成员的帧长与设备 id.
  * @param  members 实例指针数组, 元素由调用方保证不为 NULL.
  * @param  count 数组元素个数.
  * @retval NERV_OK 组内一致.
  * @retval NERV_ERROR 同一总线与指令帧标识符的成员帧长不一致或设备 id 重复.
  * @note   不同组之间允许帧长与设备 id 相同.
  */
static NERV_StatusTypeDef NERV_Motor_ValidateGroups(NERV_Motor_InstanceTypeDef *const *members,
                                                    uint8_t count)
{
    for (uint8_t first = 0U; first < count; first++)
    {
        for (uint8_t second = (uint8_t)(first + 1U); second < count; second++)
        {
            if (!NERV_Motor_SameGroup(members[first], members[second]))
            {
                continue;
            }

            if (members[first]->geo.len != members[second]->geo.len)
            {
                return NERV_ERROR;
            }

            if (members[first]->config.device_id == members[second]->config.device_id)
            {
                return NERV_ERROR;
            }
        }
    }

    return NERV_OK;
}

/**
  * @brief  为一个发送组组帧并发送.
  * @param  members 实例指针数组, 元素由调用方保证不为 NULL 且已初始化.
  * @param  count 数组元素个数.
  * @param  first 该组首个成员在数组中的下标.
  * @retval NERV_OK 帧已交给 FDCAN 发送.
  * @retval NERV_BUSY 没有空闲发送邮箱.
  * @retval NERV_ERROR 帧长超过 NERV_FDCAN_DATA_MAX 或编码失败.
  * @note   帧标识符与帧长取 members[first] 的值, 数据区按帧长清零, 只编码同一总线
  *         与标识符成员的槽位, 其余槽位为 0. 经 members[first] 的 bus 发送.
  */
static NERV_StatusTypeDef NERV_Motor_TransmitGroupFrame(NERV_Motor_InstanceTypeDef *const *members,
                                                        uint8_t count, uint8_t first)
{
    NERV_FDCAN_FrameTypeDef frame;

    if (members[first]->geo.len > NERV_FDCAN_DATA_MAX)
    {
        return NERV_ERROR;
    }

    frame.id    = members[first]->geo.tx_id;
    frame.len   = members[first]->geo.len;
    frame.flags = (uint8_t)NERV_FDCAN_FLAG_NONE;

    for (uint8_t slot = 0U; slot < frame.len; slot++)
    {
        frame.data[slot] = 0U;
    }

    for (uint8_t index = 0U; index < count; index++)
    {
        if (!NERV_Motor_SameGroup(members[index], members[first]))
        {
            continue;
        }

        if (NERV_OK != NERV_Motor_EncodeInto(members[index], frame.data))
        {
            return NERV_ERROR;
        }
    }

    return NERV_FDCAN_Transmit(members[first]->config.bus, &frame);
}

#endif /* NERV_USE_MOTOR */
