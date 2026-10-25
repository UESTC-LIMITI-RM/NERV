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
    uint8_t device_id; /**< 电调 id, 合法范围由驱动描述符的 id_min 与 id_max 决定 */
    float gear_ratio;  /**< 减速比, 必须大于 0, 输出轴角速度等于转子角速度除以本值 */
    uint32_t
        offline_ms; /**< 离线判定阈值, 单位 ms, 取大于 0 的有限值启用, NERV_DISABLE 表示关闭离线判定 */
    float
        over_temperature_c; /**< 电机过温阈值, 单位 ℃, 取大于 0 的有限值启用, NERV_DISABLE 表示关闭过温检查 */
    uint16_t
        control_limit; /**< 控制量编码值上限, 量纲与型号描述符的 control_cmd_limit 相同 (如 16384, 25000), 全型号统一按编码值配置, 取 NERV_DISABLE 表示不额外限幅 */
    bool invert_direction; /**< 安装方向取反, 指令与反馈统一乘符号 */
} NERV_Motor_ConfigTypeDef;

/**
  * @brief  电机测量值, 机构系.
  * @note   角度与角速度已按减速比换算到输出轴, 角度, 角速度与电流按安装方向
  *         取反, 温度原值透传, 正方向为机构正方向.
  * @note   角度范围为 [0, 2π), 不做多圈累积. 需要连续位置时由调用方用 NERV_Math_Wrap
  *         计算增量后自行累积.
  * @note   temperature_c 为 INT8_MIN 表示该型号无温度字段, 见 NERV_MOTOR_FB_OFF_NONE.
  */
typedef struct NERV_Motor_Measurement
{
    float angle_rad;      /**< 输出轴机械角度, 单位 rad, 机构正方向为正, 范围 [0, 2π) */
    float speed_radps;    /**< 输出轴角速度, 单位 rad/s, 机构正方向为正 */
    float current;        /**< 实际输出电流, 单位 A, 不论电机型号, 机构正方向为正 */
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
    NERV_Motor_CmdTypeDef target_cmd;          /**< 控制目标, 由 SetTarget 或 SetCmd 写入 */
    uint32_t last_rx_ms;                       /**< 最近一次有效反馈的时刻, 单位 ms */
    bool offline_armed;                        /**< 离线监督已建立计时起点, 由模块维护 */
    bool initialized;                          /**< 初始化完成标志, 由 NERV_Motor_Init 置位 */
    bool fb_fresh;                             /**< 有新反馈待结算, 中断置位, Update 清除 */
} NERV_Motor_InstanceTypeDef;

/* 调用顺序: NERV_Motor_Init 用配置建立实例, 之后周期调用 NERV_Motor_UpdateGroup
 * 完成测量值刷新, 监督与指令发送. 单个实例的测量值刷新与监督可单独调用
 * NERV_Motor_Update. 反馈经 NERV_Motor_OnRx 进入. 完整契约见各函数定义处. */

NERV_StatusTypeDef NERV_Motor_Init(NERV_Motor_InstanceTypeDef *self,
                                   const NERV_Motor_ConfigTypeDef *config);

/**
  * @brief  设置电机目标控制编码.
  * @note   target 为电机控制编码, 量纲同型号描述符的 control_cmd_limit (如 16384, 25000),
  *         与反馈电流等物理量不在同一单位. 调用方若以物理量配置, 可用
  *         NERV_Motor_CmdFromPhysical 先换算为控制编码.
  * @note   NOT ISR-safe. 写实例目标值, 与 Update 同属控制链.
  */
NERV_StatusTypeDef NERV_Motor_SetTarget(NERV_Motor_InstanceTypeDef *self, float target);

/**
  * @brief  设置电机控制目标（多字段）.
  * @note   cmd 按 kind 选择有效分支, 各驱动只支持自己协议的分支, 不支持时发送返回
  *         NERV_ERROR. 只有单一控制量的简单协议可继续用 NERV_Motor_SetTarget.
  * @note   NOT ISR-safe. 写实例目标值, 与 Update 同属控制链.
  */
NERV_StatusTypeDef NERV_Motor_SetCmd(NERV_Motor_InstanceTypeDef *self,
                                     const NERV_Motor_CmdTypeDef *cmd);

/**
  * @brief  刷新测量值, 运行在线监督并积分离线计时.
  * @note   NOT ISR-safe. 写实例测量与错误位, 更新计时基准.
  */
NERV_StatusTypeDef NERV_Motor_Update(NERV_Motor_InstanceTypeDef *self, uint32_t now_ms);

/**
  * @brief  按组刷新并下发一组电机的指令帧.
  * @note   NOT ISR-safe. 遍历实例状态, 经总线发送.
  */
NERV_StatusTypeDef NERV_Motor_UpdateGroup(NERV_Motor_InstanceTypeDef *const *members, uint8_t count,
                                          uint32_t now_ms);

/**
  * @brief  把接收到的反馈帧交给电机实例解析.
  * @note   ISR-safe. 设计为在总线接收中断中调用, 只做拷贝, 置标志与有限的状态更新,
  *         不阻塞, 不分配内存, 不调用日志.
  */
void NERV_Motor_OnRx(NERV_Motor_InstanceTypeDef *self, const NERV_FDCAN_FrameTypeDef *frame);

/**
  * @brief  读取电机机构系测量值.
  * @note   NOT ISR-safe. 测量值由 Update 在任务上下文写入, 不应在中断中读取.
  */
NERV_StatusTypeDef NERV_Motor_GetMeasurement(const NERV_Motor_InstanceTypeDef *self,
                                             NERV_Motor_MeasurementTypeDef *measurement);

/**
  * @brief  读取电机粘滞错误位.
  * @note   NOT ISR-safe. 错误位由任务上下文的 Update 与中断上下文的 OnRx 共同维护,
  *         ISR 读取可能看到不一致状态.
  */
uint32_t NERV_Motor_GetError(const NERV_Motor_InstanceTypeDef *self);

/**
  * @brief  清除电机错误位.
  * @note   NOT ISR-safe. 读改写实例错误位, 与 OnRx 可能竞争.
  */
void NERV_Motor_ClearError(NERV_Motor_InstanceTypeDef *self, uint32_t mask);

/**
  * @brief  进入安全状态, 目标清零.
  * @note   NOT ISR-safe. 写实例状态.
  */
NERV_StatusTypeDef NERV_Motor_EnterSafeState(NERV_Motor_InstanceTypeDef *self);

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_MOTOR */

#endif /* NERV_MOTOR_H */
