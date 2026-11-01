/**
  * @file    nerv_chassis.h
  * @brief   NERV 底盘模块公开接口.
  * @version 2.0.0
  * @date    2026-10-30
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_CHASSIS_H
#define NERV_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

#if (NERV_USE_CHASSIS == NERV_ENABLE)

/* 轮组数量按构型派生, 两种构型均为四轮布局, 禁止使用者覆盖. */
#if defined(NERV_CHASSIS_WHEEL_COUNT)
#error "NERV_CHASSIS_WHEEL_COUNT is derived and must not be defined by the user."
#endif

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)
#define NERV_CHASSIS_WHEEL_COUNT 4U
#elif (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)
#define NERV_CHASSIS_WHEEL_COUNT 4U
#else
#error "NERV_CHASSIS_TYPE must select NERV_CHASSIS_TYPE_MECANUM or NERV_CHASSIS_TYPE_OMNI."
#endif

#include "nerv_motor.h"
#include "nerv_pid.h"
#include "nerv_status.h"
#include "nerv_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  底盘构型枚举镜像.
  * @note   成员数值必须与 NERV_CHASSIS_TYPE_MECANUM 与 NERV_CHASSIS_TYPE_OMNI 一致,
  *         用于运行期类型检查与调试器可视化.
  */
typedef enum
{
    NERV_CHASSIS_TYPE_MECANUM_RT = 1U, /**< 麦克纳姆轮底盘 */
    NERV_CHASSIS_TYPE_OMNI_RT    = 2U, /**< 全向轮底盘 */
} NERV_Chassis_KindTypeDef;

/**
  * @brief  机体系速度.
  * @note   底盘接口只接受机体系速度. 遥控器给出的云台系指令由调用方转换, 转换用
  *         NERV_Math_RotateZ 绕 z 轴旋转, 角度为云台航向减底盘航向, 两个航向必须在
  *         同一参考系下测量. 绕 z 轴的角速度不随参考系改变, 转换时原样传递.
  */
typedef struct NERV_Chassis_Velocity
{
    float vx_mps;   /**< 车体系 x 轴线速度, 单位 m/s, 向前为正 */
    float vy_mps;   /**< 车体系 y 轴线速度, 单位 m/s, 向左为正 */
    float wz_radps; /**< 车体系 z 轴角速度, 单位 rad/s, 逆时针为正 */
} NERV_Chassis_VelocityTypeDef;

/**
  * @brief  里程计.
  * @note   位置在参考系中累积, 原点与轴向由调用方定义, 库不定义.
  * @note   底盘只用姿态的 yaw_rad 表达航向, pitch_rad 与 roll_rad 恒为 0.
  */
typedef struct NERV_Chassis_Odom
{
    float x_m;                     /**< 参考系 x 位置, 单位 m, 向前为正 */
    float y_m;                     /**< 参考系 y 位置, 单位 m, 向左为正 */
    NERV_AttitudeTypeDef attitude; /**< 姿态, yaw_rad 为相对参考系的航向 */
} NERV_Chassis_OdomTypeDef;

/**
  * @brief  底盘故障回调.
  * @param  user 注册时提供的上下文指针, 由调用者保证生命周期.
  * @param  error 当前的粘滞错误位掩码, 取值同 NERV_Chassis_ReadError 的返回值.
  * @note   由 NERV_Chassis_Update 在任务上下文调用. 回调不得阻塞, 不得递归调用
  *         底盘接口.
  */
typedef void (*NERV_Chassis_FaultCallbackTypeDef)(void *user, uint32_t error);

/**
  * @brief  底盘实例配置.
  * @note   由调用方以静态存储提供, 生命周期必须覆盖实例生命周期.
  * @note   电机实例指针指向的对象由调用方提供, 绑定后只读. 全部轮向电机必须已通过
  *         NERV_Motor_Init 初始化, 减速比与安装方向在电机实例的配置中表达.
  */
typedef struct
{
    NERV_Motor_InstanceTypeDef
        *drive_motors[NERV_CHASSIS_WHEEL_COUNT]; /**< 轮向电机实例, 按轮号 0 到 3 排列 */
    float wheel_radius_m;                        /**< 轮半径, 单位 m, 必须大于 0 */
    float center_offset_x_m;        /**< 轮组中心到底盘中心的 x 投影距, 单位 m, 必须大于 0 */
    float center_offset_y_m;        /**< 轮组中心到底盘中心的 y 投影距, 单位 m, 必须大于 0 */
    float max_translate_accel_mps2; /**< 平移加速度限幅, 单位 m/s², 0 表示不额外限幅 */
    float max_rotate_accel_radps2;  /**< 旋转加速度限幅, 单位 rad/s², 0 表示不额外限幅 */
    const NERV_PID_ConfigTypeDef
        *translate_pid; /**< 平移速度环 PID 配置指针, x轴和y轴共用, 输出为速度修正量,
                             单位 m/s, 生命周期须覆盖实例 */
    const NERV_PID_ConfigTypeDef
        *rotate_pid; /**< 旋转速度环 PID 配置指针, 输出为角速度修正量, 单位 rad/s,
                          生命周期须覆盖实例 */
    const NERV_PID_ConfigTypeDef
        *wheel_pid; /**< 轮速环 PID 配置指针, 每轮一份, 输出为电流, 单位 A, 增益与
                         限幅按安培域配置, 生命周期须覆盖实例 */
} NERV_Chassis_ConfigTypeDef;

/**
  * @brief  底盘实例.
  * @note   由调用方以静态存储提供并清零, 生命周期覆盖全部调用.
  * @note   速度环的积分量与上次偏差由模块维护, 调用方只读, 不得写入.
  * @note   config 之后的字段由模块维护, 调用方只读, 不得写入.
  */
typedef struct NERV_Chassis_Instance
{
    NERV_Chassis_ConfigTypeDef config; /**< 配置, 由 NERV_Chassis_Init 绑定, 之后只读 */

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)

    float center_distance_m; /**< 全向轮等效力臂, 单位 m, 由 NERV_Chassis_Init 预计算 */

#endif

    NERV_Chassis_VelocityTypeDef
        target_velocity; /**< 目标机体系速度, 由 NERV_Chassis_SetVelocity 写入 */
    NERV_Chassis_VelocityTypeDef
        planned_velocity;                      /**< 斜坡限幅后的规划速度, 由模块维护, 调用方只读 */
    NERV_PID_InstanceTypeDef translate_pid[2]; /**< 平移速度环 PID, 索引 0 为 x 轴, 1 为 y 轴 */
    NERV_PID_InstanceTypeDef rotate_pid;       /**< 旋转速度环 PID */
    NERV_PID_InstanceTypeDef wheel_speed_pid[NERV_CHASSIS_WHEEL_COUNT]; /**< 各轮速度环 PID */

#if (NERV_USE_CHASSIS_ODOMETRY == NERV_ENABLE)

    NERV_Chassis_OdomTypeDef odom; /**< 里程计, 由 NERV_Chassis_Update 刷新 */

#endif /* NERV_USE_CHASSIS_ODOMETRY */

    uint32_t error;          /**< 粘滞错误位, 经 NERV_Chassis_ReadError 读取 */
    uint32_t last_update_ms; /**< 上次 Update 的时刻, 单位 ms, 供里程计积分求间隔 */
    bool initialized;        /**< 初始化完成标志, 由 NERV_Chassis_Init 置位 */
    NERV_Chassis_FaultCallbackTypeDef fault_callback; /**< 故障回调, NULL 表示未注册 */
    void *fault_user;                                 /**< 故障回调上下文, 可以为 NULL */
} NERV_Chassis_InstanceTypeDef;

/* 调用顺序: NERV_Chassis_Init 用配置建立实例, 之后周期调用 NERV_Chassis_Update.
 * 目标速度经 NERV_Chassis_SetVelocity 设置. 完整契约见各函数定义处. */

NERV_StatusTypeDef NERV_Chassis_Init(NERV_Chassis_InstanceTypeDef *self,
                                     const NERV_Chassis_ConfigTypeDef *config);

NERV_StatusTypeDef NERV_Chassis_RegisterFaultCallback(NERV_Chassis_InstanceTypeDef *self,
                                                      NERV_Chassis_FaultCallbackTypeDef callback,
                                                      void *user);

NERV_StatusTypeDef NERV_Chassis_SetVelocity(NERV_Chassis_InstanceTypeDef *self,
                                            const NERV_Chassis_VelocityTypeDef *target);

NERV_StatusTypeDef NERV_Chassis_Update(NERV_Chassis_InstanceTypeDef *self, uint32_t now_ms);

#if (NERV_USE_CHASSIS_ODOMETRY == NERV_ENABLE)

NERV_StatusTypeDef NERV_Chassis_GetOdometry(const NERV_Chassis_InstanceTypeDef *self,
                                            NERV_Chassis_OdomTypeDef *odom);

NERV_StatusTypeDef NERV_Chassis_SetOdometry(NERV_Chassis_InstanceTypeDef *self,
                                            const NERV_Chassis_OdomTypeDef *odom);

#endif /* NERV_USE_CHASSIS_ODOMETRY */

uint32_t NERV_Chassis_ReadError(const NERV_Chassis_InstanceTypeDef *self);

void NERV_Chassis_ClearError(NERV_Chassis_InstanceTypeDef *self, uint32_t mask);

NERV_StatusTypeDef NERV_Chassis_EnterSafeState(NERV_Chassis_InstanceTypeDef *self);

/* 底盘布局与编号约定 (麦克纳姆轮与全向轮均为 4 轮):
 *
 * 车体系坐标系 x 指向车体前方, y 指向左方, z 指向上方. 轮号 0 到 3 依次为左前,
 * 左后, 右后, 右前, 沿底盘周向排列, 位置由配置的 center_offset_x_m 与
 * center_offset_y_m 给出轮到中心的投影距.
 *
 * 各轮驱动方向与 x 轴夹角依次为 135°, 225°, 315°, 45°, 具体物理朝向由轮向电机的
 * 安装转向吸收, 对应电机实例配置的 invert_direction, 不影响本表.
 *
 * 控制框架为串级速控: 各轮测量角速度经运动学正解得到当前机体系速度; 车体速度环把
 * 规划速度与实测速度之差修正为机体系目标速度; 运动学逆解把目标速度换算为各轮目标
 * 角速度; 每轮速度环把轮速误差换算为电流下发轮向电机. */

#ifdef __cplusplus
}
#endif

#endif /* NERV_USE_CHASSIS */

#endif /* NERV_CHASSIS_H */
