/**
  * @file    nerv_chassis_def.h
  * @brief   NERV 底盘编译期参数与派生宏.
  * @version 1.0.0
  * @date    2026-10-24
  * @copyright Copyright (c) 2026 NERV Contributors
  *
  * SPDX-License-Identifier: MIT
  */

#ifndef NERV_CHASSIS_DEF_H
#define NERV_CHASSIS_DEF_H

#include "config.h"

#if (NERV_USE_CHASSIS == NERV_ENABLE)

#include "nerv_math.h"

/* 轮数由构型推导, 使用者不得覆盖. */

#if defined(NERV_CHASSIS_WHEEL_COUNT)
#error "NERV_CHASSIS_WHEEL_COUNT is derived and must not be defined by the user."
#endif

#if defined(NERV_CHASSIS_MECANUM_WHEEL_COUNT)
#error "NERV_CHASSIS_MECANUM_WHEEL_COUNT is derived and must not be defined by the user."
#endif

#if defined(NERV_CHASSIS_OMNI_WHEEL_COUNT)
#error "NERV_CHASSIS_OMNI_WHEEL_COUNT is derived and must not be defined by the user."
#endif

#if defined(NERV_CHASSIS_STEERING_WHEEL_COUNT)
#error "NERV_CHASSIS_STEERING_WHEEL_COUNT is derived and must not be defined by the user."
#endif

/**
  * @brief 麦克纳姆轮数量.
  */
#define NERV_CHASSIS_MECANUM_WHEEL_COUNT 4U

/**
  * @brief 全向轮数量.
  */
#define NERV_CHASSIS_OMNI_WHEEL_COUNT 4U

/**
  * @brief 舵轮数量.
  */
#define NERV_CHASSIS_STEERING_WHEEL_COUNT 4U

/**
  * @brief 当前构型的车轮数量.
  * @note  由 NERV_CHASSIS_TYPE 推导, 使用者不得覆盖.
  */
#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)
#define NERV_CHASSIS_WHEEL_COUNT NERV_CHASSIS_MECANUM_WHEEL_COUNT
#elif (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)
#define NERV_CHASSIS_WHEEL_COUNT NERV_CHASSIS_OMNI_WHEEL_COUNT
#elif (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_STEERING)
#define NERV_CHASSIS_WHEEL_COUNT NERV_CHASSIS_STEERING_WHEEL_COUNT
#else
#error "Unsupported NERV_CHASSIS_TYPE."
#endif

/* 所有构型共用的硬件参数.
 * 下列宏的默认值 0.0f 是占位值. 使用者必须在 config.h 中定义实际值.
 * nerv_chassis.c 中的静态断言拒绝占位值, 避免用错误几何参数运行. */

/**
  * @brief 车轮半径, 单位 m.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_WHEEL_RADIUS_M
#define NERV_CHASSIS_WHEEL_RADIUS_M 0.0f
#endif

/**
  * @brief 驱动电机转子到车轮输出轴的减速比.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_DRIVE_GEAR_RATIO
#define NERV_CHASSIS_DRIVE_GEAR_RATIO 0.0f
#endif

/**
  * @brief 车轮线速度限幅, 单位 m/s.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_MAX_WHEEL_SPEED_MPS
#define NERV_CHASSIS_MAX_WHEEL_SPEED_MPS 0.0f
#endif

#if defined(NERV_CHASSIS_HALF_FACTOR)
#error "NERV_CHASSIS_HALF_FACTOR is a library constant and must not be defined by the user."
#endif

/**
  * @brief 二分之一系数, 用于由全长推导半长.
  */
#define NERV_CHASSIS_HALF_FACTOR 0.5f

/* 麦克纳姆轮几何参数. 按 X 形排列, 轮序为左前, 右前, 左后, 右后. */

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_MECANUM)

/**
  * @brief 前后轮轴距, 单位 m.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_MECANUM_WHEELBASE_M
#define NERV_CHASSIS_MECANUM_WHEELBASE_M 0.0f
#endif

/**
  * @brief 左右轮距, 单位 m.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_MECANUM_TRACK_M
#define NERV_CHASSIS_MECANUM_TRACK_M 0.0f
#endif

#if defined(NERV_CHASSIS_MECANUM_HALF_WHEELBASE_M)
#error "NERV_CHASSIS_MECANUM_HALF_WHEELBASE_M is derived and must not be defined by the user."
#endif

#if defined(NERV_CHASSIS_MECANUM_HALF_TRACK_M)
#error "NERV_CHASSIS_MECANUM_HALF_TRACK_M is derived and must not be defined by the user."
#endif

#if defined(NERV_CHASSIS_MECANUM_ROTATION_GAIN_M)
#error "NERV_CHASSIS_MECANUM_ROTATION_GAIN_M is derived and must not be defined by the user."
#endif

/**
  * @brief 轴距的一半, 单位 m.
  */
#define NERV_CHASSIS_MECANUM_HALF_WHEELBASE_M \
    (NERV_CHASSIS_MECANUM_WHEELBASE_M * NERV_CHASSIS_HALF_FACTOR)

/**
  * @brief 轮距的一半, 单位 m.
  */
#define NERV_CHASSIS_MECANUM_HALF_TRACK_M (NERV_CHASSIS_MECANUM_TRACK_M * NERV_CHASSIS_HALF_FACTOR)

/**
  * @brief 麦克纳姆轮绕 z 轴旋转的力臂, 单位 m.
  * @note  等于轴距一半与轮距一半之和, 与 X 形排列的解算约定配套.
  */
#define NERV_CHASSIS_MECANUM_ROTATION_GAIN_M \
    (NERV_CHASSIS_MECANUM_HALF_WHEELBASE_M + NERV_CHASSIS_MECANUM_HALF_TRACK_M)

#endif /* NERV_CHASSIS_TYPE_MECANUM */

/* 全向轮几何参数.
 * 轮序为 0, 1, 2, 3. 默认安装时依次为左前, 右前, 左后, 右后.
 * 四个车轮位于以底盘中心为圆心, 半径为 NERV_CHASSIS_OMNI_RADIUS_M 的圆周上,
 * 依次相差 90 度. 默认安装对应正方形底盘的四个角.
 * 驱动方向为逆时针切线方向.
 * NERV_CHASSIS_OMNI_WHEEL_ANGLE_OFFSET_RAD 是轮 1 即右前轮驱动方向的基准角,
 * 逆时针为正. 默认 pi/4, 即 45 度.
 * 车轮接触点位于驱动方向顺时针 90 度的半径上. 该约定与解算中绕 z 轴
 * 角速度项的符号配套. 修改 offset 会整体旋转四个车轮的驱动方向. */

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_OMNI)

/**
  * @brief 底盘中心到车轮接触点的距离, 单位 m.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_OMNI_RADIUS_M
#define NERV_CHASSIS_OMNI_RADIUS_M 0.0f
#endif

/**
  * @brief 右前轮驱动方向的基准角, 单位 rad, 逆时针为正.
  * @note  默认 pi/4, 即 45 度. 该值会整体旋转四个车轮的驱动方向.
  */
#ifndef NERV_CHASSIS_OMNI_WHEEL_ANGLE_OFFSET_RAD
#define NERV_CHASSIS_OMNI_WHEEL_ANGLE_OFFSET_RAD \
    (NERV_MATH_PI / (float)NERV_CHASSIS_OMNI_WHEEL_COUNT)
#endif

#if defined(NERV_CHASSIS_OMNI_WHEEL_ANGLE_0_RAD)
#error "NERV_CHASSIS_OMNI_WHEEL_ANGLE_0_RAD is derived and must not be defined by the user."
#endif

#if defined(NERV_CHASSIS_OMNI_WHEEL_ANGLE_1_RAD)
#error "NERV_CHASSIS_OMNI_WHEEL_ANGLE_1_RAD is derived and must not be defined by the user."
#endif

#if defined(NERV_CHASSIS_OMNI_WHEEL_ANGLE_2_RAD)
#error "NERV_CHASSIS_OMNI_WHEEL_ANGLE_2_RAD is derived and must not be defined by the user."
#endif

#if defined(NERV_CHASSIS_OMNI_WHEEL_ANGLE_3_RAD)
#error "NERV_CHASSIS_OMNI_WHEEL_ANGLE_3_RAD is derived and must not be defined by the user."
#endif

/**
  * @brief 左前轮驱动方向角, 单位 rad.
  */
#define NERV_CHASSIS_OMNI_WHEEL_ANGLE_0_RAD \
    (NERV_CHASSIS_OMNI_WHEEL_ANGLE_OFFSET_RAD + (NERV_MATH_PI * NERV_CHASSIS_HALF_FACTOR))

/**
  * @brief 右前轮驱动方向角, 单位 rad.
  */
#define NERV_CHASSIS_OMNI_WHEEL_ANGLE_1_RAD (NERV_CHASSIS_OMNI_WHEEL_ANGLE_OFFSET_RAD)

/**
  * @brief 左后轮驱动方向角, 单位 rad.
  */
#define NERV_CHASSIS_OMNI_WHEEL_ANGLE_2_RAD \
    (NERV_CHASSIS_OMNI_WHEEL_ANGLE_OFFSET_RAD + NERV_MATH_PI)

/**
  * @brief 右后轮驱动方向角, 单位 rad.
  */
#define NERV_CHASSIS_OMNI_WHEEL_ANGLE_3_RAD                    \
    (NERV_CHASSIS_OMNI_WHEEL_ANGLE_OFFSET_RAD + NERV_MATH_PI + \
     (NERV_MATH_PI * NERV_CHASSIS_HALF_FACTOR))

#endif /* NERV_CHASSIS_TYPE_OMNI */

/* 舵轮几何参数. 轮序为左前, 右前, 左后, 右后. */

#if (NERV_CHASSIS_TYPE == NERV_CHASSIS_TYPE_STEERING)

/**
  * @brief 前后轮轴距, 单位 m.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_STEERING_WHEELBASE_M
#define NERV_CHASSIS_STEERING_WHEELBASE_M 0.0f
#endif

/**
  * @brief 左右轮距, 单位 m.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_STEERING_TRACK_M
#define NERV_CHASSIS_STEERING_TRACK_M 0.0f
#endif

/**
  * @brief 转向角绝对值上限, 单位 rad.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_STEERING_MAX_ANGLE_RAD
#define NERV_CHASSIS_STEERING_MAX_ANGLE_RAD 0.0f
#endif

/**
  * @brief 转向电机转子到车轮转向轴的减速比.
  * @note  未定义时取 0.0f, 该值不能通过 nerv_chassis.c 的静态断言.
  */
#ifndef NERV_CHASSIS_STEER_GEAR_RATIO
#define NERV_CHASSIS_STEER_GEAR_RATIO 0.0f
#endif

#if defined(NERV_CHASSIS_STEERING_HALF_WHEELBASE_M)
#error "NERV_CHASSIS_STEERING_HALF_WHEELBASE_M is derived and must not be defined by the user."
#endif

#if defined(NERV_CHASSIS_STEERING_HALF_TRACK_M)
#error "NERV_CHASSIS_STEERING_HALF_TRACK_M is derived and must not be defined by the user."
#endif

/**
  * @brief 轴距的一半, 单位 m.
  */
#define NERV_CHASSIS_STEERING_HALF_WHEELBASE_M \
    (NERV_CHASSIS_STEERING_WHEELBASE_M * NERV_CHASSIS_HALF_FACTOR)

/**
  * @brief 轮距的一半, 单位 m.
  */
#define NERV_CHASSIS_STEERING_HALF_TRACK_M \
    (NERV_CHASSIS_STEERING_TRACK_M * NERV_CHASSIS_HALF_FACTOR)

#endif /* NERV_CHASSIS_TYPE_STEERING */

#endif /* NERV_USE_CHASSIS */

#endif /* NERV_CHASSIS_DEF_H */
