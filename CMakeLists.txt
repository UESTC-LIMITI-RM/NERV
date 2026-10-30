cmake_minimum_required(VERSION 3.20)

project(nerv LANGUAGES C)

add_library(nerv STATIC
    public/Src/nerv_log.c
    public/Src/nerv_math.c
    public/Src/nerv_pid.c
    public/Src/nerv_utils.c
    module/motor/Src/nerv_motor.c
    module/motor/driver/nerv_motor_driver.c
    module/motor/driver/DM/nerv_motor_driver_dm.c
    module/motor/driver/DJI/nerv_motor_driver_dji.c
    module/chassis/Src/nerv_chassis.c
    config/Src/config_check.c
)

target_include_directories(nerv PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/public/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/config/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/module/motor/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/module/motor/driver
    ${CMAKE_CURRENT_SOURCE_DIR}/module/motor/driver/DM
    ${CMAKE_CURRENT_SOURCE_DIR}/module/motor/driver/DJI
    ${CMAKE_CURRENT_SOURCE_DIR}/module/chassis/Inc
)

target_compile_features(nerv PUBLIC c_std_11)

target_link_libraries(nerv PUBLIC m)

set(NERV_PLATFORM "" CACHE STRING
    "Platform macro, for example NERV_PLATFORM_STM32H7. Required by config.h.")

set(NERV_CHASSIS_TYPE "" CACHE STRING
    "Chassis type macro, for example NERV_CHASSIS_TYPE_OMNI.")

if (NERV_PLATFORM)
    target_compile_definitions(nerv PUBLIC NERV_PLATFORM=${NERV_PLATFORM})
endif()

if (NERV_CHASSIS_TYPE)
    target_compile_definitions(nerv PUBLIC NERV_CHASSIS_TYPE=${NERV_CHASSIS_TYPE})
endif()

option(NERV_WARNINGS "Enable strict compiler warnings for the library" OFF)

if (NERV_WARNINGS)
    if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(nerv PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()
