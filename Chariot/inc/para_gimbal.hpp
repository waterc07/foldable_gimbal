/**
 ******************************************************************************
 * @file           : para_gimbal.hpp
 * @brief          : Foldable gimbal test-bench parameters
 ******************************************************************************
 */

#pragma once

#include "math_const.h"

/******************************************************************************
 *                            Motor IDs
 ******************************************************************************/
#define LOWER_PITCH_DM4310_CAN_ID    1U
#define LOWER_PITCH_DM4310_MASTER_ID 4U
#define UPPER_PITCH_DM4310_CAN_ID    2U
#define UPPER_PITCH_DM4310_MASTER_ID 5U

// The two M3508 IDs were not specified with the hardware note. Keep them
// centralized so the bench can be corrected without touching control logic.
#define LEFT_FRICTION_M3508_ID  1U
#define RIGHT_FRICTION_M3508_ID 2U
#define FEEDER_M2006_ID         3U
#define FEEDER_M2006_RATIO      36.0f

/******************************************************************************
 *                            Motor buses
 ******************************************************************************/
#define PITCH_DM4310_CAN_HANDLE   hcan1
#define FRICTION_M3508_CAN_HANDLE hcan1
#define FEEDER_M2006_CAN_HANDLE   hcan1

/******************************************************************************
 *                            Foldable pitch model
 ******************************************************************************/
#define LOWER_PITCH_OUTPUT_POLARITY 1.0f
#define UPPER_PITCH_OUTPUT_POLARITY 1.0f

// Fill these four angles after mechanical calibration. Unit: rad.
#define LOWER_PITCH_FOLDED_ANGLE   0.950f
#define UPPER_PITCH_FOLDED_ANGLE   4.680f
#define LOWER_PITCH_DEPLOYED_ANGLE 2.120f
#define UPPER_PITCH_DEPLOYED_ANGLE 3.590f
#define FOLD_JOINT_RAMP_SPEED      2.0f
#define FOLD_JOINT_ANGLE_TOLERANCE 0.1f

#define PITCH_TARGET_UPPER_LIMIT   0.6f
#define PITCH_TARGET_LOWER_LIMIT   -0.6f
#define PITCH_GRAVITY_COMPENSATE   0.0f

/******************************************************************************
 *                            PID parameters
 ******************************************************************************/
#define LOWER_PITCH_OUTER_KP             42.0f
#define LOWER_PITCH_OUTER_KI             0.5f
#define LOWER_PITCH_OUTER_KD             0.0f
#define LOWER_PITCH_OUTER_OUT_LIMIT      11.0f // 11
#define LOWER_PITCH_OUTER_IOUT_LIMIT     1.0f
#define LOWER_PITCH_INNER_KP             0.85f
#define LOWER_PITCH_INNER_KI             0.0f
#define LOWER_PITCH_INNER_KD             0.08f
#define LOWER_PITCH_INNER_OUT_LIMIT      10.0f
#define LOWER_PITCH_INNER_IOUT_LIMIT     0.5f
#define LOWER_PITCH_INNER_LOWPASS_FILTER 0.9f

#define UPPER_PITCH_OUTER_KP             40.0f
#define UPPER_PITCH_OUTER_KI             1.2f
#define UPPER_PITCH_OUTER_KD             80.0f
#define UPPER_PITCH_OUTER_OUT_LIMIT      10.0f // 5
#define UPPER_PITCH_OUTER_IOUT_LIMIT     0.5f
#define UPPER_PITCH_INNER_KP             0.80f
#define UPPER_PITCH_INNER_KI             0.00f
#define UPPER_PITCH_INNER_KD             0.05f
#define UPPER_PITCH_INNER_OUT_LIMIT      10.0f
#define UPPER_PITCH_INNER_IOUT_LIMIT     0.5f
#define UPPER_PITCH_INNER_LOWPASS_FILTER 0.9f

#define FRICTION_KP                      380.0f
#define FRICTION_KI                      0.0f
#define FRICTION_KD                      0.0f
#define FRICTION_OUT_LIMIT               15000.0f
#define FRICTION_IOUT_LIMIT              2000.0f
#define FRICTION_TARGET_ANGULAR_VELOCITY 640.0f

#define FEEDER_KP                        7000.0f
#define FEEDER_KI                        10.0f
#define FEEDER_KD                        0.0f
#define FEEDER_OUT_LIMIT                 10000.0f
#define FEEDER_IOUT_LIMIT                2000.0f
#define FEEDER_BULLET_SLOT_COUNT         8.0f
#define FEEDER_BULLET_FREQ_HZ            25.0f
#define FEEDER_TARGET_DIRECTION          1.0f

#define CAN_TX_FAILURE_LATCH_COUNT       3U
#define CAN_TX_RECOVERY_SUCCESS_COUNT    10U
#define CAN_BUS_BITRATE                  1000000U
#define CAN_STD_DATA_FRAME_WORST_BITS    135U
#define CAN_LOAD_WARNING_PERMILLE        750U

/******************************************************************************
 *                            IMU parameters
 ******************************************************************************/
#define AHRS_AUTO_FREQ            0
#define AHRS_DEFAULT_FILTER       0
#define MAHONY_KP                 0.98f
#define MAHONY_KI                 0.0f

#define GYRO_OFFSET_X             0.0f
#define GYRO_OFFSET_Y             0.0f
#define GYRO_OFFSET_Z             0.0f
#define ACCEL_OFFSET_X            0.0f
#define ACCEL_OFFSET_Y            0.0f
#define ACCEL_OFFSET_Z            0.0f
#define MAG_OFFSET_X              0.0f
#define MAG_OFFSET_Y              0.0f
#define MAG_OFFSET_Z              0.0f
#define INSTALL_SPIN_MATRIX       GSRLMath::Matrix33f((fp32[3][3]){{0, 1, 0}, {1, 0, 0}, {0, 0, -1}})

#define IMU_TEMP_CTRL_ENABLE      1
#define IMU_TEMP_CTRL_TARGET_TEMP 42.0f
#define IMU_TEMP_CTRL_KP          10.0f
#define IMU_TEMP_CTRL_KI          1.0f
#define IMU_TEMP_CTRL_KD          0.5f
#define IMU_TEMP_CTRL_OUT_LIMIT   100.0f
#define IMU_TEMP_CTRL_IOUT_LIMIT  40.0f

/******************************************************************************
 *                            Remote control
 ******************************************************************************/
#define DT7_STICK_DEAD_ZONE         0.01f
#define DT7_STICK_PITCH_SENSITIVITY 0.006f
