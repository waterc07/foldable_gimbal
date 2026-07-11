/**
 ******************************************************************************
 * @file           : tsk_gimbal.cpp
 * @brief          : Foldable gimbal test-bench task
 ******************************************************************************
 */

#include "crt_gimbal.hpp"
#include "tim.h"

/******************************************************************************
 *                            PID
 ******************************************************************************/
CascadePID::PIDParam lowerPitchOuterParam = {
    LOWER_PITCH_OUTER_KP,
    LOWER_PITCH_OUTER_KI,
    LOWER_PITCH_OUTER_KD,
    LOWER_PITCH_OUTER_OUT_LIMIT,
    LOWER_PITCH_OUTER_IOUT_LIMIT};

CascadePID::PIDParam lowerPitchInnerParam = {
    LOWER_PITCH_INNER_KP,
    LOWER_PITCH_INNER_KI,
    LOWER_PITCH_INNER_KD,
    LOWER_PITCH_INNER_OUT_LIMIT,
    LOWER_PITCH_INNER_IOUT_LIMIT};

CascadePID::PIDParam upperPitchOuterParam = {
    UPPER_PITCH_OUTER_KP,
    UPPER_PITCH_OUTER_KI,
    UPPER_PITCH_OUTER_KD,
    UPPER_PITCH_OUTER_OUT_LIMIT,
    UPPER_PITCH_OUTER_IOUT_LIMIT};

CascadePID::PIDParam upperPitchInnerParam = {
    UPPER_PITCH_INNER_KP,
    UPPER_PITCH_INNER_KI,
    UPPER_PITCH_INNER_KD,
    UPPER_PITCH_INNER_OUT_LIMIT,
    UPPER_PITCH_INNER_IOUT_LIMIT};

LowPassFilter<fp32> lowerPitchInnerLPF(LOWER_PITCH_INNER_LOWPASS_FILTER);
LowPassFilter<fp32> upperPitchInnerLPF(UPPER_PITCH_INNER_LOWPASS_FILTER);
CascadePID lowerPitchPID(lowerPitchOuterParam, lowerPitchInnerParam, nullptr, &lowerPitchInnerLPF);
CascadePID upperPitchPID(upperPitchOuterParam, upperPitchInnerParam, nullptr, &upperPitchInnerLPF);

SimplePID::PIDParam frictionPIDParam = {
    FRICTION_KP,
    FRICTION_KI,
    FRICTION_KD,
    FRICTION_OUT_LIMIT,
    FRICTION_IOUT_LIMIT};

SimplePID leftFrictionPID(SimplePID::PID_POSITION, frictionPIDParam);
SimplePID rightFrictionPID(SimplePID::PID_POSITION, frictionPIDParam);

SimplePID::PIDParam feederPIDParam = {
    FEEDER_KP,
    FEEDER_KI,
    FEEDER_KD,
    FEEDER_OUT_LIMIT,
    FEEDER_IOUT_LIMIT};

SimplePID feederPID(SimplePID::PID_POSITION, feederPIDParam);
fp32 feederBulletFrequencyHz = FEEDER_BULLET_FREQ_HZ;

/******************************************************************************
 *                            Motors
 ******************************************************************************/
static_assert(LEFT_FRICTION_M3508_ID >= 1U && LEFT_FRICTION_M3508_ID <= 4U,
              "DJI 0x200 combined frame supports motor IDs 1..4 only");
static_assert(RIGHT_FRICTION_M3508_ID >= 1U && RIGHT_FRICTION_M3508_ID <= 4U,
              "DJI 0x200 combined frame supports motor IDs 1..4 only");
static_assert(FEEDER_M2006_ID >= 1U && FEEDER_M2006_ID <= 4U,
              "DJI 0x200 combined frame supports motor IDs 1..4 only");
static_assert(LEFT_FRICTION_M3508_ID != RIGHT_FRICTION_M3508_ID &&
                  LEFT_FRICTION_M3508_ID != FEEDER_M2006_ID &&
                  RIGHT_FRICTION_M3508_ID != FEEDER_M2006_ID,
              "DJI motors sharing CAN 0x200 must use unique IDs");

MotorDM4310 lowerPitchMotor(LOWER_PITCH_DM4310_CAN_ID,
                            LOWER_PITCH_DM4310_MASTER_ID,
                            3.141593f,
                            30.0f,
                            10.0f,
                            &lowerPitchPID);

MotorDM4310 upperPitchMotor(UPPER_PITCH_DM4310_CAN_ID,
                            UPPER_PITCH_DM4310_MASTER_ID,
                            3.141593f,
                            30.0f,
                            10.0f,
                            &upperPitchPID);

MotorM3508 leftFrictionMotor(LEFT_FRICTION_M3508_ID, &leftFrictionPID);
MotorM3508 rightFrictionMotor(RIGHT_FRICTION_M3508_ID, &rightFrictionPID);
MotorM2006 feederMotor(FEEDER_M2006_ID, &feederPID, 0U, FEEDER_M2006_RATIO);

/******************************************************************************
 *                            IMU
 ******************************************************************************/
Mahony ahrs(AHRS_AUTO_FREQ, AHRS_DEFAULT_FILTER, MAHONY_KP, MAHONY_KI);

BMI088::CalibrationInfo cali = {
    {GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z},
    {ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z},
    {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z},
    {INSTALL_SPIN_MATRIX}};

SimplePID::PIDParam imuTempPIDParam = {
    IMU_TEMP_CTRL_KP,
    IMU_TEMP_CTRL_KI,
    IMU_TEMP_CTRL_KD,
    IMU_TEMP_CTRL_OUT_LIMIT,
    IMU_TEMP_CTRL_IOUT_LIMIT};

SimplePID imuTempPID(SimplePID::PID_POSITION, imuTempPIDParam);

BMI088::TemperatureCtrlConfig imuTempCtrlConfig = {
    &htim10,
    TIM_CHANNEL_1,
    &imuTempPID};

#if IMU_TEMP_CTRL_ENABLE
BMI088::TemperatureCtrlConfig *activeImuTempCtrlConfig = &imuTempCtrlConfig;
#else
BMI088::TemperatureCtrlConfig *activeImuTempCtrlConfig = nullptr;
#endif

Gimbal::ImuControlConfig gimbalImuControlConfig = {
    static_cast<bool>(IMU_TEMP_CTRL_ENABLE),
    IMU_TEMP_CTRL_TARGET_TEMP};

BMI088 imu(&ahrs,
           {&hspi1, GPIOA, GPIO_PIN_4},
           {&hspi1, GPIOB, GPIO_PIN_0},
           cali,
           nullptr,
           nullptr,
           activeImuTempCtrlConfig);

Gimbal gimbal(&lowerPitchMotor,
              &upperPitchMotor,
              &leftFrictionMotor,
              &rightFrictionMotor,
              &feederMotor,
              &imu,
              gimbalImuControlConfig);

extern "C" void gimbal_task(void *argument)
{
    gimbal.init();
    TickType_t taskLastWakeTime = xTaskGetTickCount();

    while (1) {
        gimbal.controlLoop();
        vTaskDelayUntil(&taskLastWakeTime, pdMS_TO_TICKS(1));
    }
}
