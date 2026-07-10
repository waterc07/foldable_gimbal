/**
 ******************************************************************************
 * @file           : tsk_vofa.cpp
 * @brief          : VOFA telemetry task on USART6
 ******************************************************************************
 */

#include "crt_gimbal.hpp"
#include "dvc_vofa.hpp"

extern CascadePID::PIDParam lowerPitchOuterParam;
extern CascadePID::PIDParam lowerPitchInnerParam;
extern CascadePID::PIDParam upperPitchOuterParam;
extern CascadePID::PIDParam upperPitchInnerParam;
extern CascadePID lowerPitchPID;
extern CascadePID upperPitchPID;
extern SimplePID::PIDParam frictionPIDParam;
extern SimplePID leftFrictionPID;
extern SimplePID rightFrictionPID;
extern SimplePID::PIDParam feederPIDParam;
extern SimplePID feederPID;
extern fp32 feederBulletFrequencyHz;
extern SimplePID::PIDParam imuTempPIDParam;
extern SimplePID imuTempPID;
extern Gimbal gimbal;

static Vofa<37> vofa;

static void applySimplePidParam(SimplePID &pid, SimplePID::PIDParam &param)
{
    UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
    pid.pidSetParam(param);
    pid.pidClear();
    taskEXIT_CRITICAL_FROM_ISR(irqState);
}

static void applyLowerPitchPidParam()
{
    UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
    lowerPitchPID.getOuterLoop().pidSetParam(lowerPitchOuterParam);
    lowerPitchPID.getInnerLoop().pidSetParam(lowerPitchInnerParam);
    lowerPitchPID.cascadeClear();
    taskEXIT_CRITICAL_FROM_ISR(irqState);
}

static void applyUpperPitchPidParam()
{
    UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
    upperPitchPID.getOuterLoop().pidSetParam(upperPitchOuterParam);
    upperPitchPID.getInnerLoop().pidSetParam(upperPitchInnerParam);
    upperPitchPID.cascadeClear();
    taskEXIT_CRITICAL_FROM_ISR(irqState);
}

static void applyFrictionPidParam()
{
    UBaseType_t irqState = taskENTER_CRITICAL_FROM_ISR();
    leftFrictionPID.pidSetParam(frictionPIDParam);
    leftFrictionPID.pidClear();
    rightFrictionPID.pidSetParam(frictionPIDParam);
    rightFrictionPID.pidClear();
    taskEXIT_CRITICAL_FROM_ISR(irqState);
}

static void applyFeederPidParam()
{
    applySimplePidParam(feederPID, feederPIDParam);
}

static void registerParameterListeners()
{
    static bool isRegistered = false;
    if (isRegistered) {
        return;
    }

    vofa.AddParameterListener("lower_pitch_outer_kp", [](fp32 *newValue) {
        lowerPitchOuterParam.Kp = *newValue;
        applyLowerPitchPidParam();
    });
    vofa.AddParameterListener("lower_pitch_outer_ki", [](fp32 *newValue) {
        lowerPitchOuterParam.Ki = *newValue;
        applyLowerPitchPidParam();
    });
    vofa.AddParameterListener("lower_pitch_outer_kd", [](fp32 *newValue) {
        lowerPitchOuterParam.Kd = *newValue;
        applyLowerPitchPidParam();
    });
    vofa.AddParameterListener("lower_pitch_outer_out_limit", [](fp32 *newValue) {
        lowerPitchOuterParam.outputLimit = *newValue;
        applyLowerPitchPidParam();
    });
    vofa.AddParameterListener("lower_pitch_outer_iout_limit", [](fp32 *newValue) {
        lowerPitchOuterParam.intergralLimit = *newValue;
        applyLowerPitchPidParam();
    });

    vofa.AddParameterListener("lower_pitch_inner_kp", [](fp32 *newValue) {
        lowerPitchInnerParam.Kp = *newValue;
        applyLowerPitchPidParam();
    });
    vofa.AddParameterListener("lower_pitch_inner_ki", [](fp32 *newValue) {
        lowerPitchInnerParam.Ki = *newValue;
        applyLowerPitchPidParam();
    });
    vofa.AddParameterListener("lower_pitch_inner_kd", [](fp32 *newValue) {
        lowerPitchInnerParam.Kd = *newValue;
        applyLowerPitchPidParam();
    });
    vofa.AddParameterListener("lower_pitch_inner_out_limit", [](fp32 *newValue) {
        lowerPitchInnerParam.outputLimit = *newValue;
        applyLowerPitchPidParam();
    });
    vofa.AddParameterListener("lower_pitch_inner_iout_limit", [](fp32 *newValue) {
        lowerPitchInnerParam.intergralLimit = *newValue;
        applyLowerPitchPidParam();
    });

    vofa.AddParameterListener("upper_pitch_outer_kp", [](fp32 *newValue) {
        upperPitchOuterParam.Kp = *newValue;
        applyUpperPitchPidParam();
    });
    vofa.AddParameterListener("upper_pitch_outer_ki", [](fp32 *newValue) {
        upperPitchOuterParam.Ki = *newValue;
        applyUpperPitchPidParam();
    });
    vofa.AddParameterListener("upper_pitch_outer_kd", [](fp32 *newValue) {
        upperPitchOuterParam.Kd = *newValue;
        applyUpperPitchPidParam();
    });
    vofa.AddParameterListener("upper_pitch_outer_out_limit", [](fp32 *newValue) {
        upperPitchOuterParam.outputLimit = *newValue;
        applyUpperPitchPidParam();
    });
    vofa.AddParameterListener("upper_pitch_outer_iout_limit", [](fp32 *newValue) {
        upperPitchOuterParam.intergralLimit = *newValue;
        applyUpperPitchPidParam();
    });

    vofa.AddParameterListener("upper_pitch_inner_kp", [](fp32 *newValue) {
        upperPitchInnerParam.Kp = *newValue;
        applyUpperPitchPidParam();
    });
    vofa.AddParameterListener("upper_pitch_inner_ki", [](fp32 *newValue) {
        upperPitchInnerParam.Ki = *newValue;
        applyUpperPitchPidParam();
    });
    vofa.AddParameterListener("upper_pitch_inner_kd", [](fp32 *newValue) {
        upperPitchInnerParam.Kd = *newValue;
        applyUpperPitchPidParam();
    });
    vofa.AddParameterListener("upper_pitch_inner_out_limit", [](fp32 *newValue) {
        upperPitchInnerParam.outputLimit = *newValue;
        applyUpperPitchPidParam();
    });
    vofa.AddParameterListener("upper_pitch_inner_iout_limit", [](fp32 *newValue) {
        upperPitchInnerParam.intergralLimit = *newValue;
        applyUpperPitchPidParam();
    });

    vofa.AddParameterListener("friction_kp", [](fp32 *newValue) {
        frictionPIDParam.Kp = *newValue;
        applyFrictionPidParam();
    });
    vofa.AddParameterListener("friction_ki", [](fp32 *newValue) {
        frictionPIDParam.Ki = *newValue;
        applyFrictionPidParam();
    });
    vofa.AddParameterListener("friction_kd", [](fp32 *newValue) {
        frictionPIDParam.Kd = *newValue;
        applyFrictionPidParam();
    });
    vofa.AddParameterListener("friction_out_limit", [](fp32 *newValue) {
        frictionPIDParam.outputLimit = *newValue;
        applyFrictionPidParam();
    });
    vofa.AddParameterListener("friction_iout_limit", [](fp32 *newValue) {
        frictionPIDParam.intergralLimit = *newValue;
        applyFrictionPidParam();
    });

    vofa.AddParameterListener("feeder_kp", [](fp32 *newValue) {
        feederPIDParam.Kp = *newValue;
        applyFeederPidParam();
    });
    vofa.AddParameterListener("feeder_ki", [](fp32 *newValue) {
        feederPIDParam.Ki = *newValue;
        applyFeederPidParam();
    });
    vofa.AddParameterListener("feeder_kd", [](fp32 *newValue) {
        feederPIDParam.Kd = *newValue;
        applyFeederPidParam();
    });
    vofa.AddParameterListener("feeder_out_limit", [](fp32 *newValue) {
        feederPIDParam.outputLimit = *newValue;
        applyFeederPidParam();
    });
    vofa.AddParameterListener("feeder_iout_limit", [](fp32 *newValue) {
        feederPIDParam.intergralLimit = *newValue;
        applyFeederPidParam();
    });
    vofa.AddParameterListener("feeder_bullet_freq_hz", [](fp32 *newValue) {
        UBaseType_t irqState    = taskENTER_CRITICAL_FROM_ISR();
        feederBulletFrequencyHz = *newValue > 0.0f ? *newValue : 0.0f;
        taskEXIT_CRITICAL_FROM_ISR(irqState);
    });

    vofa.AddParameterListener("imu_temp_kp", [](fp32 *newValue) {
        imuTempPIDParam.Kp = *newValue;
        applySimplePidParam(imuTempPID, imuTempPIDParam);
    });
    vofa.AddParameterListener("imu_temp_ki", [](fp32 *newValue) {
        imuTempPIDParam.Ki = *newValue;
        applySimplePidParam(imuTempPID, imuTempPIDParam);
    });
    vofa.AddParameterListener("imu_temp_kd", [](fp32 *newValue) {
        imuTempPIDParam.Kd = *newValue;
        applySimplePidParam(imuTempPID, imuTempPIDParam);
    });
    vofa.AddParameterListener("imu_temp_target", [](fp32 *newValue) {
        UBaseType_t irqState                        = taskENTER_CRITICAL_FROM_ISR();
        gimbal.m_imuControlConfig.targetTemperature = *newValue;
        taskEXIT_CRITICAL_FROM_ISR(irqState);
    });

    isRegistered = true;
}

static void writeGimbalWaveData()
{
    // vofa.writeData(gimbal.m_lowerPitchMotor->getCurrentAngle());
    // vofa.writeData(gimbal.m_upperPitchMotor->getCurrentAngle());
    // vofa.writeData(gimbal.m_pitchTargetAngle);
    // vofa.writeData(gimbal.m_eulerAngle.y);
    // vofa.writeData(gimbal.m_lowerPitchMotor->getCurrentAngularVelocity());
    // vofa.writeData(gimbal.m_upperPitchMotor->getCurrentAngularVelocity());
    vofa.writeData(gimbal.m_leftFrictionMotor->getCurrentAngularVelocity() * 60.0f / (2.0f * M_PI));
    vofa.writeData(gimbal.m_rightFrictionMotor->getCurrentAngularVelocity() * 60.0f / (2.0f * M_PI));
    vofa.writeData(gimbal.m_feederMotor->getCurrentAngularVelocity() * 60.0f / (2.0f * M_PI));
    vofa.writeData(feederBulletFrequencyHz);
    vofa.writeData(leftFrictionPID.pidGetData().output);
    vofa.writeData(rightFrictionPID.pidGetData().output);
    vofa.writeData(feederPID.pidGetData().output);
    vofa.writeData(gimbal.m_leftFrictionMotor->isMotorConected() ? 1.0f : 0.0f);
    vofa.writeData(gimbal.m_rightFrictionMotor->isMotorConected() ? 1.0f : 0.0f);
    vofa.writeData(gimbal.m_feederMotor->isMotorConected() ? 1.0f : 0.0f);
    vofa.writeData(static_cast<fp32>(gimbal.m_lastDjiCan200TxStatus));
    vofa.writeData(static_cast<fp32>(gimbal.m_lastLowerDmTxStatus));
    vofa.writeData(static_cast<fp32>(gimbal.m_lastUpperDmTxStatus));
    vofa.writeData(static_cast<fp32>(gimbal.m_lastDjiCanError));
}

extern "C" void vofa_task(void *argument)
{
    TickType_t taskLastWakeTime = xTaskGetTickCount();

    vofa.Init();
    registerParameterListeners();

    while (1) {
        writeGimbalWaveData();
        vofa.sendFrame();
        vTaskDelayUntil(&taskLastWakeTime, pdMS_TO_TICKS(10));
    }
}
