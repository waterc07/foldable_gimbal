/**
 ******************************************************************************
 * @file           : crt_gimbal.hpp
 * @brief          : Foldable gimbal test-bench controller
 ******************************************************************************
 */

#pragma once

#include "GSRL.hpp"
#include "para_gimbal.hpp"

class Gimbal
{
public:
    using Vector3f = GSRLMath::Vector3f;

    struct ImuControlConfig {
        bool enableTemperatureControl = false;
        fp32 targetTemperature        = 0.0f;
    };

    enum GimbalMode : uint8_t {
        GIMBAL_NO_FORCE = 0,
        MANUAL_CONTROL,
    };

    MotorDM4310 *m_lowerPitchMotor;
    MotorDM4310 *m_upperPitchMotor;
    MotorM3508 *m_leftFrictionMotor;
    MotorM3508 *m_rightFrictionMotor;

    BMI088 *m_imu;
    Vector3f m_eulerAngle;
    ImuControlConfig m_imuControlConfig;

    GimbalMode m_gimbalMode;
    GimbalMode m_lastGimbalMode;
    fp32 m_pitchTargetAngle;
    bool m_frictionState;
    DR16RemoteControl m_remoteControl;
    bool m_isInitComplete;

    static constexpr uint32_t m_usbTxBufSize = 32U;
    uint8_t m_usbTxBuf[m_usbTxBufSize]       = {};
    uint8_t m_usbTxSOF                       = 0x3A;
    uint8_t m_usbTxEOF                       = 0xAA;

public:
    Gimbal(MotorDM4310 *lowerPitchMotor,
           MotorDM4310 *upperPitchMotor,
           MotorM3508 *leftFrictionMotor,
           MotorM3508 *rightFrictionMotor,
           BMI088 *imu,
           ImuControlConfig imuControlConfig);

    void init();
    void controlLoop();
    void imuLoop();
    void receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage);
    void receiveRemoteControlDataFromISR(const uint8_t *rxData);
    uint8_t sendUsbData();

private:
    void modeSelect();
    void handleModeTransition();
    void targetOrientationPlan();
    void pitchControl();
    void frictionControl();
    void transmitGimbalMotorData();

    inline void setPitchAngle(const fp32 &targetAngle);
    inline fp32 gravityCompensate(fp32 baseTorque, fp32 currentAngle, fp32 compensateCoeff);
};
