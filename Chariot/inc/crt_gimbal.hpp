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
        FOLD_CONTROL,
        DEPLOY_CONTROL,
    };

    MotorDM4310 *m_lowerPitchMotor;
    MotorDM4310 *m_upperPitchMotor;
    MotorM3508 *m_leftFrictionMotor;
    MotorM3508 *m_rightFrictionMotor;
    MotorM2006 *m_feederMotor;

    BMI088 *m_imu;
    Vector3f m_eulerAngle;
    ImuControlConfig m_imuControlConfig;

    GimbalMode m_gimbalMode;
    GimbalMode m_lastGimbalMode;
    fp32 m_lowerPitchJointTargetAngle;
    fp32 m_upperPitchJointTargetAngle;
    fp32 m_pitchTargetAngle;
    bool m_frictionState;
    bool m_feederState;
    bool m_isDeployPitchReady;
    DR16RemoteControl m_remoteControl;
    bool m_isInitComplete;

    static constexpr uint32_t m_usbTxBufSize = 32U;
    uint8_t m_usbTxBuf[m_usbTxBufSize]       = {};
    uint8_t m_usbTxSOF                       = 0x3A;
    uint8_t m_usbTxEOF                       = 0xAA;
    uint8_t m_frictionFeederControlData[8]   = {};
    uint8_t m_lastDjiCan200TxStatus          = 0xFFU;
    uint8_t m_lastDjiCan1ffTxStatus          = 0xFFU;
    uint32_t m_lastDjiTxStdId                = 0U;
    uint32_t m_lastDjiCanTxFreeLevelBefore   = 0U;
    uint32_t m_lastDjiCanTxFreeLevelAfter    = 0U;
    uint32_t m_lastDjiCanError               = 0U;
    uint8_t m_lastLowerDmTxStatus            = 0xFFU;
    uint8_t m_lastUpperDmTxStatus            = 0xFFU;
    uint8_t m_canTxFailureStreak             = 0U;
    uint8_t m_canTxRecoverySuccessStreak     = 0U;
    bool m_isCanHealthy                      = false;
    bool m_canFaultLatched                   = false;
    uint32_t m_canRxFramesPerSecond          = 0U;
    uint32_t m_canTxFramesPerSecond          = 0U;
    uint32_t m_canEstimatedLoadPermille      = 0U;
    uint32_t m_canRxFifoFullCount            = 0U;
    uint32_t m_canRxFifoOverrunCount         = 0U;
    uint32_t m_canRxFifoPeakFillLevel        = 0U;
    uint32_t m_canTxMailboxFullCount         = 0U;
    uint8_t m_canTransmitErrorCounter        = 0U;
    uint8_t m_canReceiveErrorCounter         = 0U;
    bool m_isCanLoadHigh                     = false;

public:
    Gimbal(MotorDM4310 *lowerPitchMotor,
           MotorDM4310 *upperPitchMotor,
           MotorM3508 *leftFrictionMotor,
           MotorM3508 *rightFrictionMotor,
           MotorM2006 *feederMotor,
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
    void planFoldJointTargets(fp32 lowerFinalAngle, fp32 upperFinalAngle);
    fp32 rampJointTarget(fp32 currentTarget, fp32 finalTarget) const;
    bool isDeployReached() const;
    void jointAngleControl(MotorDM4310 *motor, fp32 targetAngle, fp32 outputPolarity);
    void deployPitchControl();
    void frictionControl();
    void transmitGimbalMotorData();
    void mergeDjiMotorControlData(MotorGM6020 *motor);
    void forceCanSafeState();
    void latchCanFault();
    void updateCanTxHealth(bool txCycleSucceeded);
    void updateCanLoadMonitor();

    inline void setPitchAngle(const fp32 &targetAngle);
    inline fp32 feederTargetAngularVelocity() const;
    inline fp32 gravityCompensate(fp32 baseTorque, fp32 currentAngle, fp32 compensateCoeff);
};
