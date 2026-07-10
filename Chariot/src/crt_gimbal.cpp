/**
 ******************************************************************************
 * @file           : crt_gimbal.cpp
 * @brief          : Foldable gimbal test-bench control
 ******************************************************************************
 */

#include "crt_gimbal.hpp"

#include "drv_can.h"
#include "drv_misc.h"
#include "drv_uart.h"
#include "task.h"
#include "tsk_isr.hpp"
#include "usbd_cdc_if.h"
#include <math.h>
#include <string.h>

extern CascadePID::PIDParam lowerPitchOuterParam;
extern CascadePID::PIDParam lowerPitchInnerParam;
extern CascadePID::PIDParam upperPitchOuterParam;
extern CascadePID::PIDParam upperPitchInnerParam;
extern CascadePID lowerPitchPID;
extern CascadePID upperPitchPID;
extern fp32 feederBulletFrequencyHz;

static void applyPitchPidProfile()
{
    taskENTER_CRITICAL();
    lowerPitchPID.getOuterLoop().pidSetParam(lowerPitchOuterParam);
    lowerPitchPID.getInnerLoop().pidSetParam(lowerPitchInnerParam);
    lowerPitchPID.cascadeClear();
    upperPitchPID.getOuterLoop().pidSetParam(upperPitchOuterParam);
    upperPitchPID.getInnerLoop().pidSetParam(upperPitchInnerParam);
    upperPitchPID.cascadeClear();
    taskEXIT_CRITICAL();
}

Gimbal::Gimbal(MotorDM4310 *lowerPitchMotor,
               MotorDM4310 *upperPitchMotor,
               MotorM3508 *leftFrictionMotor,
               MotorM3508 *rightFrictionMotor,
               MotorM2006 *feederMotor,
               BMI088 *imu,
               ImuControlConfig imuControlConfig)
    : m_lowerPitchMotor(lowerPitchMotor),
      m_upperPitchMotor(upperPitchMotor),
      m_leftFrictionMotor(leftFrictionMotor),
      m_rightFrictionMotor(rightFrictionMotor),
      m_feederMotor(feederMotor),
      m_imu(imu),
      m_imuControlConfig(imuControlConfig),
      m_gimbalMode(GIMBAL_NO_FORCE),
      m_lastGimbalMode(GIMBAL_NO_FORCE),
      m_lowerPitchJointTargetAngle(0.0f),
      m_upperPitchJointTargetAngle(0.0f),
      m_pitchTargetAngle(0.0f),
      m_frictionState(false),
      m_feederState(false),
      m_isDeployPitchReady(false),
      m_remoteControl(DT7_STICK_DEAD_ZONE),
      m_isInitComplete(false)
{
}

void Gimbal::init()
{
    DWT_Init();
    CAN_Init(&hcan1, can1RxCallback);
    UART_Init(&huart3, dr16RxCallback, 36);
    m_imu->init();
    m_isInitComplete = true;
}

void Gimbal::controlLoop()
{
    if (!m_isInitComplete) return;

    modeSelect();
    handleModeTransition();
    targetOrientationPlan();
    pitchControl();
    frictionControl();
    transmitGimbalMotorData();
}

uint8_t Gimbal::sendUsbData()
{
    uint32_t txbufIndex      = 0U;
    m_usbTxBuf[txbufIndex++] = m_usbTxSOF;

    const float pitchTarget = m_pitchTargetAngle;
    memcpy(m_usbTxBuf + txbufIndex, &pitchTarget, sizeof(float));
    txbufIndex += sizeof(float);

    const float pitchAngle = m_eulerAngle.y;
    memcpy(m_usbTxBuf + txbufIndex, &pitchAngle, sizeof(float));
    txbufIndex += sizeof(float);

    const float lowerAngle = m_lowerPitchMotor->getCurrentAngle();
    memcpy(m_usbTxBuf + txbufIndex, &lowerAngle, sizeof(float));
    txbufIndex += sizeof(float);

    const float upperAngle = m_upperPitchMotor->getCurrentAngle();
    memcpy(m_usbTxBuf + txbufIndex, &upperAngle, sizeof(float));
    txbufIndex += sizeof(float);

    m_usbTxBuf[txbufIndex++] = static_cast<uint8_t>(m_gimbalMode);
    m_usbTxBuf[txbufIndex++] = m_frictionState ? 1U : 0U;
    m_usbTxBuf[txbufIndex++] = m_feederState ? 1U : 0U;
    m_usbTxBuf[txbufIndex++] = m_usbTxEOF;

    return CDC_Transmit_FS(m_usbTxBuf, static_cast<uint16_t>(txbufIndex));
}

void Gimbal::imuLoop()
{
    if (!m_isInitComplete) return;

    m_eulerAngle = m_imu->solveAttitude();
    if (m_imuControlConfig.enableTemperatureControl) {
        m_imu->temperatureControl(m_imuControlConfig.targetTemperature);
    }
}

void Gimbal::receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (m_lowerPitchMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_upperPitchMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_leftFrictionMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_rightFrictionMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_feederMotor->decodeCanRxMessageFromISR(rxMessage)) return;
}

void Gimbal::receiveRemoteControlDataFromISR(const uint8_t *rxData)
{
    m_remoteControl.receiveRxDataFromISR(rxData);
}

void Gimbal::modeSelect()
{
    m_remoteControl.updateEvent();
    if (!m_remoteControl.isConnected()) {
        m_gimbalMode    = GIMBAL_NO_FORCE;
        m_frictionState = false;
        m_feederState   = false;
        return;
    }

    RemoteControl::SwitchStatus3Pos rightSwitchStatus = m_remoteControl.getRightSwitchStatus();
    switch (rightSwitchStatus) {
        case RemoteControl::SwitchStatus3Pos::SWITCH_DOWN:
            m_gimbalMode    = GIMBAL_NO_FORCE;
            m_frictionState = false;
            m_feederState   = false;
            return;

        case RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE:
            m_gimbalMode    = FOLD_CONTROL;
            m_frictionState = false;
            m_feederState   = false;
            return;

        case RemoteControl::SwitchStatus3Pos::SWITCH_UP:
            m_gimbalMode = DEPLOY_CONTROL;
            break;

        default:
            m_gimbalMode    = GIMBAL_NO_FORCE;
            m_frictionState = false;
            m_feederState   = false;
            return;
    }

    if (m_remoteControl.getLeftSwitchEvent() ==
        RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
        m_frictionState = !m_frictionState;
    }

    m_feederState = m_frictionState &&
                    m_remoteControl.getLeftSwitchStatus() == RemoteControl::SwitchStatus3Pos::SWITCH_DOWN;
}

void Gimbal::handleModeTransition()
{
    if (m_gimbalMode == m_lastGimbalMode) {
        return;
    }

    applyPitchPidProfile();

    m_lowerPitchJointTargetAngle = m_lowerPitchMotor->getCurrentAngle();
    m_upperPitchJointTargetAngle = m_upperPitchMotor->getCurrentAngle();
    m_isDeployPitchReady         = false;

    if (m_gimbalMode == DEPLOY_CONTROL) {
        setPitchAngle(m_eulerAngle.y);
    } else {
        m_pitchTargetAngle = m_eulerAngle.y;
    }

    m_lastGimbalMode = m_gimbalMode;
}

void Gimbal::targetOrientationPlan()
{
    if (m_gimbalMode != DEPLOY_CONTROL || !m_isDeployPitchReady) {
        return;
    }

    fp32 pitchIncrement = m_remoteControl.getRightStickY() * DT7_STICK_PITCH_SENSITIVITY;
    setPitchAngle(m_pitchTargetAngle - pitchIncrement);
}

void Gimbal::pitchControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_pitchTargetAngle           = m_eulerAngle.y;
            m_lowerPitchJointTargetAngle = m_lowerPitchMotor->getCurrentAngle();
            m_upperPitchJointTargetAngle = m_upperPitchMotor->getCurrentAngle();
            m_isDeployPitchReady         = false;
            m_lowerPitchMotor->openloopControl(0.0f);
            m_upperPitchMotor->openloopControl(0.0f);
            break;

        case FOLD_CONTROL:
            m_isDeployPitchReady = false;
            planFoldJointTargets(LOWER_PITCH_FOLDED_ANGLE, UPPER_PITCH_FOLDED_ANGLE);
            jointAngleControl(m_lowerPitchMotor, m_lowerPitchJointTargetAngle, LOWER_PITCH_OUTPUT_POLARITY);
            jointAngleControl(m_upperPitchMotor, m_upperPitchJointTargetAngle, UPPER_PITCH_OUTPUT_POLARITY);
            break;

        case DEPLOY_CONTROL:
            planFoldJointTargets(LOWER_PITCH_DEPLOYED_ANGLE, UPPER_PITCH_DEPLOYED_ANGLE);
            jointAngleControl(m_lowerPitchMotor, m_lowerPitchJointTargetAngle, LOWER_PITCH_OUTPUT_POLARITY);
            if (m_isDeployPitchReady || isDeployReached()) {
                if (!m_isDeployPitchReady) {
                    m_isDeployPitchReady = true;
                    setPitchAngle(m_eulerAngle.y);
                }
                deployPitchControl();
            } else {
                m_pitchTargetAngle = m_eulerAngle.y;
                jointAngleControl(m_upperPitchMotor, m_upperPitchJointTargetAngle, UPPER_PITCH_OUTPUT_POLARITY);
            }
            break;

        default:
            break;
    }
}

void Gimbal::planFoldJointTargets(fp32 lowerFinalAngle, fp32 upperFinalAngle)
{
    m_lowerPitchJointTargetAngle = rampJointTarget(m_lowerPitchJointTargetAngle, lowerFinalAngle);
    m_upperPitchJointTargetAngle = rampJointTarget(m_upperPitchJointTargetAngle, upperFinalAngle);
}

fp32 Gimbal::rampJointTarget(fp32 currentTarget, fp32 finalTarget) const
{
    const fp32 maxStep = FOLD_JOINT_RAMP_SPEED * 0.001f;
    if (maxStep <= 0.0f) {
        return finalTarget;
    }

    const fp32 delta = GSRLMath::normalizeDeltaAngle(finalTarget - currentTarget);
    if (fabsf(delta) <= maxStep) {
        return finalTarget;
    }

    return currentTarget + (delta > 0.0f ? maxStep : -maxStep);
}

bool Gimbal::isDeployReached() const
{
    const fp32 lowerError = GSRLMath::normalizeDeltaAngle(m_lowerPitchMotor->getCurrentAngle() - LOWER_PITCH_DEPLOYED_ANGLE);
    const fp32 upperError = GSRLMath::normalizeDeltaAngle(m_upperPitchMotor->getCurrentAngle() - UPPER_PITCH_DEPLOYED_ANGLE);
    return fabsf(lowerError) < FOLD_JOINT_ANGLE_TOLERANCE &&
           fabsf(upperError) < FOLD_JOINT_ANGLE_TOLERANCE;
}

void Gimbal::jointAngleControl(MotorDM4310 *motor, fp32 targetAngle, fp32 outputPolarity)
{
    fp32 fdbData[2] = {
        GSRLMath::normalizeDeltaAngle(motor->getCurrentAngle() - targetAngle),
        motor->getCurrentAngularVelocity()};

    fp32 output = motor->externalClosedloopControl(0.0f, fdbData, 2);
    motor->openloopControl(output * outputPolarity);
}

void Gimbal::deployPitchControl()
{
    fp32 fdbData[2] = {
        GSRLMath::normalizeDeltaAngle(m_eulerAngle.y - m_pitchTargetAngle),
        m_imu->getGyro().y};

    fp32 upperOutput = m_upperPitchMotor->externalClosedloopControl(0.0f, fdbData, 2);
    upperOutput      = gravityCompensate(upperOutput, m_eulerAngle.y, PITCH_GRAVITY_COMPENSATE);
    m_upperPitchMotor->openloopControl(upperOutput * UPPER_PITCH_OUTPUT_POLARITY);
}

void Gimbal::frictionControl()
{
    if (m_gimbalMode != DEPLOY_CONTROL || !m_isDeployPitchReady || !m_frictionState) {
        m_leftFrictionMotor->openloopControl(0.0f);
        m_rightFrictionMotor->openloopControl(0.0f);
        m_feederMotor->openloopControl(0.0f);
        m_feederState = false;
        return;
    }

    m_leftFrictionMotor->angularVelocityClosedloopControl(-FRICTION_TARGET_ANGULAR_VELOCITY);
    m_rightFrictionMotor->angularVelocityClosedloopControl(FRICTION_TARGET_ANGULAR_VELOCITY);

    if (m_feederState) {
        m_feederMotor->angularVelocityClosedloopControl(feederTargetAngularVelocity());
    } else {
        m_feederMotor->openloopControl(0.0f);
    }
}

void Gimbal::transmitGimbalMotorData()
{
    static bool transmitUpperPitchThisCycle = false;

    MotorGM6020 mergedDjiMotor           = *m_leftFrictionMotor + *m_rightFrictionMotor + *m_feederMotor;
    const uint8_t *mergedDjiControlData  = mergedDjiMotor.getMotorControlData();
    memcpy(m_frictionFeederControlData, mergedDjiControlData, sizeof(m_frictionFeederControlData));

    m_lastDjiCanTxFreeLevelBefore        = HAL_CAN_GetTxMailboxesFreeLevel(&FRICTION_M3508_CAN_HANDLE);
    const HAL_StatusTypeDef djiTxStatus  = CAN_Send_Data(&FRICTION_M3508_CAN_HANDLE,
                                                         const_cast<CAN_TxHeaderTypeDef *>(m_leftFrictionMotor->getMotorControlHeader()),
                                                         const_cast<uint8_t *>(mergedDjiControlData));
    m_lastDjiCanTxFreeLevelAfter         = HAL_CAN_GetTxMailboxesFreeLevel(&FRICTION_M3508_CAN_HANDLE);
    m_lastDjiCanError                    = HAL_CAN_GetError(&FRICTION_M3508_CAN_HANDLE);
    m_lastDjiTxStdId                     = m_leftFrictionMotor->getMotorControlMessageID();
    m_lastDjiCan200TxStatus              = static_cast<uint8_t>(djiTxStatus);
    m_lastDjiCan1ffTxStatus              = 0xFEU;

    if (transmitUpperPitchThisCycle) {
        const HAL_StatusTypeDef txStatus = CAN_Send_Data(&PITCH_DM4310_CAN_HANDLE,
                                                         const_cast<CAN_TxHeaderTypeDef *>(m_upperPitchMotor->getMotorControlHeader()),
                                                         const_cast<uint8_t *>(m_upperPitchMotor->getMotorControlData()));
        m_lastUpperDmTxStatus            = static_cast<uint8_t>(txStatus);
    } else {
        const HAL_StatusTypeDef txStatus = CAN_Send_Data(&PITCH_DM4310_CAN_HANDLE,
                                                         const_cast<CAN_TxHeaderTypeDef *>(m_lowerPitchMotor->getMotorControlHeader()),
                                                         const_cast<uint8_t *>(m_lowerPitchMotor->getMotorControlData()));
        m_lastLowerDmTxStatus            = static_cast<uint8_t>(txStatus);
    }
    transmitUpperPitchThisCycle = !transmitUpperPitchThisCycle;
}

inline void Gimbal::setPitchAngle(const fp32 &targetAngle)
{
    if (targetAngle > PITCH_TARGET_UPPER_LIMIT) {
        m_pitchTargetAngle = PITCH_TARGET_UPPER_LIMIT;
    } else if (targetAngle < PITCH_TARGET_LOWER_LIMIT) {
        m_pitchTargetAngle = PITCH_TARGET_LOWER_LIMIT;
    } else {
        m_pitchTargetAngle = targetAngle;
    }
}

inline fp32 Gimbal::feederTargetAngularVelocity() const
{
    return FEEDER_TARGET_DIRECTION * 2.0f * MATH_PI * feederBulletFrequencyHz / FEEDER_BULLET_SLOT_COUNT;
}

inline fp32 Gimbal::gravityCompensate(fp32 baseTorque, fp32 currentAngle, fp32 compensateCoeff)
{
    return baseTorque + compensateCoeff * cosf(currentAngle);
}
