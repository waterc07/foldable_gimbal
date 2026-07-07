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

extern CascadePID::PIDParam pitchOuterParam;
extern CascadePID::PIDParam pitchInnerParam;
extern CascadePID lowerPitchPID;
extern CascadePID upperPitchPID;

static void applyPitchPidProfile()
{
    taskENTER_CRITICAL();
    lowerPitchPID.getOuterLoop().pidSetParam(pitchOuterParam);
    lowerPitchPID.getInnerLoop().pidSetParam(pitchInnerParam);
    lowerPitchPID.cascadeClear();
    upperPitchPID.getOuterLoop().pidSetParam(pitchOuterParam);
    upperPitchPID.getInnerLoop().pidSetParam(pitchInnerParam);
    upperPitchPID.cascadeClear();
    taskEXIT_CRITICAL();
}

Gimbal::Gimbal(MotorDM4310 *lowerPitchMotor,
               MotorDM4310 *upperPitchMotor,
               MotorM3508 *leftFrictionMotor,
               MotorM3508 *rightFrictionMotor,
               BMI088 *imu,
               ImuControlConfig imuControlConfig)
    : m_lowerPitchMotor(lowerPitchMotor),
      m_upperPitchMotor(upperPitchMotor),
      m_leftFrictionMotor(leftFrictionMotor),
      m_rightFrictionMotor(rightFrictionMotor),
      m_imu(imu),
      m_imuControlConfig(imuControlConfig),
      m_gimbalMode(GIMBAL_NO_FORCE),
      m_lastGimbalMode(GIMBAL_NO_FORCE),
      m_pitchTargetAngle(0.0f),
      m_frictionState(false),
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
    uint32_t txbufIndex = 0U;
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
        return;
    }

    RemoteControl::SwitchStatus3Pos rightSwitchStatus = m_remoteControl.getRightSwitchStatus();
    if (rightSwitchStatus == RemoteControl::SwitchStatus3Pos::SWITCH_DOWN) {
        m_gimbalMode    = GIMBAL_NO_FORCE;
        m_frictionState = false;
        return;
    }

    m_gimbalMode = MANUAL_CONTROL;

    if (m_remoteControl.getLeftSwitchEvent() ==
        RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
        m_frictionState = !m_frictionState;
    }
}

void Gimbal::handleModeTransition()
{
    if (m_gimbalMode == m_lastGimbalMode) {
        return;
    }

    applyPitchPidProfile();

    if (m_gimbalMode == MANUAL_CONTROL) {
        setPitchAngle(m_eulerAngle.y);
    } else {
        m_pitchTargetAngle = m_eulerAngle.y;
    }

    m_lastGimbalMode = m_gimbalMode;
}

void Gimbal::targetOrientationPlan()
{
    if (m_gimbalMode != MANUAL_CONTROL) {
        return;
    }

    fp32 pitchIncrement = m_remoteControl.getRightStickY() * DT7_STICK_PITCH_SENSITIVITY;
    setPitchAngle(m_pitchTargetAngle - pitchIncrement);
}

void Gimbal::pitchControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_pitchTargetAngle = m_eulerAngle.y;
            m_lowerPitchMotor->openloopControl(0.0f);
            m_upperPitchMotor->openloopControl(0.0f);
            break;

        case MANUAL_CONTROL: {
            fp32 fdbData[2] = {
                GSRLMath::normalizeDeltaAngle(m_eulerAngle.y - m_pitchTargetAngle),
                m_imu->getGyro().y};

            fp32 lowerOutput = m_lowerPitchMotor->externalClosedloopControl(0.0f, fdbData, 2);
            fp32 upperOutput = m_upperPitchMotor->externalClosedloopControl(0.0f, fdbData, 2);
            lowerOutput      = gravityCompensate(lowerOutput, m_eulerAngle.y, PITCH_GRAVITY_COMPENSATE);
            upperOutput      = gravityCompensate(upperOutput, m_eulerAngle.y, PITCH_GRAVITY_COMPENSATE);

            m_lowerPitchMotor->openloopControl(lowerOutput * LOWER_PITCH_OUTPUT_POLARITY);
            m_upperPitchMotor->openloopControl(upperOutput * UPPER_PITCH_OUTPUT_POLARITY);
            break;
        }

        default:
            break;
    }
}

void Gimbal::frictionControl()
{
    if (m_gimbalMode == GIMBAL_NO_FORCE || !m_frictionState) {
        m_leftFrictionMotor->openloopControl(0.0f);
        m_rightFrictionMotor->openloopControl(0.0f);
        return;
    }

    m_leftFrictionMotor->angularVelocityClosedloopControl(FRICTION_TARGET_ANGULAR_VELOCITY);
    m_rightFrictionMotor->angularVelocityClosedloopControl(-FRICTION_TARGET_ANGULAR_VELOCITY);
}

void Gimbal::transmitGimbalMotorData()
{
    CAN_Send_Data(&PITCH_DM4310_CAN_HANDLE,
                  const_cast<CAN_TxHeaderTypeDef *>(m_lowerPitchMotor->getMotorControlHeader()),
                  const_cast<uint8_t *>(m_lowerPitchMotor->getMotorControlData()));
    CAN_Send_Data(&PITCH_DM4310_CAN_HANDLE,
                  const_cast<CAN_TxHeaderTypeDef *>(m_upperPitchMotor->getMotorControlHeader()),
                  const_cast<uint8_t *>(m_upperPitchMotor->getMotorControlData()));
    CAN_Send_Data(&FRICTION_M3508_CAN_HANDLE,
                  const_cast<CAN_TxHeaderTypeDef *>(m_leftFrictionMotor->getMotorControlHeader()),
                  const_cast<uint8_t *>(m_leftFrictionMotor->getMergedControlData(*m_rightFrictionMotor)));
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

inline fp32 Gimbal::gravityCompensate(fp32 baseTorque, fp32 currentAngle, fp32 compensateCoeff)
{
    return baseTorque + compensateCoeff * cosf(currentAngle);
}
