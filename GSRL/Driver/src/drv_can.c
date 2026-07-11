/**
 ******************************************************************************
 * @file           : drv_can.c
 * @brief          : CAN driver
 *                   Contains CAN initialization related functions,
 *                   CAN receive interrupt callback function
 *                   Use queue to save received data when using FreeRTOS,
 *                   otherwise use global variables
 *                   CAN驱动
 *                   包含CAN初始化相关函数，CAN接收中断回调函数
 *                   使用FreeRTOS时使用队列保存接收数据，否则使用全局变量
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"
#include "queue.h"

/* Typedef -----------------------------------------------------------*/

/* Define ------------------------------------------------------------*/
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
    .name = "canRxQueue"};

/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
CAN_Manage_Object_t s_can_manage_objects[2] = {0};   // CAN管理对象
static bool_t is_queue_initialized          = false; // 队列初始化标志

/**
 * @brief CAN管理实例数组
 * @note 根据board_config.h中的宏定义配置决定使用的CAN实例
 */
static CAN_TypeDef *const canInstances[2] = {
#ifdef USE_CAN1
    CAN1,
#endif
#ifdef USE_CAN2
    CAN2
#endif
};

/* Function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef can_all_pass_filter_init(CAN_HandleTypeDef *hcan);

/* User code ---------------------------------------------------------*/

/**
 * @brief 获取CAN对象
 * @param hcan CAN句柄
 * @return CAN管理对象指针
 */
static CAN_Manage_Object_t *CAN_Get_Object(CAN_HandleTypeDef *hcan)
{
    for (int i = 0; i < 2; i++) {
        if (hcan->Instance == canInstances[i]) {
            return &s_can_manage_objects[i];
        }
    }
    return NULL;
}

/**
 * @brief 初始化CAN，注册回调函数，并启动CAN
 * @param hcan CAN句柄
 * @param rxCallbackFunction 处理回调函数
 */
HAL_StatusTypeDef CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back rxCallbackFunction)
{
    // 找到对应的CAN管理对象并设置参数
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL) return HAL_ERROR;

    can_obj->hcan               = hcan;
    can_obj->rxCallbackFunction = rxCallbackFunction;
    can_obj->lastError          = HAL_CAN_ERROR_NONE;
    can_obj->isInitialized      = false;
    can_obj->busOffObserved     = false;

    // 队列模式必须在开启接收中断前完成创建；回调模式不创建冗余队列。
    if (rxCallbackFunction == NULL && is_queue_initialized == false) {
        canRxQueueHandle = osMessageQueueNew(16, sizeof(can_rx_message_t), &canRxQueue_attributes);
        if (canRxQueueHandle == NULL) return HAL_ERROR;
        is_queue_initialized = true;
    }

    if (can_all_pass_filter_init(hcan) != HAL_OK) return HAL_ERROR;

    // 启动CAN
    if (HAL_CAN_Start(hcan) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_ActivateNotification(hcan,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                         CAN_IT_RX_FIFO0_FULL |
                                         CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK) {
        HAL_CAN_Stop(hcan);
        return HAL_ERROR;
    }

    can_obj->isInitialized = true;
    return HAL_OK;
}

/**
 * @brief Poll CAN state and finish automatic bus-off recovery.
 * @return HAL_OK when CAN is listening, HAL_BUSY while bus-off is active.
 */
HAL_StatusTypeDef CAN_Service(CAN_HandleTypeDef *hcan)
{
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL || can_obj->isInitialized == false) return HAL_ERROR;

    if ((hcan->Instance->ESR & CAN_ESR_BOFF) != 0U) {
        if (can_obj->busOffObserved == false) CAN_Abort_Pending_Tx(hcan);
        can_obj->busOffObserved = true;
        hcan->ErrorCode |= HAL_CAN_ERROR_BOF;
        can_obj->lastError = hcan->ErrorCode;
        return HAL_BUSY;
    }

    HAL_CAN_StateTypeDef state = HAL_CAN_GetState(hcan);
    if (state == HAL_CAN_STATE_READY) {
        if (HAL_CAN_Start(hcan) != HAL_OK) return HAL_ERROR;
        if (HAL_CAN_ActivateNotification(hcan,
                                         CAN_IT_RX_FIFO0_MSG_PENDING |
                                             CAN_IT_RX_FIFO0_FULL |
                                             CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK)
            return HAL_ERROR;
        state = HAL_CAN_GetState(hcan);
    }

    if (state != HAL_CAN_STATE_LISTENING) {
        can_obj->lastError = HAL_CAN_GetError(hcan);
        return HAL_ERROR;
    }

    if (can_obj->busOffObserved) {
        HAL_CAN_ResetError(hcan);
        can_obj->busOffObserved = false;
        can_obj->recoveryCount++;
    }
    can_obj->lastError = HAL_CAN_GetError(hcan);
    return HAL_OK;
}

/**
 * @brief Check whether CAN can currently accept transmit requests.
 */
bool_t CAN_Is_Ready(CAN_HandleTypeDef *hcan)
{
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL || can_obj->isInitialized == false) return false;
    return HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING &&
           (hcan->Instance->ESR & CAN_ESR_BOFF) == 0U;
}

/**
 * @brief Return read-only runtime counters for load and congestion diagnostics.
 */
const CAN_Manage_Object_t *CAN_Get_Stats(CAN_HandleTypeDef *hcan)
{
    return CAN_Get_Object(hcan);
}

/**
 * @brief Drop all pending frames so stale actuator commands cannot survive a fault.
 */
HAL_StatusTypeDef CAN_Abort_Pending_Tx(CAN_HandleTypeDef *hcan)
{
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL || can_obj->isInitialized == false) return HAL_ERROR;
    if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING) return HAL_ERROR;
    return HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
}

/**
 * @brief Queue one CAN frame after checking controller and mailbox state.
 */
HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData)
{
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL || pTxHeader == NULL || pTxData == NULL) return HAL_ERROR;
    if (!CAN_Is_Ready(hcan)) {
        can_obj->txFailureCount++;
        can_obj->lastError = HAL_CAN_GetError(hcan);
        return HAL_BUSY;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
        can_obj->txFailureCount++;
        can_obj->txMailboxFullCount++;
        return HAL_BUSY;
    }

    uint32_t txMailbox = 0U;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, pTxHeader, pTxData, &txMailbox);
    if (status != HAL_OK) {
        can_obj->txFailureCount++;
    } else {
        can_obj->txFrameCount++;
    }
    can_obj->lastError = HAL_CAN_GetError(hcan);
    return status;
}

/**
 * @brief 初始化CAN全通过滤器
 *        Initalize CAN all-pass filter
 * @param None
 * @retval None
 */
static HAL_StatusTypeDef can_all_pass_filter_init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter_st    = {0}; // 全通过滤器
    can_filter_st.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh         = 0x0000;
    can_filter_st.FilterIdLow          = 0x0000;
    can_filter_st.FilterMaskIdHigh     = 0x0000;
    can_filter_st.FilterMaskIdLow      = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_FilterFIFO0;
    can_filter_st.FilterActivation     = CAN_FILTER_ENABLE;
    can_filter_st.SlaveStartFilterBank = 14;

    if (hcan->Instance == CAN1) {
        can_filter_st.FilterBank = 0;
    } else if (hcan->Instance == CAN2) {
        can_filter_st.FilterBank = 14;
    } else {
        return HAL_ERROR;
    }

    return HAL_CAN_ConfigFilter(hcan, &can_filter_st);
}

/**
 * @brief HAL库CAN中断回调函数
 *        HAL library CAN interrupt callback function
 * @param hcan CAN句柄
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL) return;

    const uint32_t fifoFillLevel = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
    if (fifoFillLevel > can_obj->rxFifoPeakFillLevel) {
        can_obj->rxFifoPeakFillLevel = fifoFillLevel;
    }

    can_rx_message_t s_rx_msg;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &s_rx_msg.header, s_rx_msg.data) == HAL_OK) {
        can_obj->rxFrameCount++;
        if (can_obj->rxCallbackFunction != NULL) {
            can_obj->rxCallbackFunction(&s_rx_msg);
            return;
        }

        // No callback was registered, so deliver through the shared queue.
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (canRxQueueHandle == NULL) {
            can_obj->rxFailureCount++;
            return;
        }
        if (xQueueIsQueueFullFromISR(canRxQueueHandle)) // 队列满,移除最早的数据
        {
            can_rx_message_t s_dummy_msg;
            xQueueReceiveFromISR(canRxQueueHandle, &s_dummy_msg, &xHigherPriorityTaskWoken);
        }
        if (xQueueSendToBackFromISR(canRxQueueHandle, &s_rx_msg, &xHigherPriorityTaskWoken) != pdPASS) {
            can_obj->rxFailureCount++;
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        can_obj->rxFailureCount++;
    }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL) return;
    can_obj->rxFifoFullCount++;
    can_obj->rxFifoPeakFillLevel = 3U;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hcan);
    if (can_obj == NULL) return;

    const uint32_t error = HAL_CAN_GetError(hcan);
    can_obj->lastError   = error;
    if ((error & HAL_CAN_ERROR_RX_FOV0) != 0U) {
        can_obj->rxFifoOverrunCount++;
        can_obj->rxFailureCount++;
    }
}
