/**
 ******************************************************************************
 * @file           : tsk_isr.hpp
 * @brief          : 中断服务程序头文件
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "crt_gimbal.hpp"

/* Exported functions prototypes ---------------------------------------------*/
extern "C" {
void dr16RxCallback(uint8_t *Buffer, uint16_t Length);
void can1RxCallback(can_rx_message_t *pRxMsg);
void can2RxCallback(can_rx_message_t *pRxMsg);
} // extern "C"
