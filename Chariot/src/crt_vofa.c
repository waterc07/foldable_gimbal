/**
 ******************************************************************************
 * @file           : crt_vofa.c
 * @brief          : vofa串口调试助手
 *                   重定向串口输出到UART6
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include "usart.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/**
 * @brief 重定向printf输出到UART6
 * @param file 文件描述符
 * @param ptr 要发送的数据指针
 * @param len 数据长度
 * @return 实际发送的字节数
 */
int _write(int file, char *ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        // 将数据通过UART6发送
        HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, HAL_MAX_DELAY);
        // HAL_UART_Transmit_DMA(&huart6, (uint8_t *)ptr, len);
        return len;
    }
    return -1;
}
