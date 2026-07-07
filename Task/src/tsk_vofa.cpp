/**
 ******************************************************************************
 * @file           : tsk_vofa.cpp
 * @brief          : Reserved debug telemetry task
 ******************************************************************************
 */

#include "crt_gimbal.hpp"

extern "C" void vofa_task(void *argument)
{
    TickType_t taskLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&taskLastWakeTime, pdMS_TO_TICKS(10));
    }
}
