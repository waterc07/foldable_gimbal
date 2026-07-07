/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for gimbal */
osThreadId_t gimbalHandle;
uint32_t gimbalBuffer[ 1024 ];
osStaticThreadDef_t gimbalControlBlock;
const osThreadAttr_t gimbal_attributes = {
  .name = "gimbal",
  .cb_mem = &gimbalControlBlock,
  .cb_size = sizeof(gimbalControlBlock),
  .stack_mem = &gimbalBuffer[0],
  .stack_size = sizeof(gimbalBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imu */
osThreadId_t imuHandle;
uint32_t imuBuffer[ 128 ];
osStaticThreadDef_t imuControlBlock;
const osThreadAttr_t imu_attributes = {
  .name = "imu",
  .cb_mem = &imuControlBlock,
  .cb_size = sizeof(imuControlBlock),
  .stack_mem = &imuBuffer[0],
  .stack_size = sizeof(imuBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vofa */
osThreadId_t vofaHandle;
uint32_t vofaBuffer[ 128 ];
osStaticThreadDef_t vofaControlBlock;
const osThreadAttr_t vofa_attributes = {
  .name = "vofa",
  .cb_mem = &vofaControlBlock,
  .cb_size = sizeof(vofaControlBlock),
  .stack_mem = &vofaBuffer[0],
  .stack_size = sizeof(vofaBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for usb */
osThreadId_t usbHandle;
uint32_t usbBuffer[ 128 ];
osStaticThreadDef_t usbControlBlock;
const osThreadAttr_t usb_attributes = {
  .name = "usb",
  .cb_mem = &usbControlBlock,
  .cb_size = sizeof(usbControlBlock),
  .stack_mem = &usbBuffer[0],
  .stack_size = sizeof(usbBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void gimbal_task(void *argument);
extern void imu_task(void *argument);
extern void vofa_task(void *argument);
extern void usb_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of gimbal */
  gimbalHandle = osThreadNew(gimbal_task, NULL, &gimbal_attributes);

  /* creation of imu */
  imuHandle = osThreadNew(imu_task, NULL, &imu_attributes);

  /* creation of vofa */
  vofaHandle = osThreadNew(vofa_task, NULL, &vofa_attributes);

  /* creation of usb */
  usbHandle = osThreadNew(usb_task, NULL, &usb_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_gimbal_task */
/**
  * @brief  Function implementing the gimbal thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

