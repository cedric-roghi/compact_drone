/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include "crsf.h"
#include "servo_control.h"
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
osThreadId_t receiverTaskHandle;
const osThreadAttr_t receiverTask_attributes = {
  .name = "receiverTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t uart_rx_buffer[64];
volatile uint16_t uart_rx_flag = 0;

extern TIM_HandleTypeDef htim1;  // Defined in tim.c

char usb_output_buffer[256];

Servo servo1, servo2;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartReceiverTask(void *argument);
void StartServoTask(void *argument);
void usb_send(const char *format, ...);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes); //Commented out to not have this task called and a thread will not be made

  /* USER CODE BEGIN RTOS_THREADS */
  receiverTaskHandle = osThreadNew(StartReceiverTask, NULL, &receiverTask_attributes);
  servoTaskHandle = osThreadNew(StartServoTask, NULL, &servoTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartServoTask(void *argument)
{
  // Initialize servos
  Servo_Init(&servo1, &htim1, TIM_CHANNEL_1);
  Servo_Init(&servo2, &htim1, TIM_CHANNEL_2);


  Servo_Start(&servo1);
  Servo_Start(&servo2);

  while (1) {
      osDelay(5000);
  }
}

void StartReceiverTask(void *argument)
{
  crsf_frame_t crsf_frame;
  crsf_rc_channels_packed_t rc_channels;
  uint32_t lastToggleTick = osKernelGetTickCount();

  // Wait for USB enum + ELRS init
  usb_send("Waiting for USB enumeration and ELRS init...\r\n");
  osDelay(2000);
  usb_send("Receiver task started with event-driven UART\r\n");

  // Start initial DMA transfer
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  while (1) {
    // Wait indefinitely for a notification from the ISR (0 CPU load while waiting)
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));

    // Handle LED toggling (runs on 1-second timeout or when notified)
    if ((osKernelGetTickCount() - lastToggleTick) >= 1000) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_8);
      lastToggleTick = osKernelGetTickCount();
    }

    // Process incoming packet if notified
    if (notificationValue > 0 && uart_rx_flag > 0) {
      uint16_t rx_size = uart_rx_flag;
      uart_rx_flag = 0;

      // Re-enable DMA immediately
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer));
      __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

      if (crsf_parse_frame(uart_rx_buffer, rx_size, &crsf_frame)) {
        if (crsf_frame.frame_type == CRSF_FRAME_RC_CHANNELS_PACKED) {
          if (crsf_extract_rc_channels(&crsf_frame, &rc_channels)) {
            
            int16_t channel_us[16];
            
            channel_us[0]  = crsf_convert_channel_to_us(rc_channels.channel_01);
            channel_us[1]  = crsf_convert_channel_to_us(rc_channels.channel_02);
            channel_us[2]  = crsf_convert_channel_to_us(rc_channels.channel_03);
            channel_us[3]  = crsf_convert_channel_to_us(rc_channels.channel_04);
            channel_us[4]  = crsf_convert_channel_to_us(rc_channels.channel_05);
            channel_us[5]  = crsf_convert_channel_to_us(rc_channels.channel_06);
            channel_us[6]  = crsf_convert_channel_to_us(rc_channels.channel_07);
            channel_us[7]  = crsf_convert_channel_to_us(rc_channels.channel_08);
            channel_us[8]  = crsf_convert_channel_to_us(rc_channels.channel_09);
            channel_us[9]  = crsf_convert_channel_to_us(rc_channels.channel_10);
            channel_us[10] = crsf_convert_channel_to_us(rc_channels.channel_11);
            channel_us[11] = crsf_convert_channel_to_us(rc_channels.channel_12);
            channel_us[12] = crsf_convert_channel_to_us(rc_channels.channel_13);
            channel_us[13] = crsf_convert_channel_to_us(rc_channels.channel_14);
            channel_us[14] = crsf_convert_channel_to_us(rc_channels.channel_15);
            channel_us[15] = crsf_convert_channel_to_us(rc_channels.channel_16);

            // --- SERVO CONTROL MAPPING CODE ---
            // Map 1000us-2000us input space to a floating point 0.0 to 180.0 degrees
            float servo1_angle = ((float)(channel_us[0] - 1000) / 1000.0f) * 180.0f;
            float servo2_angle = ((float)(channel_us[1] - 1000) / 1000.0f) * 180.0f;

            // Direct hardware update
            Servo_SetAngle(&servo1, servo1_angle);
            Servo_SetAngle(&servo2, servo2_angle);
            // ----------------------------------

            
            usb_send("CH1:%d, CH2:%d, CH3:%d, CH4:%d, CH5:%d, CH6:%d, CH7:%d, CH8:%d, "
            "CH9:%d, CH10:%d, CH11:%d, CH12:%d, CH13:%d, CH14:%d, CH15:%d, CH16:%d\r\n",
            channel_us[0], channel_us[1], channel_us[2], channel_us[3],
            channel_us[4], channel_us[5], channel_us[6], channel_us[7],
            channel_us[8], channel_us[9], channel_us[10], channel_us[11],
            channel_us[12], channel_us[13], channel_us[14], channel_us[15]);

            
          }
        }
      }
    }
    // osDelay(1);  // Keep the task responsive and prevent CPU starvation
  }
}

// This callback triggers automatically when the ELRS receiver stops transmitting data frames
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1) {
    uart_rx_flag = Size;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake up receiver task immediately
    vTaskNotifyGiveFromISR((TaskHandle_t)receiverTaskHandle, &xHigherPriorityTaskWoken);

    // Force context switch if receiver task has a higher priority
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void usb_send(const char *format, ...) {
    va_list args;
    va_start(args, format);

    // Use vsnprintf to format the string into the buffer
    int len = vsnprintf((char *)usb_output_buffer, sizeof(usb_output_buffer), format, args);
    va_end(args);

    // Ensure the buffer is null-terminated and within bounds
    if (len >= (int)sizeof(usb_output_buffer)) {
        len = sizeof(usb_output_buffer) - 1;
        usb_output_buffer[len] = '\0';
    }

    // Append \r\n if there's space
    if (len + 2 < (int)sizeof(usb_output_buffer)) {
        usb_output_buffer[len++] = '\r';
        usb_output_buffer[len++] = '\n';
    } else {
        // Replace last character(s) with \r\n if no space
        usb_output_buffer[sizeof(usb_output_buffer) - 3] = '\r';
        usb_output_buffer[sizeof(usb_output_buffer) - 2] = '\n';
        len = sizeof(usb_output_buffer) - 2;
    }

    // Send the buffer
    CDC_Transmit_FS((uint8_t *)usb_output_buffer, len);
}

/* USER CODE END Application */

