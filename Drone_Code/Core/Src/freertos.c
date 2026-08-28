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
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "crsf.h"
#include "motor_control.h"
#include <stdarg.h>
#include "icm42605.h"
#include "imu_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define fixed update period (e.g., 2ms = 500 Hz loop)
#define IMU_TASK_PERIOD_MS 2
#define DT_SECONDS         (IMU_TASK_PERIOD_MS / 1000.0f)
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

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime, // High priority for deterministic control
};

osQueueId_t imuAttitudeQueueHandle;
osQueueId_t rcCommandQueueHandle;

// Struct for normalized RC Pilot Commands (-1.0 to 1.0 or target angles)
typedef struct {
    float roll_setpoint;   // Target Roll angle (degrees)
    float pitch_setpoint;  // Target Pitch angle (degrees)
    float yaw_rate_setpoint;// Target Yaw rate (deg/s)
    float throttle;        // 0.0 to 1.0 (0% to 100% thrust)
    uint8_t armed;         // Safety switch state
} ControlSetpoint_t;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t uart_rx_buffer[64];
volatile uint16_t uart_rx_flag = 0;

extern TIM_HandleTypeDef htim1;  // Defined in tim.c

extern I2C_HandleTypeDef hi2c1;

ICM42605_t g_imu;
IMU_Attitude_t g_attitude;

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
void startImuTask(void *argument);
void usb_send(const char *format, ...);
void StartPidTask(void *argument);
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
  // Queue depth = 1 (holds only the latest attitude calculation)
  imuAttitudeQueueHandle = osMessageQueueNew(1, sizeof(IMU_Attitude_t), NULL);
  
  // Queue holding only the latest 1 RC Setpoint (length = 1)
  rcCommandQueueHandle = osMessageQueueNew(1, sizeof(ControlSetpoint_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  receiverTaskHandle = osThreadNew(StartReceiverTask, NULL, &receiverTask_attributes);
  servoTaskHandle = osThreadNew(StartServoTask, NULL, &servoTask_attributes);
  imuTaskHandle = osThreadNew(startImuTask, NULL, &imuTask_attributes);
  pidTaskHandle = osThreadNew(StartPidTask, NULL, &pidTask_attributes);
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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
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

  for (;;) {
      osDelay(1000);
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

  for (;;) {
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
            
            /*usb_send("CH1:%d, CH2:%d, CH3:%d, CH4:%d, CH5:%d, CH6:%d, CH7:%d, CH8:%d, "
            "CH9:%d, CH10:%d, CH11:%d, CH12:%d, CH13:%d, CH14:%d, CH15:%d, CH16:%d\r\n",
            channel_us[0], channel_us[1], channel_us[2], channel_us[3],
            channel_us[4], channel_us[5], channel_us[6], channel_us[7],
            channel_us[8], channel_us[9], channel_us[10], channel_us[11],
            channel_us[12], channel_us[13], channel_us[14], channel_us[15]);*/
            
          }
        }
      }
    }
    // osDelay(1);  // Keep the task responsive and prevent CPU starvation
  }
}

void startImuTask(void *argument)
{

  static ICM42605_t g_imu;
  static IMU_Attitude_t g_attitude;
  
  usb_send("IMU Task: Starting hardware init...\r\n");

  int init_status = icm42605_init(&g_imu, &hi2c1, 0x69);
  if (init_status < 0) {
      usb_send("IMU Task Error: icm42605_init failed code %d\r\n", init_status);
      osThreadExit();
  }
  
  imu_math_init(&g_attitude, 0.98f);

  uint32_t tick = osKernelGetTickCount();

  for (;;) {
    tick += pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);
    osDelayUntil(tick);

    if (icm42605_get_agt(&g_imu) == 1) { 
      // 1. Clean up IMU data & update filter
      imu_math_update(&g_attitude,
                      g_imu.acc[0], g_imu.acc[1], g_imu.acc[2],
                      g_imu.gyr[0], g_imu.gyr[1], g_imu.gyr[2],
                      DT_SECONDS);

      // 2. Non-blocking push of updated IMU_Attitude_t to PID task
      // Timeout = 0 overwrites the single item buffer instantly if full
      osMessageQueuePut(imuAttitudeQueueHandle, &g_attitude, 0, 0);
    }
  }
}

void StartPidTask(void *argument)
{
  IMU_Attitude_t attitude;
  ControlSetpoint_t setpoint = {0};

  for (;;) {
    // Blocks task execution until IMU task posts a new IMU_Attitude_t struct
    osStatus_t status = osMessageQueueGet(imuAttitudeQueueHandle, &attitude, NULL, osWaitForever);

    if (status == osOK) {
      // Fetch latest pilot setpoint (non-blocking)
      osMessageQueueGet(rcCommandQueueHandle, &setpoint, NULL, 0);

      if (!setpoint.armed) {
          // Zero outputs if disarmed
          Set_Coaxial_Motors_Thrust(0.0f, 0.0f);
          Set_Servos_Position(0.0f, 0.0f);
          continue;
      }

      // 1. Calculate errors using values directly from your struct
      float roll_error  = setpoint.roll_target  - attitude.roll;
      float pitch_error = setpoint.pitch_target - attitude.pitch;
      float yaw_error   = setpoint.yaw_target   - attitude.yaw;

      // 2. Run PID controllers
      float roll_out  = PID_Update(&pid_roll,  roll_error,  DT_SECONDS);
      float pitch_out = PID_Update(&pid_pitch, pitch_error, DT_SECONDS);
      float yaw_out   = PID_Update(&pid_yaw,   yaw_error,   DT_SECONDS);

      // 3. Coaxial Motor Mixer (Yaw controlled via differential motor torque)
      float motor1_cmd = setpoint.throttle + yaw_out;
      float motor2_cmd = setpoint.throttle - yaw_out;

      // 4. Servo Vectoring Mixer (Roll/Pitch controlled via swashplate or gimbal servos)
      float servo1_cmd = pitch_out + roll_out;
      float servo2_cmd = pitch_out - roll_out;

      // 5. Output to hardware
      Set_Coaxial_Motors_Thrust(motor1_cmd, motor2_cmd);
      Set_Servos_Position(servo1_cmd, servo2_cmd);
    }
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
  // Thread-local stack buffer prevents multi-task race conditions
  char local_buffer[256];

  va_list args;
  va_start(args, format);
  int len = vsnprintf(local_buffer, sizeof(local_buffer), format, args);
  va_end(args);

  if (len <= 0) return;

  // Retry transmission until USB CDC hardware endpoint becomes ready
  uint16_t timeout = 100;
  while (CDC_Transmit_FS((uint8_t *)local_buffer, (uint16_t)len) == USBD_BUSY && timeout > 0) {
      osDelay(1);
      timeout--;
  }
}
/* USER CODE END Application */

