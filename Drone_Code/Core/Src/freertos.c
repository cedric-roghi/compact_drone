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
#include "stm32f4xx_hal.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "tim.h"
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

#define RED_LED_PIN    GPIO_PIN_7
#define GREEN_LED_PIN  GPIO_PIN_8
#define LED_PORT       GPIOC

#define MAX_PITCH 30 // degrees absolute value so -30 to 30
#define MAX_ROLL 30  // degrees absolute value so -30 to 30
#define MAX_YAW 5    // degrees per second
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t receiverTaskHandle;
const osThreadAttr_t receiverTask_attributes = {
  .name = "receiverTask",
  .stack_size = 1024 * 4,
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
  .priority = (osPriority_t) osPriorityNormal, // Elevated to prevent priority inversion
};

osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal, // High priority for deterministic control
};

osMessageQueueId_t imuAttitudeQueueHandle;
osMessageQueueId_t rcCommandQueueHandle;

// Struct for normalized RC Pilot Commands (-1.0 to 1.0 or target angles)
typedef struct {
    float roll_setpoint;     // Target Roll angle (degrees)
    float pitch_setpoint;    // Target Pitch angle (degrees)
    float yaw_rate_setpoint; // Target Yaw rate (deg/s)
    float throttle;          // 0.0 to 1.0 (0% to 100% thrust)
    uint8_t armed;           // Safety switch state
} ControlSetpoint_t;

volatile uint32_t g_last_rc_tick = 0;

#define RC_FAILSAFE_TIMEOUT_MS 500U

volatile uint32_t g_uart_error_count = 0;

#define ROLL_KP   16.0f
#define ROLL_KI   0.0f
#define ROLL_KD   0.0f

#define PITCH_KP  16.0f
#define PITCH_KI  0.0f
#define PITCH_KD  0.0f

#define YAW_KP    2.0f
#define YAW_KI    0.0f
#define YAW_KD    0.0f

#define SERVO_PID_OUTPUT_LIMIT  500.0f
#define YAW_PID_OUTPUT_LIMIT    500.0f

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_limit;
    uint8_t initialized;
} PID_Controller_t;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern ADC_HandleTypeDef hadc1;

uint8_t uart_rx_buffer[64];
volatile uint16_t uart_rx_flag = 0;

volatile uint32_t g_shared_voltage_mv = 11100;
volatile uint8_t g_shared_battery_pct = 100;

volatile float g_yaw_trim = 0.0f; // Trim adjustment between -1.0 and 1.0 to balance motor

extern TIM_HandleTypeDef htim1, htim2;  // Defined in tim.c

extern I2C_HandleTypeDef hi2c1;

ICM42605_t g_imu;
IMU_Attitude_t g_attitude;

char usb_output_buffer[256];

Motor spitch, sroll, rotorup, rotordown; // RotorUp is CCW, RotorDown is CW
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartReceiverTask(void *argument);
void StartServoTask(void *argument);
void startImuTask(void *argument);
void usb_send(const char *format, ...);
void StartPidTask(void *argument);
static float map_float(float x, float in_min, float in_max, float out_min, float out_max);
static float pid_compute(PID_Controller_t *pid, float error, float dt);
static void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, float output_limit);
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
  MX_USB_DEVICE_Init();
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
  osDelay(2500); // Wait for USB enumeration
  uint32_t lastBlinkTick = osKernelGetTickCount();

  // Ensure LEDs start OFF by default (HIGH for active-low)
  HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);

  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

      float battery_voltage = ((float)adc_value / 4095.0f) * 3.3f * (133.0f / 33.0f);
      uint32_t voltage_mv = (uint32_t)(battery_voltage * 1000.0f);

      usb_send("Battery Voltage: %lu.%luV (ADC: %lu)\r\n", voltage_mv / 1000, voltage_mv % 1000, adc_value);

      uint8_t battery_pct = 0;
      if (voltage_mv >= 12600) {
        battery_pct = 100;
      } else if (voltage_mv <= 9900) {
        battery_pct = 0;
      } else {
        battery_pct = (uint8_t)(((float)(voltage_mv - 9900) / (12600.0f - 9900.0f)) * 100.0f);
      }

      g_shared_voltage_mv = voltage_mv;
      g_shared_battery_pct = battery_pct;

      if (adc_value < 3103) {
        if ((osKernelGetTickCount() - lastBlinkTick) >= 250) {
          HAL_GPIO_TogglePin(LED_PORT, RED_LED_PIN);
          lastBlinkTick = osKernelGetTickCount();
        }
      } else {
        // Voltage is safe: force Red LED completely OFF (SET = HIGH = OFF)
        HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        lastBlinkTick = osKernelGetTickCount();
      }
    }
    HAL_ADC_Stop(&hadc1);

    osDelay(1000);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartServoTask(void *argument)
{
  // Initialize servos
  motor_Init(&spitch, SERVO_MID_PULSE_TICKS, SERVO_MIN_PULSE_TICKS, SERVO_MAX_PULSE_TICKS, &htim1, TIM_CHANNEL_1);
  motor_Init(&sroll, SERVO_MID_PULSE_TICKS, SERVO_MIN_PULSE_TICKS, SERVO_MAX_PULSE_TICKS, &htim1, TIM_CHANNEL_2);

  // Initialize ESCs
  motor_Init(&rotorup, ESC_MIN_PULSE_TICKS, ESC_MIN_PULSE_TICKS, ESC_MAX_PULSE_TICKS, &htim2, TIM_CHANNEL_1);
  motor_Init(&rotordown,  ESC_MIN_PULSE_TICKS, ESC_MIN_PULSE_TICKS, ESC_MAX_PULSE_TICKS, &htim2, TIM_CHANNEL_2);

  motor_Start(&spitch);
  motor_Start(&sroll);
  motor_Start(&rotorup);
  motor_Start(&rotordown);

  for (;;) {
      osDelay(1000);
  }
}

void StartReceiverTask(void *argument)
{
    crsf_frame_t crsf_frame;
    crsf_rc_channels_packed_t rc_channels;
    uint32_t last_green_toggle_tick = 0;
    static uint32_t telemetry_counter = 0;

    osDelay(3000);
    usb_send("Receiver task started and active\r\n");

    // Green LED OFF
    HAL_GPIO_WritePin(LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);

    // Start UART DMA reception
    HAL_StatusTypeDef uart_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    if (uart_status != HAL_OK)
    {
        usb_send("ERROR: Initial UART RX start failed: %d\r\n", uart_status);
    }

    g_last_rc_tick = osKernelGetTickCount();

    for (;;)
    {
        uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        uint32_t now = osKernelGetTickCount();
        if ((now - g_last_rc_tick) > pdMS_TO_TICKS(RC_FAILSAFE_TIMEOUT_MS))
        {
            HAL_GPIO_WritePin(LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
            last_green_toggle_tick = 0;
        }

        if ((notificationValue > 0) && (uart_rx_flag > 0))
        {
            uint16_t rx_size = uart_rx_flag;
            uart_rx_flag = 0;

            // Inside StartReceiverTask loop:
            if (huart1.RxState != HAL_UART_STATE_READY)
            {
                HAL_UART_AbortReceive(&huart1);
            }

            uart_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, sizeof(uart_rx_buffer));
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

            if (uart_status != HAL_OK)
            {
                usb_send("ERROR: UART RX restart failed: %d\r\n", uart_status);
            }

            if (crsf_parse_frame(uart_rx_buffer, rx_size, &crsf_frame))
            {
                g_last_rc_tick = osKernelGetTickCount();

                if ((g_last_rc_tick - last_green_toggle_tick) >= pdMS_TO_TICKS(250))
                {
                    HAL_GPIO_TogglePin(LED_PORT, GREEN_LED_PIN);
                    last_green_toggle_tick = g_last_rc_tick;
                }

                telemetry_counter++;
                if (telemetry_counter >= 64)
                {
                    telemetry_counter = 0;
                    uint8_t tx_buf[16];
                    uint8_t tx_size = 0;

                    if (crsf_generate_battery_frame((uint16_t)g_shared_voltage_mv, 0, 0, g_shared_battery_pct, tx_buf, &tx_size))
                    {
                        HAL_UART_Transmit(&huart1, tx_buf, tx_size, 10);
                    }
                }

                if (crsf_frame.frame_type == CRSF_FRAME_RC_CHANNELS_PACKED)
                {
                    if (crsf_extract_rc_channels(&crsf_frame, &rc_channels))
                    {
                        ControlSetpoint_t new_cmd;

                        new_cmd.roll_setpoint = map_float((float)rc_channels.channel_01, 172.0f, 1811.0f, -(float)MAX_ROLL, (float)MAX_ROLL);
                        new_cmd.pitch_setpoint = map_float((float)rc_channels.channel_02, 172.0f, 1811.0f, -(float)MAX_PITCH, (float)MAX_PITCH);
                        new_cmd.throttle = map_float((float)rc_channels.channel_03, 172.0f, 1811.0f, 0.0f, 1.0f);
                        new_cmd.yaw_rate_setpoint = map_float((float)rc_channels.channel_04, 172.0f, 1811.0f, -(float)MAX_YAW, (float)MAX_YAW);
                        new_cmd.armed = (rc_channels.channel_07 > 1500) ? 1 : 0;

                        if (osMessageQueuePut(rcCommandQueueHandle, &new_cmd, 0, 0) != osOK)
                        {
                            ControlSetpoint_t discarded;
                            osMessageQueueGet(rcCommandQueueHandle, &discarded, NULL, 0);
                            osMessageQueuePut(rcCommandQueueHandle, &new_cmd, 0, 0);
                        }
                    }
                }
            }
        }
    }
}

void startImuTask(void *argument)
{
    osDelay(1000);
    usb_send("IMU Task: Starting hardware init...\r\n");

    // Initialize IMU
    int init_status = icm42605_init(&g_imu, &hi2c1, 0x69);
    if (init_status < 0)
    {
        usb_send("IMU Task Error: icm42605_init failed code %d\r\n", init_status);
        osThreadExit();
    }

    usb_send("IMU initialized successfully\r\n");

    // Initialize attitude filter
    imu_math_init(&g_attitude, 0.98f);

    // Fixed-rate timing
    TickType_t tick = xTaskGetTickCount();
    uint32_t print_counter = 0;

    for (;;)
    {
        tick += pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));

        // Read IMU
        if (icm42605_get_agt(&g_imu) == 1)
        {
            imu_math_update(&g_attitude, g_imu.acc[0], g_imu.acc[1], g_imu.acc[2], g_imu.gyr[0], g_imu.gyr[1], g_imu.gyr[2], DT_SECONDS);

            // Debug output
            print_counter++;
            if (print_counter >= 100)
            {
                print_counter = 0;
                usb_send("Roll: %d | Pitch: %d | Yaw: %d\r\n", (int)g_attitude.roll, (int)g_attitude.pitch, (int)g_attitude.yaw);
            }

            // Send latest attitude
            if (osMessageQueuePut(imuAttitudeQueueHandle, &g_attitude, 0, 0) != osOK)
            {
                // Queue depth = 1. Discard old attitude and replace it with newest.
                IMU_Attitude_t discarded;
                osMessageQueueGet(imuAttitudeQueueHandle, &discarded, NULL, 0);
                osMessageQueuePut(imuAttitudeQueueHandle, &g_attitude, 0, 0);
            }
        }
    }
}

void StartPidTask(void *argument)
{
    IMU_Attitude_t current_attitude;
    ControlSetpoint_t current_setpoint = {
        .roll_setpoint = 0.0f,
        .pitch_setpoint = 0.0f,
        .yaw_rate_setpoint = 0.0f,
        .throttle = 0.0f,
        .armed = 0
    };

    // PID controllers
    PID_Controller_t roll_pid;
    PID_Controller_t pitch_pid;
    PID_Controller_t yaw_pid;

    pid_init(&roll_pid, ROLL_KP, ROLL_KI, ROLL_KD, SERVO_PID_OUTPUT_LIMIT);
    pid_init(&pitch_pid, PITCH_KP, PITCH_KI, PITCH_KD, SERVO_PID_OUTPUT_LIMIT);
    pid_init(&yaw_pid, YAW_KP, YAW_KI, YAW_KD, YAW_PID_OUTPUT_LIMIT);

    usb_send("PID Task started\r\n");

    for (;;)
    {
      static uint32_t actuator_divider = 0;
        // Wait for the newest IMU sample.
        if (osMessageQueueGet(imuAttitudeQueueHandle, &current_attitude, NULL, portMAX_DELAY) != osOK)
        {
            continue;
        }

        // Get newest RC command
        osMessageQueueGet(rcCommandQueueHandle, &current_setpoint, NULL, 0);

        // HARD RC FAILSAFE
        uint32_t now = osKernelGetTickCount();
        if ((now - g_last_rc_tick) > pdMS_TO_TICKS(RC_FAILSAFE_TIMEOUT_MS))
        {
            current_setpoint.armed = 0;
            current_setpoint.throttle = 0.0f;
            current_setpoint.roll_setpoint = 0.0f;
            current_setpoint.pitch_setpoint = 0.0f;
            current_setpoint.yaw_rate_setpoint = 0.0f;
        }

        // DISARMED STATE
        if (!current_setpoint.armed)
        {
            motor_SetPulse(&rotorup, ESC_MIN_PULSE_TICKS);
            motor_SetPulse(&rotordown, ESC_MIN_PULSE_TICKS);
            motor_SetPulse(&spitch, SERVO_MID_PULSE_TICKS);
            motor_SetPulse(&sroll, SERVO_MID_PULSE_TICKS);
            // Reset PID state.
            roll_pid.integral = 0.0f;
            pitch_pid.integral = 0.0f;
            yaw_pid.integral = 0.0f;
            roll_pid.prev_error = 0.0f;
            pitch_pid.prev_error = 0.0f;
            yaw_pid.prev_error = 0.0f;
            roll_pid.initialized = 0;
            pitch_pid.initialized = 0;
            yaw_pid.initialized = 0;

            continue;
        }

        // Calculate attitude errors
        float roll_error = current_setpoint.roll_setpoint - current_attitude.roll;
        float pitch_error = current_setpoint.pitch_setpoint - current_attitude.pitch;

        // Roll / pitch PID
        float roll_correction = pid_compute(&roll_pid, roll_error, DT_SECONDS);
        float pitch_correction = pid_compute(&pitch_pid, pitch_error, DT_SECONDS);

        // Yaw rate using direct gyroscope Z-axis reading to avoid filter delay and noise amplification
        float yaw_error = current_setpoint.yaw_rate_setpoint - g_imu.gyr[2];
        float yaw_correction = pid_compute(&yaw_pid, yaw_error, DT_SECONDS);

        // Calculate throttle
        if (current_setpoint.throttle < 0.0f) current_setpoint.throttle = 0.0f;
        if (current_setpoint.throttle > 1.0f) current_setpoint.throttle = 1.0f;

        uint32_t base_throttle_ticks;
        int32_t up_motor_ticks;
        int32_t down_motor_ticks;

        if (current_setpoint.throttle < 0.01f)
        {
            // Force absolute minimum idle pulse when throttle is zero
            base_throttle_ticks = ESC_MIN_PULSE_TICKS;
            up_motor_ticks = ESC_MIN_PULSE_TICKS;
            down_motor_ticks = ESC_MIN_PULSE_TICKS;
        }
        else
        {
            base_throttle_ticks = ESC_MIN_PULSE_TICKS + (uint32_t)(current_setpoint.throttle * (float)(ESC_MAX_PULSE_TICKS - ESC_MIN_PULSE_TICKS));

            // Yaw motor mixing
            float trim_offset = g_yaw_trim * 50.0f;
            up_motor_ticks = (int32_t)base_throttle_ticks - (int32_t)yaw_correction + (int32_t)trim_offset;
            down_motor_ticks = (int32_t)base_throttle_ticks + (int32_t)yaw_correction - (int32_t)trim_offset;
        }

        // Clamp motors
        if (up_motor_ticks < (int32_t)ESC_MIN_PULSE_TICKS) up_motor_ticks = ESC_MIN_PULSE_TICKS;
        if (up_motor_ticks > (int32_t)ESC_MAX_PULSE_TICKS) up_motor_ticks = ESC_MAX_PULSE_TICKS;
        if (down_motor_ticks < (int32_t)ESC_MIN_PULSE_TICKS) down_motor_ticks = ESC_MIN_PULSE_TICKS;
        if (down_motor_ticks > (int32_t)ESC_MAX_PULSE_TICKS) down_motor_ticks = ESC_MAX_PULSE_TICKS;

        // Servo output
        int32_t pitch_servo_ticks = SERVO_MID_PULSE_TICKS + (int32_t)pitch_correction;
        int32_t roll_servo_ticks = SERVO_MID_PULSE_TICKS + (int32_t)roll_correction;

        // Clamp servos
        if (pitch_servo_ticks < (int32_t)SERVO_MIN_PULSE_TICKS) pitch_servo_ticks = SERVO_MIN_PULSE_TICKS;
        if (pitch_servo_ticks > (int32_t)SERVO_MAX_PULSE_TICKS) pitch_servo_ticks = SERVO_MAX_PULSE_TICKS;
        if (roll_servo_ticks < (int32_t)SERVO_MIN_PULSE_TICKS) roll_servo_ticks = SERVO_MIN_PULSE_TICKS;
        if (roll_servo_ticks > (int32_t)SERVO_MAX_PULSE_TICKS) roll_servo_ticks = SERVO_MAX_PULSE_TICKS;

        // Output actuators at 50 Hz (every 20 ms / 10 IMU loops)
        if (++actuator_divider >= 10)
        {
            actuator_divider = 0;
            motor_SetPulse(&rotorup, (uint32_t)up_motor_ticks);
            motor_SetPulse(&rotordown, (uint32_t)down_motor_ticks);
            motor_SetPulse(&spitch, (uint32_t)pitch_servo_ticks);
            motor_SetPulse(&sroll, (uint32_t)roll_servo_ticks);
        }
    }
}

// This callback triggers automatically when data reception completes or line goes idle
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
  char local_buffer[256];

  va_list args;
  va_start(args, format);
  int len = vsnprintf(local_buffer, sizeof(local_buffer), format, args);
  va_end(args);

  if (len <= 0) return;

  // Non-blocking transmission attempt: drops packet instantly if USB is busy
  CDC_Transmit_FS((uint8_t *)local_buffer, (uint16_t)len);
}

// PID computation function
static float pid_compute(PID_Controller_t *pid, float error, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    // Proportional
    float p = pid->kp * error;

    // Derivative
    float derivative = 0.0f;
    
    // Prevent derivative kick on startup
    if (pid->initialized)
    {
      derivative = (error - pid->prev_error) / dt;
    }
    else
    {
      pid->initialized = 1;
    }
    float d = pid->kd * derivative;

    // Integral
    pid->integral += error * dt;
    float i = pid->ki * pid->integral;

    // Limit integral contribution
    if (i > pid->output_limit)
    {
      i = pid->output_limit;
      if (pid->ki != 0.0f) pid->integral = pid->output_limit / pid->ki;
    }
    else if (i < -pid->output_limit)
    {
      i = -pid->output_limit;
      if (pid->ki != 0.0f) pid->integral = -pid->output_limit / pid->ki;
    }

    // Save error
    pid->prev_error = error;

    // Total output
    float output = p + i + d;

    // Output saturation
    if (output > pid->output_limit) output = pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

static void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = output_limit;
    pid->initialized = 0;
}

// Helper function for linear mapping
static float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* USER CODE END Application */