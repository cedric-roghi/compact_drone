#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f4xx_hal.h"

// Servo pulse width in timer ticks (Configured for a 1 MHz timer clock)
// This gives a 1 microsecond resolution per tick, yielding 1000 steps of precision!
#define SERVO_MIN_PULSE_TICKS 500   // 1.0ms pulse (0 degrees)
#define SERVO_MID_PULSE_TICKS 1500   // 1.5ms pulse (90 degrees)
#define SERVO_MAX_PULSE_TICKS 2500   // 2.0ms pulse (180 degrees)

// Servo structure
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t min_pulse;
    uint32_t max_pulse;
} Servo;

// Function prototypes
void Servo_Init(Servo *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(Servo *servo, float angle); // Angle from 0.0 to 180.0
void Servo_Start(Servo *servo);
void Servo_Stop(Servo *servo);

#endif // MOTOR_CONTROL_H