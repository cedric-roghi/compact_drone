#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f4xx_hal.h"

// Motor pulse width in timer ticks (Configured for a 1 MHz timer clock)
// This gives a 1 microsecond resolution per tick, yielding 1000 steps of precision!
#define SERVO_MIN_PULSE_TICKS 1200   // 60 degrees (90° - 30°)
#define SERVO_MID_PULSE_TICKS 1500   // 90 degrees (Center)
#define SERVO_MAX_PULSE_TICKS 1800   // 120 degrees (90° + 30°)

#define ESC_MIN_PULSE_TICKS 16000   // 1.0ms pulse (0%)
#define ESC_MAX_PULSE_TICKS 32000   // 2.0ms pulse (100%)

// Motor structureS
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t default_pulse_tick;
    uint32_t min_pulse;
    uint32_t max_pulse;
} Motor;

// Function prototypes
void motor_Init(Motor *motor, uint32_t default_pulse_tick, uint32_t min_pulse_tick, uint32_t max_pulse_tick, TIM_HandleTypeDef *htim, uint32_t channel);
void motor_SetAngle(Motor *motor, float angle); // Angle from 0.0 to 180.0
void motor_SetPulse(Motor *motor, uint32_t pulse_ticks);
void motor_Start(Motor *motor);
void motor_Stop(Motor *motor);

#endif // MOTOR_CONTROL_H