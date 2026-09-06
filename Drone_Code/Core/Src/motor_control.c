#include "motor_control.h"
#include <stdint.h>

// Initialize motor (assumes TIM1 is already configured for PWM)
void motor_Init(Motor *motor, uint32_t default_pulse_tick, uint32_t min_pulse_tick, uint32_t max_pulse_tick, TIM_HandleTypeDef *htim, uint32_t channel) {
    motor->htim = htim;
    motor->channel = channel;
    motor->default_pulse_tick = default_pulse_tick;
    motor->min_pulse =  min_pulse_tick;
    motor->max_pulse = max_pulse_tick;
}

// Set motor angle (0° to 180°) with instance-specific calibration limits
void servo_SetAngle(Motor *motor, float angle) {
    // 1. Structural Guardrails
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // 2. Calculate pulse width using the instance limits instead of global macros
    uint32_t pulse = motor->min_pulse + (uint32_t)((angle / 180.0f) * (motor->max_pulse - motor->min_pulse));
    
    // 3. Update the hardware register
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pulse);
}

void motor_SetPulse(Motor *motor, uint32_t pulse_ticks)
{
    // Optional: Clamp the pulse value to your safe limits to prevent out-of-bounds register values
    if (pulse_ticks < motor->min_pulse) {
        pulse_ticks = motor->min_pulse;
    } else if (pulse_ticks > motor->max_pulse) {
        pulse_ticks = motor->max_pulse;
    }

    // Update the timer compare register directly
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pulse_ticks);
}

// Start motor PWM
void motor_Start(Motor *motor) {
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
}

// Stop motor PWM
void motor_Stop(Motor *motor) {
    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
}