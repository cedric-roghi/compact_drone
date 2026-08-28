#include "motor_control.h"

// Initialize servo (assumes TIM1 is already configured for PWM)
void Servo_Init(Servo *servo, TIM_HandleTypeDef *htim, uint32_t channel) {
    servo->htim = htim;
    servo->channel = channel;
    servo->min_pulse = SERVO_MIN_PULSE_TICKS;
    servo->max_pulse = SERVO_MAX_PULSE_TICKS;
}

// Set servo angle (0° to 180°) with instance-specific calibration limits
void Servo_SetAngle(Servo *servo, float angle) {
    // 1. Structural Guardrails
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // 2. Calculate pulse width using the instance limits instead of global macros
    uint32_t pulse = servo->min_pulse + (uint32_t)((angle / 180.0f) * (servo->max_pulse - servo->min_pulse));
    
    // 3. Update the hardware register
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}

// Start servo PWM
void Servo_Start(Servo *servo) {
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

// Stop servo PWM
void Servo_Stop(Servo *servo) {
    HAL_TIM_PWM_Stop(servo->htim, servo->channel);
}