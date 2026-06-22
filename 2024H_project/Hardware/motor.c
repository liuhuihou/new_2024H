#include "motor.h"

int limit_PWM(int value, int low, int high)
{
    if (value > high) return high;
    if (value < low)  return low;
    return value;
}

/* Hold both wheels stopped and direction pins low. */
static void Motor_Coast(void)
{
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
    DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
}

/*
 * Apply forward PWM duty to each wheel. Negative inputs are clamped to 0
 * (the car is forward-only). Direction pins are held low so the PWM half of
 * the H-bridge drives the wheel forward.
 */
void Set_PWM(int pwmLeft, int pwmRight)
{
    pwmLeft  = limit_PWM(pwmLeft,  0, PWM_MAX);
    pwmRight = limit_PWM(pwmRight, 0, PWM_MAX);

    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
    DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);

    DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)pwmLeft,  GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)pwmRight, GPIO_PWM_0_C1_IDX);

    if (pwmLeft == 0 && pwmRight == 0)
    {
        Motor_Coast();
    }
}
