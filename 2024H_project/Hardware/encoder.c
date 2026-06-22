#include "encoder.h"

uint32_t gpio_interrup1, gpio_interrup2;
volatile int Get_Encoder_countA, Get_Encoder_countB;

/* Read raw quadrature channel levels. */
static uint8_t Encoder_Left_A(void)  { return (uint8_t)((GPIOA->DIN31_0 >> 25) & 1U); }
static uint8_t Encoder_Left_B(void)  { return (uint8_t)((GPIOA->DIN31_0 >> 26) & 1U); }
static uint8_t Encoder_Right_A(void) { return (uint8_t)((GPIOB->DIN31_0 >> 20) & 1U); }
static uint8_t Encoder_Right_B(void) { return (uint8_t)((GPIOB->DIN31_0 >> 24) & 1U); }

/* GROUP1 interrupt covers both GPIOA (encoder A) and GPIOB (encoder B) edges. */
void GROUP1_IRQHandler(void)
{
    gpio_interrup1 = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    gpio_interrup2 = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    /* Encoder A */
    if ((gpio_interrup1 & ENCODERA_E1A_PIN) == ENCODERA_E1A_PIN)
    {
        if (!Encoder_Left_B()) Get_Encoder_countA--;
        else                   Get_Encoder_countA++;
    }
    else if ((gpio_interrup1 & ENCODERA_E1B_PIN) == ENCODERA_E1B_PIN)
    {
        if (!Encoder_Left_A()) Get_Encoder_countA++;
        else                   Get_Encoder_countA--;
    }

    /* Encoder B */
    if ((gpio_interrup2 & ENCODERB_E2A_PIN) == ENCODERB_E2A_PIN)
    {
        if (!Encoder_Right_B()) Get_Encoder_countB--;
        else                    Get_Encoder_countB++;
    }
    else if ((gpio_interrup2 & ENCODERB_E2B_PIN) == ENCODERB_E2B_PIN)
    {
        if (!Encoder_Right_A()) Get_Encoder_countB++;
        else                    Get_Encoder_countB--;
    }

    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}

float Calculate_Motor_RPM(int encoder_count, int sample_time_ms)
{
    const int ENCODER_LINES   = 13;
    const int MULTIPLY_FACTOR = 2;
    const int GEAR_RATIO      = 30;
    int pulses_per_revolution = ENCODER_LINES * MULTIPLY_FACTOR;   /* 26 */
    float motor_rpm = (float)encoder_count * 60000.0f /
                      (float)(pulses_per_revolution * sample_time_ms);
    return motor_rpm / (float)GEAR_RATIO;
}
