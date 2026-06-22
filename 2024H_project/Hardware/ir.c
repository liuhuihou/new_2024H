#include "ir.h"

/*
 * IR sensor pins are configured as pull-up inputs by SysConfig
 * (DH1/DH2 on GPIOA, DH3/DH4 on GPIOB). No runtime init is required, but
 * IR_Init() is provided for symmetry and future use.
 */
void IR_Init(void)
{
    /* nothing to do: pins set up in SYSCFG_DL_GPIO_init */
}

/* Return 1 if the given sensor is over black, applying the polarity option. */
static uint8_t ir_sensor_active(uint32_t raw_level)
{
#if IR_BLACK_IS_HIGH
    return raw_level ? 1U : 0U;
#else
    return raw_level ? 0U : 1U;
#endif
}

/*
 * Read all four sensors into a 4-bit state, MSB = DH1 (leftmost).
 * bit set => that sensor is on the black line.
 */
uint8_t IR_Read(void)
{
    uint32_t dh1 = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN);
    uint32_t dh2 = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN);
    uint32_t dh3 = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN);
    uint32_t dh4 = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN);

    uint8_t s = 0;
    s |= (uint8_t)(ir_sensor_active(dh1) << 3);
    s |= (uint8_t)(ir_sensor_active(dh2) << 2);
    s |= (uint8_t)(ir_sensor_active(dh3) << 1);
    s |= (uint8_t)(ir_sensor_active(dh4) << 0);
    return s;
}

/*
 * Weighted line-position error. Positive = line is to the RIGHT of center
 * (car must steer right). Sensor weights, left to right: -3 -1 +1 +3.
 * Returns the average weight of the active sensors; 0 when centered (0b0110)
 * or when no sensor is active (caller should check IR_AllWhite separately).
 */
int IR_Error(void)
{
    static const int weight[4] = { -3, -1, +1, +3 };  /* DH1..DH4 */
    uint8_t s = IR_Read();
    int sum = 0;
    int n   = 0;

    for (int i = 0; i < 4; i++)
    {
        if (s & (1U << (3 - i)))   /* bit 3 = DH1 = weight[0] */
        {
            sum += weight[i];
            n++;
        }
    }
    if (n == 0)
    {
        return 0;   /* line lost; handled by state machine via IR_AllWhite */
    }
    return sum / n;
}

uint8_t IR_OnLine(void)   { return (IR_Read() != 0x00) ? 1U : 0U; }
uint8_t IR_AllWhite(void) { return (IR_Read() == 0x00) ? 1U : 0U; }
uint8_t IR_AllBlack(void) { return (IR_Read() == 0x0F) ? 1U : 0U; }
