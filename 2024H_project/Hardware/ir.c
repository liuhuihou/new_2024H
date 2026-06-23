#include "ir.h"

/*
 * IR sensor pins are configured as pull-up inputs by SysConfig
 * (DH1/DH2 on GPIOA, DH3/DH4 on GPIOB). No runtime init is required.
 *
 * Readings are passed through a per-bit integrate-and-hold glitch filter
 * (see IR_FILTER_N): each sensor has a small up/down counter, and its reported
 * bit only flips after IR_FILTER_N consecutive raw samples agree. This removes
 * the transient "all white" mis-reads that were being mistaken for an arc end
 * (premature stop). IR_Update() must be called once per control tick to clock
 * the filter; IR_Read() returns the filtered state.
 */

/* Per-sensor integrators (0..IR_FILTER_N) and the debounced output bits. */
static uint8_t s_count[4]  = {0, 0, 0, 0};
static uint8_t s_filtered  = 0;   /* stable 4-bit state, MSB = DH1 */

void IR_Init(void)
{
    int i;
    for (i = 0; i < 4; i++)
        s_count[i] = 0;
    s_filtered = 0;
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

/* Read the four sensors once, polarity-corrected, MSB = DH1 (leftmost). */
uint8_t IR_ReadRaw(void)
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
 * Clock the glitch filter with one fresh sample. For each bit: count up toward
 * IR_FILTER_N while raw=black, down toward 0 while raw=white; the reported bit
 * is set only at the top of the count and cleared only at the bottom, giving
 * full hysteresis (a change needs IR_FILTER_N consecutive agreeing samples).
 */
void IR_Update(void)
{
    uint8_t raw = IR_ReadRaw();
    int i;

    for (i = 0; i < 4; i++)
    {
        uint8_t mask = (uint8_t)(1U << (3 - i));   /* bit3 = DH1 */

        if (raw & mask)
        {
            if (s_count[i] < IR_FILTER_N) s_count[i]++;
            if (s_count[i] >= IR_FILTER_N) s_filtered |= mask;   /* confirm black */
        }
        else
        {
            if (s_count[i] > 0) s_count[i]--;
            if (s_count[i] == 0) s_filtered &= (uint8_t)~mask;   /* confirm white */
        }
    }
}

/*
 * Return the filtered 4-bit state, MSB = DH1 (leftmost).
 * bit set => that sensor is debounced "on the black line".
 */
uint8_t IR_Read(void)
{
    return s_filtered;
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
