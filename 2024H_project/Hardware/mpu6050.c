#include "mpu6050.h"
#include "i2c.h"
#include "board.h"   /* delay_ms */

/*
 * Minimal MPU6050 driver: wake the device, set gyro full-scale to +-500 dps,
 * then integrate the Z-axis rate into a heading angle. Only yaw is needed for
 * the line-follow heading hold, so accel/temperature are not read.
 *
 * Registers used:
 *   0x6B PWR_MGMT_1    : 0x00 wakes the device, uses internal 8 MHz osc
 *   0x1B GYRO_CONFIG   : FS_SEL in bits[4:3]; 0x08 = +-500 dps
 *   0x1A CONFIG        : DLPF; 0x03 = ~44 Hz gyro bandwidth (less noise)
 *   0x19 SMPLRT_DIV    : sample-rate divider
 *   0x75 WHO_AM_I      : returns 0x68 on a genuine MPU6050
 *   0x47 GYRO_ZOUT_H/L : signed 16-bit Z angular rate
 */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75
#define REG_GYRO_ZOUT_H   0x47

/* +-500 dps full scale -> 65.5 LSB per deg/s (datasheet). */
#define GYRO_LSB_PER_DPS  65.5f
#define TICK_DT_S         0.010f   /* control tick = 10 ms */

static int   s_ready     = 0;
static float s_bias_z    = 0.0f;   /* gyro Z zero offset, raw LSB */
static float s_rate_z    = 0.0f;   /* latest rate, deg/s */
static float s_yaw       = 0.0f;   /* integrated heading, deg */

/* Read the signed 16-bit Z gyro register pair. Returns 0 on success. */
static int read_gyro_z_raw(int16_t *out)
{
    uint8_t b[2];
    if (I2C_ReadRegs(MPU6050_ADDR, REG_GYRO_ZOUT_H, b, 2) != 0)
        return 1;
    *out = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    return 0;
}

int MPU_Init(void)
{
    uint8_t who = 0;

    s_ready  = 0;
    s_bias_z = 0.0f;
    s_rate_z = 0.0f;
    s_yaw    = 0.0f;

    /* Wake from sleep (clears the default SLEEP bit). */
    if (I2C_WriteReg(MPU6050_ADDR, REG_PWR_MGMT_1, 0x00) != 0)
        return 1;
    delay_ms(50);

    if (I2C_ReadReg(MPU6050_ADDR, REG_WHO_AM_I, &who) != 0)
        return 2;
    if (who != 0x68)
        return 3;

    /* ~44 Hz DLPF, 1 kHz base -> SMPLRT_DIV 0x09 = 100 Hz sample rate. */
    if (I2C_WriteReg(MPU6050_ADDR, REG_CONFIG, 0x03) != 0)      return 4;
    if (I2C_WriteReg(MPU6050_ADDR, REG_SMPLRT_DIV, 0x09) != 0)  return 5;
    /* Gyro +-500 dps. */
    if (I2C_WriteReg(MPU6050_ADDR, REG_GYRO_CONFIG, 0x08) != 0) return 6;

    s_ready = 1;
    return 0;
}

void MPU_Calibrate(uint16_t samples)
{
    int32_t acc = 0;
    uint16_t got = 0;
    uint16_t i;
    int16_t raw;

    if (!s_ready || samples == 0) return;

    for (i = 0; i < samples; i++)
    {
        if (read_gyro_z_raw(&raw) == 0)
        {
            acc += raw;
            got++;
        }
        delay_ms(2);
    }
    if (got > 0)
        s_bias_z = (float)acc / (float)got;

    s_yaw = 0.0f;   /* fresh reference after calibration */
}

void MPU_UpdateYaw(void)
{
    int16_t raw;

    if (!s_ready) return;
    if (read_gyro_z_raw(&raw) != 0) return;   /* skip this tick on I2C glitch */

    s_rate_z = ((float)raw - s_bias_z) / GYRO_LSB_PER_DPS;   /* deg/s */
    s_yaw   += s_rate_z * TICK_DT_S;                          /* integrate */
}

float MPU_GetYaw(void)    { return s_yaw; }
void  MPU_ZeroYaw(void)   { s_yaw = 0.0f; }
float MPU_GetGyroZ(void)  { return s_rate_z; }
int   MPU_IsReady(void)   { return s_ready; }
