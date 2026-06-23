#ifndef _MPU6050_H
#define _MPU6050_H

#include "ti_msp_dl_config.h"

/*
 * MPU6050 6-axis IMU on hardware I2C0 (PA0=SDA, PA1=SCL).
 *
 * Used here only for heading (yaw): the Z-axis gyro is integrated in the 10 ms
 * control tick into a relative heading angle in degrees. This corrects the
 * "exits the curve at an angle, then blind-drives off the straight" problem:
 * heading is held by closing the loop on yaw instead of on wheel-odometry
 * difference (which is fooled by wheel slip on the arcs).
 *
 *   MPU_Init()     once at boot (after SYSCFG_DL_init), returns 0 on success.
 *   MPU_Calibrate() with the car held still: averages the gyro Z bias.
 *   MPU_UpdateYaw() once per control tick (10 ms): integrates gyro Z.
 *   MPU_GetYaw()   current relative heading (deg, +CCW or +CW per mounting).
 *   MPU_ZeroYaw()  reset the accumulated heading to 0 (new reference).
 */

#define MPU6050_ADDR      0x68    /* AD0 low (default) */

int   MPU_Init(void);            /* 0 = ok, non-zero = not found / I2C error */
void  MPU_Calibrate(uint16_t samples);  /* car must be stationary */
void  MPU_UpdateYaw(void);       /* call every 10 ms tick */
float MPU_GetYaw(void);          /* accumulated heading, degrees */
void  MPU_ZeroYaw(void);         /* set current heading as 0 reference */
float MPU_GetGyroZ(void);        /* latest Z angular rate, deg/s (bias removed) */
int   MPU_IsReady(void);         /* 1 once MPU_Init succeeded */

#endif /* _MPU6050_H */
