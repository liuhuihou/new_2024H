#ifndef _CONTROL_H
#define _CONTROL_H

#include "ti_msp_dl_config.h"

/*
 * Cascaded controller running in the 10 ms TIMG0 tick.
 *   Outer loop (mode-dependent) -> left/right target RPM difference
 *   Inner loop  (per wheel PI)  -> PWM duty
 *
 * Modes:
 *   CTRL_STOP     : both wheels held at 0.
 *   CTRL_STRAIGHT : drive forward at base speed, hold heading by equalizing
 *                   left/right distance (encoder dead-reckoning, no line).
 *   CTRL_LINE     : follow the black arc using the 4-way IR PD error, with an
 *                   optional curvature feed-forward bias.
 */
typedef enum
{
    CTRL_STOP = 0,
    CTRL_STRAIGHT,
    CTRL_LINE
} ControlMode;

/* Runtime-tunable gain identifiers (for UART live tuning, see Control/tune.c). */
typedef enum
{
    GAIN_KP_SPEED = 0,
    GAIN_KI_SPEED,
    GAIN_FF_LEFT,
    GAIN_FF_RIGHT,
    GAIN_KP_HEADING,
    GAIN_KD_HEADING,
    GAIN_KP_LINE,
    GAIN_KD_LINE,
    GAIN_COUNT
} ControlGain;

/* Called once from main() after SYSCFG_DL_init(). */
void Control_Init(void);

/* The 10 ms control tick. Defined as TIMER_0_INST_IRQHandler in control.c;
 * also exposed so it can be unit-stepped if ever needed. */
void Control_Tick(void);

/* Mode + tuning setters (safe to call from the foreground loop). */
void  Control_SetMode(ControlMode mode);
ControlMode Control_GetMode(void);
void  Control_SetBaseSpeed(float rpm);        /* forward base speed (RPM) */
void  Control_SetLineBias(float bias_rpm);    /* arc feed-forward differential */

/* Lock the current heading as the straight-line target (zeroes gyro yaw). Call
 * when the car must drive blind in a straight line (e.g. crossing the white
 * A4 patch) so it holds its present heading instead of drifting. */
void  Control_LockHeading(void);
int   Control_UsingGyro(void);                /* 1 if MPU6050 heading hold active */
float Control_GetYaw(void);                   /* integrated heading (deg), 0 if no IMU */

/* 1 once the car has been laterally centered on the line (IR error 0) for at
 * least min_ticks consecutive control ticks. Used to wait for re-centering
 * before blind-crossing the white patch. */
int   Control_IsCentered(int min_ticks);

/* Per-segment distance accumulators (encoder counts since last reset). */
void Control_ResetDistance(void);
int  Control_GetDistanceL(void);
int  Control_GetDistanceR(void);

/* Latest measured wheel speeds (RPM), for telemetry. */
float Control_GetRpmL(void);
float Control_GetRpmR(void);

/* Consecutive 10 ms ticks the line has been lost while in CTRL_LINE.
 * Used by the path layer to confirm "arc ended / vertex reached". */
int Control_GetLineLostTicks(void);

/* Runtime gain access for the UART tuner. */
void  Control_SetGain(ControlGain id, float value);
float Control_GetGain(ControlGain id);

#endif /* _CONTROL_H */
