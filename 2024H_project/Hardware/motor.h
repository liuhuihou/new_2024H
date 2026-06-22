#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"

/*
 * Forward-only differential drive on an A4950-style driver.
 * Left  wheel: TIMA1 CCP0 (PB2) PWM, direction pins AIN held for forward.
 * Right wheel: TIMA1 CCP1 (PB3) PWM, direction pins BIN held for forward.
 * Set_PWM takes non-negative duties (0..8000); car never reverses.
 */
#define PWM_MAX 7000

void Set_PWM(int pwmLeft, int pwmRight);
int  limit_PWM(int value, int low, int high);

#endif /* _MOTOR_H */
