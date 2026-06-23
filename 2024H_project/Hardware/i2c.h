#ifndef _I2C_H
#define _I2C_H

#include "ti_msp_dl_config.h"

/*
 * Blocking I2C0 controller helpers for the MPU6050 (PA0=SDA, PA1=SCL).
 * Hardware I2C0 is configured by SYSCFG_DL_I2C_MPU_init() at 100 kHz.
 *
 * All calls are polled/blocking with a bounded spin so a missing device can
 * never hang the control loop. Return 0 on success, non-zero on error/timeout.
 */

int I2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t value);
int I2C_ReadReg(uint8_t dev_addr, uint8_t reg, uint8_t *value);
int I2C_ReadRegs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* _I2C_H */
