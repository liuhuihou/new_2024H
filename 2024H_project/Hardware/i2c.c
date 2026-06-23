#include "i2c.h"

/*
 * Blocking I2C0 controller transfers via MSPM0 DriverLib. The MPU6050 is the
 * only device on this bus. Register read uses the standard "write reg addr
 * (no STOP) then repeated-START read" sequence.
 *
 * Every wait loop is bounded by I2C_SPIN so a disconnected/!ACKing device
 * times out instead of hanging the 10 ms control tick.
 */

#define I2C_SPIN  100000U   /* bounded spin count for each blocking wait */

/* Wait until the controller bus is idle (not busy). 0 = ok, 1 = timeout. */
static int wait_bus_idle(void)
{
    uint32_t spin = I2C_SPIN;
    while (DL_I2C_getControllerStatus(I2C_MPU_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
    {
        if (--spin == 0U) return 1;
    }
    return 0;
}

/* Wait while a controller transfer is in progress. 0 = ok, 1 = timeout. */
static int wait_not_busy(void)
{
    uint32_t spin = I2C_SPIN;
    while (DL_I2C_getControllerStatus(I2C_MPU_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY)
    {
        if (--spin == 0U) return 1;
    }
    return 0;
}

/* True if the last transfer flagged an error (NACK / arbitration loss). */
static int had_error(void)
{
    return (DL_I2C_getControllerStatus(I2C_MPU_INST)
            & DL_I2C_CONTROLLER_STATUS_ERROR) ? 1 : 0;
}

int I2C_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    if (wait_bus_idle()) return 1;

    tx[0] = reg;
    tx[1] = value;

    DL_I2C_flushControllerTXFIFO(I2C_MPU_INST);
    DL_I2C_fillControllerTXFIFO(I2C_MPU_INST, tx, 2);

    DL_I2C_startControllerTransfer(I2C_MPU_INST, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    if (wait_not_busy()) return 2;
    if (had_error())     return 3;
    return 0;
}

int I2C_ReadRegs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint32_t spin;

    if (buf == 0 || len == 0) return 1;
    if (wait_bus_idle())      return 1;

    /* Phase 1: write the register address, no STOP (repeated start follows). */
    DL_I2C_flushControllerTXFIFO(I2C_MPU_INST);
    DL_I2C_fillControllerTXFIFO(I2C_MPU_INST, &reg, 1);
    DL_I2C_startControllerTransferAdvanced(I2C_MPU_INST, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_ENABLE);

    if (wait_not_busy()) return 2;
    if (had_error())     return 3;

    /* Phase 2: repeated start + read `len` bytes, STOP at the end. */
    DL_I2C_startControllerTransfer(I2C_MPU_INST, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    for (i = 0; i < len; i++)
    {
        spin = I2C_SPIN;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_MPU_INST))
        {
            if (had_error()) return 4;
            if (--spin == 0U) return 5;
        }
        buf[i] = DL_I2C_receiveControllerData(I2C_MPU_INST);
    }

    if (wait_not_busy()) return 6;
    if (had_error())     return 7;
    return 0;
}

int I2C_ReadReg(uint8_t dev_addr, uint8_t reg, uint8_t *value)
{
    return I2C_ReadRegs(dev_addr, reg, value, 1);
}
