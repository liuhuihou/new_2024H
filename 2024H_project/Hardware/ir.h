#ifndef _IR_H
#define _IR_H

#include "ti_msp_dl_config.h"

/*
 * 4-way IR line sensor. Layout left-to-right along the car's front:
 *   DH1 (PA27) | DH2 (PA12) | DH3 (PB16) | DH4 (PB17)
 * On the field a sensor over the BLACK line is "active". The module turns its
 * indicator LED on over black. The raw GPIO level for "black" depends on the
 * module; IR_BLACK_IS_HIGH selects the polarity (flip on the bench if needed).
 * Pins use the SysConfig-generated IR_DHx_PORT / IR_DHx_PIN macros.
 *
 * State is packed MSB-first: state = (DH1<<3)|(DH2<<2)|(DH3<<1)|DH4.
 * On track, the middle two sensors sit on the line -> state == 0b0110.
 */

/* Set to 1 if a sensor reads GPIO HIGH on black, 0 if it reads LOW on black. */
#ifndef IR_BLACK_IS_HIGH
#define IR_BLACK_IS_HIGH 1
#endif

void    IR_Init(void);
uint8_t IR_Read(void);        /* 4-bit state, bit set = on black line */
int     IR_Error(void);       /* weighted position error, 0 = centered */
uint8_t IR_OnLine(void);      /* any sensor on black (state != 0) */
uint8_t IR_AllWhite(void);    /* no sensor on black (state == 0) */
uint8_t IR_AllBlack(void);    /* all four on black (state == 0b1111) */

#endif /* _IR_H */
