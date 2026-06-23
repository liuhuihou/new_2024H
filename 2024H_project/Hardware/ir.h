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

/* Set to 1 if a sensor reads GPIO HIGH on black, 0 if it reads LOW on black.
 * LF04 line sensor outputs LOW on black / HIGH on white, so use 0. */
#ifndef IR_BLACK_IS_HIGH
#define IR_BLACK_IS_HIGH 0
#endif

/* Digital glitch filter depth, in 10 ms control ticks. A sensor bit only
 * changes its reported state after IR_FILTER_N consecutive raw samples agree,
 * so transient mis-reads (reflections, line gaps, edge-of-arc, electrical
 * noise) lasting up to (IR_FILTER_N - 1) ticks are rejected. This stops a
 * brief false "all white" from being counted as a vertex (premature stop).
 * Larger = steadier but slower to confirm a real change. */
#ifndef IR_FILTER_N
#define IR_FILTER_N 3
#endif

void    IR_Init(void);
void    IR_Update(void);      /* sample + filter; call once per control tick */
uint8_t IR_Read(void);        /* filtered 4-bit state, bit set = on black line */
uint8_t IR_ReadRaw(void);     /* unfiltered instantaneous state (debug only) */
int     IR_Error(void);       /* weighted position error, 0 = centered */
uint8_t IR_OnLine(void);      /* any sensor on black (state != 0) */
uint8_t IR_AllWhite(void);    /* no sensor on black (state == 0) */
uint8_t IR_AllBlack(void);    /* all four on black (state == 0b1111) */

#endif /* _IR_H */
