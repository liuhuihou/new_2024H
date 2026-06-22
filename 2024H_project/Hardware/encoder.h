#ifndef _ENCODER_H
#define _ENCODER_H

#include "ti_msp_dl_config.h"

/* Raw quadrature counts, updated in GROUP1_IRQHandler (GPIOA+GPIOB). */
extern volatile int Get_Encoder_countA;   /* encoder A (PA25/PA26) */
extern volatile int Get_Encoder_countB;   /* encoder B (PB20/PB24) */

/*
 * Convert an encoder delta to motor RPM.
 * 13 lines * 2 (edge factor) = 26 pulses/rev at the encoder; /30 gearbox.
 */
float Calculate_Motor_RPM(int encoder_count, int sample_time_ms);

#endif /* _ENCODER_H */
