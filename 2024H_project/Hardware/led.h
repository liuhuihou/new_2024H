#ifndef _LED_H
#define _LED_H

#include "ti_msp_dl_config.h"

/* LED on PB9, active-low (clear pin = on). */
void LED_ON(void);
void LED_OFF(void);
void LED_Toggle(void);
void LED_Flash(uint16_t period);   /* toggle every `period` calls */

#endif /* _LED_H */
