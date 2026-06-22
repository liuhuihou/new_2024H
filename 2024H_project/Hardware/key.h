#ifndef _KEY_H
#define _KEY_H

#include "ti_msp_dl_config.h"
#include "board.h"

/* KEY on PA18. Reference hardware reads pin HIGH when pressed. */
#define KEY DL_GPIO_readPins(KEY_PORT, KEY_key_PIN)

/* Single/double click scan. Returns 0=none, 1=single click, 2=double click.
 * `time` is the double-click wait window in scan ticks. */
uint8_t click_N_Double(uint8_t time);

/* 2-second long-press detection: returns 1 once when held. */
uint8_t Long_Press(void);

#endif /* _KEY_H */
