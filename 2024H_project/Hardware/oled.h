#ifndef _OLED_H
#define _OLED_H

#include "ti_msp_dl_config.h"

/*
 * 0.96" SSD1306 128x64 OLED driver, 4-wire software SPI (bit-banged).
 *
 * Wiring (see 硬件连接线路/README.md):
 *   RST -> PB14   DC -> PB15   SCL(D0/SCLK) -> PA28   SDA(D1/MOSI) -> PA31
 *
 * The panel is addressed in 8 pages of 128 columns (page = 8 vertical pixels).
 * Text is drawn with a built-in 6x8 font, so the screen is 21 chars x 8 lines.
 * CS is tied low on the module, so it is not driven here.
 */

#define OLED_COLS   128
#define OLED_PAGES  8          /* 64 px / 8 */
#define OLED_CHAR_W 6          /* 6x8 font -> 21 chars per line */

void OLED_Init(void);
void OLED_Clear(void);

/* Place the text cursor. x = column 0..20 (char cell), y = page/line 0..7. */
void OLED_SetCursor(uint8_t x, uint8_t y);

void OLED_ShowChar(uint8_t x, uint8_t y, char ch);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);

/* Unsigned integer, `len` digits, right-justified with leading spaces. */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len);

/* Signed value scaled by 10 shown as d.d (e.g. 123 -> "12.3"). For RPM. */
void OLED_ShowTenths(uint8_t x, uint8_t y, int32_t value_x10, uint8_t int_len);

#endif /* _OLED_H */
