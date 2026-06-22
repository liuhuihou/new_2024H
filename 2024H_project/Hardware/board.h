#ifndef _BOARD_H_
#define _BOARD_H_

#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"
#include "led.h"
#include "key.h"
#include "motor.h"
#include "encoder.h"
#include "ir.h"

#define ABS(a) ((a) > 0 ? (a) : (-(a)))

typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

/* Global run/stop flag, toggled by the key (kept for compatibility). */
extern int Flag_Stop;

/* SysTick free-running counter (24-bit). */
#define SysTickMAX_COUNT 0xFFFFFF
#define SysTickFre       80000000

uint32_t Systick_getTick(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void Board_Init(void);

#endif /* _BOARD_H_ */
