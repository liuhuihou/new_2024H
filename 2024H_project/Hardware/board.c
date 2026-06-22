#include "ti_msp_dl_config.h"
#include "board.h"

/*
 * Board support: SysTick-based busy-wait delays and printf -> UART0 redirect.
 * SysTick is configured by SYSCFG_DL_SYSTICK_init (free-running, 24-bit).
 */

/* Read the current SysTick counter value (counts down). */
uint32_t Systick_getTick(void)
{
    return SysTick->VAL;
}

/* Microsecond busy-wait using the SysTick down-counter. */
void delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t told, tnow;
    uint32_t tcnt = 0;
    const uint32_t reload = SysTickMAX_COUNT + 1U;

    if (us > (SysTickMAX_COUNT / (SysTickFre / 1000000U)))
    {
        us = SysTickMAX_COUNT / (SysTickFre / 1000000U);
    }
    ticks = us * (SysTickFre / 1000000U);

    told = Systick_getTick();
    while (1)
    {
        tnow = Systick_getTick();
        if (tnow != told)
        {
            /* SysTick counts DOWN, so elapsed = told - tnow (mod reload). */
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}

#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE { int handle; };
#endif
FILE __stdout;
void _sys_exit(int x) { x = x; }
#endif

/* printf redirect to UART0 (blocking). */
int fputc(int ch, FILE *stream)
{
    (void)stream;
    while (DL_UART_isBusy(UART_0_INST) == true)
    {
    }
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)ch);
    return ch;
}
