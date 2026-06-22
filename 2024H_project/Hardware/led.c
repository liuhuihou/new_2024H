#include "led.h"

void LED_ON(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_led_PIN);   /* active-low */
}

void LED_OFF(void)
{
    DL_GPIO_setPins(LED_PORT, LED_led_PIN);
}

void LED_Toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_led_PIN);
}

/* Toggle once every `period` calls; period==0 forces on. */
void LED_Flash(uint16_t period)
{
    static uint16_t cnt = 0;
    if (period == 0)
    {
        LED_ON();
        return;
    }
    if (++cnt >= period)
    {
        LED_Toggle();
        cnt = 0;
    }
}
