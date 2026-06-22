#include "key.h"

/*
 * Key scan with single/double-click detection.
 * Reads KEY (PA18); pin reads HIGH when pressed on the reference hardware.
 * Returns: 0 = no action, 1 = single click, 2 = double click.
 */
uint8_t click_N_Double(uint8_t time)
{
    static uint8_t  flag_key, count_key, double_key = 0;
    static uint16_t count_single, Forever_count;

    if (KEY > 0)
    {
        Forever_count++;
    }
    else
    {
        Forever_count = 0;
    }

    if ((KEY > 0) && (flag_key == 0))
    {
        flag_key = 1;   /* first edge */
    }

    if (count_key == 0)
    {
        if (flag_key == 1)
        {
            double_key++;
            count_key = 1;
        }
        if (double_key == 2)
        {
            double_key   = 0;
            count_single = 0;
            return 2;       /* double click */
        }
    }

    if (KEY == 0)
    {
        flag_key  = 0;
        count_key = 0;
    }

    if (double_key == 1)
    {
        count_single++;
        if (count_single > time && Forever_count < time)
        {
            double_key   = 0;
            count_single = 0;
            return 1;       /* single click */
        }
        if (Forever_count > time)
        {
            double_key   = 0;
            count_single = 0;
        }
    }
    return 0;
}

/* Long press (~2 s at 10 ms scan): returns 1 once. */
uint8_t Long_Press(void)
{
    static uint16_t long_count, pressed;
    if (pressed == 0 && KEY != 0)
    {
        long_count++;
    }
    else
    {
        long_count = 0;
    }
    if (long_count > 200)
    {
        pressed    = 1;
        long_count = 0;
        return 1;
    }
    if (pressed == 1 && KEY == 0)
    {
        pressed = 0;
    }
    return 0;
}
