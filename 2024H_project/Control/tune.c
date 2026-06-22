#include "board.h"
#include "control.h"
#include "tune.h"
#include <stdlib.h>

/*
 * Minimal newline-terminated command parser over UART0 RX (polled).
 * See tune.h for the command syntax.
 */

#define TUNE_BUF_LEN 32

static char    g_buf[TUNE_BUF_LEN];
static uint8_t g_len = 0;

static const char *gain_name(ControlGain id)
{
    switch (id)
    {
        case GAIN_KP_SPEED:   return "KpSpd";
        case GAIN_KI_SPEED:   return "KiSpd";
        case GAIN_FF_LEFT:    return "FFL";
        case GAIN_FF_RIGHT:   return "FFR";
        case GAIN_KP_HEADING: return "KpHead";
        case GAIN_KD_HEADING: return "KdHead";
        case GAIN_KP_LINE:    return "KpLine";
        case GAIN_KD_LINE:    return "KdLine";
        default:              return "?";
    }
}

static void print_all(void)
{
    printf("--- gains ---\r\n");
    for (int i = 0; i < (int)GAIN_COUNT; i++)
    {
        printf("g %d %-7s = %.3f\r\n", i, gain_name((ControlGain)i),
               Control_GetGain((ControlGain)i));
    }
}

/* Parse and apply one complete command line. */
static void handle_line(char *s)
{
    /* skip leading spaces */
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '\0') return;

    char cmd = *s++;
    while (*s == ' ' || *s == '\t') s++;

    switch (cmd)
    {
        case 'p':
            print_all();
            break;

        case 'g':
        {
            int id = atoi(s);
            /* advance past the integer to the value */
            while (*s == '-' || (*s >= '0' && *s <= '9')) s++;
            while (*s == ' ' || *s == '\t') s++;
            float val = (float)atof(s);
            if (id >= 0 && id < (int)GAIN_COUNT)
            {
                Control_SetGain((ControlGain)id, val);
                printf("set g %d %s = %.3f\r\n", id, gain_name((ControlGain)id), val);
            }
            else
            {
                printf("bad gain id %d\r\n", id);
            }
            break;
        }

        case 'b':
        {
            float val = (float)atof(s);
            Control_SetBaseSpeed(val);
            printf("set base = %.2f rpm\r\n", val);
            break;
        }

        case 'f':
        {
            float val = (float)atof(s);
            Control_SetLineBias(val);
            printf("set bias = %.2f rpm\r\n", val);
            break;
        }

        default:
            printf("? cmd '%c' (p / g id val / b val / f val)\r\n", cmd);
            break;
    }
}

void Tune_Poll(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST))
    {
        char c = (char)DL_UART_Main_receiveData(UART_0_INST);

        if (c == '\r' || c == '\n')
        {
            if (g_len > 0)
            {
                g_buf[g_len] = '\0';
                handle_line(g_buf);
                g_len = 0;
            }
        }
        else if (g_len < (TUNE_BUF_LEN - 1))
        {
            g_buf[g_len++] = c;
        }
        else
        {
            /* overflow: reset the line */
            g_len = 0;
        }
    }
}
