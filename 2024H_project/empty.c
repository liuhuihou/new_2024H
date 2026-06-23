#include "board.h"
#include "control.h"
#include "path.h"
#include "tune.h"
#include "show.h"

/*
 * 2024 EDC Problem H - autonomous line-following car (TI MSPM0G3507).
 *
 * Operation:
 *   - Single-click KEY to cycle the selected requirement (1..4).
 *   - Double-click KEY to start the run.
 *   - The 10 ms TIMG0 tick runs the cascaded speed/line controller.
 *   - The foreground loop runs the event-driven path state machine and
 *     prints telemetry over UART0 (115200 8N1, PA10/PA11).
 *
 * Sound/light: LED on PB9 + UART printf at each vertex and on stop.
 */

int Flag_Stop = 1;

static PathReq g_selected = REQ_1;
static uint8_t g_running  = 0;

static void print_banner(void)
{
    printf("\r\n==== 2024H Auto Car (MSPM0G3507) ====\r\n");
    printf("KEY single-click: select REQ 1..4   double-click: START\r\n");
    printf("UART tune: 'p' list, 'g id val', 'b val', 'f val'\r\n");
    printf("Selected: REQ_%d\r\n", (int)g_selected);
}

int main(void)
{
    uint32_t telemetry = 0;
    uint32_t oled_div  = 0;

    SYSCFG_DL_init();

    /* Make sure real UART output works even if a regenerated config re-enabled
     * loopback (belt-and-suspenders; SYSCFG_DL_UART_0_init already disables). */
    DL_UART_Main_disableLoopbackMode(UART_0_INST);

    DL_Timer_startCounter(PWM_0_INST);

    /* Encoder edge interrupts (GROUP1 = GPIOA + GPIOB). */
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    Control_Init();
    IR_Init();

    /* Encoder edges must preempt the control tick so counts are never missed:
     * GROUP1 (GPIO) highest priority (0), TIMG0 control tick lower (1). */
    NVIC_SetPriority(ENCODERA_INT_IRQN, 0);
    NVIC_SetPriority(ENCODERB_INT_IRQN, 0);

    /* 10 ms control tick. */
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 1);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    Path_Init(g_selected);
    Show_Init();
    print_banner();

    while (1)
    {
        Tune_Poll();   /* UART0 live gain/speed tuning, any phase */

        if (!g_running)
        {
            /* selection phase: single-click cycles requirement, double starts */
            uint8_t k = click_N_Double(50);
            if (k == 1)
            {
                int next = (int)g_selected + 1;
                if (next > (int)REQ_4) next = (int)REQ_1;
                g_selected = (PathReq)next;
                Path_Init(g_selected);
                printf("Selected: REQ_%d\r\n", (int)g_selected);
            }
            else if (k == 2)
            {
                g_running = 1;
                Path_Start();
            }
        }
        else
        {
            Path_Tick();

            if (++telemetry >= 20U)   /* ~ every 200 ms */
            {
                telemetry = 0;
                printf("st=%d v=%d IR=0x%X Lrpm=%.1f Rrpm=%.1f\r\n",
                       (int)Control_GetMode(), Path_VertexCount(), IR_Read(),
                       Control_GetRpmL(), Control_GetRpmR());
            }

            if (Path_IsFinished())
            {
                /* run complete: hold stopped, allow re-arm with a click */
                if (click_N_Double(50) == 2)
                {
                    g_running = 0;
                    Path_Init(g_selected);
                    print_banner();
                }
            }
        }

        if (++oled_div >= 10U)   /* refresh OLED ~ every 100 ms */
        {
            oled_div = 0;
            Show_Update(g_selected, g_running, (uint8_t)Path_IsFinished());
        }

        delay_ms(10);
    }
}
