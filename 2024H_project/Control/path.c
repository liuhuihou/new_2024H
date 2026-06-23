#include "board.h"
#include "control.h"
#include "path.h"

/*
 * Path state machine. Runs in the foreground loop; the actual motor control
 * is done by control.c in the 10 ms tick. This module only decides the mode
 * (STRAIGHT vs LINE vs STOP) from debounced IR events and counts vertices.
 */

/* ---- tunables ---- */
#define ACQUIRE_N      3        /* ticks of "on line" to confirm arc acquired   */
#define VERTEX_LOST_N  6        /* ticks of sustained line-loss to call a vertex */
#define BASE_RPM      60.0f     /* forward base speed for straight + line        */
#define LINE_BIAS      0.0f     /* arc curvature feed-forward (0 = pure PD)      */

/* Vertices to pass before the final stop, per requirement. Confirm on bench. */
static int vertices_for_req(PathReq req)
{
    switch (req)
    {
        case REQ_1: return 1;    /* reach B */
        case REQ_2: return 4;    /* B, C, D, A */
        case REQ_3: return 4;    /* C, B, D, A */
        case REQ_4: return 16;   /* 4 laps x 4 vertices */
        default:    return 1;
    }
}

typedef enum
{
    PS_IDLE = 0,
    PS_STRAIGHT,
    PS_LINE,
    PS_STOP
} PathState;

static PathReq   g_req          = REQ_1;
static PathState g_state        = PS_IDLE;
static int       g_vertex_count = 0;
static int       g_vertex_goal  = 1;
static int       g_finished     = 0;

static int g_acquire_streak = 0;   /* consecutive "on line" reads in STRAIGHT */

static void announce_vertex(int n)
{
    /* sound/light surrogate: LED on + serial print at each vertex */
    LED_ON();
    printf("PASS vertex %d  IR=0x%X\r\n", n, IR_Read());
}

static void announce_stop(void)
{
    LED_ON();
    printf("ARRIVED, STOP. vertices=%d\r\n", g_vertex_count);
}

void Path_Init(PathReq req)
{
    g_req          = req;
    g_vertex_goal  = vertices_for_req(req);
    g_state        = PS_IDLE;
    g_vertex_count = 0;
    g_finished     = 0;
    g_acquire_streak = 0;
    Control_SetMode(CTRL_STOP);
    Control_SetBaseSpeed(0.0f);
    printf("Path init: REQ_%d, vertex goal=%d\r\n", (int)req, g_vertex_goal);
}

void Path_Start(void)
{
    if (g_state != PS_IDLE)
    {
        return;
    }
    g_acquire_streak = 0;
    Control_ResetDistance();
    Control_SetBaseSpeed(BASE_RPM);
    Control_SetLineBias(LINE_BIAS);
    Control_SetMode(CTRL_STRAIGHT);
    g_state = PS_STRAIGHT;
    LED_OFF();
    printf("Path start (STRAIGHT)\r\n");
}

void Path_Stop(void)
{
    Control_SetMode(CTRL_STOP);
    g_state    = PS_STOP;
    g_finished = 1;
    announce_stop();
}

/* Register one passed vertex; returns 1 if this was the final one. */
static int register_vertex(void)
{
    g_vertex_count++;
    announce_vertex(g_vertex_count);
    return (g_vertex_count >= g_vertex_goal) ? 1 : 0;
}

void Path_Tick(void)
{
    switch (g_state)
    {
        case PS_STRAIGHT:
        {
            /* Wait until the IR sensors catch the black arc. */
            if (IR_OnLine())
            {
                if (++g_acquire_streak >= ACQUIRE_N)
                {
                    g_acquire_streak = 0;

                    /* For REQ_1 the goal is simply "reach the arc at B":
                     * acquiring the line == arriving at B -> stop. */
                    if (g_vertex_goal == 1)
                    {
                        if (register_vertex())
                        {
                            Path_Stop();
                        }
                        break;
                    }

                    Control_SetMode(CTRL_LINE);
                    g_state = PS_LINE;
                    LED_OFF();
                    printf("-> LINE\r\n");
                }
            }
            else
            {
                g_acquire_streak = 0;
            }
            break;
        }

        case PS_LINE:
        {
            /* Vertex = arc ends. The controller keeps steering with the held
             * correction while the line is lost; only a *sustained* loss
             * (VERTEX_LOST_N ticks) counts as the arc's end, so a brief gap or
             * a fast corner does not trigger a false vertex. */
            if (Control_GetLineLostTicks() >= VERTEX_LOST_N)
            {
                int last = register_vertex();
                if (last)
                {
                    Path_Stop();
                }
                else
                {
                    g_acquire_streak = 0;
                    Control_ResetDistance();
                    Control_SetMode(CTRL_STRAIGHT);
                    g_state = PS_STRAIGHT;
                    printf("-> STRAIGHT\r\n");
                }
            }
            break;
        }

        case PS_STOP:
        case PS_IDLE:
        default:
            break;
    }
}

int Path_VertexCount(void) { return g_vertex_count; }
int Path_IsFinished(void)  { return g_finished; }
