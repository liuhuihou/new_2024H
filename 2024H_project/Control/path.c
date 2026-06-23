#include "board.h"
#include "control.h"
#include "path.h"

/*
 * Path state machine. Runs in the foreground loop; the actual motor control
 * is done by control.c in the 10 ms tick. This module only decides the mode
 * (STRAIGHT vs LINE vs STOP) and counts progress.
 *
 * Two flavours:
 *   - REQ_1/3/4 (legacy): a "vertex" is a sustained line-loss; count vertices
 *     until the goal, then stop.
 *   - REQ_2 (stadium one-lap): the track is an oval (2 semicircles + 2 straights)
 *     whose two straights each carry a white A4 patch in the middle. Line-loss is
 *     NOT a stop signal; instead, hitting the white patch makes the car blind-drive
 *     straight across it (heading-hold), re-acquire the line on the far side, and
 *     log a landmark. The lap ends on odometry (one full lap) confirmed by the
 *     landmark count, so the return/line-loss logic can never stop the car early.
 */

/* ---- legacy tunables (REQ_1/3/4) ---- */
#define ACQUIRE_N      3        /* ticks of "on line" to confirm arc acquired   */
#define VERTEX_LOST_N  6        /* ticks of sustained line-loss to call a vertex */
#define BASE_RPM      60.0f     /* forward base speed for straight + line        */
#define LINE_BIAS      0.0f     /* arc curvature feed-forward (0 = pure PD)      */

/*
 * ---- REQ_2 stadium one-lap, odometry constants ----
 * Encoder: 13 lines x2 edges x30 gear = 780 counts/wheel-rev; wheel d=6.6 cm
 * (circumference 20.74 cm) -> ~37.6 counts/cm.
 *   one lap = 2*straight(80.5) + 2*semicircle(R=58) = 525.4 cm  ~= 19765 counts
 *   A4 white patch (long edge)  = 29.7 cm            ~= 1117 counts
 */
#define LAP_COUNTS        19765L  /* one full lap (525.4 cm)                        */
#define GAP_MIN_COUNTS      680L  /* ~18 cm: shorter white = glitch, not a patch    */
#define GAP_MAX_COUNTS     1880L  /* ~50 cm: blind-driven past this w/o line=abnormal */
#define LANDMARK_MIN_CNT   3760L  /* ~100 cm: min spacing between counted patches    */
#define LAP_LANDMARKS         2   /* white patches crossed in one lap               */
#define CENTERED_TICKS        2   /* ticks centered on the line to call it "on center" */
/* require >= 90% lap before a landmark may end the run; hard backstop at 110%. */
#define LAP_STOP_NUM          9
#define LAP_STOP_DEN         10
#define LAP_BACKSTOP_NUM     11
#define LAP_BACKSTOP_DEN     10

/* Vertices/landmarks to pass before the final stop, per requirement. */
static int vertices_for_req(PathReq req)
{
    switch (req)
    {
        case REQ_1: return 1;             /* reach B */
        case REQ_2: return LAP_LANDMARKS; /* stadium lap: 2 white-patch crossings */
        case REQ_3: return 4;             /* C, B, D, A */
        case REQ_4: return 16;            /* 4 laps x 4 vertices */
        default:    return 1;
    }
}

typedef enum
{
    PS_IDLE = 0,
    PS_STRAIGHT,
    PS_LINE,
    PS_CROSS_GAP,   /* REQ_2: blind-driving straight across a white A4 patch */
    PS_STOP
} PathState;

static PathReq   g_req          = REQ_1;
static PathState g_state        = PS_IDLE;
static int       g_vertex_count = 0;
static int       g_vertex_goal  = 1;
static int       g_finished     = 0;

static int g_acquire_streak = 0;   /* consecutive "on line" reads in STRAIGHT/GAP */

/* ---- REQ_2 lap-mode state ---- */
static int  g_lap_mode            = 0;   /* 1 when req == REQ_2 */
static long g_total_counts        = 0;   /* distance booked since start */
static long g_last_landmark_total = 0;   /* total at the previous counted patch */

/* Average wheel distance (centerline counts) since the last reset. */
static int seg_dist(void)
{
    return (Control_GetDistanceL() + Control_GetDistanceR()) / 2;
}

/* Book the current segment into the running total and start a fresh segment. */
static void advance_total_and_reset(void)
{
    g_total_counts += seg_dist();
    Control_ResetDistance();
}

static void announce_vertex(int n)
{
    /* sound/light surrogate: LED on + serial print at each vertex/landmark */
    LED_ON();
    printf("PASS %d  IR=0x%X  total=%ld\r\n", n, IR_Read(), g_total_counts);
}

static void announce_stop(void)
{
    LED_ON();
    printf("ARRIVED, STOP. count=%d total=%ld\r\n", g_vertex_count, g_total_counts);
}

void Path_Init(PathReq req)
{
    g_req          = req;
    g_lap_mode     = (req == REQ_2) ? 1 : 0;
    g_vertex_goal  = vertices_for_req(req);
    g_state        = PS_IDLE;
    g_vertex_count = 0;
    g_finished     = 0;
    g_acquire_streak = 0;
    g_total_counts        = 0;
    g_last_landmark_total = 0;
    Control_SetMode(CTRL_STOP);
    Control_SetBaseSpeed(0.0f);
    printf("Path init: REQ_%d, %s, goal=%d\r\n",
           (int)req, g_lap_mode ? "STADIUM-LAP" : "vertex", g_vertex_goal);
}

void Path_Start(void)
{
    if (g_state != PS_IDLE)
    {
        return;
    }
    g_acquire_streak = 0;
    g_total_counts        = 0;
    g_last_landmark_total = 0;
    Control_ResetDistance();
    Control_SetBaseSpeed(BASE_RPM);
    Control_SetLineBias(LINE_BIAS);
    Control_LockHeading();          /* start straight from the current heading */
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

/* Register one passed vertex (legacy modes); returns 1 if it was the final one. */
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
            /* Wait until the IR sensors catch the black line. */
            if (IR_OnLine())
            {
                if (++g_acquire_streak >= ACQUIRE_N)
                {
                    g_acquire_streak = 0;

                    if (!g_lap_mode)
                    {
                        /* REQ_1: acquiring the arc at B == arriving -> stop. */
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
                    else
                    {
                        /* lap mode: book the start/acquire distance, follow line */
                        advance_total_and_reset();
                        Control_SetMode(CTRL_LINE);
                        g_state = PS_LINE;
                        LED_OFF();
                        printf("-> LINE (total=%ld)\r\n", g_total_counts);
                    }
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
            if (g_lap_mode)
            {
                /*
                 * Sustained white = entering an A4 patch (or, rarely, a real
                 * line gap). Do NOT stop: blind-drive straight across it and
                 * re-acquire on the far side. The IR glitch filter already
                 * rejects momentary white, so reaching here means real white.
                 */
                if (IR_AllWhite())
                {
                    /* g_centered_streak survives the first white tick (white
                     * skips the centering update), so this reflects whether the
                     * car was on the centerline just before the patch. If not,
                     * a lateral offset is about to be carried across the patch -
                     * surface it on UART so KD_LINE / speed can be tuned. */
                    if (!Control_IsCentered(CENTERED_TICKS))
                        printf("WARN: entering patch off-center\r\n");

                    advance_total_and_reset();         /* book the line-follow run */
                    Control_LockHeading();             /* hold current heading blind */
                    Control_SetMode(CTRL_STRAIGHT);    /* heading-hold across patch */
                    g_state = PS_CROSS_GAP;
                    printf("-> CROSS_GAP (total=%ld)\r\n", g_total_counts);
                }
                /* Backstop: never overrun a lap, even if a landmark was missed. */
                else if (g_total_counts + seg_dist()
                         >= LAP_COUNTS * LAP_BACKSTOP_NUM / LAP_BACKSTOP_DEN)
                {
                    advance_total_and_reset();
                    Path_Stop();
                }
            }
            else
            {
                /* legacy: sustained line-loss is a vertex */
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
                        Control_LockHeading();      /* hold heading on this leg */
                        Control_SetMode(CTRL_STRAIGHT);
                        g_state = PS_STRAIGHT;
                        printf("-> STRAIGHT\r\n");
                    }
                }
            }
            break;
        }

        case PS_CROSS_GAP:
        {
            int gap = seg_dist();   /* distance blind-driven into the white patch */

            if (IR_OnLine())
            {
                if (++g_acquire_streak >= ACQUIRE_N)
                {
                    long total_now;
                    int is_patch;

                    g_acquire_streak = 0;
                    advance_total_and_reset();
                    total_now = g_total_counts;

                    /* A confirmed A4 patch: white length in band, and far enough
                     * from the last one (so one patch can't be counted twice). */
                    is_patch = (gap >= GAP_MIN_COUNTS) && (gap <= GAP_MAX_COUNTS)
                            && (total_now - g_last_landmark_total >= LANDMARK_MIN_CNT);

                    if (is_patch)
                    {
                        g_vertex_count++;
                        g_last_landmark_total = total_now;
                        announce_vertex(g_vertex_count);

                        /* Lap done: enough patches AND ~a full lap travelled. */
                        if (g_vertex_count >= LAP_LANDMARKS &&
                            total_now >= LAP_COUNTS * LAP_STOP_NUM / LAP_STOP_DEN)
                        {
                            Path_Stop();
                            break;
                        }
                    }
                    else
                    {
                        printf("gap ignored (len=%d cnt)\r\n", gap);
                    }

                    Control_SetMode(CTRL_LINE);
                    g_state = PS_LINE;
                }
            }
            else
            {
                g_acquire_streak = 0;
                /*
                 * Blind-driven past the max patch length with no line. Keep
                 * going straight (do not stop on line-loss); only the lap
                 * backstop may end the run.
                 */
                if ((long)gap > GAP_MAX_COUNTS &&
                    g_total_counts + gap >= LAP_COUNTS * LAP_BACKSTOP_NUM / LAP_BACKSTOP_DEN)
                {
                    advance_total_and_reset();
                    Path_Stop();
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
