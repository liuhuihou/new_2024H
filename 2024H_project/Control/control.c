#include "board.h"
#include "control.h"

/*
 * Cascaded motion controller. Runs entirely inside the 10 ms TIMG0 interrupt.
 *
 * Encoder mapping (matches the speed-loop reference):
 *   left  count = +Get_Encoder_countB
 *   right count = -Get_Encoder_countA
 *
 * Inner loop: per-wheel PI on RPM, with a feed-forward base PWM term so the
 * integrator only trims. Outer loop: per mode, sets the left/right target RPM.
 *
 * All gains are runtime-tunable through Control_SetGain() so they can be
 * adjusted over UART without recompiling (see Control/tune.c).
 */

#define TICK_MS    10
#define PWM_PERIOD 8000     /* PWM timer count (matches SysConfig) */

/*
 * Inner PI output limit. Sits just below the PWM period so the loop can use
 * almost the full duty range (forward-only). Previously 5200, which capped the
 * top speed at ~65% duty; raised to 7800 to unlock speed for requirement 4.
 */
#define PWM_LIMIT  7800

/* ---- runtime-tunable gains (start values; tune on hardware) ---- */
static float KP_SPEED   = 10.0f;   /* inner PI proportional */
static float KI_SPEED   = 0.35f;   /* inner PI integral     */
static float FF_LEFT    = 65.0f;   /* base PWM per target RPM, left  */
static float FF_RIGHT   = 58.0f;   /* base PWM per target RPM, right */
static float INT_LIMIT  = 1500.0f; /* integrator clamp (raised with PWM_LIMIT) */

static float KP_HEADING = 0.20f;   /* straight heading-hold P (on dist diff) */
static float KD_HEADING = 0.50f;   /* straight heading-hold D */

static float KP_LINE    = 6.0f;    /* line-follow P (on IR weighted error) */
static float KD_LINE    = 3.0f;    /* line-follow D */

/* ---- state ---- */
static volatile ControlMode g_mode = CTRL_STOP;
static float g_base_rpm   = 0.0f;     /* forward base speed */
static float g_line_bias  = 0.0f;     /* arc curvature feed-forward (RPM) */

static float g_int_left  = 0.0f;
static float g_int_right = 0.0f;

static volatile int g_dist_left  = 0;   /* counts since reset */
static volatile int g_dist_right = 0;

static float g_rpm_left  = 0.0f;        /* measured, for telemetry */
static float g_rpm_right = 0.0f;

static int   g_prev_head_err = 0;
static int   g_prev_line_err = 0;

/* Line-lost memory: when all sensors read white mid-arc, keep applying the
 * last steering correction for a few ticks instead of giving up immediately.
 * The path layer decides whether a sustained white means "vertex reached". */
static float g_last_line_corr = 0.0f;
static int   g_line_lost_ticks = 0;

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

void Control_Init(void)
{
    g_mode       = CTRL_STOP;
    g_base_rpm   = 0.0f;
    g_line_bias  = 0.0f;
    g_int_left   = 0.0f;
    g_int_right  = 0.0f;
    g_dist_left  = 0;
    g_dist_right = 0;
    g_prev_head_err = 0;
    g_prev_line_err = 0;
    g_last_line_corr = 0.0f;
    g_line_lost_ticks = 0;
    Set_PWM(0, 0);
}

void Control_SetMode(ControlMode mode)
{
    if (mode != g_mode)
    {
        g_int_left  = 0.0f;
        g_int_right = 0.0f;
        g_prev_head_err = 0;
        g_prev_line_err = 0;
        g_last_line_corr = 0.0f;
        g_line_lost_ticks = 0;
    }
    g_mode = mode;
}

ControlMode Control_GetMode(void)        { return g_mode; }
void  Control_SetBaseSpeed(float rpm)    { g_base_rpm  = rpm; }
void  Control_SetLineBias(float bias)    { g_line_bias = bias; }

void Control_ResetDistance(void)
{
    __disable_irq();
    g_dist_left  = 0;
    g_dist_right = 0;
    __enable_irq();
}

int Control_GetDistanceL(void) { return g_dist_left; }
int Control_GetDistanceR(void) { return g_dist_right; }
float Control_GetRpmL(void)    { return g_rpm_left; }
float Control_GetRpmR(void)    { return g_rpm_right; }

int Control_GetLineLostTicks(void) { return g_line_lost_ticks; }

/* Runtime gain access for the UART tuner. */
void Control_SetGain(ControlGain id, float value)
{
    switch (id)
    {
        case GAIN_KP_SPEED:   KP_SPEED   = value; break;
        case GAIN_KI_SPEED:   KI_SPEED   = value; break;
        case GAIN_FF_LEFT:    FF_LEFT    = value; break;
        case GAIN_FF_RIGHT:   FF_RIGHT   = value; break;
        case GAIN_KP_HEADING: KP_HEADING = value; break;
        case GAIN_KD_HEADING: KD_HEADING = value; break;
        case GAIN_KP_LINE:    KP_LINE    = value; break;
        case GAIN_KD_LINE:    KD_LINE    = value; break;
        default: break;
    }
}

float Control_GetGain(ControlGain id)
{
    switch (id)
    {
        case GAIN_KP_SPEED:   return KP_SPEED;
        case GAIN_KI_SPEED:   return KI_SPEED;
        case GAIN_FF_LEFT:    return FF_LEFT;
        case GAIN_FF_RIGHT:   return FF_RIGHT;
        case GAIN_KP_HEADING: return KP_HEADING;
        case GAIN_KD_HEADING: return KD_HEADING;
        case GAIN_KP_LINE:    return KP_LINE;
        case GAIN_KD_LINE:    return KD_LINE;
        default:              return 0.0f;
    }
}

/* Inner PI for one wheel; returns clamped PWM duty. */
static int speed_pi(float target_rpm, float meas_rpm, float ff_gain, float *integ)
{
    float err = target_rpm - meas_rpm;
    *integ += err;
    *integ = clampf(*integ, -INT_LIMIT, INT_LIMIT);
    float pwm = ff_gain * target_rpm + KP_SPEED * err + KI_SPEED * (*integ);
    if (pwm < 0.0f) pwm = 0.0f;                 /* forward-only */
    if (pwm > (float)PWM_LIMIT) pwm = (float)PWM_LIMIT;
    return (int)pwm;
}

void Control_Tick(void)
{
    int rawA, rawB;
    int left_count, right_count;
    float target_left, target_right;

    /* Clock the IR glitch filter once per tick so IR_AllWhite()/IR_Error()
     * below operate on the debounced state (prevents premature vertex/stop). */
    IR_Update();

    __disable_irq();
    rawA = Get_Encoder_countA;
    rawB = Get_Encoder_countB;
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    __enable_irq();

    left_count  =  rawB;
    right_count = -rawA;

    g_dist_left  += left_count;
    g_dist_right += right_count;

    g_rpm_left  = Calculate_Motor_RPM(left_count,  TICK_MS);
    g_rpm_right = Calculate_Motor_RPM(right_count, TICK_MS);
    if (g_rpm_left  < 0.0f) g_rpm_left  = -g_rpm_left;
    if (g_rpm_right < 0.0f) g_rpm_right = -g_rpm_right;

    if (g_mode == CTRL_STOP)
    {
        g_int_left = g_int_right = 0.0f;
        Set_PWM(0, 0);
        return;
    }

    if (g_mode == CTRL_STRAIGHT)
    {
        int head_err = g_dist_left - g_dist_right;
        float corr = KP_HEADING * (float)head_err
                   + KD_HEADING * (float)(head_err - g_prev_head_err);
        g_prev_head_err = head_err;
        target_left  = g_base_rpm - corr;
        target_right = g_base_rpm + corr;
        g_line_lost_ticks = 0;
    }
    else /* CTRL_LINE */
    {
        float corr;
        if (IR_AllWhite())
        {
            /* Line lost: hold the last correction so the car keeps curving the
             * same way through the gap (or the arc end) instead of jerking
             * straight. The path layer counts sustained white as a vertex. */
            g_line_lost_ticks++;
            corr = g_last_line_corr;
        }
        else
        {
            int line_err = IR_Error();
            corr = KP_LINE * (float)line_err
                 + KD_LINE * (float)(line_err - g_prev_line_err);
            g_prev_line_err = line_err;
            g_last_line_corr = corr;
            g_line_lost_ticks = 0;
        }
        target_left  = g_base_rpm + g_line_bias + corr;
        target_right = g_base_rpm - g_line_bias - corr;
    }

    if (target_left  < 0.0f) target_left  = 0.0f;
    if (target_right < 0.0f) target_right = 0.0f;

    int pwm_left  = speed_pi(target_left,  g_rpm_left,  FF_LEFT,  &g_int_left);
    int pwm_right = speed_pi(target_right, g_rpm_right, FF_RIGHT, &g_int_right);
    Set_PWM(pwm_left, pwm_right);
}

/* 10 ms periodic interrupt from TIMG0 (TIMER_0). */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            Control_Tick();
            break;
        default:
            break;
    }
}
