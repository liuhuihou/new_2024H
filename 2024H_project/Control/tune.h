#ifndef _TUNE_H
#define _TUNE_H

/*
 * UART0 live tuner. Poll Tune_Poll() from the foreground loop. It reads any
 * bytes waiting in the UART0 RX FIFO, assembles newline-terminated commands,
 * and applies them so PID gains and base speed can be changed without a
 * recompile/reflash.
 *
 * Command syntax (end each line with CR or LF):
 *   p              print all gains and base speed
 *   g <id> <val>   set gain <id> (0..7) to <val>   e.g.  g 6 4.5
 *   b <val>        set base speed (RPM)             e.g.  b 60
 *   f <val>        set arc feed-forward bias (RPM)
 * Gain ids: 0 KpSpd 1 KiSpd 2 FFL 3 FFR 4 KpHead 5 KdHead 6 KpLine 7 KdLine
 */

void Tune_Poll(void);

#endif /* _TUNE_H */
