#ifndef _SHOW_H
#define _SHOW_H

#include "ti_msp_dl_config.h"
#include "path.h"

/*
 * OLED status screen for the line-following car. Organizes the telemetry that
 * empty.c already prints over UART0 into a fixed 128x64 layout:
 *
 *   Line0: "2024H Auto Car"
 *   Line1: "REQ:n  STATE"        (SELECT / RUN / DONE)
 *   Line2: "MODE: STOP/STRT/LINE"
 *   Line3: "Vertex: nn"
 *   Line4: "IR: b b b b"         (4-way sensor bits)
 *   Line6: "L:xxxx.x rpm"
 *   Line7: "R:xxxx.x rpm"
 *
 * Static labels are drawn once in Show_Init; Show_Update only rewrites the
 * changing fields so the bit-banged refresh stays cheap.
 */

void Show_Init(void);

/* Refresh dynamic fields. `running` 0 = selecting, 1 = running; `finished`
 * marks a completed run. Pulls the rest from Control_*/IR_*/Path_*. */
void Show_Update(PathReq req, uint8_t running, uint8_t finished);

#endif /* _SHOW_H */
