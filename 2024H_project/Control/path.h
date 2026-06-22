#ifndef _PATH_H
#define _PATH_H

#include "ti_msp_dl_config.h"

/*
 * Event-driven path runner for problem H. No hard-coded distances: the car
 * starts driving straight, switches to line-following when the IR sensors
 * catch the black arc, and switches back to straight (counting a vertex)
 * when the sensors run fully onto white at the arc's end.
 *
 * A "requirement" only differs in how many vertices are passed before the
 * final stop. Vertex counts are confirmed on the bench against real IR edges.
 */
typedef enum
{
    REQ_1 = 1,   /* A -> B, stop. (1 vertex event) */
    REQ_2,       /* A B C D A loop, stop at A. */
    REQ_3,       /* bowtie A C B D A, stop at A. */
    REQ_4        /* requirement-3 path x4 laps. */
} PathReq;

void Path_Init(PathReq req);   /* select requirement, arm the runner */
void Path_Start(void);         /* begin moving (called on key/start) */
void Path_Stop(void);          /* force stop */
void Path_Tick(void);          /* call from the foreground loop (~10 ms) */

int  Path_VertexCount(void);   /* vertices passed so far */
int  Path_IsFinished(void);    /* 1 once the car has stopped at the goal */

#endif /* _PATH_H */
