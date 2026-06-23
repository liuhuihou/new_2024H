#include "show.h"
#include "oled.h"
#include "control.h"
#include "ir.h"

/*
 * Maps controller/path/IR telemetry onto the OLED. Labels are written once;
 * Show_Update repaints only the value cells each call.
 */

static const char *mode_text(ControlMode m)
{
    switch (m)
    {
        case CTRL_STOP:     return "STOP";
        case CTRL_STRAIGHT: return "STRT";
        case CTRL_LINE:     return "LINE";
        default:            return "----";
    }
}

void Show_Init(void)
{
    OLED_Init();
    OLED_Clear();

    OLED_ShowString(0, 0, "2024H Auto Car");
    OLED_ShowString(0, 1, "REQ:   STATE");
    OLED_ShowString(0, 2, "MODE:");
    OLED_ShowString(0, 3, "Vertex:");
    OLED_ShowString(0, 4, "IR:");
    OLED_ShowString(0, 6, "L:");
    OLED_ShowString(0, 7, "R:");
    OLED_ShowString(9, 6, "rpm");
    OLED_ShowString(9, 7, "rpm");
}

void Show_Update(PathReq req, uint8_t running, uint8_t finished)
{
    uint8_t ir = IR_Read();
    int i;

    /* REQ number + run state */
    OLED_ShowChar(4, 1, (char)('0' + (int)req));
    if (finished)
        OLED_ShowString(7, 1, "DONE  ");
    else if (running)
        OLED_ShowString(7, 1, "RUN   ");
    else
        OLED_ShowString(7, 1, "SELECT");

    /* control mode */
    OLED_ShowString(6, 2, mode_text(Control_GetMode()));

    /* vertices passed */
    OLED_ShowNum(8, 3, (uint32_t)Path_VertexCount(), 2);

    /* 4 IR bits, MSB (DH1) first */
    for (i = 0; i < 4; i++)
        OLED_ShowChar((uint8_t)(4 + i * 2), 4, (ir & (0x08 >> i)) ? '1' : '0');

    /* wheel speeds, one decimal place; RPM*10 as integer */
    OLED_ShowTenths(2, 6, (int32_t)(Control_GetRpmL() * 10.0f), 4);
    OLED_ShowTenths(2, 7, (int32_t)(Control_GetRpmR() * 10.0f), 4);
}
