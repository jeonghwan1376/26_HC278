#include "app_command.h"

#include "confirm_conveyor.h"
#include "rejector.h"
#include "robot_process.h"
#include "TrayLoader.h"
#include "uart.h"

#include <stdio.h>
#include <string.h>

void AppCommand_ProcessLine(const char *line)
{
    RobotProcessTarget_t target;
    char command_type;

    if ((line == NULL) || (line[0] == '\0'))
    {
        return;
    }

    /* 제품 좌표 검출 직후 검사 컨베이어 정지 */
    if ((strcmp(line, "p") == 0) || (strcmp(line, "P") == 0))
    {
        ConfirmConveyor_Stop();
        return;
    }

    if ((strcmp(line, "t") == 0) ||
    (strcmp(line, "T") == 0))
    {
        char message[128];

        TrayLoader_NotifyNormalProductDetected();

        snprintf(
            message,
            sizeof(message),
            "T_STATUS,N=%lu,DMA=%u,SUP=%lu,FIRST=%lu\r\n",
            (unsigned long)TrayLoader_GetNormalProductCount(),
            (unsigned int)TrayLoader_HasDmaError(),
            (unsigned long)TrayLoader_GetSuppliedTrayCount(),
            (unsigned long)TrayLoader_GetFirstPlacementStartedCount()
        );

        UART_SendMessage(message);
        return;
    }

    /* 웹캠 최종 불량 판정: 리젝터 왕복 1회 요청 */
    if ((strcmp(line, "f") == 0) || (strcmp(line, "F") == 0))
    {
        Rejector_RequestCycle();
        return;
    }

    /* 좌표 명령: G,X,Y,Z */
    if (sscanf(line, "%c,%f,%f,%f",
               &command_type,
               &target.x_mm,
               &target.y_mm,
               &target.z_mm) == 4)
    {
        if ((command_type == 'G') || (command_type == 'g'))
        {
            (void)RobotProcess_SubmitTarget(&target);
        }
    }
}
