/**
  ******************************************************************************
  * @file    robot_process.c
  * @brief   직교로봇의 제품 흡착 및 트레이 적재 공정 제어
  *
  * @details
  * 이 파일은 다음 동작을 순차적으로 수행한다.
  *
  * 1. 프로그램 시작 후 직교로봇을 초기 위치로 이동
  * 2. 카메라로부터 전달받은 제품 좌표로 이동
  * 3. 진공 펌프를 사용하여 제품 흡착
  * 4. 현재 적재 순서에 따라 다음 트레이 공급 요청
  * 5. PC9/PC10 기반으로 추적 중인 트레이 선택
  * 6. 첫 번째 또는 두 번째 적재 위치에 맞춰 로봇과 트레이 동기화
  * 7. 제품 적재 후 초기 위치로 복귀
  *
  * main.c와 직접 결합하지 않도록 UART 상태 전송, 트레이 공급 요청,
  * 검사 컨베이어 재시작은 콜백 함수로 전달받는다.
  ******************************************************************************
  */
#include "robot_process.h"

#include "main.h"
#include "cmsis_os.h"
#include "tracking.h"
#include "TrayLoader.h"
#include "xyz_motion.h"

#include <stddef.h>
#include <stdio.h>

/*
 * main.c에서 등록한 애플리케이션 콜백 저장 영역
 * send_status : 로봇 상태 번호를 UART로 전송
 * request_tray_supply : 다음 트레이 공급 요청 플래그 설정
 * start_confirm_conveyor : 검사 컨베이어 재시작
 */
static RobotProcessCallbacks_t robot_callbacks;

static volatile uint8_t robot_pending_target_valid = 0U;
static RobotProcessTarget_t robot_pending_target;

static void RobotProcess_MoveAndWait(float x_mm, float y_mm, float z_mm);           // 직교로봇 이동 명령 후 동작 완료까지 대기
static void RobotProcess_SendStatus(const char *message);                           // 등록된 콜백을 통해 상태 문자열 전송
static void RobotProcess_VacuumOn(void);                                            // 진공 펌프 ON
static void RobotProcess_VacuumOff(void);                                           // 진공 펌프 OFF
static int32_t RobotProcess_PrepareTray(int8_t tray_index, uint8_t place_count);    // 현재 트레이 위치와 적재 순서에 따라 로봇의 적재 시작 Y 좌표 결정
// static void RobotProcess_SendTraySyncDebug(void);

/* ========================================================================== */
/* Public functions                                                           */
/* ========================================================================== */
void RobotProcess_Init(const RobotProcessCallbacks_t *callbacks)
{
    /*
     * 콜백 구조체를 먼저 NULL 상태로 초기화한다.
     * 일부 콜백만 등록되는 경우에도 잘못된 함수 호출을 방지한다.
     */
    robot_callbacks.send_status = NULL;
    // robot_callbacks.request_tray_supply = NULL;  // 기존 트레이 공급
    robot_callbacks.notify_first_placement_started = NULL;  // 새로운 기준의 트레이 공급
    robot_callbacks.start_confirm_conveyor = NULL;

    robot_pending_target_valid = 0U;

    /* 유효한 콜백 구조체가 전달된 경우 전체 내용을 복사한다. */
    if (callbacks != NULL)
    {
        robot_callbacks = *callbacks;
    }

    /* 프로그램 시작 시 펌프가 우발적으로 켜지지 않도록 OFF 처리한다. */
    RobotProcess_VacuumOff();
}

void RobotProcess_RunInitialSequence(void)
{
    /* 시스템 초기 안정화 대기 */
    osDelay(ROBOT_PROCESS_INITIAL_DELAY_MS);

    /* 직교로봇 초기 위치 이동 */
    RobotProcess_MoveAndWait(ROBOT_PROCESS_HOME_X_MM, ROBOT_PROCESS_HOME_Y_MM, ROBOT_PROCESS_HOME_Z_MM);

    /* 상태 1: 직교로봇 초기 위치 이동 완료 */
    RobotProcess_SendStatus("1\r\n");
}

void RobotProcess_Run(const RobotProcessTarget_t *target)
{
    int8_t tray_index;
    int32_t robot_start_y;
    int32_t robot_end_y = 0;
    uint8_t place_count;

    /* 잘못된 포인터가 전달된 경우 공정을 시작하지 않는다. */
    if (target == NULL)
    {
        return;
    }

    /* 1. 카메라로 받은 제품 좌표 상부로 이동 */
    RobotProcess_MoveAndWait(target->x_mm, target->y_mm, target->z_mm);
    /* 상태 2: 제품 흡착 준비 위치 이동 완료 */
    RobotProcess_SendStatus("2\r\n");

    /* 2. 진공 흡착 */
    RobotProcess_VacuumOn();

    /* 제품 흡착 높이까지 Z축 하강 */
    RobotProcess_MoveAndWait(target->x_mm, target->y_mm, ROBOT_PROCESS_PICK_Z_MM);
    /* 상태 3: 제품 흡착 위치 도달 */
    RobotProcess_SendStatus("3\r\n");
    /* 제품을 흡착한 상태로 Z축 상승 */
    RobotProcess_MoveAndWait(target->x_mm, target->y_mm, ROBOT_PROCESS_LIFT_Z_MM);
    /* 상태 4: 제품 흡착 및 상승 완료 */
    RobotProcess_SendStatus("4\r\n");

    /*
     * 3. 현재 적재 순서를 한 번만 읽어 이번 사이클 전체에서 동일하게 사용
     *
     * Tray_GetPlaceCount():
     *   0U = 첫 번째 적재
     *   1U = 두 번째 적재
     */
    place_count = Tray_GetPlaceCount();

    /* 4. 검사 컨베이어 재시작 */
    if (robot_callbacks.start_confirm_conveyor != NULL)
    {
        robot_callbacks.start_confirm_conveyor();
    }

    /* 5. 사용할 트레이 선택 및 적재 시작 위치 결정 */
    tray_index = Tray_SelectForPlacement();

    if (tray_index < 0) // 적재할 트레이가 없는 상황
    {
        /* 현재 Tray_SelectForPlacement()은 사용 가능한 트레이가 생길 때까지 대기하는 구조이므로 일반적으로 이 경로에는 들어오지 않는다. */
        RobotProcess_VacuumOff();

        /* 직교로봇 초기 위치 복귀 */
        RobotProcess_MoveAndWait(ROBOT_PROCESS_HOME_X_MM, ROBOT_PROCESS_HOME_Y_MM, ROBOT_PROCESS_HOME_Z_MM);
        /* 상태 7: 예외 종료 후 사이클 완료 처리 */
        RobotProcess_SendStatus("7\r\n");

        return;
    }

    /* 첫 번째/두 번째 적재 여부와 트레이 위치를 기준으로 시작 Y를 결정한다. */
    robot_start_y = RobotProcess_PrepareTray(tray_index, place_count);

    /* 6. 트레이와 함께 이동하면서 제품 적재 */
    Tray_DoPlacementMove(robot_start_y, &robot_end_y);
    Tray_UpdatePlaceFlag();

    /* 7. 원점 복귀 및 사이클 완료 통보 */
    RobotProcess_MoveAndWait(ROBOT_PROCESS_HOME_X_MM, ROBOT_PROCESS_HOME_Y_MM, ROBOT_PROCESS_HOME_Z_MM);
    /* 상태 7: 제품 흡착 및 트레이 적재 전체 사이클 완료 */
    RobotProcess_SendStatus("7\r\n");
}

/* ====================================================== */
uint8_t RobotProcess_SubmitTarget(const RobotProcessTarget_t *target)
{
    uint8_t accepted = 0U;
    uint32_t primask;

    if (target == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (robot_pending_target_valid == 0U)
    {
        robot_pending_target = *target;
        __DMB();
        robot_pending_target_valid = 1U;
        accepted = 1U;
    }

    __set_PRIMASK(primask);
    return accepted;
}

uint8_t RobotProcess_RunPending(void)
{
    RobotProcessTarget_t target;
    uint8_t has_target = 0U;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    if (robot_pending_target_valid != 0U)
    {
        target = robot_pending_target;
        robot_pending_target_valid = 0U;
        has_target = 1U;
    }

    __set_PRIMASK(primask);

    if (has_target != 0U)
    {
        RobotProcess_Run(&target);
    }

    return has_target;
}

uint8_t RobotProcess_HasPendingTarget(void)
{
    return robot_pending_target_valid;
}
/* ====================================================== */

static int32_t RobotProcess_PrepareTray(int8_t tray_index, uint8_t place_count)
{
    int32_t tray_y;
    int32_t robot_start_y = 0;

    /*
     * 현재 추적 중인 트레이의 Y 위치 확인
     * PC10 보정이 완료된 트레이는 PC10 기준 Y=-100 mm부터 계산된다.
     */
    tray_y = Tray_GetY((uint8_t)tray_index);

    if (place_count == 0U)
    {
        /* 첫 번째 적재 */
        if (tray_y < 0)
        {
            /* 트레이가 아직 적재 기준 Y=0에 도달하지 않은 경우, 로봇을 먼저 X=TRAY_X_MM, Y=0, Z=60으로 이동시킨 뒤 대기한다. */
            robot_start_y = 0;

            RobotProcess_MoveAndWait((float)TRAY_X_MM, (float)robot_start_y, ROBOT_PROCESS_LIFT_Z_MM);
            /* 트레이가 로봇 적재 시작 위치에 도달할 때까지 대기 */
            Tray_WaitAtY(tray_index, robot_start_y, ROBOT_PROCESS_FIRST_WAIT_ADVANCE_MM);
            /* 트레이와 로봇의 기계적 응답 오차 보정을 위한 추가 대기 */
            osDelay(ROBOT_PROCESS_FIRST_POST_WAIT_MS);
        }
        else
        {
            /* 트레이가 이미 Y=0을 지난 경우 현재 위치에 맞춰 동기화한다. */
            // Tray_MoveWaitAndSync_Check(tray_index, TRAY_FRONT_Y_OFFSET_MM, &robot_start_y, ROBOT_PROCESS_FIRST_SYNC_DELAY_MM);
        }
    }
    else
    {
        /* 두 번째 적재 */
        Tray_MoveWaitAndSync_Check(tray_index, TRAY_SECOND_PLACE_OFFSET_FROM_FRONT_MM, TRAY_SECOND_SYNC_LEAD_MM, 
            &robot_start_y, ROBOT_PROCESS_SECOND_SYNC_DELAY_MM);
    }

    return robot_start_y;
}

static void RobotProcess_MoveAndWait(float x_mm, float y_mm, float z_mm)
{
    MoveToXYZ_mm(x_mm, y_mm, z_mm);
    while (XYZ_IsBusy() != 0U)
    {
        osDelay(1U);
    }
}

static void RobotProcess_SendStatus(const char *message)
{
    /* 문자열과 콜백 함수가 모두 유효한 경우에만 상태를 전송한다. */
    if ((message != NULL) && (robot_callbacks.send_status != NULL))
    {
        robot_callbacks.send_status(message);
    }
}

static void RobotProcess_VacuumOn(void)
{
    /* 펌프 Active-Low: RESET = ON */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
}

static void RobotProcess_VacuumOff(void)
{
    /* 펌프 Active-Low: SET = OFF */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
}

/* tracking.c에서 호출하는 애플리케이션 훅 */
void Tray_OnFirstPlacementMoveStarted(void)
{
    /*
     * 첫 번째 제품의 추적 적재 이동이 시작된 직후 호출된다.
     * main.c에서 등록한 TrayLoader_NotifyFirstPlacementStarted()를
     * 호출하여 첫 적재 시작 카운터를 증가시킨다.
     */
    if (robot_callbacks.notify_first_placement_started != NULL)
    {
        robot_callbacks.notify_first_placement_started();
    }
}

/* tracking.c에서 호출하는 애플리케이션 훅 */
void Tray_OnTrackMoveDone(void)
{
    /* 트레이 추적 이동 완료 콜백 */
    RobotProcess_SendStatus("5\r\n");
}

void Tray_PumpOff(void)
{
    RobotProcess_VacuumOff();
}
