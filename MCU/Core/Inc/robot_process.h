#ifndef ROBOT_PROCESS_H
#define ROBOT_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 다음 트레이 공급 요청 시점
 *
 * 1U: 현재 트레이의 첫 번째 적재 사이클에서 다음 트레이 공급
 * 2U: 현재 트레이의 두 번째 적재 사이클에서 다음 트레이 공급
 */

/* 직교로봇 위치 및 적재 동기화 파라미터 */
#ifndef ROBOT_PROCESS_INITIAL_DELAY_MS
#define ROBOT_PROCESS_INITIAL_DELAY_MS                 3000U
#endif

#ifndef ROBOT_PROCESS_HOME_X_MM
#define ROBOT_PROCESS_HOME_X_MM                        100.0f
#endif

#ifndef ROBOT_PROCESS_HOME_Y_MM
#define ROBOT_PROCESS_HOME_Y_MM                        0.0f
#endif

#ifndef ROBOT_PROCESS_HOME_Z_MM
#define ROBOT_PROCESS_HOME_Z_MM                        50.0f
#endif

#ifndef ROBOT_PROCESS_PICK_Z_MM
#define ROBOT_PROCESS_PICK_Z_MM                        0.0f
#endif

#ifndef ROBOT_PROCESS_LIFT_Z_MM
#define ROBOT_PROCESS_LIFT_Z_MM                        60.0f
#endif

#ifndef ROBOT_PROCESS_FIRST_WAIT_ADVANCE_MM
#define ROBOT_PROCESS_FIRST_WAIT_ADVANCE_MM            5
#endif

#ifndef ROBOT_PROCESS_FIRST_POST_WAIT_MS
#define ROBOT_PROCESS_FIRST_POST_WAIT_MS               15U   // 5U->6U->10U
#endif

#ifndef ROBOT_PROCESS_FIRST_SYNC_DELAY_MM
#define ROBOT_PROCESS_FIRST_SYNC_DELAY_MM              0
#endif

#ifndef ROBOT_PROCESS_SECOND_SYNC_DELAY_MM
#define ROBOT_PROCESS_SECOND_SYNC_DELAY_MM             32   // 20->0->-10(260721/03:50)->42(260721/23:47)->20(260722/21:14) -> -32(260724/04:24)

#endif

typedef struct
{
    float x_mm;
    float y_mm;
    float z_mm;
} RobotProcessTarget_t;

typedef void (*RobotProcessStatusCallback_t)(const char *message);
typedef void (*RobotProcessVoidCallback_t)(void);

typedef struct
{
    RobotProcessStatusCallback_t send_status;
    // RobotProcessVoidCallback_t request_tray_supply;  // 기존 트레이 공급 시 사용
    RobotProcessVoidCallback_t notify_first_placement_started;  // 새로운 기준의 트레이 공급
    RobotProcessVoidCallback_t start_confirm_conveyor;
} RobotProcessCallbacks_t;

/*
 * 콜백을 등록하고 진공 펌프를 OFF 상태로 초기화한다.
 * callbacks가 NULL이거나 일부 콜백이 NULL이어도 로봇 동작 자체는 수행된다.
 */
void RobotProcess_Init(const RobotProcessCallbacks_t *callbacks);

/* 프로그램 시작 후 초기 대기, 원점 이동, 상태 "1" 송신을 수행한다. */
void RobotProcess_RunInitialSequence(void);

/* 제품 흡착부터 트레이 적재, 원점 복귀, 상태 "7" 송신까지 수행한다. */
void RobotProcess_Run(const RobotProcessTarget_t *target);

/* UART 명령 태스크에서 좌표를 제출하고 로봇 태스크에서 실행하기 위한 단일 슬롯 */
uint8_t RobotProcess_SubmitTarget(const RobotProcessTarget_t *target);
uint8_t RobotProcess_RunPending(void);
uint8_t RobotProcess_HasPendingTarget(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PROCESS_H */
