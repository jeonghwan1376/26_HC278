#ifndef TRACKING_H
#define TRACKING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ======================================================
 * Tray tracking parameter
 * ====================================================== */

#define MAX_TRAY_COUNT              8

#define TRAY_X_MM                   253   // 트레이 적재 X좌표
#define TRAY_TRACK_MOVE_MM          50    // 적재 중 트레이 추적 이동 거리

/*
 * Tray_GetY()가 반환하는 값은 트레이 앞쪽 기준점의 Y 위치이다.
 *
 * 부호 기준:
 *   +값 = 트레이 이동 방향(+Y) 쪽
 *   -값 = 트레이 이동 반대 방향(-Y) 쪽
 */
#define TRAY_FIRST_PLACE_OFFSET_FROM_FRONT_MM    0
#define TRAY_SECOND_PLACE_OFFSET_FROM_FRONT_MM   0  // 두 번째 적재 오프셋(수정해야 함)

/*
 * 로봇이 트레이보다 먼저 이동해서 기다리는 거리
 */
#define TRAY_FIRST_SYNC_LEAD_MM                  80
#define TRAY_SECOND_SYNC_LEAD_MM                 40

#define TRAY_SYNC_TOLERANCE_MM      1

/* ======================================================
 * Load rail velocity parameter
 * ====================================================== */

#define LOAD_RAIL_STEPS_PER_REV     1600.0f
#define LOAD_RAIL_MM_PER_REV        84.0f
#define LOAD_RAIL_MM_PER_STEP       (LOAD_RAIL_MM_PER_REV / LOAD_RAIL_STEPS_PER_REV)

/*
 * PC9: 새 트레이 최초 등록 위치
 * PC10: 적재 가능 위치 Y=0 직전의 정밀 보정 위치
 */
#define TRAY_PC9_TO_LOAD_Y0_MM       425
#define TRAY_PC9_START_Y_MM          (-TRAY_PC9_TO_LOAD_Y0_MM)

#define TRAY_PC10_TO_LOAD_Y0_MM      100
#define TRAY_PC10_START_Y_MM         (-TRAY_PC10_TO_LOAD_Y0_MM)

#define LOAD_RAIL_STEP_PER_SEC      500.0f
#define LOAD_RAIL_SPEED_MM_PER_SEC  (LOAD_RAIL_STEP_PER_SEC * LOAD_RAIL_MM_PER_STEP)

/* 직교로봇 Y축을 load rail과 비슷한 속도로 움직이기 위한 TIM2 ARR */
#define TRAY_TRACK_TIM2_ARR         951U   // 1427(default) -> 713(200%) -> 951(150%) | 1141(125%)

/* 직교 로봇 y축 제한 범위 */
#define TRAY_POSITION_LIMIT_Y_MM    420


/*
 * 두 번째 적재 동기화 재보정값
 *
 * 로봇이 최초 계산 위치로 이동하는 동안 트레이가 기다림 기준점을
 * 지나간 경우, 현재 트레이 위치를 기준으로 로봇 위치를 다시 맞춘다.
 */
#define TRAY_SECOND_SYNC_TOLERANCE_MM       1
#define TRAY_SECOND_SYNC_REARM_LEAD_MM      10
#define TRAY_SECOND_SYNC_MAX_CORRECTIONS    8U

/*
 * PC9와 PC10 사이의 실제 거리
 *
 * -100 - (-425) = 325 mm
 */
#define TRAY_PC9_TO_PC10_DISTANCE_MM  (TRAY_PC9_TO_LOAD_Y0_MM - TRAY_PC10_TO_LOAD_Y0_MM)

/* 기본 트레이 속도: 26.25 mm/s */
#define LOAD_RAIL_NOMINAL_SPEED_MILLI_MM_PER_SEC  26250

/*
 * 잘못된 센서 매칭이나 센서 노이즈로
 * 비정상적인 속도가 적용되지 않도록 제한
 */
#define LOAD_RAIL_MIN_VALID_SPEED_MILLI_MM_PER_SEC  15000
#define LOAD_RAIL_MAX_VALID_SPEED_MILLI_MM_PER_SEC  40000

#define TRAY_PC10_MATCH_TOLERANCE_MM  60


/*
 * 직교로봇의 트레이 추적 이동 속도.
 *
 * 실제 속도가 39.39 mm/s라면 39390으로 설정한다.
 * 정확한 값은 실측값을 사용하는 것이 가장 좋다.
 */
#define TRAY_TRACK_ROBOT_SPEED_MILLI_MM_PER_SEC  39390

/*
 * 계산 결과에 추가하는 최종 미세 보정값.
 *
 * 양수: 적재 시작을 늦춤
 * 음수: 적재 시작을 당김
 */
#define TRAY_SECOND_SYNC_FINE_TUNE_MM  20 // 10mm 타이밍 늦춤

/* 비정상적인 계산 결과 제한 */
#define TRAY_SECOND_DYNAMIC_DELAY_MIN_MM  0
#define TRAY_SECOND_DYNAMIC_DELAY_MAX_MM  40

/* ======================================================
 * Tray timing debug
 * ====================================================== */

typedef struct
{
  uint32_t tray_sequence;
  int8_t tray_index;
  uint8_t place_count;
  uint8_t sync_mode;               /* 1: y<0 fixed Y=0 wait, 2: dynamic sync */

  int32_t select_y_mm;
  int32_t placement_offset_from_front_mm; /* 앞쪽 기준 -> 실제 적재점 오프셋 */
  int32_t placement_y_at_select_mm;       /* 선택 시 실제 적재점 Y */
  int32_t sync_lead_y_mm;                 /* 로봇 선행 대기 거리 */
  int32_t robot_start_y_mm;
  int32_t wait_condition_y_mm;
  int32_t tray_y_after_robot_move_mm;
  int32_t wait_break_y_mm;
  int32_t late_after_robot_move_mm; /* >0이면 로봇 도착 전에 트레이가 조건점을 이미 통과 */

  uint32_t move_start_tick;
  uint32_t robot_arrive_tick;
  uint32_t wait_break_tick;
} TrayDebugInfo_t;

void Tray_DebugBegin(int32_t tray_index, uint8_t place_count, uint8_t sync_mode, int32_t select_y_mm,
  int32_t placement_offset_from_front_mm, int32_t placement_y_at_select_mm, int32_t sync_lead_y_mm,
  int32_t robot_start_y_mm, int32_t wait_condition_y_mm);

void Tray_DebugRobotArrived(int32_t tray_index);
void Tray_GetDebugInfo(TrayDebugInfo_t *out_info);

/* ======================================================
 * Public functions
 * ====================================================== */

void Tray_Init(void);

void Tray_AddDetection(uint32_t now_tick);

/* SensorCallbacks_t에 직접 연결 가능한 PC9/PC10 이벤트 처리 함수 */
void Tray_HandlePc9Detected(uint32_t now_tick);
void Tray_HandlePc10Detected(uint32_t now_tick);

/* PC10 감지: PC9에서 등록된 동일 트레이를 Y=-100 mm로 보정 */
uint8_t Tray_CorrectAtPC10(uint32_t now_tick);

void Tray_UpdatePosition(uint32_t now_tick);

int8_t Tray_SelectForPlacement(void);
int32_t Tray_GetY(uint8_t index);

void Tray_WaitAtY(int32_t tray_index, int32_t wait_y, int32_t wait_advance_y);
void Tray_MoveWaitAndSync(int32_t tray_index, int32_t offset_y, int32_t *robot_start_y, int32_t wait_advance_y);
void Tray_MoveWaitAndSync_Check(int32_t tray_index, int32_t placement_offset_from_front_y,
  int32_t sync_lead_y, int32_t *robot_start_y, int32_t wait_delay_y);
void Tray_DoPlacementMove(int32_t robot_start_y, int32_t *robot_end_y);
void Tray_UpdatePlaceFlag(void);

uint8_t Tray_GetPlaceCount(void);
int8_t Tray_GetCurrentIndex(void);

/* ======================================================
 * Callback functions
 *
 * main.c에서 실제 동작을 연결한다.
 * tracking.c는 UART나 펌프 핀을 직접 알지 않도록 분리한다.
 * ====================================================== */

void Tray_OnFirstPlacementMoveStarted(void);  // 트레이 공급 요청을 여기서 하기 위해 추가함
void Tray_OnTrackMoveDone(void);
void Tray_PumpOff(void);

#ifdef __cplusplus
}
#endif

#endif
