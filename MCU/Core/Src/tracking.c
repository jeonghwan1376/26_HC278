#include "tracking.h"
#include "xyz_motion.h"
#include "cmsis_os.h"
#include <string.h>

extern TIM_HandleTypeDef htim4;

/* ======================================================
 * Private typedef
 * ====================================================== */

typedef struct
{
  uint8_t active;        // 트레이 정보 사용 중
  uint8_t entered;       // 적재 가능 영역 진입 여부
  uint8_t reserved;      // 로봇이 이 트레이를 사용하기로 예약했는지
  uint8_t pc10_corrected;  // PC10 기준 위치 보정 완료 여부

  uint32_t position_tick;  // 현재 위치 기준이 설정된 시각
  int32_t reference_y_mm;  // position_tick 시점의 기준 Y좌표
  int32_t y_mm;          // 현재 트레이 Y좌표
  uint32_t sequence;     // 실제 트레이 순번
  
  /*
   * PC9 감지 시각과 해당 트레이의 측정 속도
   *
   * speed_milli_mm_per_sec:
   *   실제 mm/s 값의 1000배
   *
   * 예:
   *   26.25 mm/s → 26250
   */
  uint32_t pc9_detect_tick;
  int32_t speed_milli_mm_per_sec;
} TrayTrack_t;

/* ======================================================
 * Private variables
 * ====================================================== */

static volatile TrayTrack_t tray_list[MAX_TRAY_COUNT];

static volatile uint8_t tray_write_index = 0;
static volatile int8_t current_tray_index = -1;
static volatile uint8_t tray_place_count = 0;

static volatile uint32_t tray_sequence_counter = 0;
static volatile TrayDebugInfo_t tray_debug_info;

/* 디버깅용 */
static volatile int32_t tray_last_remain_y = 0;
static volatile int32_t tray_last_wait_condition_y = 0;
static volatile int32_t tray_last_break_y = 0;


static volatile uint32_t tray_prediction_elapsed_ms = 0U;
static volatile uint32_t tray_prediction_max_elapsed_ms = 0U;

/* ======================================================
 * Private function prototypes
 * ====================================================== */

static int8_t Tray_FindNearestActive(void);


static int8_t Tray_FindFreeSlot(void);

static int8_t Tray_FindPc10CorrectionTarget(void);
static void Tray_Reserve(uint8_t index);
static void Tray_Clear(uint8_t index);

static int32_t Tray_CalculateSecondSyncDelay(uint8_t tray_index);
static int32_t Tray_CalculatePredictiveStartY(uint8_t tray_index, int32_t tray_front_y,
   int32_t placement_offset_from_front_y, int32_t arrival_reserve_y, int32_t tracking_compensation_y);

/* ======================================================
 * Weak callbacks
 * main.c에서 같은 이름의 함수를 만들면 main.c 함수가 우선 사용
 * ====================================================== */

__weak void Tray_OnFirstPlacementMoveStarted(void)
{
  // robot_process.c에 구현되어 있음
}

__weak void Tray_OnTrackMoveDone(void)
{
  /* main.c에서 UART_Send_IT("5\r\n") 등을 연결 */
}

__weak void Tray_PumpOff(void)
{
  /*
 * 애플리케이션 계층에서 펌프 OFF 동작을 구현한다.
 * 현재 실제 구현은 robot_process.c에 있다.
 */
}

/* ======================================================
 * Public functions
 * ====================================================== */

void Tray_Init(void)
{
  memset((void *)tray_list, 0, sizeof(tray_list));

  tray_write_index = 0;
  current_tray_index = -1;
  tray_place_count = 0;
  tray_sequence_counter = 0;
  memset((void *)&tray_debug_info, 0, sizeof(tray_debug_info));

  tray_last_remain_y = 0;
  tray_last_wait_condition_y = 0;
  tray_last_break_y = 0;

  tray_prediction_elapsed_ms = 0U;
  tray_prediction_max_elapsed_ms = 0U;

  /* Load rail: PB2 ENA, PF4 DIR, TIM4 CH1 STEP PWM */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}


/* ====================================================== */
void Tray_HandlePc9Detected(uint32_t now_tick)
{
  Tray_AddDetection(now_tick);
}

void Tray_HandlePc10Detected(uint32_t now_tick)
{
  (void)Tray_CorrectAtPC10(now_tick);
}
/* ====================================================== */


void Tray_AddDetection(uint32_t now_tick)
{
  // uint8_t index = tray_write_index;

  int8_t free_index = Tray_FindFreeSlot();
  uint8_t index;

  if (free_index < 0)
  {
    /* 모든 슬롯이 사용 중이면 기존 활성 트레이를 덮어쓰지 않는다. */
    return;
  }

  index = (uint8_t)free_index;

  /*
  * active를 마지막에 1로 설정한다.
  * 다른 RTOS task가 Tray_UpdatePosition()을 동시에 실행해도
  * detect_tick/y_mm가 완성되기 전의 슬롯을 읽지 않도록 한다.
  */
  tray_list[index].active = 0U; // 1->0
  tray_list[index].entered = 0U;
  tray_list[index].reserved = 0U;
  tray_list[index].pc10_corrected = 0U;

  tray_list[index].position_tick = now_tick;
  tray_list[index].reference_y_mm = TRAY_PC9_START_Y_MM;
  tray_list[index].y_mm = TRAY_PC9_START_Y_MM;

  tray_list[index].pc9_detect_tick = now_tick;
  tray_list[index].speed_milli_mm_per_sec = LOAD_RAIL_NOMINAL_SPEED_MILLI_MM_PER_SEC;

  tray_sequence_counter++;
  tray_list[index].sequence = tray_sequence_counter;

  __DMB();
  tray_list[index].active = 1U;

  // tray_write_index++;
  tray_write_index = index + 1U;
  if (tray_write_index >= MAX_TRAY_COUNT)
  {
    tray_write_index = 0U;
  }
}

uint8_t Tray_CorrectAtPC10(uint32_t now_tick)
{
  int8_t tray_index;

  /* PC10 감지 직전까지의 추정 위치를 먼저 갱신한다. */
  Tray_UpdatePosition(now_tick);

  /*
   * PC9에서 등록됐지만 아직 PC10 보정을 받지 않은 트레이 중
   * 현재 추정 위치가 Y=-100 mm에 가장 가까운 트레이를 선택한다.
   * reserved 상태여도 같은 트레이이므로 보정 대상에 포함한다.
   */
  tray_index = Tray_FindPc10CorrectionTarget();
  if (tray_index < 0)
  {
    return 0U;
  }
  
  /* PC10 실제 감지 위치로 좌표 재설정 */
  tray_list[(uint8_t)tray_index].position_tick = now_tick;
  tray_list[(uint8_t)tray_index].reference_y_mm = TRAY_PC10_START_Y_MM;
  tray_list[(uint8_t)tray_index].y_mm = TRAY_PC10_START_Y_MM;
  /*
   * 트레이별 측정 속도를 사용하지 않고
   * 모든 트레이에 동일한 명목 속도 사용
   */
  tray_list[(uint8_t)tray_index].speed_milli_mm_per_sec = LOAD_RAIL_NOMINAL_SPEED_MILLI_MM_PER_SEC;
  tray_list[(uint8_t)tray_index].entered = 0U;

  __DMB();
  tray_list[(uint8_t)tray_index].pc10_corrected = 1U;
  return 1U;
}

void Tray_UpdatePosition(uint32_t now_tick)
{
  for (uint8_t i = 0U; i < MAX_TRAY_COUNT; i++)
  {
    if (tray_list[i].active == 1U)
    {
      uint32_t elapsed_ms = now_tick - tray_list[i].position_tick;
      int32_t moved_mm = (int32_t)(((int64_t)elapsed_ms * tray_list[i].speed_milli_mm_per_sec) / 1000000LL);

      tray_list[i].y_mm = tray_list[i].reference_y_mm + moved_mm;
      if (tray_list[i].y_mm >= 0)
      {
        tray_list[i].entered = 1U;
      }
    }
  }
}

int32_t Tray_GetY(uint8_t index)
{
  if ((index >= MAX_TRAY_COUNT) || (tray_list[index].active == 0U))
  {
    return 999999;
  }

  return tray_list[index].y_mm;
}

int8_t Tray_SelectForPlacement(void)
{
  int8_t tray_index = -1;

  if (tray_place_count == 0U)
  {
    /* 1번째 적재: 새 트레이 선택 */
    while (tray_index < 0)
    {
      Tray_UpdatePosition(HAL_GetTick());
      tray_index = Tray_FindNearestActive();
      osDelay(1);
    }

    Tray_Reserve((uint8_t)tray_index);
    current_tray_index = tray_index;
  }
  else
  {
    uint32_t wait_start_tick;
    /* 2번째 적재: 반드시 1번째 적재에 사용한 같은 예약 트레이 사용 */
    tray_index = current_tray_index;

    if ((tray_index < 0) || (tray_index >= MAX_TRAY_COUNT) || (tray_list[(uint8_t)tray_index].active == 0U) || (tray_list[(uint8_t)tray_index].reserved == 0U))
    {
      return -1;
    }

    /*
     * PC10 보정 대기 시작 시각 기록
     */
    wait_start_tick = HAL_GetTick();

    /* 두 번째 적재는 PC10 보정 완료 후 진행 */
    while (tray_list[(uint8_t)tray_index].pc10_corrected == 0U)
    {
      if ((HAL_GetTick() - wait_start_tick) >= 2000U)
      {
        return -1;
      }
      osDelay(1);
    }

    /* 기다리는 동안 이동한 트레이 위치를 최신값으로 갱신 */
    Tray_UpdatePosition(HAL_GetTick());
  }
  return tray_index;
}

void Tray_DebugBegin(int32_t tray_index, uint8_t place_count, uint8_t sync_mode, int32_t select_y_mm,
    int32_t placement_offset_from_front_mm, int32_t placement_y_at_select_mm, int32_t sync_lead_y_mm,
    int32_t robot_start_y_mm, int32_t wait_condition_y_mm)
{
  memset((void *)&tray_debug_info, 0, sizeof(tray_debug_info));

  tray_debug_info.tray_index = (int8_t)tray_index;
  tray_debug_info.place_count = place_count;
  tray_debug_info.sync_mode = sync_mode;
  tray_debug_info.select_y_mm = select_y_mm;
  tray_debug_info.placement_offset_from_front_mm = placement_offset_from_front_mm;
  tray_debug_info.placement_y_at_select_mm = placement_y_at_select_mm;
  tray_debug_info.sync_lead_y_mm = sync_lead_y_mm;
  tray_debug_info.robot_start_y_mm = robot_start_y_mm;
  tray_debug_info.wait_condition_y_mm = wait_condition_y_mm;
  tray_debug_info.move_start_tick = HAL_GetTick();

  if ((tray_index >= 0) && (tray_index < MAX_TRAY_COUNT))
  {
    tray_debug_info.tray_sequence = tray_list[(uint8_t)tray_index].sequence;
  }
}

void Tray_DebugRobotArrived(int32_t tray_index)
{
  Tray_UpdatePosition(HAL_GetTick());

  tray_debug_info.robot_arrive_tick = HAL_GetTick();
  tray_debug_info.tray_y_after_robot_move_mm = Tray_GetY((uint8_t)tray_index);

  tray_debug_info.late_after_robot_move_mm = tray_debug_info.tray_y_after_robot_move_mm - tray_debug_info.wait_condition_y_mm;
}

void Tray_GetDebugInfo(TrayDebugInfo_t *out_info)
{
  if (out_info == NULL)
  {
    return;
  }

  *out_info = tray_debug_info;
}

void Tray_WaitAtY(int32_t tray_index, int32_t wait_y, int32_t wait_advance_y)
{
  int32_t tray_y = 0;
  int32_t start_condition_y = 0;

  /*
   * wait_advance_y = 0  : tray_y >= wait_y일 때 동작
   * wait_advance_y = 10 : tray_y >= wait_y - 10일 때 동작, 즉 더 빠름
   * wait_advance_y = -5 : tray_y >= wait_y + 5일 때 동작, 즉 더 늦음
   */
  start_condition_y = wait_y - wait_advance_y;

  while (1)
  {
    Tray_UpdatePosition(HAL_GetTick());

    tray_y = Tray_GetY((uint8_t)tray_index);

    if (tray_y >= start_condition_y)
    {
      tray_last_break_y = tray_y;
      tray_debug_info.wait_break_y_mm = tray_y;
      tray_debug_info.wait_break_tick = HAL_GetTick();
      break;
    }
    osDelay(1);
  }
}

void Tray_MoveWaitAndSync(int32_t tray_index, int32_t offset_y, int32_t *robot_start_y, int32_t wait_advance_y)
{
  int32_t tray_y = 0;

  Tray_UpdatePosition(HAL_GetTick());

  tray_y = Tray_GetY((uint8_t)tray_index);

  /* 현재 트레이 위치보다 offset_y만큼 앞쪽으로 로봇 이동 */
  *robot_start_y = tray_y + offset_y;

  MoveToXYZ_mm(TRAY_X_MM, *robot_start_y, 60);
  while (XYZ_IsBusy())
  {
    osDelay(1);
  }

  /* 로봇은 먼저 대기 위치에 도착, 이후 트레이가 조건 위치에 도달할 때까지 대기 */
  Tray_WaitAtY(tray_index, *robot_start_y, wait_advance_y);
}

void Tray_MoveWaitAndSync_Check(int32_t tray_index, int32_t placement_offset_from_front_y, 
  int32_t sync_lead_y, int32_t *robot_start_y, int32_t wait_delay_y)
{
  int32_t tray_front_y;
  int32_t placement_y_at_select;
  int32_t wait_condition_front_y;
  int32_t tray_front_y_after_move;
  int32_t effective_wait_delay_y;

  uint32_t prediction_start_tick;

  if (robot_start_y == NULL)
  {
    return;
  }

  if ((tray_index < 0) || (tray_index >= MAX_TRAY_COUNT))
  {
    return;
  }

  effective_wait_delay_y = wait_delay_y;

  /*
    * 두 번째 적재에서는 로봇과 트레이의
    * 추적 이동 상대속도를 반영한 보정거리 사용
    */
  if (tray_place_count == 1U)
  {
    effective_wait_delay_y = Tray_CalculateSecondSyncDelay( (uint8_t)tray_index);
  }

  /*
    * PC9 또는 PC10 기준으로 현재 트레이 위치 계산
    */
  Tray_UpdatePosition(HAL_GetTick());

  tray_front_y = Tray_GetY((uint8_t)tray_index);
  placement_y_at_select = tray_front_y + placement_offset_from_front_y;
  prediction_start_tick = HAL_GetTick();

  /*
  * 예측 목표 위치:
  *
  * robot_start_y
  * = 현재 트레이 위치
  * + 실제 적재점 오프셋
  * + 로봇 이동 중 예상되는 트레이 이동거리
  * + 로봇 도착 후 남겨둘 대기거리
  * - 추적 이동 상대속도 보정거리
  *
  * sync_lead_y는 이제
  * "로봇 도착 후 트레이가 추가로 이동해야 하는 여유거리"로 사용한다.
  */
  *robot_start_y = Tray_CalculatePredictiveStartY((uint8_t)tray_index, tray_front_y, placement_offset_from_front_y, sync_lead_y, effective_wait_delay_y);

  tray_prediction_elapsed_ms = HAL_GetTick() - prediction_start_tick;

  if (tray_prediction_elapsed_ms > tray_prediction_max_elapsed_ms)
  {
    tray_prediction_max_elapsed_ms = tray_prediction_elapsed_ms;
  }

  /*
    * 트레이 앞쪽 기준의 적재 시작 조건
    *
    * 실제 적재점:
    * tray_front + placement_offset
    *
    * 적재 시작:
    * 실제 적재점 >= robot_start + compensation
    *
    * 트레이 앞쪽 기준으로 정리:
    * tray_front >= robot_start
    *             + compensation
    *             - placement_offset
    */
  wait_condition_front_y = *robot_start_y + effective_wait_delay_y - placement_offset_from_front_y;

  Tray_DebugBegin(tray_index, tray_place_count, 2U, tray_front_y,
      placement_offset_from_front_y, placement_y_at_select,
      sync_lead_y, *robot_start_y, wait_condition_front_y);

  /*
    * 예측된 목표 Y로 한 번만 이동
    */
  MoveToXYZ_mm((float)TRAY_X_MM, (float)(*robot_start_y), 60.0f);
  while (XYZ_IsBusy() != 0U)
  {
    osDelay(1U);
  }

  /*
    * 로봇 도착 시점의 트레이 위치 확인
    */
  Tray_UpdatePosition(HAL_GetTick());

  tray_front_y_after_move = Tray_GetY((uint8_t)tray_index);
  tray_last_remain_y = wait_condition_front_y - tray_front_y_after_move;
  tray_last_wait_condition_y = wait_condition_front_y;
  tray_debug_info.robot_start_y_mm = *robot_start_y;
  tray_debug_info.wait_condition_y_mm = wait_condition_front_y;
  tray_debug_info.robot_arrive_tick = HAL_GetTick();
  tray_debug_info.tray_y_after_robot_move_mm = tray_front_y_after_move;
  tray_debug_info.late_after_robot_move_mm = tray_front_y_after_move - wait_condition_front_y;

  /*
    * 로봇은 이미 예측된 적재 대기 위치에 도착해 있다.
    * 트레이의 실제 적재점이 정확한 시작 조건에 도달할 때까지 기다린다.
    */
  while (1)
  {
    Tray_UpdatePosition(HAL_GetTick());

    tray_front_y = Tray_GetY((uint8_t)tray_index);

    if (tray_front_y >= wait_condition_front_y)
    {
      tray_last_break_y = tray_front_y;
      tray_debug_info.wait_break_y_mm = tray_front_y;
      tray_debug_info.wait_break_tick = HAL_GetTick();

      break;
    }
    osDelay(1U);
  }
}


void Tray_DoPlacementMove(int32_t robot_start_y, int32_t *robot_end_y)
{
  /*
   * 현재 트레이의 첫 번째 제품 적재 이동이 실제로 시작된 시점에
   * 다음 트레이 공급 조건을 통보한다.
   *
   * tray_place_count:
   *   0U = 첫 번째 적재
   *   1U = 두 번째 적재
   */
  if (tray_place_count == 0U)
  {
    Tray_OnFirstPlacementMoveStarted();
  }
  
  /* 트레이와 Y좌표가 맞은 순간부터 컨베이어와 같은 속도로 추적하며 Z=12까지 내려감 */
  *robot_end_y = robot_start_y + TRAY_TRACK_MOVE_MM;

  MoveToXYZ_ConstSpeed_mm(TRAY_X_MM, *robot_end_y, 12, TRAY_TRACK_TIM2_ARR);
  // /*
  //  * 현재 트레이의 첫 번째 제품 적재 이동이 실제로 시작된 시점에
  //  * 다음 트레이 공급 조건을 통보한다.
  //  *
  //  * tray_place_count:
  //  *   0U = 첫 번째 적재
  //  *   1U = 두 번째 적재
  //  */

  while (XYZ_IsBusy())
  {
    osDelay(1);
  }

  /* 트레이 추적 이동 완료 알림, 실제 UART 송신은 main.c의 Tray_OnTrackMoveDone()에서 처리 */
  Tray_OnTrackMoveDone();

  /* 제품 놓기, 실제 펌프 OFF는 main.c의 Tray_PumpOff()에서 처리 */
  Tray_PumpOff();

  /* 제품을 놓은 뒤 살짝 더 이동하며 상승 */
  MoveToXYZ_ConstSpeed_mm(TRAY_X_MM, *robot_end_y + 10, 20, TRAY_TRACK_TIM2_ARR);
  while (XYZ_IsBusy())
  {
    osDelay(1);
  }
}

void Tray_UpdatePlaceFlag(void)
{
  if (tray_place_count == 0U)
  {
    /* 1번째 적재 완료, 다음 제품은 같은 트레이에 2번째 적재 */
    tray_place_count = 1U;
  }
  else
  {
    /* 2번째 적재 완료 시점에만 현재 트레이 정보를 삭제 */
    if ((current_tray_index >= 0) && (current_tray_index < MAX_TRAY_COUNT))
    {
      Tray_Clear((uint8_t)current_tray_index);
    }

    current_tray_index = -1;
    tray_place_count = 0;
  }
}

uint8_t Tray_GetPlaceCount(void)
{
  return tray_place_count;
}

int8_t Tray_GetCurrentIndex(void)
{
  return current_tray_index;
}

/* ======================================================
 * Private functions
 * ====================================================== */

 static int8_t Tray_FindFreeSlot(void)
{
  for (uint8_t offset = 0U; offset < MAX_TRAY_COUNT; offset++)
  {
    uint8_t index = (uint8_t)((tray_write_index + offset) % MAX_TRAY_COUNT);

    if (tray_list[index].active == 0U)
    {
      return (int8_t)index;
    }
  }

  return -1;
}

 static int8_t Tray_FindNearestActive(void)
{
  int8_t best_index = -1;
  int32_t best_y = -100000;

  for (uint8_t i = 0; i < MAX_TRAY_COUNT; i++)
  {
    if ((tray_list[i].active == 1) && (tray_list[i].reserved == 0))
    {
      /*
       * y가 가장 큰 트레이를 선택.
       * 예: -300, -100, +20 중에서는 +20 선택.
       * 즉, 적재 영역에 가장 가까운 트레이 선택.
       */
      if (tray_list[i].y_mm > best_y)
      {
        best_y = tray_list[i].y_mm;
        best_index = i;
      }
    }
  }

  return best_index;
}

static int8_t Tray_FindPc10CorrectionTarget(void)
{
    int8_t best_index = -1;
    int32_t best_error_mm = TRAY_PC10_MATCH_TOLERANCE_MM + 1;

    for (uint8_t i = 0U; i < MAX_TRAY_COUNT; i++)
    {
      if ((tray_list[i].active == 0U) || (tray_list[i].pc10_corrected != 0U))
      {
        continue;
      }

      int32_t error_mm = tray_list[i].y_mm - TRAY_PC10_START_Y_MM;

      if (error_mm < 0)
      {
        error_mm = -error_mm;
      }

      /* PC10 예상 위치에서 너무 먼 트레이는 이번 PC10 감지 대상이 아니다. */
      if (error_mm > TRAY_PC10_MATCH_TOLERANCE_MM)
      {
        continue;
      }

      if (error_mm < best_error_mm)
      {
        best_error_mm = error_mm;
        best_index = (int8_t)i;
      }
    }

    return best_index;
}

static void Tray_Reserve(uint8_t index)
{
  if (index < MAX_TRAY_COUNT)
  {
    tray_list[index].reserved = 1;
  }
}

static void Tray_Clear(uint8_t index)
{
  if (index < MAX_TRAY_COUNT)
  {
    tray_list[index].active = 0;
    tray_list[index].entered = 0;
    tray_list[index].reserved = 0;
    tray_list[index].pc10_corrected = 0U;

    tray_list[index].position_tick = 0U;
    tray_list[index].reference_y_mm = 0;
    tray_list[index].y_mm = 0;

    tray_list[index].pc9_detect_tick = 0U;
    tray_list[index].speed_milli_mm_per_sec = LOAD_RAIL_NOMINAL_SPEED_MILLI_MM_PER_SEC;

    tray_list[index].sequence = 0U;
  }
}

static int32_t Tray_CalculateSecondSyncDelay(uint8_t tray_index)
{
    int32_t tray_speed;
    int32_t robot_speed;
    int32_t dynamic_delay;

    if ((tray_index >= MAX_TRAY_COUNT) ||
        (tray_list[tray_index].active == 0U))
    {
        return TRAY_SECOND_SYNC_FINE_TUNE_MM;
    }

    tray_speed = tray_list[tray_index].speed_milli_mm_per_sec;

    robot_speed = TRAY_TRACK_ROBOT_SPEED_MILLI_MM_PER_SEC;

    if (robot_speed <= 0)
    {
      return TRAY_SECOND_SYNC_FINE_TUNE_MM;
    }

    /*
     * 로봇 추적 속도가 트레이 속도보다 빠른 경우:
     *
     * delay
     * = track_distance
     *   × (robot_speed - tray_speed)
     *   ÷ robot_speed
     */
    if (robot_speed > tray_speed)
    {
      dynamic_delay = (int32_t)(((int64_t)TRAY_TRACK_MOVE_MM * (robot_speed - tray_speed) + (robot_speed / 2)) / robot_speed);
    }
    else
    {
      dynamic_delay = 0;
    }

    dynamic_delay += TRAY_SECOND_SYNC_FINE_TUNE_MM;

    if (dynamic_delay < TRAY_SECOND_DYNAMIC_DELAY_MIN_MM)
    {
      dynamic_delay = TRAY_SECOND_DYNAMIC_DELAY_MIN_MM;
    }

    if (dynamic_delay > TRAY_SECOND_DYNAMIC_DELAY_MAX_MM)
    {
      dynamic_delay = TRAY_SECOND_DYNAMIC_DELAY_MAX_MM;
    }

    return dynamic_delay;
}

static int32_t Tray_CalculatePredictiveStartY(uint8_t tray_index, int32_t tray_front_y, 
  int32_t placement_offset_from_front_y, int32_t arrival_reserve_y, int32_t tracking_compensation_y)
{
    int32_t tray_speed;
    int32_t robot_target_y;
    int32_t next_target_y;
    int32_t tray_advance_y;

    uint32_t move_time_ms;

    if ((tray_index >= MAX_TRAY_COUNT) || (tray_list[tray_index].active == 0U))
    {
      return tray_front_y + placement_offset_from_front_y;
    }

    tray_speed = tray_list[tray_index] .speed_milli_mm_per_sec;

    /*
     * 최초 계산:
     * 로봇 이동시간을 아직 모르는 상태이므로
     * 트레이 이동거리를 0으로 두고 시작한다.
     */
    robot_target_y = tray_front_y + placement_offset_from_front_y + arrival_reserve_y - tracking_compensation_y;

    /*
     * 목표 Y에 따라 로봇 이동시간이 달라지고,
     * 이동시간에 따라 트레이 이동거리도 달라진다.
     *
     * 따라서 세 번 반복 계산하여 목표값을 수렴시킨다.
     */
    for (uint8_t iteration = 0U; iteration < 2U; iteration++)
    {
      move_time_ms = XYZ_EstimateMoveTimeMs((float)TRAY_X_MM, (float)robot_target_y, 60.0f);

      /*
        * speed_milli_mm_per_sec는
        * 실제 mm/s 값의 1000배이므로:
        *
        * 이동거리(mm)
        * = speed_milli × time_ms / 1,000,000
        */
      tray_advance_y = (int32_t)(((int64_t)tray_speed * move_time_ms + 500000LL) / 1000000LL);
      next_target_y = tray_front_y + placement_offset_from_front_y + tray_advance_y + arrival_reserve_y - tracking_compensation_y;

      /*
        * 계산값 차이가 1mm 이하면 수렴한 것으로 판단한다.
        */
      if ((next_target_y >= (robot_target_y - 1)) && (next_target_y <= (robot_target_y + 1)))
      {
        robot_target_y = next_target_y;
        break;
      }

    robot_target_y = next_target_y;
  }

  return robot_target_y;
}
