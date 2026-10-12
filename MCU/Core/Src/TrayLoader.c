#include "TrayLoader.h"
#include "TrayLoaderDmaProfiles.h"
#include "cmsis_os.h"

#include "uart.h"
#include <stdio.h>

/*
 * CubeMX에서 생성된 타이머 핸들
 * - TIM5 CH4 / PA3 : 트레이 로더 벨트 STEP
 * - TIM8 CH2 / PC7 : 볼스크류 STEP
 * - TIM5_UP / TIM8_UP DMA가 각 펄스 종료 시 다음 ARR을 전송한다.
 */
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

/* ============================================================
 * 기구 및 공급 설정
 * ============================================================ */
#define TRAY_LOADER_SUPPLY_MAX_COUNT        (4U)
#define TRAY_LOADER_FIRST_SUPPLY_BELT_MM    (108)
#define TRAY_LOADER_NEXT_SUPPLY_BELT_MM     (45)
#define TRAY_LOADER_SCREW_STROKE_MM         (130)

/* DMA 프로파일을 생성할 때 사용한 ARR 설정값 */
#define LOADER_BELT_ARR_SLOW                (1999U)
#define LOADER_SCREW_ARR_SLOW               (1999U)

/*
 * PWM mode 2를 사용한다. Update 순간 출력이 LOW가 되고,
 * CCR 시점에 HIGH가 되므로 DMA 완료 IRQ가 다음 rising edge 전에
 * 타이머를 정지할 수 있다. 1 MHz 기준 100은 LOW 100 us이다.
 */
#define LOADER_STEP_LOW_TICKS               (100U)

#define LOADER_BELT_TIM                     (&htim5)
#define LOADER_BELT_CHANNEL                 (TIM_CHANNEL_4)
#define LOADER_SCREW_TIM                    (&htim8)
#define LOADER_SCREW_CHANNEL                (TIM_CHANNEL_2)

/* 벨트축 GPIO */
#define LOADER_BELT_EN_PORT                 GPIOE
#define LOADER_BELT_EN_PIN                  GPIO_PIN_4
#define LOADER_BELT_DIR_PORT                GPIOE
#define LOADER_BELT_DIR_PIN                 GPIO_PIN_6

/* 볼스크류축 GPIO */
#define LOADER_SCREW_EN_PORT                GPIOF
#define LOADER_SCREW_EN_PIN                 GPIO_PIN_8
#define LOADER_SCREW_DIR_PORT               GPIOF
#define LOADER_SCREW_DIR_PIN                GPIO_PIN_9

/* ============================================================
 * 내부 상태
 * ============================================================ */
static volatile uint8_t belt_dma_running_l = 0U;
static volatile uint8_t screw_dma_running_l = 0U;
static volatile uint8_t move_in_progress_l = 0U;
static volatile uint8_t dma_error_l = 0U;

static volatile int32_t cur_belt_mm = 0;
static volatile int32_t cur_screw_mm = 0;
static volatile int32_t pending_belt_mm = 0;
static volatile int32_t pending_screw_mm = 0;

static volatile uint8_t tray_supply_count_l = 0U;

static volatile uint32_t normal_product_count_l = 0U;
static volatile uint32_t first_placement_started_count_l = 0U;
static volatile uint32_t supplied_tray_count_l = 0U;
static volatile uint8_t auto_supply_running_l = 0U;


static volatile uint8_t belt_dma_done_debug_l = 0U;

/* ============================================================
 * 내부 함수 선언
 * ============================================================ */
static uint8_t start_belt_move_mm(int32_t dx_mm);
static uint8_t start_screw_move_mm(int32_t dx_mm);
static void stop_belt_dma(void);
static void stop_screw_dma(void);
static void stop_all_loader_dma(void);

static const uint32_t *select_belt_profile(uint32_t distance_mm,
                                           uint32_t *length_out);
static const uint16_t *select_screw_profile(uint32_t distance_mm,
                                            uint32_t *length_out);

static uint8_t start_axis_dma(TIM_HandleTypeDef *htim,
                              uint32_t channel,
                              const void *profile,
                              uint32_t profile_length,
                              uint32_t initial_arr,
                              void (*complete_callback)(DMA_HandleTypeDef *),
                              void (*error_callback)(DMA_HandleTypeDef *));

static void belt_dma_complete_callback(DMA_HandleTypeDef *hdma);
static void screw_dma_complete_callback(DMA_HandleTypeDef *hdma);
static void belt_dma_error_callback(DMA_HandleTypeDef *hdma);
static void screw_dma_error_callback(DMA_HandleTypeDef *hdma);
static void loader_dma_error_common(DMA_HandleTypeDef *failed_hdma);

static void TrayLoader_WaitDone_Blocking(void);

/* ============================================================
 * 초기화 및 기본 이동
 * ============================================================ */
void TrayLoader_Init(void)
{
    belt_dma_running_l = 0U;
    screw_dma_running_l = 0U;
    move_in_progress_l = 0U;
    dma_error_l = 0U;

    cur_belt_mm = 0;
    cur_screw_mm = 0;
    pending_belt_mm = 0;
    pending_screw_mm = 0;

    tray_supply_count_l = 0U;
    normal_product_count_l = 0U;
    first_placement_started_count_l = 0U;
    supplied_tray_count_l = 0U;
    auto_supply_running_l = 0U;

    HAL_GPIO_WritePin(LOADER_BELT_EN_PORT,
                      LOADER_BELT_EN_PIN,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LOADER_SCREW_EN_PORT,
                      LOADER_SCREW_EN_PIN,
                      GPIO_PIN_RESET);

    stop_all_loader_dma();

    /* CubeMX의 Update DMA 연결이 누락되었는지 즉시 확인한다. */
    if ((LOADER_BELT_TIM->hdma[TIM_DMA_ID_UPDATE] == NULL) ||
        (LOADER_SCREW_TIM->hdma[TIM_DMA_ID_UPDATE] == NULL))
    {
        dma_error_l = 1U;
    }
}

void MoveTo_Belt_Screw_mm(int32_t belt_mm, int32_t screw_mm)
{
    int32_t d_belt_mm;
    int32_t d_screw_mm;
    uint8_t belt_ok;
    uint8_t screw_ok;

    if ((move_in_progress_l != 0U) || (dma_error_l != 0U))
    {
        return;
    }

    d_belt_mm = belt_mm - cur_belt_mm;
    d_screw_mm = screw_mm - cur_screw_mm;

    /*
     * 현재 자동공급 시퀀스에서 사용하는 이동거리만 DMA 프로파일을
     * 보유한다. 지원하지 않는 거리이면 모터를 시작하지 않는다.
     */
    if ((d_belt_mm != 0) &&
        (select_belt_profile((uint32_t)((d_belt_mm > 0) ? d_belt_mm : -d_belt_mm),
                             NULL) == NULL))
    {
        dma_error_l = 1U;
        return;
    }

    if ((d_screw_mm != 0) &&
        (select_screw_profile((uint32_t)((d_screw_mm > 0) ? d_screw_mm : -d_screw_mm),
                              NULL) == NULL))
    {
        dma_error_l = 1U;
        return;
    }

    pending_belt_mm = d_belt_mm;
    pending_screw_mm = d_screw_mm;
    move_in_progress_l = 1U;

    belt_ok = start_belt_move_mm(d_belt_mm);
    screw_ok = start_screw_move_mm(d_screw_mm);

    if ((belt_ok == 0U) || (screw_ok == 0U))
    {
        stop_all_loader_dma();
        pending_belt_mm = 0;
        pending_screw_mm = 0;
        move_in_progress_l = 0U;
        dma_error_l = 1U;
        return;
    }

    /* 두 축 이동량이 모두 0이면 즉시 완료된다. */
    TrayLoader_UpdateDone();
}

void TrayLoader_UpdateDone(void)
{
    uint32_t primask;

    if (move_in_progress_l == 0U)
    {
        return;
    }

    if ((belt_dma_running_l != 0U) ||
        (screw_dma_running_l != 0U))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (move_in_progress_l != 0U)
    {
        cur_belt_mm += pending_belt_mm;
        cur_screw_mm += pending_screw_mm;

        pending_belt_mm = 0;
        pending_screw_mm = 0;
        move_in_progress_l = 0U;
    }

    __DMB();
    __set_PRIMASK(primask);
}

uint8_t TrayLoader_IsBusy(void)
{
    return move_in_progress_l;
}

uint8_t TrayLoader_HasDmaError(void)
{
    return dma_error_l;
}

void TrayLoader_ClearDmaError(void)
{
    if (move_in_progress_l == 0U)
    {
        dma_error_l = 0U;
    }
}

int32_t TrayLoader_GetBelt_mm(void)
{
    return cur_belt_mm;
}

int32_t TrayLoader_GetScrew_mm(void)
{
    return cur_screw_mm;
}

void TrayLoader_SetCurrentPosition_mm(int32_t belt_mm, int32_t screw_mm)
{
    if (move_in_progress_l != 0U)
    {
        return;
    }

    cur_belt_mm = belt_mm;
    cur_screw_mm = screw_mm;
}

/* ============================================================
 * 자동 공급 정책
 * ============================================================ */
void TrayLoader_NotifyNormalProductDetected(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    normal_product_count_l++;
    __set_PRIMASK(primask);
}

void TrayLoader_NotifyFirstPlacementStarted(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    first_placement_started_count_l++;
    __set_PRIMASK(primask);
}

void TrayLoader_ProcessAutoSupply(void)
{
    uint32_t normal_count;
    uint32_t first_started_count;
    uint32_t supplied_count;
    uint32_t required_normal_count;
    uint8_t should_supply = 0U;
    uint32_t primask;

    if (dma_error_l != 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if ((auto_supply_running_l == 0U))
    {
        normal_count = normal_product_count_l;
        first_started_count = first_placement_started_count_l;
        supplied_count = supplied_tray_count_l;
        required_normal_count = (2U * (supplied_count + 1U)) - 1U;

        if ((normal_count >= required_normal_count) &&
            ((supplied_count == 0U) ||
             (first_started_count >= supplied_count)))
        {
            auto_supply_running_l = 1U;
            should_supply = 1U;
        }
    }

    __set_PRIMASK(primask);

    if (should_supply == 0U)
    {
        return;
    }
    UART_SendMessage("LOADER_AUTO_START\r\n");  // 임시 로그 확인

    TrayLoader_RunOneSupply_AutoReset();

    primask = __get_PRIMASK();
    __disable_irq();

    if (dma_error_l == 0U)
    {
        supplied_tray_count_l++;
    }

    auto_supply_running_l = 0U;
    __set_PRIMASK(primask);
}

uint32_t TrayLoader_GetNormalProductCount(void)
{
    return normal_product_count_l;
}

uint32_t TrayLoader_GetFirstPlacementStartedCount(void)
{
    return first_placement_started_count_l;
}

uint32_t TrayLoader_GetSuppliedTrayCount(void)
{
    return supplied_tray_count_l;
}

/* ============================================================
 * 공급 시퀀스
 * ============================================================ */
void TrayLoader_ResetSupplyCounter(void)
{
    if (move_in_progress_l == 0U)
    {
        tray_supply_count_l = 0U;
    }
}

uint8_t TrayLoader_GetSupplyCounter(void)
{
    return tray_supply_count_l;
}

void TrayLoader_RunOneSupply_AutoReset(void)
{
    TrayLoader_WaitDone_Blocking();

    if (dma_error_l != 0U)
    {
        return;
    }

    /* 0. 예외 처리(Fail-safe): 카운트가 이미 최대치면 초기 위치로 복귀 */
    if (tray_supply_count_l >= TRAY_LOADER_SUPPLY_MAX_COUNT)
    {
        MoveTo_Belt_Screw_mm(0, cur_screw_mm);
        TrayLoader_WaitDone_Blocking();

        if (dma_error_l != 0U)
        {
            return;
        }

        tray_supply_count_l = 0U;
    }

    /* 1. 첫 번째 공급(Count==0)일 때만 108mm 이동 
          (두 번째부터는 아래 3번 단계에서 이미 45mm 이동해 둔 상태임) */
    if (tray_supply_count_l == 0U)
    {
        MoveTo_Belt_Screw_mm(TRAY_LOADER_FIRST_SUPPLY_BELT_MM, cur_screw_mm);
        TrayLoader_WaitDone_Blocking();

        if (dma_error_l != 0U)
        {
            return;
        }
    }

    /* 2. 볼스크류 왕복 (트레이 공급 실행) */
    RunTrayBallScrewCycle();

    if (dma_error_l != 0U)
    {
        return;
    }

    tray_supply_count_l++;

    /* 3. 완료 후 다음 위치로 미리 이동 혹은 전체 리셋 */
    if (tray_supply_count_l >= TRAY_LOADER_SUPPLY_MAX_COUNT)
    {
        // 마지막 공급을 마쳤으므로 벨트를 처음 위치(0)로 완전 복귀
        MoveTo_Belt_Screw_mm(0, cur_screw_mm);
        TrayLoader_WaitDone_Blocking();

        if (dma_error_l == 0U)
        {
            tray_supply_count_l = 0U; // 모든 사이클 완료 후 카운트 초기화
        }
    }
    else
    {
        // 아직 공급 횟수가 남았다면, 다음 신호가 오기 전에 미리 45mm 이동
        RunTrayBeltCycle();
    }
}

void RunTrayBallScrewCycle(void)
{
    MoveTo_Belt_Screw_mm(cur_belt_mm, cur_screw_mm + TRAY_LOADER_SCREW_STROKE_MM);
    TrayLoader_WaitDone_Blocking();

    if (dma_error_l != 0U)
    {
        return;
    }

    osDelay(100U);

    MoveTo_Belt_Screw_mm(cur_belt_mm, cur_screw_mm - TRAY_LOADER_SCREW_STROKE_MM);
    TrayLoader_WaitDone_Blocking();
}

void RunTrayBeltCycle(void)
{
    MoveTo_Belt_Screw_mm(cur_belt_mm + TRAY_LOADER_NEXT_SUPPLY_BELT_MM, cur_screw_mm);
    TrayLoader_WaitDone_Blocking();
}

/* ============================================================
 * DMA 시작/완료 처리
 * ============================================================ */
static uint8_t start_belt_move_mm(int32_t dx_mm)
{
    uint32_t distance_mm;
    uint32_t profile_length;
    const uint32_t *profile;

    UART_SendMessage("BELT_ENTER\r\n");

    stop_belt_dma();

    if (dx_mm == 0)
    {
        UART_SendMessage("BELT_ZERO\r\n");
        belt_dma_running_l = 0U;
        return 1U;
    }

    distance_mm = (uint32_t)((dx_mm > 0) ? dx_mm : -dx_mm);
    profile = select_belt_profile(distance_mm, &profile_length);

    if (profile == NULL)
    {
        UART_SendMessage("BELT_PROFILE_NULL\r\n");
        return 0U;
    }

    UART_SendMessage("BELT_PROFILE_OK\r\n");

    HAL_GPIO_WritePin(LOADER_BELT_DIR_PORT,
                      LOADER_BELT_DIR_PIN,
                      (dx_mm > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    belt_dma_running_l = 1U;

    if (start_axis_dma(LOADER_BELT_TIM,
                       LOADER_BELT_CHANNEL,
                       profile,
                       profile_length,
                       LOADER_BELT_ARR_SLOW,
                       belt_dma_complete_callback,
                       belt_dma_error_callback) == 0U)
    {
        UART_SendMessage("BELT_DMA_START_FAIL\r\n");

        belt_dma_running_l = 0U;
        return 0U;
    }

    UART_SendMessage("BELT_DMA_STARTED\r\n");

    return 1U;
}

static uint8_t start_screw_move_mm(int32_t dx_mm)
{
    uint32_t distance_mm;
    uint32_t profile_length;
    const uint16_t *profile;

    stop_screw_dma();

    if (dx_mm == 0)
    {
        screw_dma_running_l = 0U;
        return 1U;
    }

    distance_mm = (uint32_t)((dx_mm > 0) ? dx_mm : -dx_mm);
    profile = select_screw_profile(distance_mm, &profile_length);

    if (profile == NULL)
    {
        return 0U;
    }

    HAL_GPIO_WritePin(LOADER_SCREW_DIR_PORT,
                      LOADER_SCREW_DIR_PIN,
                      (dx_mm > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    screw_dma_running_l = 1U;

    if (start_axis_dma(LOADER_SCREW_TIM,
                       LOADER_SCREW_CHANNEL,
                       profile,
                       profile_length,
                       LOADER_SCREW_ARR_SLOW,
                       screw_dma_complete_callback,
                       screw_dma_error_callback) == 0U)
    {
        screw_dma_running_l = 0U;
        return 0U;
    }

    return 1U;
}

static uint8_t start_axis_dma(TIM_HandleTypeDef *htim,
                              uint32_t channel,
                              const void *profile,
                              uint32_t profile_length,
                              uint32_t initial_arr,
                              void (*complete_callback)(DMA_HandleTypeDef *),
                              void (*error_callback)(DMA_HandleTypeDef *))
{
    DMA_HandleTypeDef *hdma;
    HAL_StatusTypeDef status;

    if ((htim == NULL) ||
        (profile == NULL) ||
        (profile_length == 0U))
    {
        return 0U;
    }

    hdma = htim->hdma[TIM_DMA_ID_UPDATE];
    if (hdma == NULL)
    {
        return 0U;
    }

    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
    (void)HAL_TIM_PWM_Stop(htim, channel);
    (void)HAL_DMA_Abort(hdma);

    hdma->XferCpltCallback = complete_callback;
    hdma->XferHalfCpltCallback = NULL;
    hdma->XferErrorCallback = error_callback;

    __HAL_TIM_SET_AUTORELOAD(htim, initial_arr);
    __HAL_TIM_SET_COMPARE(htim, channel, LOADER_STEP_LOW_TICKS);
    __HAL_TIM_SET_COUNTER(htim, 0U);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

    status = HAL_DMA_Start_IT(hdma, (uint32_t)profile, (uint32_t)&htim->Instance->ARR, profile_length);
    if (status != HAL_OK)
    {
        UART_SendMessage("HAL_DMA_START_FAIL\r\n");

        return 0U;
    }
    UART_SendMessage("HAL_DMA_START_OK\r\n");

    __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);

    status = HAL_TIM_PWM_Start(htim, channel);
    if (status != HAL_OK)
    {
        UART_SendMessage("HAL_PWM_START_FAIL\r\n");

        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
        (void)HAL_DMA_Abort(hdma);
        return 0U;
    }
    UART_SendMessage("HAL_PWM_START_OK\r\n");

    return 1U;
}

static void belt_dma_complete_callback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;

    belt_dma_done_debug_l = 1U;

    stop_belt_dma();
    TrayLoader_UpdateDone();
}

static void screw_dma_complete_callback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    stop_screw_dma();
    TrayLoader_UpdateDone();
}

static void belt_dma_error_callback(DMA_HandleTypeDef *hdma)
{
    loader_dma_error_common(hdma);
}

static void screw_dma_error_callback(DMA_HandleTypeDef *hdma)
{
    loader_dma_error_common(hdma);
}

static void loader_dma_error_common(DMA_HandleTypeDef *failed_hdma)
{
    DMA_HandleTypeDef *belt_hdma = LOADER_BELT_TIM->hdma[TIM_DMA_ID_UPDATE];
    DMA_HandleTypeDef *screw_hdma = LOADER_SCREW_TIM->hdma[TIM_DMA_ID_UPDATE];

    dma_error_l = 1U;

    __HAL_TIM_DISABLE_DMA(LOADER_BELT_TIM, TIM_DMA_UPDATE);
    __HAL_TIM_DISABLE_DMA(LOADER_SCREW_TIM, TIM_DMA_UPDATE);
    (void)HAL_TIM_PWM_Stop(LOADER_BELT_TIM, LOADER_BELT_CHANNEL);
    (void)HAL_TIM_PWM_Stop(LOADER_SCREW_TIM, LOADER_SCREW_CHANNEL);

    belt_dma_running_l = 0U;
    screw_dma_running_l = 0U;

    /* 실패한 DMA 자체는 HAL_DMA_IRQHandler()가 오류 처리를 마친다. */
    if ((belt_hdma != NULL) && (belt_hdma != failed_hdma))
    {
        (void)HAL_DMA_Abort(belt_hdma);
    }

    if ((screw_hdma != NULL) && (screw_hdma != failed_hdma))
    {
        (void)HAL_DMA_Abort(screw_hdma);
    }

    pending_belt_mm = 0;
    pending_screw_mm = 0;
    move_in_progress_l = 0U;
}

static void stop_belt_dma(void)
{
    __HAL_TIM_DISABLE_DMA(LOADER_BELT_TIM, TIM_DMA_UPDATE);
    (void)HAL_TIM_PWM_Stop(LOADER_BELT_TIM, LOADER_BELT_CHANNEL);
    belt_dma_running_l = 0U;
}

static void stop_screw_dma(void)
{
    __HAL_TIM_DISABLE_DMA(LOADER_SCREW_TIM, TIM_DMA_UPDATE);
    (void)HAL_TIM_PWM_Stop(LOADER_SCREW_TIM, LOADER_SCREW_CHANNEL);
    screw_dma_running_l = 0U;
}

static void stop_all_loader_dma(void)
{
    DMA_HandleTypeDef *belt_hdma = LOADER_BELT_TIM->hdma[TIM_DMA_ID_UPDATE];
    DMA_HandleTypeDef *screw_hdma = LOADER_SCREW_TIM->hdma[TIM_DMA_ID_UPDATE];

    stop_belt_dma();
    stop_screw_dma();

    if (belt_hdma != NULL)
    {
        (void)HAL_DMA_Abort(belt_hdma);
    }

    if (screw_hdma != NULL)
    {
        (void)HAL_DMA_Abort(screw_hdma);
    }
}

/* ============================================================
 * Flash DMA 프로파일 선택
 * ============================================================ */
static const uint32_t *select_belt_profile(uint32_t distance_mm, uint32_t *length_out)
{
    const uint32_t *profile = NULL;
    uint32_t length = 0U;

    switch (distance_mm)
    {
        case 45U:
            profile = tray_loader_belt_45_arr_dma;
            length = TRAY_LOADER_BELT_45_ARR_DMA_LENGTH;
            break;

        case 108U:
            profile = tray_loader_belt_108_arr_dma;
            length = TRAY_LOADER_BELT_108_ARR_DMA_LENGTH;
            break;

        case 243U:
            profile = tray_loader_belt_243_arr_dma;
            length = TRAY_LOADER_BELT_243_ARR_DMA_LENGTH;
            break;

        default:
            break;
    }

    if (length_out != NULL)
    {
        *length_out = length;
    }

    return profile;
}

static const uint16_t *select_screw_profile(uint32_t distance_mm, uint32_t *length_out)
{
    const uint16_t *profile = NULL;
    uint32_t length = 0U;

    if (distance_mm == TRAY_LOADER_SCREW_STROKE_MM)
    {
        profile = tray_loader_screw_130_arr_dma;
        length = TRAY_LOADER_SCREW_130_ARR_DMA_LENGTH;
    }

    if (length_out != NULL)
    {
        *length_out = length;
    }

    return profile;
}

/* ============================================================
 * 보조 함수
 * ============================================================ */
static void TrayLoader_WaitDone_Blocking(void)
{
    uint32_t start_tick;
    uint8_t debug_sent = 0U;
    char message[192];

    start_tick = HAL_GetTick();

    while ((TrayLoader_IsBusy() != 0U) &&
           (dma_error_l == 0U))
    {
        uint32_t elapsed_ms =
            HAL_GetTick() - start_tick;

        TrayLoader_UpdateDone();

        /*
         * 첫 번째 벨트 이동 시작 후 500ms 시점의
         * TIM5/DMA 상태를 한 번 출력한다.
         */
        if ((debug_sent == 0U) &&
            (elapsed_ms >= 500U) &&
            (belt_dma_running_l != 0U))
        {
            snprintf(
                message,
                sizeof(message),
                "BELT_DBG,CNT=%lu,ARR=%lu,NDTR=%lu,"
                "CEN=%u,UDE=%u,CC4E=%u,DMA_EN=%u\r\n",
                (unsigned long)TIM5->CNT,
                (unsigned long)TIM5->ARR,
                (unsigned long)DMA1_Stream6->NDTR,
                (unsigned int)(
                    (TIM5->CR1 & TIM_CR1_CEN) != 0U
                ),
                (unsigned int)(
                    (TIM5->DIER & TIM_DIER_UDE) != 0U
                ),
                (unsigned int)(
                    (TIM5->CCER & TIM_CCER_CC4E) != 0U
                ),
                (unsigned int)(
                    (DMA1_Stream6->CR &
                     DMA_SxCR_EN) != 0U
                )
            );

            UART_SendMessage(message);
            debug_sent = 1U;
        }
        if (belt_dma_done_debug_l != 0U)
        {
            belt_dma_done_debug_l = 0U;
            UART_SendMessage("BELT_DMA_COMPLETE\r\n");
        }

        osDelay(1U);
    }
}
