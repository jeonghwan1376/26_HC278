#include "rejector.h"

/*
 * main.c에서 생성된 TIM 핸들 사용
 * TIM1 CH1 : 리젝터 STEP PWM
 */
extern TIM_HandleTypeDef htim1;

/* TIM 설정
 * 초기
 * PSC : 15, ARR : 999, pulse : 499 -> 하드웨어상 걸림 이슈 발생
 *
 * 안정화(1) : ARR : 1199, pulse : 599 -> 하드웨어상 걸림 이슈 발생
 * 안정화(2) : ARR : 1499, pulse : 749
 * 안정화(3) : ARR : 1999, pulse : 999
 */

/* ============================================================
 * 설정값
 * ============================================================ */

/* 160도 회전에 필요한 step 수, 기존 main.c 값 유지 */
#define REJECTOR_STEP_160DEG      (444U)    // 160도 : 711 -> 150도 : 667 -> 140도 : 622 -> 100도 : 444 -> 90도 : 400(260721/23:59)

/* DIR 신호 안정화용 NOP, 기존 StepMotor_Start()의 3000 NOP 유지 */
#define REJECTOR_DIR_DELAY_NOP    (3000U)

/* ============================================================
 * GPIO 핀 정의
 * ============================================================ */

/* ENA : PE11 */
#define REJECTOR_EN_PORT          GPIOE
#define REJECTOR_EN_PIN           GPIO_PIN_11

/* DIR : PF13 */
#define REJECTOR_DIR_PORT         GPIOF
#define REJECTOR_DIR_PIN          GPIO_PIN_13

/* ============================================================
 * 내부 변수
 * ============================================================ */

 typedef enum
{
    REJECTOR_CYCLE_IDLE = 0,
    REJECTOR_CYCLE_FORWARD,
    REJECTOR_CYCLE_REVERSE
} RejectorCycleState_t;


static volatile uint16_t rejector_step_count = 0;
static volatile uint8_t rejector_running = 0;

static volatile uint8_t rejector_request_count = 0U;
static RejectorCycleState_t rejector_cycle_state = REJECTOR_CYCLE_IDLE;

/* ============================================================
 * 외부 공개 함수
 * ============================================================ */

void Rejector_Init(void)
{
    rejector_step_count = 0;
    rejector_running = 0;

    rejector_request_count = 0U;
    rejector_cycle_state = REJECTOR_CYCLE_IDLE;

    /* 기존 main.c 기준: GPIOE PIN11 RESET이 Enable 상태 */
    HAL_GPIO_WritePin(REJECTOR_EN_PORT, REJECTOR_EN_PIN, GPIO_PIN_RESET);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(&htim1);

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
}

void Rejector_Start(GPIO_PinState dir)
{
    if (rejector_running)
    {
        return;
    }

    rejector_step_count = 0;
    rejector_running = 1;

    /*
     * 방향 설정
     * 방향이 반대면 호출부에서 GPIO_PIN_RESET / GPIO_PIN_SET 순서를 바꾸거나
     * 여기서 SET/RESET 의미를 반대로 바꾸면 됨
     */
    HAL_GPIO_WritePin(REJECTOR_DIR_PORT, REJECTOR_DIR_PIN, dir);

    /* DIR 신호 안정화 */
    for (volatile uint32_t i = 0; i < REJECTOR_DIR_DELAY_NOP; i++)
    {
        __NOP();
    }

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim1);
}


/* ====================================================== */
void Rejector_RequestCycle(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (rejector_request_count < 255U)
    {
        rejector_request_count++;
    }

    __set_PRIMASK(primask);
}

void Rejector_Process(void)
{
    uint8_t start_new_cycle = 0U;
    uint32_t primask;

    if (rejector_cycle_state == REJECTOR_CYCLE_IDLE)
    {
        primask = __get_PRIMASK();
        __disable_irq();

        if (rejector_request_count > 0U)
        {
            rejector_request_count--;
            start_new_cycle = 1U;
        }

        __set_PRIMASK(primask);

        if (start_new_cycle != 0U)
        {
            rejector_cycle_state = REJECTOR_CYCLE_FORWARD;
            Rejector_Start(GPIO_PIN_RESET);
        }
        return;
    }

    if ((rejector_cycle_state == REJECTOR_CYCLE_FORWARD) &&
        (Rejector_IsBusy() == 0U))
    {
        rejector_cycle_state = REJECTOR_CYCLE_REVERSE;
        Rejector_Start(GPIO_PIN_SET);
        return;
    }

    if ((rejector_cycle_state == REJECTOR_CYCLE_REVERSE) &&
        (Rejector_IsBusy() == 0U))
    {
        rejector_cycle_state = REJECTOR_CYCLE_IDLE;
    }
}
/* ====================================================== */

uint8_t Rejector_IsBusy(void)
{
    return rejector_running;
}

void Rejector_TIM1Callback(void)
{
    if (!rejector_running)
    {
        return;
    }

    rejector_step_count++;

    if (rejector_step_count >= REJECTOR_STEP_160DEG)
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_Base_Stop_IT(&htim1);

        rejector_running = 0;
        rejector_step_count = 0;
    }
}
