#include "xyz_motion.h"  // XYZ 모션 제어 함수 선언과 main.h 접근을 위한 헤더 포함

/*
 * main.c에서 생성된 TIM 핸들을 사용하기 위한 extern 선언
 * htim2는 XYZ축 step pulse 주기 생성용 타이머
 */
extern TIM_HandleTypeDef htim2;  // main.c에서 생성된 TIM2 핸들을 이 파일에서 사용

/* 설정값 */

/* 모터 드라이버 마이크로스텝 설정 */
#define XYZ_STEPS_PER_REV     (1600U)  // 모터 1회전당 스텝 수, 드라이버 마이크로스텝 설정 기준

/*
 * 벨트 구동축 기준 mm 변환값
 * mm_per_rev = BELT_PITCH_MM * PULLEY_TEETH
 * 예: 3.0mm pitch × 20T = 60mm/rev
 */
#define XYZ_BELT_PITCH_MM     (3.0f)   // 벨트 피치, 풀리 톱니 하나당 이동 거리
#define XYZ_PULLEY_TEETH      (20.0f)  // 풀리 톱니 수

/* STEP 펄스 HIGH 유지용 NOP */
#define XYZ_STEP_PULSE_NOP    (80U)    // STEP 핀 HIGH 상태를 짧게 유지하기 위한 NOP 반복 횟수

/*
 * TIM2 tick frequency
 * 기존 코드 기준:
 * SYSCLK 16MHz, PSC 15이면 TIM2 tick = 1MHz
 */
#define XYZ_TIM2_TICK_HZ      (1000000U)  // TIM2 카운터 클럭 주파수, ARR 계산에 사용

/* S-curve Q16 설정 */
#define XYZ_SCURVE_Q          (16U)       // 고정소수점 Q16 연산에서 소수부 비트 수
#define XYZ_SCURVE_ONE_Q16    (65535U)    // Q16 기준 1.0에 해당하는 값

/*
 * 적재 중 Z축 하강 속도
 *
 * 값이 작을수록 빠름.
 * TRAY_TRACK_TIM2_ARR = 713보다 작게 설정해야 Z축이 더 빠르게 움직인다.
 *
 * 예:
 * 713  → Y축 동기화 속도
 * 350  → 약 2배 빠름
 * 250  → 약 2.8배 빠름
 */
#define PLACE_Z_FAST_TIM2_ARR          10U // 350->200->10

/* 타입 정의 */

typedef struct
{
    uint32_t arr_slow;       // 저속 구간 ARR 값, 값이 클수록 step 주파수가 낮음
    uint32_t arr_fast;       // 고속 구간 ARR 값, 값이 작을수록 step 주파수가 높음
    uint32_t ramp_steps;     // 가속 또는 감속에 사용할 기본 step 수

    uint32_t total_steps;    // 전체 이동 step 수, Bresenham의 major_steps와 같음
    uint32_t arr_cur;        // 현재 적용 중인 ARR 값

    uint32_t ramp_eff;       // 실제 적용할 ramp step 수, 짧은 이동에서는 자동 축소됨

    uint32_t v_slow_q16;     // 저속 step frequency를 Q16으로 표현한 값
    uint32_t v_fast_q16;     // 고속 step frequency를 Q16으로 표현한 값
    uint32_t v_diff_q16;     // 고속 주파수와 저속 주파수의 차이
} XYZ_Ramp_t;                // XYZ축 S-curve 가감속 상태 구조체

typedef struct
{
    uint32_t x_steps;        // X축 이동에 필요한 step 수
    uint32_t y_steps;        // Y축 이동에 필요한 step 수
    uint32_t z_steps;        // Z축 이동에 필요한 step 수

    uint32_t major_steps;    // 세 축 중 가장 큰 step 수, 전체 인터럽트 반복 기준
    uint32_t major_count;    // 현재까지 진행한 major step 카운트

    int32_t err_x;           // Bresenham X축 누적 오차
    int32_t err_y;           // Bresenham Y축 누적 오차
    int32_t err_z;           // Bresenham Z축 누적 오차

    uint8_t x_en;            // X축 이동 사용 여부
    uint8_t y_en;            // Y축 이동 사용 여부
    uint8_t z_en;            // Z축 이동 사용 여부
} XYZ_BresenhamMotion_t;     // XYZ축 Bresenham 보간 상태 구조체

/* 내부 변수 */

/* 현재 스텝(step 관리) */
static volatile int32_t cur_x_steps = 0;
static volatile int32_t cur_y_steps = 0;
static volatile int32_t cur_z_steps = 0;

static volatile int32_t pending_dx_steps = 0;
static volatile int32_t pending_dy_steps = 0;
static volatile int32_t pending_dz_steps = 0;

/* 이동 중 플래그 */
static volatile uint8_t move_in_progress = 0;  // XYZ축 이동 중이면 1, 정지 상태이면 0

/* Bresenham 보간 상태 */
static volatile XYZ_BresenhamMotion_t bres;  // 현재 이동에 대한 Bresenham 보간 상태 저장 변수

/* 고정속도 이동 모드 */
static volatile uint8_t const_speed_mode = 0;
static volatile uint32_t const_speed_arr = 1999;

/* 가감속 설정 arr_slow: 저속 ARR, arr_fast: 고속 ARR, ramp_steps: 가속/감속에 사용할 step 수 */
static volatile XYZ_Ramp_t ramp_x = {.arr_slow   = 2999, .arr_fast   = 199, .ramp_steps = 400};

/* 내부 함수 선언 */
// static uint32_t XYZ_mm_to_steps(float mm);																// mm 단위 거리를 step 수로 변환하는 내부 함수
// static void XYZ_StartMove(float dx_mm, float dy_mm, float dz_mm);									// 상대 이동량을 기준으로 실제 이동 시작
static int32_t XYZ_mm_to_abs_steps(float mm);   // 목표 mm를 절대 step 좌표로 변환
static void XYZ_StartMoveSteps(int32_t dx_steps, int32_t dy_steps, int32_t dz_steps);

static void XYZ_SetTimerARR(TIM_HandleTypeDef *htim, uint32_t arr);										// 타이머 ARR 값을 설정하는 함수
static uint32_t XYZ_ScurveSmoothstep_Q16(uint32_t u_q16);												// smoothstep S-curve 보간 함수

static uint32_t XYZ_EstimateArrAtStep(uint32_t total_steps, uint32_t ramp_eff, uint32_t step_now);

static void XYZ_RampPrepare(volatile XYZ_Ramp_t *r, uint32_t total_steps);								// 이동 시작 전 ramp 값 준비
static void XYZ_RampUpdateScurve(volatile XYZ_Ramp_t *r, uint32_t step_now, TIM_HandleTypeDef *htim);	// 현재 step 위치에 맞게 ARR 갱신

void MoveToXYZ_ConstSpeed_mm(float x_mm, float y_mm, float z_mm, uint32_t arr_fixed);				// 적재 시 컨베이어와 속도 맞춘 동작

/* 외부 공개 함수 */
void XYZ_Init(void)
{
    cur_x_steps = 0;
    cur_y_steps = 0;
    cur_z_steps = 0;

    pending_dx_steps = 0;
    pending_dy_steps = 0;
    pending_dz_steps = 0;

    move_in_progress = 0;  // 이동 상태 플래그를 정지 상태로 초기화

    bres.x_steps = 0;  // X축 step 수 초기화
    bres.y_steps = 0;  // Y축 step 수 초기화
    bres.z_steps = 0;  // Z축 step 수 초기화

    bres.major_steps = 0;  // 전체 기준 step 수 초기화
    bres.major_count = 0;  // 현재 진행 step 카운트 초기화

    bres.err_x = 0;  // X축 Bresenham 오차 초기화
    bres.err_y = 0;  // Y축 Bresenham 오차 초기화
    bres.err_z = 0;  // Z축 Bresenham 오차 초기화

    bres.x_en = 0;  // X축 이동 비활성화
    bres.y_en = 0;  // Y축 이동 비활성화
    bres.z_en = 0;  // Z축 이동 비활성화

    HAL_TIM_Base_Stop_IT(&htim2);  // TIM2 인터럽트 정지

    const_speed_mode = 0;
    const_speed_arr = 1999;
}

void MoveToXYZ_mm(float x_mm, float y_mm, float z_mm)
{
    if (move_in_progress)
    {
        return;
    }

    int32_t target_x_steps = XYZ_mm_to_abs_steps(x_mm);
    int32_t target_y_steps = XYZ_mm_to_abs_steps(y_mm);
    int32_t target_z_steps = XYZ_mm_to_abs_steps(z_mm);

    int32_t dx_steps = target_x_steps - cur_x_steps;
    int32_t dy_steps = target_y_steps - cur_y_steps;
    int32_t dz_steps = target_z_steps - cur_z_steps;

    if ((dx_steps == 0) && (dy_steps == 0) && (dz_steps == 0))
    {
        return;
    }

    pending_dx_steps = dx_steps;
    pending_dy_steps = dy_steps;
    pending_dz_steps = dz_steps;

    const_speed_mode = 0;
    move_in_progress = 1;

    XYZ_StartMoveSteps(dx_steps, dy_steps, dz_steps);
}

uint8_t XYZ_IsBusy(void)
{
    return move_in_progress;  // 현재 XYZ 이동 중 여부 반환
}

void XYZ_SetCurrentPosition_mm(float x_mm, float y_mm, float z_mm)
{
    if (move_in_progress)  // 이동 중인지 확인
    {
        return;  // 이동 중에는 현재 좌표 강제 변경을 허용하지 않음
    }

    cur_x_steps = XYZ_mm_to_abs_steps(x_mm);
    cur_y_steps = XYZ_mm_to_abs_steps(y_mm);
    cur_z_steps = XYZ_mm_to_abs_steps(z_mm);
}


uint32_t XYZ_EstimateMoveTimeMs(float x_mm, float y_mm, float z_mm)
{
    int32_t target_x_steps;
    int32_t target_y_steps;
    int32_t target_z_steps;

    int32_t dx_steps;
    int32_t dy_steps;
    int32_t dz_steps;

    uint32_t sx;
    uint32_t sy;
    uint32_t sz;

    uint32_t major_steps;
    uint32_t ramp_eff;

    uint64_t total_time_us;
    uint32_t arr;

    /* 이 함수는 로봇이 정지한 상태에서 호출해야 한다. */
    if (move_in_progress != 0U)
    {
        return 0U;
    }

    target_x_steps = XYZ_mm_to_abs_steps(x_mm);
    target_y_steps = XYZ_mm_to_abs_steps(y_mm);
    target_z_steps = XYZ_mm_to_abs_steps(z_mm);

    dx_steps = target_x_steps - cur_x_steps;
    dy_steps = target_y_steps - cur_y_steps;
    dz_steps = target_z_steps - cur_z_steps;

    sx = (dx_steps >= 0) ? (uint32_t)dx_steps : (uint32_t)(-dx_steps);
    sy = (dy_steps >= 0) ? (uint32_t)dy_steps : (uint32_t)(-dy_steps);
    sz = (dz_steps >= 0) ? (uint32_t)dz_steps : (uint32_t)(-dz_steps);

    major_steps = sx;

    if (sy > major_steps)
    {
        major_steps = sy;
    }

    if (sz > major_steps)
    {
        major_steps = sz;
    }

    if (major_steps == 0U)
    {
        return 0U;
    }

    ramp_eff = ramp_x.ramp_steps;

    if ((ramp_eff > 0U) && (major_steps <= (2U * ramp_eff)))
    {
        ramp_eff = major_steps / 2U;
    }

    /* 첫 번째 인터럽트는 arr_slow로 시작한다. */
    total_time_us = (uint64_t)ramp_x.arr_slow + 1ULL;

    // /*
    //  * 실제 XYZ_RampUpdateScurve()에서 적용하는 ARR을 각 major step에 대해 동일하게 계산한다.
    //  * 마지막 step 이후 move_in_progress가 해제되는 다음 인터럽트 주기까지 포함한다.
    //  */

    /*
    * 가속 구간 계산
    */
    for (uint32_t step = 1U; step <= ramp_eff; step++)
    {
        arr = XYZ_EstimateArrAtStep( major_steps, ramp_eff, step);

        total_time_us += (uint64_t)arr + 1ULL;
    }

    /*
    * 정속 구간은 ARR이 전부 동일하므로
    * 반복하지 않고 한 번에 합산한다.
    */
    if (major_steps > ((2U * ramp_eff) + 1U))
    {
        uint32_t constant_steps = major_steps - (2U * ramp_eff) - 1U;

        total_time_us += (uint64_t)constant_steps * ((uint64_t)ramp_x.arr_fast + 1ULL);
    }

    /*
    * 감속 구간 계산
    */
    if (major_steps > ramp_eff)
    {
        uint32_t decel_start = major_steps - ramp_eff;

        for (uint32_t step = decel_start; step <= major_steps; step++)
        {
            arr = XYZ_EstimateArrAtStep( major_steps, ramp_eff, step);

            total_time_us += (uint64_t)arr + 1ULL;
        }
    }

    /* 마이크로초를 밀리초로 올림 변환 */
    return (uint32_t)((total_time_us + 999ULL) / 1000ULL);
}


/*
 * TIM2 인터럽트에서 호출하는 함수, main.c의 HAL_TIM_PeriodElapsedCallback() 안에서:
 * if (htim->Instance == TIM2)
 * {
 *     XYZ_TIM2Callback();
 * }
 */
void XYZ_TIM2Callback(void)
{
    if (!move_in_progress)  // 이동 중이 아닌지 확인
    {
        return;  // 이동 중이 아니면 아무 처리 없이 종료
    }

    if (bres.major_count >= bres.major_steps)  // 목표 major step 수에 도달했는지 확인
    {
        HAL_TIM_Base_Stop_IT(&htim2);  // TIM2 인터럽트 정지

        cur_x_steps += pending_dx_steps;
        cur_y_steps += pending_dy_steps;
        cur_z_steps += pending_dz_steps;

        pending_dx_steps = 0;
        pending_dy_steps = 0;
        pending_dz_steps = 0;

        const_speed_mode = 0;  // 추가
        move_in_progress = 0;  // 이동 완료 상태로 변경
        
        return;  // 콜백 종료
    }

    bres.major_count++;  // 이번 TIM2 인터럽트에서 major step 하나 진행

    if (!const_speed_mode)	// 추가
    {
        XYZ_RampUpdateScurve(&ramp_x, bres.major_count, &htim2);
    }

    uint8_t step_x = 0;  // 이번 주기에 X축 STEP을 발생시킬지 저장
    uint8_t step_y = 0;  // 이번 주기에 Y축 STEP을 발생시킬지 저장
    uint8_t step_z = 0;  // 이번 주기에 Z축 STEP을 발생시킬지 저장

    /* Bresenham 계산 : 이번 TIM2 주기에 어떤 축을 움직일지 결정 */
    if (bres.x_en)  // X축 이동이 필요한 경우
    {
        bres.err_x += bres.x_steps;  // X축 오차 누적

        if (bres.err_x >= (int32_t)bres.major_steps)  // 누적 오차가 기준 step을 넘었는지 확인
        {
            bres.err_x -= bres.major_steps;  // 기준 step만큼 오차 보정
            step_x = 1;  // 이번 주기에 X축 STEP 발생
        }
    }

    if (bres.y_en)  // Y축 이동이 필요한 경우
    {
        bres.err_y += bres.y_steps;  // Y축 오차 누적

        if (bres.err_y >= (int32_t)bres.major_steps)  // 누적 오차가 기준 step을 넘었는지 확인
        {
            bres.err_y -= bres.major_steps;  // 기준 step만큼 오차 보정
            step_y = 1;  // 이번 주기에 Y축 STEP 발생
        }
    }

    if (bres.z_en)  // Z축 이동이 필요한 경우
    {
        bres.err_z += bres.z_steps;  // Z축 오차 누적

        if (bres.err_z >= (int32_t)bres.major_steps)  // 누적 오차가 기준 step을 넘었는지 확인
        {
            bres.err_z -= bres.major_steps;  // 기준 step만큼 오차 보정
            step_z = 1;  // 이번 주기에 Z축 STEP 발생
        }
    }

    /* STEP 핀 동시 SET : Y축은 Y1, Y2 두 개의 모터를 동시에 구동 */
    if (step_x)  // 이번 주기에 X축 STEP이 필요한 경우
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  // X축 STEP 핀 HIGH
    }

    if (step_y)  // 이번 주기에 Y축 STEP이 필요한 경우
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);  // Y1축 STEP 핀 HIGH
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  // Y2축 STEP 핀 HIGH
    }

    if (step_z)  // 이번 주기에 Z축 STEP이 필요한 경우
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Z축 STEP 핀 HIGH
    }

    for (volatile uint32_t i = 0; i < XYZ_STEP_PULSE_NOP; i++)  // STEP HIGH 유지용 짧은 지연 반복
    {
        __NOP();  // 아무 동작도 하지 않는 명령, 짧은 시간 지연용
    }

    /* STEP 핀 동시 RESET */
    if (step_x)  // X축 STEP을 발생시켰다면
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);  // X축 STEP 핀 LOW
    }

    if (step_y)  // Y축 STEP을 발생시켰다면
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // Y1축 STEP 핀 LOW
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  // Y2축 STEP 핀 LOW
    }

    if (step_z)  // Z축 STEP을 발생시켰다면
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Z축 STEP 핀 LOW
    }
}

/* 내부 함수 */

static int32_t XYZ_mm_to_abs_steps(float mm)
{
    const float mm_per_rev = XYZ_BELT_PITCH_MM * XYZ_PULLEY_TEETH;
    const float steps_per_mm = ((float)XYZ_STEPS_PER_REV) / mm_per_rev;

    if (mm >= 0.0f)
    {
        return (int32_t)(mm * steps_per_mm + 0.5f);
    }
    else
    {
        return (int32_t)(mm * steps_per_mm - 0.5f);
    }
}

static void XYZ_StartMoveSteps(int32_t dx_steps, int32_t dy_steps, int32_t dz_steps)
{
    uint32_t sx = (dx_steps >= 0) ? (uint32_t)dx_steps : (uint32_t)(-dx_steps);
    uint32_t sy = (dy_steps >= 0) ? (uint32_t)dy_steps : (uint32_t)(-dy_steps);
    uint32_t sz = (dz_steps >= 0) ? (uint32_t)dz_steps : (uint32_t)(-dz_steps);

    if ((sx == 0U) && (sy == 0U) && (sz == 0U))
    {
        move_in_progress = 0;
        return;
    }

    /* 방향 설정 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, (dx_steps > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, (dy_steps > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, (dy_steps > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, (dz_steps > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* DIR 신호 안정화 */
    for (volatile uint32_t i = 0; i < 2000; i++)
    {
        __NOP();
    }

    bres.x_steps = sx;
    bres.y_steps = sy;
    bres.z_steps = sz;

    bres.major_steps = sx;

    if (sy > bres.major_steps)
    {
        bres.major_steps = sy;
    }

    if (sz > bres.major_steps)
    {
        bres.major_steps = sz;
    }

    bres.major_count = 0;

    bres.err_x = 0;
    bres.err_y = 0;
    bres.err_z = 0;

    bres.x_en = (sx > 0U);
    bres.y_en = (sy > 0U);
    bres.z_en = (sz > 0U);

    if (const_speed_mode)
    {
        XYZ_SetTimerARR(&htim2, const_speed_arr);
    }
    else
    {
        XYZ_RampPrepare(&ramp_x, bres.major_steps);
        XYZ_SetTimerARR(&htim2, ramp_x.arr_cur);
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    HAL_TIM_Base_Start_IT(&htim2);
}

static void XYZ_SetTimerARR(TIM_HandleTypeDef *htim, uint32_t arr)
{
    __HAL_TIM_SET_AUTORELOAD(htim, arr);  // 타이머 ARR 값을 변경하여 주기를 변경
    __HAL_TIM_SET_COUNTER(htim, 0);       // 타이머 카운터를 0으로 초기화
}

static uint32_t XYZ_ScurveSmoothstep_Q16(uint32_t u_q16)
{
    if (u_q16 >= XYZ_SCURVE_ONE_Q16)  // 입력값이 1.0 이상인지 확인
    {
        return XYZ_SCURVE_ONE_Q16;  // 1.0 이상이면 최대값으로 제한
    }

    uint64_t u = u_q16;  // Q16 입력값을 64비트로 확장
    uint64_t u2 = (u * u) >> XYZ_SCURVE_Q;  // u^2 계산 후 Q16 스케일 유지
    uint64_t u3 = (u2 * u) >> XYZ_SCURVE_Q; // u^3 계산 후 Q16 스케일 유지

    uint64_t y = (3U * u2) - (2U * u3);  // smoothstep 공식 y = 3u^2 - 2u^3 계산

    if (y > XYZ_SCURVE_ONE_Q16)  // 계산값이 Q16 최대값을 넘었는지 확인
    {
        y = XYZ_SCURVE_ONE_Q16;  // 최대값으로 제한
    }

    return (uint32_t)y;  // Q16 smoothstep 결과 반환
}


static uint32_t XYZ_EstimateArrAtStep(uint32_t total_steps, uint32_t ramp_eff, uint32_t step_now)
{
    uint32_t arr_slow = ramp_x.arr_slow;
    uint32_t arr_fast = ramp_x.arr_fast;

    uint32_t v_slow_q16;
    uint32_t v_fast_q16;
    uint32_t v_diff_q16;
    uint32_t scale_q16;
    uint32_t v_cur_q16;
    uint32_t arr;

    if (total_steps == 0U)
    {
        return arr_slow;
    }

    v_slow_q16 = (uint32_t)(((uint64_t)XYZ_TIM2_TICK_HZ << XYZ_SCURVE_Q) / (uint64_t)(arr_slow + 1U));

    v_fast_q16 = (uint32_t)(((uint64_t)XYZ_TIM2_TICK_HZ << XYZ_SCURVE_Q) / (uint64_t)(arr_fast + 1U));

    if (v_fast_q16 > v_slow_q16)
    {
        v_diff_q16 = v_fast_q16 - v_slow_q16;
    }
    else
    {
        return arr_slow;
    }

    if (ramp_eff == 0U)
    {
        return arr_fast;
    }

    if (step_now <= ramp_eff)
    {
        uint32_t u_q16 = (uint32_t)(((uint64_t)step_now * XYZ_SCURVE_ONE_Q16) / ramp_eff);

        scale_q16 = XYZ_ScurveSmoothstep_Q16(u_q16);
    }
    else if (step_now >= (total_steps - ramp_eff))
    {
        uint32_t remain = total_steps - step_now;
        uint32_t u_q16 = (uint32_t)(((uint64_t)remain * XYZ_SCURVE_ONE_Q16) / ramp_eff);

        scale_q16 = XYZ_ScurveSmoothstep_Q16(u_q16);
    }
    else
    {
        scale_q16 = XYZ_SCURVE_ONE_Q16;
    }

    v_cur_q16 = v_slow_q16 + (uint32_t)(((uint64_t)v_diff_q16 * scale_q16) / XYZ_SCURVE_ONE_Q16);

    if (v_cur_q16 == 0U)
    {
        return arr_slow;
    }

    arr = (uint32_t)((((uint64_t)XYZ_TIM2_TICK_HZ << XYZ_SCURVE_Q) / v_cur_q16) - 1U);

    if (arr < arr_fast)
    {
        arr = arr_fast;
    }

    if (arr > arr_slow)
    {
        arr = arr_slow;
    }

    return arr;
}


static void XYZ_RampPrepare(volatile XYZ_Ramp_t *r, uint32_t total_steps)
{
    r->total_steps = total_steps;  // 전체 이동 step 수 저장
    r->arr_cur = r->arr_slow;      // 현재 ARR을 저속 ARR로 초기화

    r->ramp_eff = r->ramp_steps;   // 기본 ramp step 수를 실제 ramp step 수로 설정

    /* 이동 거리가 짧으면 가속/감속 구간을 자동 축소, 전체 step이 2R보다 작으면 삼각 프로파일 형태가 됨 */
    if ((r->ramp_eff > 0U) && (total_steps <= (2U * r->ramp_eff)))  // 전체 이동이 가속+감속 구간보다 짧은지 확인
    {
        r->ramp_eff = total_steps / 2U;  // ramp 구간을 전체 이동거리의 절반으로 축소
    }

    /* ARR를 직접 보간하지 않고, 실제 step frequency를 S-curve 보간한 뒤 ARR로 변환 : freq = TIM2_TICK_HZ / (ARR + 1) */
    r->v_slow_q16 = (uint32_t)(((uint64_t)XYZ_TIM2_TICK_HZ << XYZ_SCURVE_Q) / (uint64_t)(r->arr_slow + 1U));  // 저속 ARR에 해당하는 주파수를 Q16으로 계산

    r->v_fast_q16 = (uint32_t)(((uint64_t)XYZ_TIM2_TICK_HZ << XYZ_SCURVE_Q) / (uint64_t)(r->arr_fast + 1U));  // 고속 ARR에 해당하는 주파수를 Q16으로 계산

    if (r->v_fast_q16 > r->v_slow_q16)  // 고속 주파수가 저속 주파수보다 큰지 확인
    {
        r->v_diff_q16 = r->v_fast_q16 - r->v_slow_q16;  // 주파수 차이 계산
    }
    else
    {
        r->v_diff_q16 = 0U;  // 비정상 설정이면 차이를 0으로 처리
    }
}

static void XYZ_RampUpdateScurve(volatile XYZ_Ramp_t *r, uint32_t step_now, TIM_HandleTypeDef *htim)
{
    uint32_t N = r->total_steps;  // 전체 이동 step 수를 지역 변수로 복사
    uint32_t R = r->ramp_eff;     // 실제 ramp step 수를 지역 변수로 복사

    if (N == 0U)  // 전체 step 수가 0인지 확인
    {
        return;  // 이동할 step이 없으면 함수 종료
    }

    if ((R == 0U) || (r->v_diff_q16 == 0U))  // ramp 구간이 없거나 속도 차이가 없는 경우
    {
        if (r->arr_cur != r->arr_fast)  // 현재 ARR이 고속 ARR이 아닌지 확인
        {
            r->arr_cur = r->arr_fast;  // 현재 ARR을 고속 ARR로 변경
            XYZ_SetTimerARR(htim, r->arr_cur);  // 타이머에 고속 ARR 적용
        }

        return;  // ramp 갱신 종료
    }

    uint32_t scale_q16;  // S-curve 비율 값을 저장할 변수

    if (step_now <= R)  // 현재 step이 가속 구간 안에 있는지 확인
    {
        /* 가속 구간 : slow speed -> fast speed */
        uint32_t u_q16 = (uint32_t)(((uint64_t)step_now * XYZ_SCURVE_ONE_Q16) / R);  // 가속 진행률을 Q16으로 계산

        scale_q16 = XYZ_ScurveSmoothstep_Q16(u_q16);  // 가속 진행률에 S-curve 적용
    }
    else if (step_now >= (N - R))  // 현재 step이 감속 구간 안에 있는지 확인
    {
        /* 감속 구간 : fast speed -> slow speed */
        uint32_t remain = N - step_now;  // 감속 종료까지 남은 step 수 계산
        uint32_t u_q16 = (uint32_t)(((uint64_t)remain * XYZ_SCURVE_ONE_Q16) / R);  // 남은 step 기준 감속 진행률 계산

        scale_q16 = XYZ_ScurveSmoothstep_Q16(u_q16);  // 감속 진행률에 S-curve 적용
    }
    else
    {
        /* 정속 구간 */
        scale_q16 = XYZ_SCURVE_ONE_Q16;  // 정속 구간에서는 최대 속도 비율 1.0 사용
    }

    uint32_t v_cur_q16 = r->v_slow_q16 + (uint32_t)(((uint64_t)r->v_diff_q16 * scale_q16) / XYZ_SCURVE_ONE_Q16);  // 현재 S-curve 비율에 따른 현재 주파수 계산

    if (v_cur_q16 == 0U)  // 현재 주파수가 0인지 확인
    {
        return;  // 0으로 나누기 방지를 위해 종료
    }

    uint32_t arr = (uint32_t)((((uint64_t)XYZ_TIM2_TICK_HZ << XYZ_SCURVE_Q) / v_cur_q16) - 1U);  // 현재 주파수에 대응하는 ARR 값 계산

    if (arr < r->arr_fast)  // 계산된 ARR이 고속 ARR보다 작으면 너무 빠른 상태
    {
        arr = r->arr_fast;  // 최소 ARR을 고속 ARR로 제한
    }

    if (arr > r->arr_slow)  // 계산된 ARR이 저속 ARR보다 크면 너무 느린 상태
    {
        arr = r->arr_slow;  // 최대 ARR을 저속 ARR로 제한
    }

    if (arr != r->arr_cur)  // ARR 값이 이전과 달라졌는지 확인
    {
        r->arr_cur = arr;  // 현재 ARR 값 갱신
        XYZ_SetTimerARR(htim, arr);  // TIM2에 새 ARR 값 적용
    }
}

void MoveToXYZ_ConstSpeed_mm(float x_mm, float y_mm, float z_mm, uint32_t arr_fixed)
{
    if (move_in_progress)
    {
        return;
    }

    int32_t target_x_steps = XYZ_mm_to_abs_steps(x_mm);
    int32_t target_y_steps = XYZ_mm_to_abs_steps(y_mm);
    int32_t target_z_steps = XYZ_mm_to_abs_steps(z_mm);

    int32_t dx_steps = target_x_steps - cur_x_steps;
    int32_t dy_steps = target_y_steps - cur_y_steps;
    int32_t dz_steps = target_z_steps - cur_z_steps;

    if ((dx_steps == 0) && (dy_steps == 0) && (dz_steps == 0))
    {
        return;
    }

    pending_dx_steps = dx_steps;
    pending_dy_steps = dy_steps;
    pending_dz_steps = dz_steps;

    const_speed_mode = 1;
    const_speed_arr = arr_fixed;

    move_in_progress = 1;

    XYZ_StartMoveSteps(dx_steps, dy_steps, dz_steps);
}
