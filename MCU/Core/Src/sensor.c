#include "sensor.h"      // Sensor_Init(), Sensor_Update(), SensorCallbacks_t 선언이 있는 헤더 파일 포함
#include <stddef.h>      // NULL 사용을 위한 표준 헤더 파일 포함

#define SENSOR_DEBOUNCE_MS       10U    // 센서 입력이 10ms 동안 유지되어야 안정된 입력으로 인정
#define SENSOR_EVENT_COOLDOWN_MS 200U   // 같은 센서 이벤트가 200ms 안에 중복 발생하지 않도록 제한

typedef struct                         // 센서 디바운싱 처리를 위한 구조체 정의 시작
{                                      // 구조체 내부 변수 시작
    GPIO_TypeDef *port;                // 센서가 연결된 GPIO 포트 주소 저장
    uint16_t pin;                      // 센서가 연결된 GPIO 핀 번호 저장

    GPIO_PinState prev_stable;         // 이전에 안정 상태로 인정된 센서 값 저장
    GPIO_PinState last_sample;         // 직전에 읽은 센서 샘플 값 저장
    GPIO_PinState active_state;        // 센서가 감지되었다고 판단할 GPIO 상태 저장

    uint32_t last_change_tick;         // 센서 입력 값이 마지막으로 변한 시각 저장
    uint32_t last_event_tick;          // 센서 이벤트가 마지막으로 발생한 시각 저장
} SensorDebounce_t;                    // 구조체 이름을 SensorDebounce_t로 정의

/* 트레이 감지 센서 */
static SensorDebounce_t tray_sensor_pc9;    // 트레이 감지용 포토센서 상태 변수, PC9 사용
static SensorDebounce_t tray_sensor_pc10;    // 적재 영역 Y=0 기준 -100 mm에 설치한 PC10 센서

static SensorCallbacks_t sensor_callbacks; // 센서 감지 시 main.c에서 실행할 콜백 함수 저장 변수

static void SensorDebounce_Init(SensorDebounce_t *sensor, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState active_state) // 센서 디바운싱 구조체 초기화 함수
{                                                                                                                        // 함수 시작
    uint32_t now_tick = HAL_GetTick();
    GPIO_PinState initial_state = HAL_GPIO_ReadPin(port, pin); // 현재 센서 핀 상태를 처음 상태로 읽음

    sensor->port = port;                         // 구조체에 GPIO 포트 정보 저장
    sensor->pin = pin;                           // 구조체에 GPIO 핀 번호 저장

    sensor->prev_stable = initial_state;         // 현재 입력값을 초기 안정 상태로 저장
    sensor->last_sample = initial_state;         // 현재 입력값을 마지막 샘플 값으로 저장
    sensor->active_state = active_state;         // 감지 상태로 판단할 GPIO 상태 저장

    sensor->last_change_tick = now_tick;    // 현재 시간을 마지막 변화 시각으로 저장
    
    /* 초기 200ms 안의 첫 이벤트도 허용 */
    sensor->last_event_tick = now_tick - SENSOR_EVENT_COOLDOWN_MS;
}                                                // SensorDebounce_Init 함수 끝

static uint8_t SensorDebounce_Update(SensorDebounce_t *sensor, uint32_t now_tick) // 센서 상태를 읽고 감지 이벤트 발생 여부를 반환하는 함수
{
    GPIO_PinState now_state = HAL_GPIO_ReadPin(sensor->port, sensor->pin);        // 현재 센서 GPIO 상태 읽기

    if (now_state != sensor->last_sample)        // 현재 입력값이 직전 샘플 값과 다르면
    {                                            // 입력 변화 처리 시작
        sensor->last_sample = now_state;         // 마지막 샘플 값을 현재 입력값으로 갱신
        sensor->last_change_tick = now_tick;     // 입력이 변한 시간을 현재 시간으로 갱신
    }                                            // 입력 변화 처리 끝

    /* 동일한 상태가 10ms 이상 유지되었는지 확인 */
    if ((now_tick - sensor->last_change_tick) < SENSOR_DEBOUNCE_MS)
    {
        return 0U;
    }

    /* 이전 안정 상태와 같으면 새로운 이벤트가 아님 */
    if (now_state == sensor->prev_stable)
    {
        return 0U;
    }

    /* 새로운 안정 상태로 갱신 */
    sensor->prev_stable = now_state;

    /* BJ300 계열 NPN Active-Low 기준: RESET 진입만 감지 이벤트 */
    if (now_state != sensor->active_state)
    {
        return 0U;
    }

    /* 동일 센서에서 200ms 이내 중복 감지 방지 */
    if ((now_tick - sensor->last_event_tick) < SENSOR_EVENT_COOLDOWN_MS)
    {
        return 0U;
    }

    sensor->last_event_tick = now_tick;

    return 1U;
}                                              // SensorDebounce_Update 함수 끝

void Sensor_Init(const SensorCallbacks_t *callbacks) // 센서 모듈 초기화 함수
{                                                    // 함수 시작
    if (callbacks != NULL)                          // 전달받은 콜백 구조체 주소가 NULL이 아니면
    {                                                // 콜백 등록 처리 시작
        sensor_callbacks = *callbacks;              // 전달받은 콜백 함수들을 내부 변수에 복사
    }                                                // 콜백 등록 처리 끝
    else                                             // 전달받은 콜백 구조체가 NULL이면
    {                                                // 콜백 미사용 처리 시작
        /* 구조체 전체를 0으로 초기화 */
        sensor_callbacks = (SensorCallbacks_t){0};
    }                                                // 콜백 미사용 처리 끝

    SensorDebounce_Init(&tray_sensor_pc9, GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // PC9 트레이 감지 센서를 active-low 방식으로 초기화
    SensorDebounce_Init(&tray_sensor_pc10, GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // PC10 제품 감지 센서를 active-low 방식으로 초기화
}   // Sensor_Init 함수 끝

void Sensor_Update(uint32_t now_tick)   // 주기적으로 호출되어 PC8, PC9 센서를 확인하는 함수
{
    uint8_t pc9_detected = SensorDebounce_Update(&tray_sensor_pc9, now_tick);
    uint8_t pc10_detected = SensorDebounce_Update(&tray_sensor_pc10, now_tick);

    if ((pc9_detected != 0U) && (sensor_callbacks.on_tray_pc9_detected != NULL))
    {
        sensor_callbacks.on_tray_pc9_detected(now_tick);
    }
    if ((pc10_detected != 0U) && (sensor_callbacks.on_tray_pc10_detected != NULL))
    {
        sensor_callbacks.on_tray_pc10_detected(now_tick);
    }
}                                                              // Sensor_Update 함수 끝
