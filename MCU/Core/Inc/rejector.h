#ifndef REJECTOR_H
#define REJECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * 개요
 * - 리젝터 스텝모터 제어 모듈
 * - TIM1 CH1 PWM을 STEP 신호로 사용
 * - GPIOF PIN13을 DIR 신호로 사용
 * - GPIOE PIN11을 ENA 신호로 사용
 */

/* 초기화 */
void Rejector_Init(void);

/* 리젝터 1회 회전 시작 */
void Rejector_Start(GPIO_PinState dir);

/* 리젝터 왕복 1회 요청 및 태스크 상태 처리 */
void Rejector_RequestCycle(void);
void Rejector_Process(void);

/* 리젝터 동작 중 여부 */
uint8_t Rejector_IsBusy(void);

/* TIM1 주기 인터럽트에서 호출 */
void Rejector_TIM1Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* REJECTOR_H */
