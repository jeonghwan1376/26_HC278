#ifndef XYZ_MOTION_H
#define XYZ_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * 개요
 * - 직교로봇 X/Y/Z축 이동 제어 모듈
 * - mm 단위 목표 좌표를 입력받아 현재 좌표 기준 상대 이동 수행
 * - TIM2 인터럽트에서 Bresenham 보간으로 X/Y/Z축 동시 이동
 * - S-curve 기반 가감속 적용
 */

/* 초기화 */
void XYZ_Init(void);

/* 절대 좌표 이동 */
void MoveToXYZ_mm(float x_mm, float y_mm, float z_mm);

/* 이동 중 여부 확인 */
uint8_t XYZ_IsBusy(void);

/* 현재 직교로봇 위치에서 목표 XYZ까지 이동하는 데 걸리는 시간을 현재 S-curve 설정 기준으로 예측한다. */
uint32_t XYZ_EstimateMoveTimeMs(float x_mm, float y_mm, float z_mm);

/* 현재 좌표 강제 설정 */
void XYZ_SetCurrentPosition_mm(float x_mm, float y_mm, float z_mm);

/* TIM2 주기 인터럽트에서 호출 */
void XYZ_TIM2Callback(void);

/* 컨베이어와 속도 동기화 */
void MoveToXYZ_ConstSpeed_mm(float x_mm, float y_mm, float z_mm, uint32_t arr_fixed);

#ifdef __cplusplus
}
#endif

#endif /* XYZ_MOTION_H */
