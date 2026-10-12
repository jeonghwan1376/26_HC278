#ifndef TRAY_LOADER_H
#define TRAY_LOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * 개요
 * - 트레이 로더 벨트축 / 볼스크류축 제어 모듈
 * - TIM5  : 벨트축 STEP PWM
 * - TIM8 : 볼스크류축 STEP PWM
 * - mm 단위 절대 위치 이동
 * - 사다리꼴 형태의 ARR 기반 가감속 적용
 */

/* 초기화 */
void TrayLoader_Init(void);

/* 절대 좌표 이동 */
void MoveTo_Belt_Screw_mm(int32_t belt_mm, int32_t screw_mm);

/* 트레이 로더 동작 완료 갱신 */
void TrayLoader_UpdateDone(void);

/* 트레이 로더 이동 중 여부 */
uint8_t TrayLoader_IsBusy(void);

uint8_t TrayLoader_HasDmaError(void);
void TrayLoader_ClearDmaError(void);

/* 현재 위치 확인 */
int32_t TrayLoader_GetBelt_mm(void);
int32_t TrayLoader_GetScrew_mm(void);

/* 현재 위치 강제 설정 */
void TrayLoader_SetCurrentPosition_mm(int32_t belt_mm, int32_t screw_mm);

/* 동작 시퀀스 */
void RunTrayBallScrewCycle(void);
void RunTrayBeltCycle(void);

/*
 * 트레이 1개 자동 공급
 * - 1개째: 벨트축 115mm 위치 이동 후 볼스크류 왕복
 * - 2~4개째: 벨트축 45mm씩 이동 후 볼스크류 왕복
 * - 4개 공급 완료 후 벨트축만 0mm 복귀
 */
void TrayLoader_RunOneSupply_AutoReset(void);

/* 웹캠 정상 제품 수 + 각 트레이 첫 적재 시작을 이용한 자동 공급 정책 */
void TrayLoader_NotifyNormalProductDetected(void);
void TrayLoader_NotifyFirstPlacementStarted(void);
void TrayLoader_ProcessAutoSupply(void);
uint32_t TrayLoader_GetNormalProductCount(void);
uint32_t TrayLoader_GetFirstPlacementStartedCount(void);
uint32_t TrayLoader_GetSuppliedTrayCount(void);

/* 공급 카운터 관리 */
void TrayLoader_ResetSupplyCounter(void);
uint8_t TrayLoader_GetSupplyCounter(void);

/* UART 숫자 명령 처리 */
void ProcessCommand(int32_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* TRAY_LOADER_H */
