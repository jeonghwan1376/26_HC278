#ifndef INC_UART_H_
#define INC_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stddef.h>
#include <stdint.h>

#define UART_RX_LINE_SIZE   64U

/* USART 인터럽트 기반 송수신 모듈 초기화 */
HAL_StatusTypeDef UART_Comm_Init(UART_HandleTypeDef *huart);

/* 송신 큐에 문자열 추가. 실제 송신은 인터럽트로 순차 처리된다. */
HAL_StatusTypeDef UART_Send_IT(const char *msg);

/* RobotProcessStatusCallback_t에 직접 등록하기 위한 void 반환 래퍼 */
void UART_SendMessage(const char *msg);

/* 수신 링 버퍼에서 완성된 한 줄을 꺼낸다. 성공 시 1, 없으면 0 */
uint8_t UART_GetLine(char *out_line, size_t out_size);

/* HAL_BUSY 등으로 시작하지 못한 송신을 다시 시도한다. 태스크에서 주기 호출 */
void UART_Process(void);

uint8_t UART_GetRxOverflow(void);
void UART_ClearRxOverflow(void);
uint8_t UART_GetTxOverflow(void);
void UART_ClearTxOverflow(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_UART_H_ */
