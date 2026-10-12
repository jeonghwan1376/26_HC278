#ifndef INC_CONFIRM_CONVEYOR_H_
#define INC_CONFIRM_CONVEYOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void ConfirmConveyor_Init(void);
void ConfirmConveyor_Start(void);
void ConfirmConveyor_Stop(void);
uint8_t ConfirmConveyor_IsRunning(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_CONFIRM_CONVEYOR_H_ */
