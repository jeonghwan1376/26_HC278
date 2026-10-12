#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef void (*SensorEventCallback_t)(uint32_t now_tick);

typedef struct
{
    /* PC9: 새 트레이 최초 등록 */
    SensorEventCallback_t on_tray_pc9_detected;

    /* PC10: 기존 트레이 위치를 Y=-100 mm로 보정 */
    SensorEventCallback_t on_tray_pc10_detected;
} SensorCallbacks_t;

void Sensor_Init(const SensorCallbacks_t *callbacks);
void Sensor_Update(uint32_t now_tick);

#ifdef __cplusplus
}
#endif

#endif /* INC_SENSOR_H_ */
