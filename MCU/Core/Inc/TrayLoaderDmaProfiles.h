#ifndef TRAY_LOADER_DMA_PROFILES_H
#define TRAY_LOADER_DMA_PROFILES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* TIM5/TIM8 Update DMA ARR profiles stored as const data in Flash. */

#define TRAY_LOADER_BELT_45_ARR_DMA_LENGTH (1200U)
extern const uint32_t tray_loader_belt_45_arr_dma[TRAY_LOADER_BELT_45_ARR_DMA_LENGTH];

#define TRAY_LOADER_BELT_108_ARR_DMA_LENGTH (2880U)
extern const uint32_t tray_loader_belt_108_arr_dma[TRAY_LOADER_BELT_108_ARR_DMA_LENGTH];

#define TRAY_LOADER_BELT_243_ARR_DMA_LENGTH (6480U)
extern const uint32_t tray_loader_belt_243_arr_dma[TRAY_LOADER_BELT_243_ARR_DMA_LENGTH];

#define TRAY_LOADER_SCREW_130_ARR_DMA_LENGTH (20800U)
extern const uint16_t tray_loader_screw_130_arr_dma[TRAY_LOADER_SCREW_130_ARR_DMA_LENGTH];

#ifdef __cplusplus
}
#endif

#endif /* TRAY_LOADER_DMA_PROFILES_H */
