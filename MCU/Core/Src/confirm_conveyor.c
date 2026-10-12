#include "confirm_conveyor.h"

#include "main.h"

extern TIM_HandleTypeDef htim3;

static volatile uint8_t confirm_conveyor_running = 0U;

void ConfirmConveyor_Init(void)
{
    /* PB13: ENA, PB15: DIR, TIM3 CH1: STEP PWM */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

    ConfirmConveyor_Start();
}

void ConfirmConveyor_Start(void)
{
    if (confirm_conveyor_running != 0U)
    {
        return;
    }

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) == HAL_OK)
    {
        confirm_conveyor_running = 1U;
    }
}

void ConfirmConveyor_Stop(void)
{
    if (confirm_conveyor_running == 0U)
    {
        return;
    }

    (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    confirm_conveyor_running = 0U;
}

uint8_t ConfirmConveyor_IsRunning(void)
{
    return confirm_conveyor_running;
}
