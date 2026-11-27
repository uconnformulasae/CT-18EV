#pragma once
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_init(CAN_HandleTypeDef* hcan);
void app_start();
void app_step(IWDG_HandleTypeDef* hiwdg, uint8_t reset_cause);

//
void HAL_CAN_RxFifo0MsgPendingCallback_Impl(CAN_HandleTypeDef *hcan);
void HAL_TIM_PeriodElapsedCallback_Impl(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif