#pragma once
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// C++ entry point called from main.c
void app_init(CAN_HandleTypeDef* hcan);
void app_loop(IWDG_HandleTypeDef* hiwdg);

//
void HAL_CAN_RxFifo0MsgPendingCallback_Impl(CAN_HandleTypeDef *hcan);
void HAL_TIM_PeriodElapsedCallback_Impl(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif