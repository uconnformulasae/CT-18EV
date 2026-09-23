#ifndef PINOUT_H
#define PINOUT_H

#include "main.h"

/* RTD button (active high) */
#define PIN_RTD_BUTTON_PORT   GPIOB
#define PIN_RTD_BUTTON        GPIO_PIN_14
#define PIN_RTD_BUTTON_PULL   GPIO_PULLDOWN
#define PIN_RTD_BUTTON_ACTIVE GPIO_PIN_SET

/* RTD dash indicator */
#define PIN_RTD_LIGHT_PORT GPIOB
#define PIN_RTD_LIGHT      GPIO_PIN_1

/* RTD buzzer */
#define PIN_RTD_BUZZER_PORT GPIOB
#define PIN_RTD_BUZZER      GPIO_PIN_2

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

#define ADC_TPS1 (&hadc1) /* ADC1 ch10 */
#define ADC_TPS2 (&hadc2) /* ADC2 ch11 */
#define ADC_BPS  (&hadc3) /* ADC3 ch12 */

#endif /* PINOUT_H */
