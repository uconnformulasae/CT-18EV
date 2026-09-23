/* Non-blocking pedal and brake sampling */

#include "adc.h"

static volatile uint32_t dual_raw; /* ADC1 in [15:0], ADC2 in [31:16] */
static volatile uint16_t bps_raw;
static volatile uint8_t dual_done;
static volatile uint8_t bps_done;
static volatile uint16_t err_count;

static uint16_t tps1_latched;
static uint16_t tps2_latched;
static uint16_t bps_latched;
static uint8_t have_samples;
static uint16_t overrun_count;

#ifdef ADC_HOST

uint32_t adc_host_arm_count;
uint32_t adc_host_calibrate_count;
uint32_t adc_host_armed_before_calibration;

static void adc_calibrate(void)
{
    adc_host_calibrate_count++;
}

static void adc_arm(void)
{
    if (adc_host_calibrate_count == 0u) {
        adc_host_armed_before_calibration++;
    }
    adc_host_arm_count++;
}

#else

#include "main.h"
#include "pinout.h"

static void adc_calibrate(void)
{
    if (HAL_ADCEx_Calibration_Start(ADC_TPS1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(ADC_TPS2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(ADC_BPS) != HAL_OK) {
        Error_Handler();
    }
}

static void adc_arm(void)
{
    /* Set EXTTRIG on slave ADC2 */
    if (HAL_ADC_Start(ADC_TPS2) != HAL_OK) {
        (void)HAL_ADC_Stop(ADC_TPS2);
        err_count++;
    }

    /* ADC1+ADC2 dual regular DMA */
    if (HAL_ADCEx_MultiModeStart_DMA(ADC_TPS1, (uint32_t *)&dual_raw, 1u) != HAL_OK) {
        (void)HAL_ADCEx_MultiModeStop_DMA(ADC_TPS1);
        err_count++;
    }

    if (HAL_ADC_Start_DMA(ADC_BPS, (uint32_t *)&bps_raw, 1u) != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(ADC_BPS);
        err_count++;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        dual_done = 1u;
    } else if (hadc->Instance == ADC3) {
        bps_done = 1u;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    err_count++;
}

#endif

void adc_init(void)
{
    dual_raw = 0u;
    bps_raw = 0u;
    dual_done = 0u;
    bps_done = 0u;
    err_count = 0u;

    tps1_latched = 0u;
    tps2_latched = 0u;
    bps_latched = 0u;
    have_samples = 0u;
    overrun_count = 0u;

    adc_calibrate();
    adc_arm();
}

void adc_update(void)
{
#ifndef ADC_HOST
    dual_done = 0u;
    bps_done = 0u;
    adc_arm();
    uint32_t timeout = 5000u;
    while ((!dual_done || !bps_done) && --timeout) {
    }
#endif

    if (dual_done && bps_done) {
        const uint32_t dual = dual_raw;
        tps1_latched = (uint16_t)(dual & 0xFFFFu);
        tps2_latched = (uint16_t)(dual >> 16);
        bps_latched = bps_raw;
        have_samples = 1u;
    } else if (have_samples) {
        /* Hold previous on overrun */
        overrun_count++;
    }

#ifdef ADC_HOST
    dual_done = 0u;
    bps_done = 0u;
    adc_arm();
#endif
}

uint16_t adc_tps1(void)
{
    return tps1_latched;
}

uint16_t adc_tps2(void)
{
    return tps2_latched;
}

uint16_t adc_bps(void)
{
    return bps_latched;
}

uint8_t adc_have_samples(void)
{
    return have_samples;
}

uint16_t adc_overruns(void)
{
    return overrun_count;
}

uint16_t adc_errors(void)
{
    return err_count;
}
