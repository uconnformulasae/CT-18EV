/**
 * @file    adc.h
 * @brief   Non-blocking pedal and brake sampling.
 *
 * TPS1/TPS2 run on ADC1+ADC2 in dual regular simultaneous mode. ADC1's DMA
 * carries both results in one 32-bit word, ADC1 in the low half and ADC2 in
 * the high half; ADC2 has no DMA channel of its own on STM32F1. BPS is on
 * ADC3 with its own DMA. One conversion per control tick, read on the next.
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);

/* Latch the conversion started last tick and begin the next. Once per tick. */
void adc_update(void);

/* Raw counts from the most recent completed conversion. */
uint16_t adc_tps1(void);
uint16_t adc_tps2(void);
uint16_t adc_bps(void);

/* 0 until the first conversion completes. */
uint8_t adc_have_samples(void);

/* Conversions not complete by the following tick. */
uint16_t adc_overruns(void);

/* HAL start failures and DMA errors. */
uint16_t adc_errors(void);

#endif /* ADC_H */
