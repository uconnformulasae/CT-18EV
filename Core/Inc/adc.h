#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);
void adc_update(void);

/* Raw ADC counts */
uint16_t adc_tps1(void);
uint16_t adc_tps2(void);
uint16_t adc_bps(void);

uint8_t adc_have_samples(void);
uint16_t adc_overruns(void);
uint16_t adc_errors(void);

#endif /* ADC_H */
