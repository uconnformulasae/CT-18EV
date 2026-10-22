#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#include "config.h"

typedef struct {
    float tps1_avg;
    float tps2_avg;
    uint8_t bse;
    uint32_t fault_ms;
} control_t;

typedef struct {
    float tps1; /* 0..1 */
    float tps2;
    float tps; /* Average of both */
    uint8_t tps1_oor;
    uint8_t tps2_oor;
    uint8_t tps_disagree;
} control_pedals_t;

/* Everything in the 0x555 debug frame */
typedef struct {
    uint8_t rtd_timeout;
    uint32_t bps_adc;
    uint32_t tps1_adc;
    uint32_t tps2_adc;
    control_pedals_t pedals;
    uint8_t bse;
    uint8_t ready_to_drive;
    uint8_t disable;
    uint8_t inverter_enabled;
    uint8_t inverter_lockout;
} control_status_t;

void control_init(control_t *c);

/* Calibrate APPS voltages and check them (T.4.2) */
control_pedals_t control_pedals(control_t *c, float tps1_v, float tps2_v);

/* Brake pressure sensor open or shorted (T.4.3.4) */
uint8_t control_bps_oor(float bps_v);

/* Brake pressure sensor volts to psi, clamped to the sensor range */
float control_bps_psi(float bps_v);

/* Brake + throttle latch (EV.4.7): trips at BSE_TRIP_TPS, clears below BSE_CLEAR_TPS */
uint8_t control_bse(control_t *c, uint8_t brake, float tps);

/* Returns 1 once a fault has persisted past DISABLE_DEBOUNCE_MS, starts tripped */
uint8_t control_fault_debounce(control_t *c, uint8_t fault, uint32_t dt_ms);

/* TPS to 0..1 across the drive range */
float control_torque_map(float tps);

/* Torque request (0.1 Nm) for a mapped pedal, capped by DC current limit */
int32_t control_torque_limit(float x, uint32_t rpm, uint32_t dcl_a);

void control_pack_inverter(uint8_t *d, int32_t torque, uint8_t lockout, uint8_t disable,
                           uint8_t heartbeat);
void control_pack_debug(uint8_t *d, const control_status_t *s);

/* 0x55C: psi x10 [0:1], raw ADC [2:3], little endian; bit 0 pressed, bit 1 oor [4] */
void control_pack_bps(uint8_t *d, float psi, uint16_t adc, uint8_t pressed, uint8_t oor);

/* 0x55E: TPS1/TPS2 raw ADC [0:1] [2:3], TPS1/TPS2 0.01% [4:5] [6:7], little endian */
void control_pack_pedals(uint8_t *d, const control_pedals_t *p, uint16_t tps1_adc,
                         uint16_t tps2_adc);

#endif /* CONTROL_H */
