#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#include "bms_limits.h"


/* 0x55A byte 6 */
#define CONTROL_LIMITS_LIVE    0x01 /* fresh, valid 0x203 */
#define CONTROL_LIMITS_LIMP    0x02 /* using limp values */
#define CONTROL_LIMITS_STALE   0x04 /* BMS_LIMITS_WHY_STALE */
#define CONTROL_LIMITS_VERSION 0x08 /* BMS_LIMITS_WHY_VERSION */
#define CONTROL_LIMITS_COUNTER 0x10 /* BMS_LIMITS_WHY_COUNTER */

typedef struct {
    uint32_t bps_adc;
    uint32_t tps1_adc;
    uint32_t tps2_adc;
} control_adcs_t;

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
} control_throttle_t;

typedef struct {
    float bps;
    uint8_t bps_oor;
    uint8_t brake_pressed;
} control_brakes_t;

typedef struct  {
    control_throttle_t throttle;
    control_brakes_t brakes;
    control_adcs_t adcs;
} control_data_t;

/* Everything in the 0x555 debug frame */
typedef struct {
    uint8_t rtd_timeout;
    uint32_t bps_adc;
    uint32_t tps1_adc;
    uint32_t tps2_adc;
    control_throttle_t pedals;
    uint8_t bse;
    uint8_t ready_to_drive;
    uint8_t disable;
    uint8_t inverter_enabled;
    uint8_t inverter_lockout;
} control_status_t;

void control_init(control_t *c);

/* Calibrate APPS voltages and check them (T.4.2) */
control_throttle_t control_throttle(control_t *c, float tps1_v, float tps2_v);

control_brakes_t control_brakes(float bps);

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

/* Torque request (0.1 Nm) for a mapped pedal, capped by the BMS's discharge power limit (W) */
int32_t control_torque_limit(float x, uint32_t rpm, uint32_t dpl_w);

void control_pack_inverter(uint8_t *d, int32_t torque, uint8_t lockout, uint8_t disable,
                           uint8_t heartbeat);
void control_pack_debug(
    uint8_t *buf, 
    const control_data_t *controls, 
    uint8_t rtd_timeout, 
    uint8_t inverter_lockout, 
    uint8_t inverter_enabled,
    uint8_t bse_error,
    uint8_t ready_to_drive,
    uint8_t should_disable_inverter
);

/* 0x55C: psi x10 [0:1], raw ADC [2:3], little endian; bit 0 pressed, bit 1 oor [4] */
void control_pack_bps(uint8_t *d, float psi, uint16_t adc, uint8_t pressed, uint8_t oor);

/* 0x55A LVCU_Limits: drive torque limit [0:1] and regen torque limit (magnitude) [2:3],
   0.1 Nm; DPL in use, 0.1 kW [4:5]; little endian, saturating; CONTROL_LIMITS_* [6] */
void control_pack_limits(uint8_t *d, int32_t drive_x10, int32_t regen_x10, const bms_limits_t *lim);

#endif /* CONTROL_H */
