#ifndef REGEN_H
#define REGEN_H

#include <stdint.h>

#include "bms_limits.h"
#include "config.h"

/* 0x559: cut [0], CPL in use, kW, saturating [1], max regen, Nm [2] */
#define REGEN_DEBUG_CAN_ID 0x559
#define REGEN_DEBUG_DLC    3

#define REGEN_CUTOFF_RPM 500u
#define REGEN_RAMP_RPM   650u
#define REGEN_RPM_MAX    6000.0f

/* Regen torque at REGEN_RPM_MAX (Nm) */
#define REGEN_PEAK_NM 100.0f

/* Slew below BAND (0.1 Nm), rates in 0.1 Nm/ms */
#define REGEN_SLEW_BAND 100
#define REGEN_SLEW_UP   15
#define REGEN_SLEW_DOWN 20
#define REGEN_DT_MAX_MS 10u

typedef struct {
    float pedal;
    float max_nm;
    uint32_t cpl_w; /* charge power limit in use (W), 0 when the limits are not live */
    int32_t target;
    int32_t torque;
    uint8_t cut;
    uint8_t limits_live;
} regen_debug_t;

void regen_init(void);

/* Returns commanded torque (0.1 Nm). Regen torque is capped at lim->cpl_w over the
   motor speed (efficiency 1), and is 0 when the limits are not live. */
int32_t regen_update(int32_t drive_torque, uint32_t motor_speed_rpm, float tps,
                     const bms_limits_t *lim, uint8_t cut, uint32_t dt_ms);

const regen_debug_t *regen_get_debug(void);
void regen_pack_debug(uint8_t *d);

#endif /* REGEN_H */
