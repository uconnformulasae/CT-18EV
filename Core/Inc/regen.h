#ifndef REGEN_H
#define REGEN_H

#include <stdint.h>

#include "soc_kf.h"

#define REGEN_DEBUG_CAN_ID 0x559

#define REGEN_CELL_CHARGE_A 25.0f

#define REGEN_CUTOFF_RPM 500u
#define REGEN_RAMP_RPM   650u
#define REGEN_RPM_MAX    6000.0f

/* Regen torque at REGEN_RPM_MAX (Nm) */
#define REGEN_PEAK_NM 100.0f

/* Regen fades in from zero at SOC_OFF to full strength at SOC_FULL */
#define REGEN_SOC_OFF  0.95f
#define REGEN_SOC_FULL 0.80f

/* Slew below BAND (0.1 Nm), rates in 0.1 Nm/ms */
#define REGEN_SLEW_BAND 100
#define REGEN_SLEW_UP   15
#define REGEN_SLEW_DOWN 20
#define REGEN_DT_MAX_MS 10u

typedef struct {
    float pedal;
    float max_nm;
    float soc_scale;
    int32_t target;
    int32_t torque;
    uint8_t cut;
    uint8_t soc_ok;
} regen_debug_t;

void regen_init(void);

/* Returns commanded torque (0.1 Nm) */
int32_t regen_update(int32_t drive_torque, uint32_t motor_speed_rpm, float tps,
                     const soc_kf_debug_t *kf, uint8_t cut, uint32_t dt_ms);

const regen_debug_t *regen_get_debug(void);
void regen_pack_debug(uint8_t *d);

#endif /* REGEN_H */
