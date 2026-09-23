#ifndef REGEN_H
#define REGEN_H

#include <stdint.h>

#define REGEN_TPS_THRESHOLD 0.03f
#define REGEN_CUTOFF_RPM 500u
#define REGEN_MIN_RAMP_RPM 550u
#define REGEN_MAX_SOC_PERCENT 80u

/* Regen torque (0.1 Nm, negative) */
#define REGEN_TORQUE_AT_ENTRY  (-200)
#define REGEN_TORQUE_AT_CUTOFF (-250)

typedef struct {
    uint8_t active;           /* Regen active */
    uint32_t entry_speed_rpm; /* Entry speed (RPM) */
    int32_t torque;           /* Applied torque (0.1 Nm) */
} regen_debug_t;

void regen_init(void);

/* Returns commanded torque (0.1 Nm) */
int32_t regen_update(int32_t driver_torque, uint32_t motor_speed_rpm, float tps_combined,
                     uint8_t brake_pressed, uint8_t soc_percent, uint8_t bms_live);

const regen_debug_t *regen_get_debug(void);

#endif /* REGEN_H */
