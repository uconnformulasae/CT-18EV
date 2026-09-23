#include "regen.h"

static regen_debug_t dbg;

void regen_init(void)
{
    dbg.active = 0;
    dbg.entry_speed_rpm = 0;
    dbg.torque = 0;
}

const regen_debug_t *regen_get_debug(void)
{
    return &dbg;
}

int32_t regen_update(int32_t driver_torque, uint32_t motor_speed_rpm, float tps_combined,
                     uint8_t brake_pressed, uint8_t soc_percent, uint8_t bms_live)
{
    const uint8_t coasting = (tps_combined < REGEN_TPS_THRESHOLD) &&
                             (motor_speed_rpm > REGEN_CUTOFF_RPM) && !brake_pressed;

    if (!coasting || !bms_live || soc_percent > REGEN_MAX_SOC_PERCENT) {
        dbg.active = 0;
        dbg.torque = 0;
        return driver_torque;
    }

    if (!dbg.active) {
        dbg.active = 1;
        dbg.entry_speed_rpm = motor_speed_rpm;
    } else if (motor_speed_rpm > dbg.entry_speed_rpm) {
        dbg.entry_speed_rpm = motor_speed_rpm;
    }

    int32_t torque;
    if (dbg.entry_speed_rpm > REGEN_MIN_RAMP_RPM) {
        const float ratio = (float)(motor_speed_rpm - REGEN_CUTOFF_RPM) /
                            (float)(dbg.entry_speed_rpm - REGEN_CUTOFF_RPM);
        torque = REGEN_TORQUE_AT_CUTOFF +
                 (int32_t)(ratio * (float)(REGEN_TORQUE_AT_ENTRY - REGEN_TORQUE_AT_CUTOFF));
    } else {
        torque = REGEN_TORQUE_AT_CUTOFF;
    }

    dbg.torque = torque;
    return torque;
}
