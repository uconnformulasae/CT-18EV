/**
 * @file    soc_kf.h
 * @brief   Pack SoC via a 1-RC EKF. TELEMETRY ONLY - never gates the control
 *          path; Orion's SoC stays authoritative for regen.
 */

#ifndef SOC_KF_H
#define SOC_KF_H

#include <stdint.h>

#define SOC_KF_CAN_ID_STATE 0x558

#define SOC_KF_NS 100
#define SOC_KF_NP 4

#define SOC_KF_IBAT_DISCHARGE_POSITIVE 1

#define SOC_KF_Q_Z_PER_S  1.1e-9f
#define SOC_KF_Q_V1_PER_S 1.1e-5f
#define SOC_KF_R_MEAS     1e-3f

#define SOC_KF_BLACKOUT_I_A 300.0f
#define SOC_KF_V1_MAX_V     0.5f
#define SOC_KF_OCV_SLOPE_MIN 0.1f

#define SOC_KF_NIS_GATE     49.0f
#define SOC_KF_GATE_INFLATE 1.5f

#define SOC_KF_P0_Z_OCV   1e-4f
#define SOC_KF_P0_Z_ORION 1e-2f
#define SOC_KF_P0_V1      1e-4f

#define SOC_KF_STALE_MS        200
#define SOC_KF_REST_CURRENT_A  2.0f
#define SOC_KF_REST_MS         2000
#define SOC_KF_INIT_TIMEOUT_MS 3000
#define SOC_KF_P_FLOOR         1e-12f
#define SOC_KF_P_MAX           1.0f

#define SOC_KF_VCELL_MIN 2.0f
#define SOC_KF_VCELL_MAX 4.35f

#define SOC_KF_FLAG_INIT     0x01
#define SOC_KF_FLAG_OCV_INIT 0x02
#define SOC_KF_FLAG_BMS_LIVE 0x04
#define SOC_KF_FLAG_GATED    0x08
#define SOC_KF_FLAG_FROZEN   0x10
#define SOC_KF_FLAG_CLAMPED  0x20
#define SOC_KF_FLAG_VBAD     0x40

typedef struct {
    float soc;
    float v1;
    float innovation;
    float p00;
    float p11;
    float i_pack;
    float v_pack;
    float charge_ah;
    uint8_t temp_c;
    uint8_t orion_soc;
    uint8_t flags;
} soc_kf_debug_t;

void soc_kf_init(void);

void soc_kf_feed_bms(int16_t ibat_raw, uint16_t vbat_raw, uint8_t btmp_raw, uint8_t soc_raw,
                     uint32_t tick_ms);

void soc_kf_update(uint32_t tick_ms);

const soc_kf_debug_t *soc_kf_get_debug(void);

void soc_kf_pack_state(uint8_t *d);

#endif /* SOC_KF_H */
