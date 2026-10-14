#ifndef SOC_DERATE_H
#define SOC_DERATE_H

#include <stdint.h>

#include "config.h"
#include "soc_kf.h"

#define SOC_DERATE_CAN_ID 0x55A

/* Which input set the active DCL */
#define SOC_DERATE_SRC_BMS 0 /* KF invalid or BMS DCL is 0 */
#define SOC_DERATE_SRC_SOC 1

typedef struct {
    float x;
    float amps;
} soc_derate_pt_t;

typedef struct {
    const soc_derate_pt_t *pts;
    uint8_t n;
} soc_derate_curve_t;

typedef struct {
    uint8_t id;
    soc_derate_curve_t soc;
} soc_derate_tune_t;

typedef struct {
    float gap;      /* Avg cell SoC minus weakest cell SoC */
    float weak_soc; /* KF SoC minus gap */
    uint32_t dcl;
    uint8_t src;
    uint8_t cell_live;
} soc_derate_debug_t;

void soc_derate_init(void);

/* Orion 0x79F low and average opencell voltage (0.1 mV) */
void soc_derate_feed_cells(uint16_t low_ocv_raw, uint16_t avg_ocv_raw, uint32_t tick_ms);

/* Returns DCL (A) */
uint32_t soc_derate_dcl(const soc_kf_debug_t *kf, uint32_t bms_dcl, uint32_t tick_ms,
                        uint32_t dt_ms);

const soc_derate_debug_t *soc_derate_get_debug(void);
void soc_derate_pack_debug(uint8_t *d);

#endif /* SOC_DERATE_H */
