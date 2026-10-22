#ifndef SOC_DERATE_TUNES_H
#define SOC_DERATE_TUNES_H

/* Active tune */
#ifndef SOC_DERATE_TUNE
#define SOC_DERATE_TUNE flat
#endif

/*
 * Tune setup:
 *   id        logged in 0x55A byte 0,
 *   weak_cell 0 runs the SoC curve on pack SoC, 1 on the weakest cell, in between
 *             takes that much of the gap off pack SoC.
 *   soc       SoC (0-1) -> DCL (A), as {SoC, amps} points sorted by SoC.
 *             Interpolated between points, held flat past the ends.
 *   temp      pack temp from Orion 0x600 (C, signed) -> DCL (A), same format.
 *             Optional, the lower of the two wins.
 */

SOC_DERATE_TUNE_DEF(flat,
    .id = 0,
    .soc = CURVE({0.00f, 165.0f}));

/*
 * Full current down to 25%, then about 90% of what a P50B holds for 10 s
 * at 35 C without going below 2.8 V (soc_kf_tables.h). The pack is that warm
 * by the time SoC is this low.
 * Temp backs off from 47 C so the pack never reaches the 60 C trip.
 */
SOC_DERATE_TUNE_DEF(endurance,
    .id = 1,
    .weak_cell = 0.5f,
    .soc = CURVE({0.05f, 0.0f}, {0.10f, 20.0f}, {0.15f, 60.0f}, {0.20f, 120.0f},
                 {0.25f, 165.0f}),
    .temp = CURVE({47.0f, 165.0f}, {51.0f, 120.0f}, {55.0f, 65.0f}, {58.0f, 30.0f}));

#endif /* SOC_DERATE_TUNES_H */
