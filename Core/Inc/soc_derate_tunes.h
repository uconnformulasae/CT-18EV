#ifndef SOC_DERATE_TUNES_H
#define SOC_DERATE_TUNES_H


/* Active tune */
#define SOC_DERATE_TUNE endurance

/*
 * Tune setup:
 *   id   logged in 0x55A byte 0,
 *   soc  weakest cell SoC (0-1) -> DCL (A), as {SoC, amps} points sorted by SoC.
 *        Interpolated between points, held flat past the ends.
 */

SOC_DERATE_TUNE_DEF(flat,
    .id = 0,
    .soc = CURVE({0.00f, 165.0f}));

/* Weak cell held at 2.8 V under sustained load, P50B OCV and R0+R1 at 25 C */
SOC_DERATE_TUNE_DEF(endurance,
    .id = 1,
    .soc = CURVE({0.05f, 0.0f}, {0.10f, 13.0f}, {0.15f, 43.0f}, {0.20f, 84.0f},
                 {0.25f, 123.0f}, {0.30f, 149.0f}, {0.35f, 165.0f}));

#endif /* SOC_DERATE_TUNES_H */
