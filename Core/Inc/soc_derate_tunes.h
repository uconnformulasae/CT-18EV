#ifndef SOC_DERATE_TUNES_H
#define SOC_DERATE_TUNES_H


/* Active tune */
#define SOC_DERATE_TUNE flat

/*
 * Tune setup:
 *   id   logged in 0x55A byte 0,
 *   soc  weakest cell SoC (0-1) -> DCL (A), as {SoC, amps} points sorted by SoC.
 *        Interpolated between points, held flat past the ends.
 */

SOC_DERATE_TUNE_DEF(flat,
    .id = 0,
    .soc = CURVE({0.00f, 165.0f}));

/* -5 A per SoC % below 95% */
SOC_DERATE_TUNE_DEF(endurance,
    .id = 1,
    .soc = CURVE({0.62f, 0.0f}, {0.95f, 165.0f}));

#endif /* SOC_DERATE_TUNES_H */
