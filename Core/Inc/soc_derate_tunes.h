#ifndef SOC_DERATE_TUNES_H
#define SOC_DERATE_TUNES_H


/* Active tune */
#define SOC_DERATE_TUNE endurance

/*
 * Tune setup:
 *   id   logged in 0x55A byte 0,
 *   weak_cell  how much the weakest cell pulls the map input down, 0-1.
 *              Map runs on pack SoC - weak_cell * gap (0 = pack only, 1 = weakest cell).
 *   soc        map input SoC (0-1) -> DCL (A), as {SoC, amps} points sorted by SoC.
 *              Interpolated between points, held flat past the ends.
 */

SOC_DERATE_TUNE_DEF(flat,
    .id = 0,
    .weak_cell = 1.0f,
    .soc = CURVE({0.00f, 165.0f}));

/* Cell held at 2.8 V under sustained load, P50B OCV and R0+R1 at 25 C.
   Half weight on the gap, so the weakest cell can dip a little below that. */
SOC_DERATE_TUNE_DEF(endurance,
    .id = 1,
    .weak_cell = 0.5f,
    .soc = CURVE({0.05f, 0.0f}, {0.10f, 13.0f}, {0.15f, 43.0f}, {0.20f, 84.0f},
                 {0.25f, 123.0f}, {0.30f, 149.0f}, {0.35f, 165.0f}));

#endif /* SOC_DERATE_TUNES_H */
