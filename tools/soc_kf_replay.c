/* Replay a logged BMS stream through soc_kf and compare against the SoC the
 * car actually published on 0x558. Includes the .c directly to reach the
 * static state, the same way soc_kf_test.c does.
 *
 *   cc -DSOC_KF_HOST -I../Core/Inc -o soc_kf_replay soc_kf_replay.c -lm
 *   ./soc_kf_replay log.csv [options]
 *
 * The CSV needs a header line and these columns, in order:
 *
 *   t_ms,ibat_raw,vbat_raw,btmp_raw,soc_raw[,kf_logged]
 *
 * The four raw fields are the 0x600 payload exactly as soc_kf_feed_bms() takes
 * them - int16 0.1 A/bit, uint16 0.1 V/bit, uint8 1 C/bit, uint8 0.5 %/bit -
 * NOT engineering units. kf_logged is optional and is the SoC the car reported
 * in 0x558 bytes 0-1, in percent, used only for the comparison column.
 *
 * IMPORTANT: feed the log at the real 0x600 rate. A logger that subsamples
 * (an AiM EVO5 records these at ~31 Hz against a 50 Hz broadcast) will change
 * which seed path the filter takes and quietly invalidate the replay, so the
 * detected rate is printed and flagged. Resample to the broadcast rate first.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../Core/Src/soc_kf.c"

static const char *FLAG_NAME[8] = {"INIT", "OCV", "LIVE", "GATED", "FROZ", "CLAMP", "VBAD", "b7"};

static void flags_str(uint8_t f, char *out, size_t n)
{
    out[0] = '\0';
    for (int b = 0; b < 8; b++) {
        if (!(f & (1u << b)))
            continue;
        if (out[0] != '\0')
            strncat(out, "|", n - strlen(out) - 1);
        strncat(out, FLAG_NAME[b], n - strlen(out) - 1);
    }
    if (out[0] == '\0')
        strncat(out, "-", n - 1);
}

static uint16_t bswap16(uint16_t v)
{
    return (uint16_t)((v >> 8) | (v << 8));
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s LOG.csv [--period MS] [--every S] [--swap-current]\n"
            "                  [--swap-voltage] [--quiet]\n\n"
            "  --period MS      soc_kf_update() cadence, default 90 (the ~11 Hz\n"
            "                   main loop). Set it to the real loop period.\n"
            "  --every S        print a row this often, default 120.\n"
            "  --swap-current   byte-swap ibat_raw before feeding, to test a\n"
            "                   suspected endianness mismatch on 0x600 b6-7.\n"
            "  --swap-voltage   same for vbat_raw, 0x600 b4-5.\n"
            "  --quiet          summary only.\n",
            argv0);
}

int main(int argc, char **argv)
{
    const char *path = NULL;
    uint32_t period_ms = 90u;
    double every_s = 120.0;
    int swap_i = 0, swap_v = 0, quiet = 0;

    for (int k = 1; k < argc; k++) {
        if (!strcmp(argv[k], "--period") && k + 1 < argc)
            period_ms = (uint32_t)strtoul(argv[++k], NULL, 10);
        else if (!strcmp(argv[k], "--every") && k + 1 < argc)
            every_s = strtod(argv[++k], NULL);
        else if (!strcmp(argv[k], "--swap-current"))
            swap_i = 1;
        else if (!strcmp(argv[k], "--swap-voltage"))
            swap_v = 1;
        else if (!strcmp(argv[k], "--quiet"))
            quiet = 1;
        else if (argv[k][0] != '-' && path == NULL)
            path = argv[k];
        else {
            usage(argv[0]);
            return 2;
        }
    }
    if (path == NULL) {
        usage(argv[0]);
        return 2;
    }

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        perror(path);
        return 1;
    }

    char line[512];
    if (fgets(line, sizeof line, f) == NULL) { /* header */
        fprintf(stderr, "%s: empty\n", path);
        fclose(f);
        return 1;
    }

    soc_kf_init();

    uint32_t t = 0u, t_first = 0u, t_prev = 0u, last_update = 0u;
    unsigned long rows = 0, updates = 0;
    double next_print = 0.0;
    double sum_sq = 0.0, max_abs = 0.0;
    unsigned long compared = 0;
    uint8_t flags_seen = 0u;
    double gap_max_ms = 0.0;

    if (!quiet)
        printf("%9s %9s %9s %9s %9s %10s  %s\n", "t(s)", "I(A)", "V(V)", "kf(%)", "logged", "diff",
               "flags");

    while (fgets(line, sizeof line, f) != NULL) {
        long tv;
        int ib, vb, bt, sr;
        double klog = NAN;
        const int n = sscanf(line, "%ld,%d,%d,%d,%d,%lf", &tv, &ib, &vb, &bt, &sr, &klog);
        if (n < 5)
            continue;

        t = (uint32_t)tv;
        if (rows == 0u) {
            t_first = t;
            t_prev = t;
            last_update = t;
        } else if (t - t_prev > (uint32_t)gap_max_ms) {
            gap_max_ms = (double)(t - t_prev);
        }

        uint16_t iraw = (uint16_t)ib;
        uint16_t vraw = (uint16_t)vb;
        if (swap_i)
            iraw = bswap16(iraw);
        if (swap_v)
            vraw = bswap16(vraw);

        soc_kf_feed_bms((int16_t)iraw, vraw, (uint8_t)bt, (uint8_t)sr, t);

        if (t - last_update >= period_ms) {
            soc_kf_update(t);
            last_update = t;
            updates++;

            flags_seen |= s.dbg.flags;
            const double est = s.dbg.soc * 100.0;
            if (!isnan(klog)) {
                const double d = est - klog;
                sum_sq += d * d;
                if (fabs(d) > max_abs)
                    max_abs = fabs(d);
                compared++;
            }

            const double el = (double)(t - t_first) / 1000.0;
            if (!quiet && el >= next_print) {
                char fs[64];
                flags_str(s.dbg.flags, fs, sizeof fs);
                printf("%9.1f %9.2f %9.1f %9.3f ", el, s.dbg.i_pack, s.dbg.v_pack, est);
                if (isnan(klog))
                    printf("%9s %10s", "-", "-");
                else
                    printf("%9.3f %+10.3f", klog, est - klog);
                printf("  %s\n", fs);
                next_print = el + every_s;
            }
        }
        t_prev = t;
        rows++;
    }
    fclose(f);

    if (rows < 2u) {
        fprintf(stderr, "%s: need at least 2 data rows\n", path);
        return 1;
    }

    const double span_s = (double)(t - t_first) / 1000.0;
    const double feed_hz = (double)(rows - 1u) / span_s;

    char fs[64];
    flags_str(flags_seen, fs, sizeof fs);
    printf("\n%s\n", path);
    printf("  rows %lu over %.1f s -> feed %.1f Hz, %lu updates at %u ms\n", rows, span_s, feed_hz,
           updates, period_ms);
    printf("  largest gap between rows: %.0f ms (STALE_MS is %d)\n", gap_max_ms, SOC_KF_STALE_MS);
    printf("  flags seen: 0x%02X %s\n", flags_seen, fs);
    printf("  final SoC %.3f%%  charge %.3f Ah  p00 %.3e (sigma %.2f%%)\n", s.dbg.soc * 100.0,
           s.dbg.charge_ah, s.p00, sqrt(s.p00) * 100.0);
    if (compared > 0u)
        printf("  vs logged: RMS %.3f%%, max %.3f%% over %lu updates\n", sqrt(sum_sq / compared),
               max_abs, compared);
    if (swap_i || swap_v)
        printf("  NOTE: byte-swapped %s%s%s before feeding\n", swap_i ? "current" : "",
               (swap_i && swap_v) ? " and " : "", swap_v ? "voltage" : "");
    if (feed_hz < 40.0)
        printf("  WARNING: feed is %.1f Hz. 0x600 broadcasts at 50 Hz; a subsampled log\n"
               "           changes the rest timer and the seed path. Resample first.\n",
               feed_hz);
    return 0;
}
