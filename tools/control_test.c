/* Host tests for control (pedals, BSE, fault debounce, torque, CAN frames) */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../Core/Src/control.c"

#define STEP_MS 5u

static int failures = 0;
static int checks = 0;

static void check(int cond, const char *what)
{
    checks++;
    if (!cond) {
        failures++;
        printf("  FAIL: %s\n", what);
    }
}

static void check_eq(long got, long want, const char *what)
{
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL: %s (got %ld want %ld)\n", what, got, want);
    }
}

static void check_near(float got, float want, float tol, const char *what)
{
    checks++;
    if (fabsf(got - want) > tol) {
        failures++;
        printf("  FAIL: %s (got %g want %g)\n", what, got, want);
    }
}

/* Pedal fraction to sensor volts */
static float v1(float f)
{
    return TPS1_0PER + f * (TPS1_100PER - TPS1_0PER);
}

static float v2(float f)
{
    return TPS2_0PER + f * (TPS2_100PER - TPS2_0PER);
}

static control_pedals_t pedals_at(float f1, float f2)
{
    control_t c;
    control_init(&c);
    return control_pedals(&c, v1(f1), v2(f2));
}

/* ---- old main.c logic, verbatim, to prove the move changed nothing ---- */

static float old_tmap_lut(float tps)
{
    return (fmaxf(TMAP_DRIVE_START, fminf(tps, TMAP_DEADBAND_HIGH)) - TMAP_DRIVE_START) *
           (1.0f / (TMAP_DEADBAND_HIGH - TMAP_DRIVE_START));
}

static int32_t old_torque_lut(float tps, uint32_t motor_speed, uint32_t current_limit)
{
    const float power_limit = (float)(TORQUE_POWER_NUM * current_limit) /
                              fmaxf(TORQUE_SPEED_DIV_MIN, (float)motor_speed * TORQUE_RPM_TO_DIV);

    float torque_limit_local = fminf((float)TORQUE_LIMIT_NM_X10, power_limit);

    if (motor_speed >= TORQUE_HIGH_SPEED_RPM) {
        torque_limit_local = TORQUE_HIGH_SPEED_NM_X10;
    }

    return (int32_t)(tps * torque_limit_local);
}

/* Old debounce: loop sets the flag, TIM2 ISR counts it between loop passes */
typedef struct {
    uint8_t start;
    uint16_t counter;
} old_debounce_t;

static uint8_t old_debounce_step(old_debounce_t *o, uint8_t fault)
{
    o->start = fault;
    const uint8_t disable = o->counter > 10; /* DISABLE_DEBOUNCE_TRIP */

    /* TIM2 tick before the next pass */
    if (o->start) {
        o->counter += 1;
        if (o->counter > 100) { /* DISABLE_DEBOUNCE_MAX */
            o->counter = 100;
        }
    } else {
        o->counter = 0;
    }
    return disable;
}

static void old_pack_inverter(uint8_t *d, int32_t torque_request, uint8_t inverter_lockout,
                              uint8_t should_disable_inverter, uint8_t heartbeat_counter)
{
    if (inverter_lockout == 1) {
        d[0] = torque_request & 0xFF;
        d[1] = torque_request >> 8 & 0xFF;
        d[2] = 0x00;
        d[3] = 0x00;
        d[4] = 0x01;
        d[5] = 0x00 | 0x02 | (heartbeat_counter << 4);
        d[6] = 0x00;
        d[7] = 0x00;
    } else {
        d[0] = torque_request & 0xFF;
        d[1] = torque_request >> 8 & 0xFF;
        d[2] = 0x00;
        d[3] = 0x00;
        d[4] = 0x01;
        d[5] = (~should_disable_inverter & 0x01) | 0x02 | (heartbeat_counter << 4);
        d[6] = 0x00;
        d[7] = 0x00;
    }
}

static void test_matches_old(void)
{
    printf("test_matches_old\n");
    int bad = 0;

    for (int i = -20; i <= 120; i++) {
        const float tps = (float)i * 0.01f;
        bad += control_torque_map(tps) != old_tmap_lut(tps);
    }
    check_eq(bad, 0, "torque map identical");

    bad = 0;
    const uint32_t dcls[] = {0, 1, 60, 100, 121, 125, 165, 200, 400};
    for (uint32_t rpm = 0; rpm <= 7000; rpm += 50) {
        for (size_t k = 0; k < sizeof(dcls) / sizeof(dcls[0]); k++) {
            for (int i = 0; i <= 20; i++) {
                const float x = (float)i * 0.05f;
                bad += control_torque_limit(x, rpm, dcls[k]) != old_torque_lut(x, rpm, dcls[k]);
            }
        }
    }
    check_eq(bad, 0, "torque limit identical");

    /* Same fault pattern through both debounces */
    bad = 0;
    control_t c;
    control_init(&c);
    old_debounce_t o = {1, 999}; /* DISABLE_DEBOUNCE_INIT, flag set at boot */
    srand(18);
    uint8_t fault = 0;
    for (int i = 0; i < 20000; i++) {
        if (rand() % 23 == 0) {
            fault = !fault;
        }
        bad += control_fault_debounce(&c, fault, STEP_MS) != old_debounce_step(&o, fault);
    }
    check_eq(bad, 0, "debounce identical over random faults");

    bad = 0;
    uint8_t a[8], b[8];
    const int32_t torques[] = {0, 1, 437, 2200, -1, -437, -32768, 32767};
    for (size_t k = 0; k < sizeof(torques) / sizeof(torques[0]); k++) {
        for (int bits = 0; bits < 4; bits++) {
            for (uint8_t hb = 0; hb <= CAN_HEARTBEAT_MASK; hb++) {
                control_pack_inverter(a, torques[k], bits & 1, (bits >> 1) & 1, hb);
                old_pack_inverter(b, torques[k], bits & 1, (bits >> 1) & 1, hb);
                bad += memcmp(a, b, 8) != 0;
            }
        }
    }
    check_eq(bad, 0, "inverter frame identical");
}

/* ---- APPS (T.4.2) ---- */

static void test_pedals(void)
{
    printf("test_pedals\n");
    control_pedals_t p;

    p = pedals_at(0.0f, 0.0f);
    check_near(p.tps1, 0.0f, 1e-5f, "tps1 0% at calibration low");
    check_near(p.tps2, 0.0f, 1e-5f, "tps2 0% at calibration low");
    p = pedals_at(1.0f, 1.0f);
    check_near(p.tps1, 1.0f, 1e-5f, "tps1 100% at calibration high");
    check_near(p.tps2, 1.0f, 1e-5f, "tps2 100% at calibration high");
    p = pedals_at(0.5f, 0.5f);
    check_near(p.tps1, 0.5f, 1e-5f, "tps1 linear");
    check_near(p.tps, 0.5f, 1e-5f, "combined is the average");

    control_t c;
    control_init(&c);
    p = control_pedals(&c, TPS1_0PER - 0.3f, TPS2_100PER + 0.3f);
    check_near(p.tps1, 0.0f, 1e-6f, "clamped below 0");
    check_near(p.tps2, 1.0f, 1e-6f, "clamped above 1");

    /* Out of range: limits themselves are in range */
    control_init(&c);
    check_eq(control_pedals(&c, TPS1_FAULT_LOW, v2(0.0f)).tps1_oor, 0, "tps1 at low limit ok");
    check_eq(control_pedals(&c, TPS1_FAULT_LOW - 0.01f, v2(0.0f)).tps1_oor, 1, "tps1 below range");
    check_eq(control_pedals(&c, TPS1_FAULT_HIGH + 0.01f, v2(0.0f)).tps1_oor, 1, "tps1 above range");
    check_eq(control_pedals(&c, v1(0.0f), TPS2_FAULT_LOW - 0.01f).tps2_oor, 1, "tps2 below range");
    check_eq(control_pedals(&c, v1(0.0f), TPS2_FAULT_HIGH + 0.01f).tps2_oor, 1, "tps2 above range");
    check_eq(control_pedals(&c, 0.0f, v2(0.0f)).tps1_oor, 1, "tps1 open circuit");
    check_eq(control_pedals(&c, v1(0.0f), 0.0f).tps2_oor, 1, "tps2 open circuit");
    check_eq(control_pedals(&c, 3.3f, v2(0.0f)).tps1_oor, 1, "tps1 short to supply");

    /* Disagreement over APPS_TRIP_PERCENT */
    check_eq(pedals_at(0.50f, 0.55f).tps_disagree, 0, "5% apart ok");
    check_eq(pedals_at(0.50f, 0.59f).tps_disagree, 0, "9% apart ok");
    check_eq(pedals_at(0.50f, 0.62f).tps_disagree, 1, "12% apart trips");
    check_eq(pedals_at(0.80f, 0.20f).tps_disagree, 1, "60% apart trips");
    check_eq(pedals_at(0.30f, 0.0f).tps_disagree, 1, "one sensor stuck at 0");

    /* Filter is off (TPS_IIR_RATIO 0): follows input every step */
    control_init(&c);
    control_pedals(&c, v1(0.2f), v2(0.2f));
    check_near(control_pedals(&c, v1(0.9f), v2(0.9f)).tps, 0.9f, 1e-5f, "no lag with filter off");
}

/* ---- Brake/throttle plausibility (EV.4.7) ---- */

static void test_bse(void)
{
    printf("test_bse\n");
    control_t c;

    control_init(&c);
    check_eq(control_bse(&c, 1, BSE_TRIP_TPS - 0.01f), 0, "brake under trip throttle ok");
    check_eq(control_bse(&c, 0, 1.0f), 0, "full throttle without brake ok");
    check_eq(control_bse(&c, 1, BSE_TRIP_TPS), 1, "brake at trip throttle trips");

    check_eq(control_bse(&c, 0, 0.50f), 1, "stays latched after brake release");
    check_eq(control_bse(&c, 0, BSE_CLEAR_TPS), 1, "still latched at clear threshold");
    check_eq(control_bse(&c, 0, BSE_CLEAR_TPS - 0.01f), 0, "clears below 5% throttle");
    check_eq(control_bse(&c, 0, 0.50f), 0, "needs brake again to re-trip");

    control_init(&c);
    control_bse(&c, 1, 0.30f);
    check_eq(control_bse(&c, 1, 0.0f), 0, "clears with brake still on once off throttle");
}

static void test_bps_oor(void)
{
    printf("test_bps_oor\n");

    check_eq(control_bps_oor(0.0f), 1, "open circuit");
    check_eq(control_bps_oor(BPS_FAULT_LOW - 0.01f), 1, "just under low limit");
    check_eq(control_bps_oor(BPS_FAULT_LOW), 0, "at low limit");
    check_eq(control_bps_oor(0.5f), 0, "resting sensor");
    check_eq(control_bps_oor(BPS_SETPOINT_V + 0.1f), 0, "brake pressed");
    check_eq(control_bps_oor(BPS_FAULT_HIGH), 0, "at high limit");
    check_eq(control_bps_oor(BPS_FAULT_HIGH + 0.01f), 1, "just over high limit");
    check_eq(control_bps_oor(ADC_BPS_VREF_V), 1, "short to supply");
    check(BPS_FAULT_LOW < BPS_SETPOINT_V, "setpoint above low limit");
}

static void test_bps_psi(void)
{
    printf("test_bps_psi\n");

    check_near(control_bps_psi(BPS_0PSI_V), 0.0f, 0.01f, "zero at 0.5 V");
    check_near(control_bps_psi(BPS_FULL_V), BPS_FULL_PSI, 0.01f, "full scale at 4.5 V");
    check_near(control_bps_psi(2.5f), 1000.0f, 0.1f, "midscale");
    check_near(control_bps_psi(BPS_SETPOINT_V), 100.0f, 0.1f, "brake setpoint");
    check_near(control_bps_psi(0.3f), 0.0f, 0.01f, "below zero clamps");
    check_near(control_bps_psi(4.7f), BPS_FULL_PSI, 0.01f, "above full scale clamps");
}

/* ---- Fault debounce ---- */

static int steps_to_trip(control_t *c, uint32_t dt_ms)
{
    for (int i = 1; i <= 1000; i++) {
        if (control_fault_debounce(c, 1, dt_ms)) {
            return i;
        }
    }
    return -1;
}

static void test_debounce(void)
{
    printf("test_debounce\n");
    control_t c;

    control_init(&c);
    check_eq(control_fault_debounce(&c, 0, STEP_MS), 1, "starts tripped");
    check_eq(control_fault_debounce(&c, 0, STEP_MS), 0, "clears after a clean step");

    /* Onset step reads 0, so trips once more than 50 ms has built up */
    const int n = steps_to_trip(&c, STEP_MS);
    check_eq(n, 12, "trips on the 12th 5 ms step");
    check_eq((n - 1) * (int)STEP_MS, 55, "55 ms after the fault appears");
    check((n - 1) * (int)STEP_MS <= 100, "cuts within 100 ms (T.4.2.5)");

    check_eq(control_fault_debounce(&c, 0, STEP_MS), 1, "one step lag on clear");
    check_eq(control_fault_debounce(&c, 0, STEP_MS), 0, "then clear");

    /* Glitch shorter than the debounce never trips */
    int tripped = 0;
    for (int i = 0; i < 9; i++) {
        tripped |= control_fault_debounce(&c, 1, STEP_MS);
    }
    control_fault_debounce(&c, 0, STEP_MS);
    for (int i = 0; i < 9; i++) {
        tripped |= control_fault_debounce(&c, 1, STEP_MS);
    }
    check_eq(tripped, 0, "repeated short glitches reset, never trip");

    /* Slow loop still trips on time */
    control_fault_debounce(&c, 0, STEP_MS);
    check_eq(steps_to_trip(&c, 20u), 4, "trips on time with 20 ms steps");

    /* Long fault doesn't wrap */
    control_init(&c);
    control_fault_debounce(&c, 0, STEP_MS);
    for (int i = 0; i < 1000; i++) {
        control_fault_debounce(&c, 1, 10000000u);
    }
    check_eq(control_fault_debounce(&c, 1, STEP_MS), 1, "still tripped after hours");
}

/* ---- Torque ---- */

static void test_torque_map(void)
{
    printf("test_torque_map\n");
    check_near(control_torque_map(0.0f), 0.0f, 1e-6f, "0 at rest");
    check_near(control_torque_map(TMAP_DRIVE_START), 0.0f, 1e-6f, "0 at drive start");
    check_near(control_torque_map(TMAP_DEADBAND_HIGH), 1.0f, 1e-5f, "1 at top deadband");
    check_near(control_torque_map(1.0f), 1.0f, 1e-5f, "1 at full pedal");
    check_near(control_torque_map((TMAP_DRIVE_START + TMAP_DEADBAND_HIGH) / 2.0f), 0.5f, 1e-5f,
               "linear in between");
}

static void test_torque_limit(void)
{
    printf("test_torque_limit\n");
    check_eq(control_torque_limit(1.0f, 0, 400), TORQUE_LIMIT_NM_X10, "capped at max torque");
    check_eq(control_torque_limit(0.0f, 0, 400), 0, "no pedal no torque");
    check_eq(control_torque_limit(0.5f, 0, 400), TORQUE_LIMIT_NM_X10 / 2, "scales with pedal");

    /* 4200 * DCL / max(230.4, rpm * 0.1076) */
    check_eq(control_torque_limit(1.0f, 0, 100), 1822, "100 A at standstill");
    check_eq(control_torque_limit(1.0f, 4000, 165), 1610, "165 A at 4000 rpm");
    check_eq(control_torque_limit(1.0f, 5000, 60), 468, "60 A at 5000 rpm");
    check_eq(control_torque_limit(1.0f, 3000, 0), 0, "0 A gives 0 torque");

    check(control_torque_limit(1.0f, 4000, 100) < control_torque_limit(1.0f, 4000, 150),
          "less current, less torque");
    check(control_torque_limit(1.0f, 5000, 150) < control_torque_limit(1.0f, 3000, 150),
          "more speed, less torque");

    /* Known issue, fixed separately: the fixed 30 Nm ignores the DCL */
    check_eq(control_torque_limit(1.0f, 6000, 165), (long)TORQUE_HIGH_SPEED_NM_X10,
             "30 Nm above 6000 rpm");
    check_eq(control_torque_limit(1.0f, 6000, 0), (long)TORQUE_HIGH_SPEED_NM_X10,
             "above 6000 rpm ignores DCL (known issue)");
}

/* ---- CAN frames ---- */

static void test_pack_inverter(void)
{
    printf("test_pack_inverter\n");
    uint8_t d[8];

    control_pack_inverter(d, 1234, 0, 0, 5);
    check_eq(d[0] | d[1] << 8, 1234, "torque");
    check_eq(d[4], 0x01, "forward");
    check_eq(d[5] & 0x01, 1, "enabled");
    check_eq(d[5] & 0x02, 0x02, "bit 1 always set");
    check_eq(d[5] >> 4, 5, "heartbeat");
    check_eq(d[2] | d[3] | d[6] | d[7], 0, "unused bytes zero");

    control_pack_inverter(d, -437, 0, 0, 0);
    check_eq((int16_t)(d[0] | d[1] << 8), -437, "negative torque (regen)");

    control_pack_inverter(d, 1234, 0, 1, 0);
    check_eq(d[5] & 0x01, 0, "disable clears enable");

    control_pack_inverter(d, 1234, 1, 0, 0);
    check_eq(d[5] & 0x01, 0, "lockout holds enable low");

    control_pack_inverter(d, 0, 0, 0, CAN_HEARTBEAT_MASK);
    check_eq(d[5] >> 4, CAN_HEARTBEAT_MASK, "heartbeat top value");
}

static void test_pack_debug(void)
{
    printf("test_pack_debug\n");
    control_status_t s = {0};
    uint8_t d[8];

    s.rtd_timeout = 7;
    s.bps_adc = 0x0ABC;
    s.tps1_adc = 0x0123;
    s.tps2_adc = 0x0FFF;
    s.pedals.tps1 = 0.25f;
    s.pedals.tps2 = 0.999f;
    s.pedals.tps = 0.5f;
    control_pack_debug(d, &s);
    check_eq(d[0], 7, "rtd timeout");
    check_eq(d[1], 0xAB, "bps adc / 16");
    check_eq(d[2], 0x12, "tps1 adc / 16");
    check_eq(d[3], 0xFF, "tps2 adc / 16");
    check_eq(d[4], 0, "no flags");
    check_eq(d[5], 25, "tps1 %");
    check_eq(d[6], 99, "tps2 % truncates");
    check_eq(d[7], 50, "throttle %");

    const struct {
        int bit;
        const char *name;
    } bits[] = {{0, "disable"},  {1, "ready to drive"}, {2, "bse"},     {3, "tps1 oor"},
                {4, "tps2 oor"}, {5, "disagree"},       {6, "enabled"}, {7, "lockout"}};
    for (int i = 0; i < 8; i++) {
        control_status_t f = {0};
        f.disable = bits[i].bit == 0;
        f.ready_to_drive = bits[i].bit == 1;
        f.bse = bits[i].bit == 2;
        f.pedals.tps1_oor = bits[i].bit == 3;
        f.pedals.tps2_oor = bits[i].bit == 4;
        f.pedals.tps_disagree = bits[i].bit == 5;
        f.inverter_enabled = bits[i].bit == 6;
        f.inverter_lockout = bits[i].bit == 7;
        control_pack_debug(d, &f);
        check_eq(d[4], 1 << bits[i].bit, bits[i].name);
    }
}

static void test_pack_bps(void)
{
    printf("test_pack_bps\n");
    uint8_t d[CAN_BPS_DLC];

    control_pack_bps(d, 1234.56f, 0x0ABC, 0, 0);
    check_eq(d[0] | d[1] << 8, 12346, "psi x10 little endian, rounds");
    check_eq(d[2] | d[3] << 8, 0x0ABC, "full 12 bit adc");
    check_eq(d[4], 0, "no flags");

    control_pack_bps(d, BPS_FULL_PSI, 0x0FFF, 0, 0);
    check_eq(d[0] | d[1] << 8, 20000, "full scale fits 16 bits");
    check_eq(d[2] | d[3] << 8, 0x0FFF, "max adc");

    control_pack_bps(d, 0.0f, 0, 1, 0);
    check_eq(d[0] | d[1] << 8, 0, "zero psi");
    check_eq(d[4], 0x01, "brake pressed bit");
    control_pack_bps(d, 0.0f, 0, 0, 1);
    check_eq(d[4], 0x02, "oor bit");
}

static void test_pack_pedals(void)
{
    printf("test_pack_pedals\n");
    control_pedals_t p = {0};
    uint8_t d[CAN_PEDALS_DLC];

    p.tps1 = 0.2468f;
    p.tps2 = 0.25f;
    control_pack_pedals(d, &p, 0x0ABC, 0x0FFF);
    check_eq(d[0] | d[1] << 8, 0x0ABC, "tps1 full 12 bit adc");
    check_eq(d[2] | d[3] << 8, 0x0FFF, "tps2 full 12 bit adc");
    check_eq(d[4] | d[5] << 8, 2468, "tps1 0.01%");
    check_eq(d[6] | d[7] << 8, 2500, "tps2 0.01%");

    /* Finer than 0x555's whole percent, either side of the BSE trip */
    p.tps1 = BSE_TRIP_TPS - 0.0004f;
    p.tps2 = BSE_TRIP_TPS + 0.0004f;
    control_pack_pedals(d, &p, 0, 0);
    check_eq(d[4] | d[5] << 8, 2496, "just under trip");
    check_eq(d[6] | d[7] << 8, 2504, "just over trip");

    p.tps1 = 1.0f;
    p.tps2 = 0.0f;
    control_pack_pedals(d, &p, 0, 0);
    check_eq(d[4] | d[5] << 8, 10000, "full pedal");
    check_eq(d[6] | d[7] << 8, 0, "no pedal");

    p.tps1 = 1.5f;
    p.tps2 = -0.5f;
    control_pack_pedals(d, &p, 0, 0);
    check_eq(d[4] | d[5] << 8, 10000, "clamped high");
    check_eq(d[6] | d[7] << 8, 0, "clamped low");
}

int main(void)
{
    test_matches_old();
    test_pedals();
    test_bse();
    test_bps_oor();
    test_bps_psi();
    test_debounce();
    test_torque_map();
    test_torque_limit();
    test_pack_inverter();
    test_pack_debug();
    test_pack_bps();
    test_pack_pedals();

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
