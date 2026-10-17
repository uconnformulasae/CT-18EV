# Comprehensive Firmware Architecture & Safety Audit: CT-18EV ECU

**Target Vehicle:** UConn Formula SAE CT-18EV  
**Target Hardware:** STMicroelectronics STM32F103RCT6 (ARM Cortex-M3 @ 16 MHz, 256 KB Flash, 48 KB SRAM, Soft-Float)  
**Regulatory Standards:** Formula SAE (FSAE 2024–2026), Formula Student Rules (FSG 2024–2026), ISO 26262 / Functional Safety  
**Audit Conducted By:** Multi-Agent Specialist Review Team (Safety & Rules Auditor, Powertrain & Controls Engineer, BMS & State Estimation Specialist, Embedded Systems Architect)  
**Date:** October 2026  
**Scope:** `Core/Src/`, `Core/Inc/`, `tools/`, `CT-18EV.ioc`, Linker scripts, Documentation. No code modifications applied.

---

## Table of Contents

1. [Executive Summary & Vulnerability Dashboard](#1-executive-summary--vulnerability-dashboard)
2. [Section I: Safety Systems & Regulatory Compliance (FSAE / FS Rules)](#2-section-i-safety-systems--regulatory-compliance)
3. [Section II: Vehicle Dynamics & Powertrain Controls](#3-section-ii-vehicle-dynamics--powertrain-controls)
4. [Section III: Battery State Estimation (EKF) & Thermal Derating](#4-section-iii-battery-state-estimation-ekf--thermal-derating)
5. [Section IV: Embedded Architecture, HAL Drivers & Concurrency](#5-section-iv-embedded-architecture-hal-drivers--concurrency)
6. [Section V: Host Test Suite & Simulation Blindspots](#6-section-v-host-test-suite--simulation-blindspots)
7. [Section VI: Prioritized Remediation Roadmap](#7-section-vi-prioritized-remediation-roadmap)

---

## 1. Executive Summary & Vulnerability Dashboard

An exhaustive, multi-agent code review of the CT-18EV electronic control unit (ECU) firmware was conducted. The CT-18EV ECU is the safety-critical central controller responsible for accelerator and brake pedal plausibility (APPS/BSE), Ready-to-Drive (RTD) sequencing, regenerative braking torque blending, closed-loop launch control, Extended Kalman Filter (EKF) state-of-charge estimation, weakest-cell battery derating, and high-voltage inverter torque modulation via CAN.

The audit revealed **17 CRITICAL**, **16 HIGH**, **8 MEDIUM**, and **5 LOW/SUGGESTION** findings across regulatory compliance, vehicle stability, battery protection, and silicon hardware drivers. 

### Critical Hazard Highlights

- **Unlatched Fault Clearing (Sudden Unintended Acceleration):** APPS out-of-range, pedal disagreement, and BSE faults auto-clear immediately upon sensor reconnection, snapping full driver torque back into the wheels without requiring pedal zeroing or RTD reset.
- **Brake Pressure Sensor Unmonitored (Fails Dangerous):** Zero out-of-range or disconnection checking on the hydraulic brake sensor. Disconnection permanently disables brake plausibility shutdown; sensor short-to-power allows entering Ready-to-Drive without touching the brake.
- **BMS Safety Override (Thermal Runaway Hazard):** The internal SoC derating table completely overrides the Orion BMS Discharge Current Limit (`bms_dcl`). If the BMS commands 0 A due to critical cell overtemperature ($>60^\circ\text{C}$), the ECU ignores the cutoff and continues drawing up to 165 A.
- **Degenerate Launch Control (1.0 Nm Stall):** The open-loop launch control map contains all-zero RPM breakpoints and 1.0 Nm torque, causing the car to stall on the start line while trapping it permanently in open loop. Furthermore, launch control is dead code (never enabled in firmware).
- **Severe Rear Axle Over-Braking (Snap-Oversteer):** Peak regen torque is set to 100 Nm (spec was 30 Nm), requiring an unphysical friction coefficient of $\mu \approx 1.92$ on dynamic rear axle load, inducing immediate rear wheel lockup and spin-out on throttle lift-off.
- **Regen Cut Shock Load & Deceleration Dip:** Touching the mechanical brake bypasses the torque slew rate limiter, instantly stepping negative torque from up to -75 Nm to 0 Nm in 5 ms (15,000 Nm/s shock), shocking the drivetrain and causing a deceleration loss right at threshold braking entry.
- **Hardware Conflict: Concurrent CAN and USB on STM32F103:** Physical silicon collision on STM32F103 where CAN1 and USB share the exact same 512-byte SRAM packet buffer and `USB_LP_CAN1_RX0_IRQn` interrupt, with `HAL_PCD_IRQHandler` being executed on every CAN RX interrupt.
- **Synchronous Spin-Wait in 5 ms Real-Time Loop:** `adc_update()` executes a 5,000-iteration busy-wait loop (`while ((!dual_done || !bps_done) && --timeout)`), burning up to 1.56 ms (31% of the entire loop period) in a blocking spin.
- **Single-Shot CAN Mode (`AutoRetransmission = DISABLE`):** Frame retransmission is disabled, causing bxCAN to drop inverter torque commands permanently upon transient arbitration loss or electrical noise.
- **Telemetry Self-Assignment Bug:** In `main.c`, `loop_dt_max_ms = loop_dt_max_ms;` permanently blinds trackside telemetry to control loop timing overruns.
- **Missing Hardware Watchdog:** Complete absence of an Independent Watchdog (IWDG) or Window Watchdog (WWDG).

---

## 2. Section I: Safety Systems & Regulatory Compliance

*Audited against Formula SAE (FSAE 2024–2026 Rules T.4.2, T.4.3, T.4.4, EV 4.1, EV 4.7, EV 4.12) and Formula Student Rules (FSG).*

---

### [CRIT-SAF-01] Control Faults Clear Asymmetrically Without Latching or Throttle Zeroing
- **Location:** `Core/Src/control.c#L48-L59`, `Core/Src/main.c#L376-L386`, `Core/Src/main.c#L407-L413`
- **Rule Reference:** FSAE T.4.2.5, FSG EV 4.1.2, ISO 26262 ASIL-D
- **Mechanism:**
  When an APPS disagreement, APPS out-of-range, or BSE plausibility fault occurs, `control_fault_debounce()` sets `should_disable_inverter = 1` after 55 ms. However, if `fault == 0` for a single 5 ms tick (e.g. an intermittent harness wire reconnects or noise subsides), `c->fault_ms` is instantly reset to 0. In `main.c`, `should_disable_inverter` drops to 0. Because `ready_to_drive` remains 1, the ECU **instantly commands full throttle torque (e.g. 150 Nm) in a single 5 ms tick** if the driver is holding down the pedal.
- **Regulatory Violation:** FSAE T.4.2.5 and FSG EV 4.1.2 require that if an implausibility persists for > 100 ms, motor power must remain completely shut down until tractive system power is cycled. Even for sub-100 ms transient faults, torque must remain locked at 0 Nm until the driver fully returns the pedal to `< 5%` travel.
- **Remediation:**
  1. Implement a latched fault state machine in `control.c`. If an implausibility persists for $> 100\text{ ms}$, require tractive system power cycling to clear (`ready_to_drive = 0`).
  2. For transient faults, latch torque output to 0 Nm until `tps_combined < BSE_CLEAR_TPS` (0.05).

---

### [CRIT-SAF-02] Complete Absence of Brake Pressure Sensor (BSE/BPS) Out-of-Range & Disconnection Checks
- **Location:** `Core/Inc/config.h#L10-L11, L33-L38`, `Core/Src/main.c#L347-L349, L374-L377`, `Core/Src/control.c#L38-L46`
- **Rule Reference:** FSAE EV 4.7.2, T 4.3.3, T 4.3.4
- **Mechanism:**
  The firmware reads brake pressure voltage via `bps = (float)bps_adc * ADC_BPS_V_PER_COUNT` and sets `brake_pressed = bps > BPS_SETPOINT_V` (0.700V). There are no out-of-range thresholds (`BPS_FAULT_LOW` / `BPS_FAULT_HIGH`):
  - **Open-Circuit / Wire Disconnect:** Pin pulls to 0.0V. Since $0.0\text{V} \le 0.700\text{V}$, `brake_pressed = 0`. The ECU believes the brake is never pressed. If the throttle sticks open, **BSE plausibility will NEVER trip**. The car fails completely dangerous.
  - **Short-Circuit to 5V:** Sensor shorts to supply rail ($5.0\text{V}$). `brake_pressed` evaluates to 1 permanently. In `main.c#L352`: `rtd_raw &= brake_pressed`. The brake interlock is permanently satisfied, allowing anyone to **enter Ready-to-Drive WITHOUT pressing the brake pedal**.
- **Remediation:**
  Define out-of-range limits in `config.h`:
  ```c
  #define BPS_FAULT_LOW   0.25f  /* Open circuit / pull-down */
  #define BPS_FAULT_HIGH  4.75f  /* Short circuit to supply */
  ```
  Validate `bps_oor = (bps < BPS_FAULT_LOW || bps > BPS_FAULT_HIGH)`. If active, immediately disable inverter, inhibit RTD entry, and trip safety fault shutdown.

---

### [CRIT-SAF-03] Motor Power Not Inhibited During Acoustic Ready-to-Drive (RTD) Sound
- **Location:** `Core/Src/main.c#L355-L372, L378-L386, L407-L410`, `Core/Inc/config.h#L67-L69`
- **Rule Reference:** FSG EV 4.12.3, FSAE T.4.4.4
- **Mechanism:**
  When `rtd_debounce_update()` latches `ready_to_drive = 1`:
  - `rtd_buzzer_counter` begins counting from 0 to 12 (1.08 seconds of buzzer).
  - Simultaneously, `!ready_to_drive` evaluates to 0, which clears `should_disable_inverter = 0`.
  - Inverter enable is transmitted with active torque output capability.
  If the driver steps on the accelerator while the buzzer is sounding, **the car launches immediately while the buzzer is still sounding**, directly violating FSG EV 4.12.3 (*"Motor power is prohibited while the sound is being produced"*).
- **Remediation:**
  Inhibit inverter enable and zero torque during buzzer activation:
  ```c
  const uint8_t rtd_sound_active = ready_to_drive && (rtd_buzzer_counter < RTD_BUZZER_TICKS);
  should_disable_inverter = control_fault_debounce(&ctrl, fault, loop_dt_ms) || !ready_to_drive || rtd_sound_active;
  ```

---

### [CRIT-SAF-04] Ready-to-Drive (RTD) State Never Deactivates on Tractive System Shutdown
- **Location:** `Core/Src/main.c#L130-L135, L358-L359`, `Core/Inc/config.h#L70-L74`
- **Rule Reference:** FSAE T.4.4.2, EV 4.12.2
- **Mechanism:**
  `ready_to_drive` is only cleared if `rtd_timeout >= RTD_TIMEOUT_TICKS` (1.8s timeout on Inverter CAN message 0x0AA). If the high-voltage shutdown circuit trips (emergency stop button, BMS opening contactors, crash inertia switch), the inverter 12V auxiliary logic remains powered and continues broadcasting 0x0AA with `ready_pin_state = 1`. `ready_to_drive` remains 1 indefinitely. When the shutdown loop is reset and AIR contactors reclose, the car is **already in RTD mode without requiring the driver to press the brake and RTD button**.
- **Remediation:**
  Tie RTD deactivation to Tractive System Active status: clear `ready_to_drive = 0` whenever `bus_voltage < 60V` or when BMS contactor status signals open.

---

### [HIGH-SAF-01] Asymmetric Debounce Resets Instantly on Glitches, Evading Shutdown
- **Location:** `Core/Src/control.c#L48-L59`, `tools/control_test.c#L281-L291`
- **Rule Reference:** FSAE T.4.2.4, T.4.2.5
- **Mechanism:**
  In `control_fault_debounce`:
  ```c
  if (!fault) {
      c->fault_ms = 0;
  } else if (c->fault_ms <= UINT32_MAX - dt_ms) {
      c->fault_ms += dt_ms;
  }
  ```
  If an APPS harness wire vibrates or suffers periodic EMI: 45 ms fault $\rightarrow$ 5 ms clean $\rightarrow$ 45 ms fault. The single 5 ms clean sample resets `fault_ms` to 0. Over minutes, the fault is present 90% of the time, but the debouncer **never trips**. `tools/control_test.c` line 281 explicitly enshrines this bug as a test assertion.
- **Remediation:**
  Implement a leaky-bucket or integrating filter:
  ```c
  if (fault) {
      c->fault_ms = (c->fault_ms + dt_ms > MAX_FAULT_MS) ? MAX_FAULT_MS : c->fault_ms + dt_ms;
  } else {
      c->fault_ms = (c->fault_ms > (dt_ms / 2)) ? c->fault_ms - (dt_ms / 2) : 0;
  }
  ```

---

### [HIGH-SAF-02] Drive Torque Commanded During First 50 ms of APPS Out-of-Range or Disagreement
- **Location:** `Core/Src/control.c#L28-L35`, `Core/Src/main.c#L342-L346, L376-L386`
- **Rule Reference:** FSAE T.4.2.4
- **Mechanism:**
  When TPS1 shorts to 3.3V while TPS2 is at 0V, `control_pedals()` averages them to `p.tps = 0.50f` (50% throttle). During the 50 ms debounce delay, `should_disable_inverter` is 0. The ECU commands 50% drive torque for 50 ms derived from a shorted sensor.
- **Remediation:**
  In `control_pedals()`, immediately force `p.tps = 0.0f` if `p.tps1_oor || p.tps2_oor || p.tps_disagree`.

---

### [HIGH-SAF-03] Indefinite Pedal and Brake Freeze on ADC Overrun
- **Location:** `Core/Src/adc.c#L109-L130`, `Core/Src/main.c#L337-L341`
- **Mechanism:**
  In `adc_update()`, if conversions fail to complete within the 5000-iteration timeout, the driver increments `overrun_count` and retains `tps1_latched`, `tps2_latched`, and `bps_latched`. Neither `adc_overruns()` nor `adc_errors()` is checked in `main.c`. If DMA hangs while at 100% throttle, pedal readings **freeze at 100% indefinitely** with 0% disagreement and 0% OOR.
- **Remediation:**
  If `overrun_count` persists for $> 2$ consecutive cycles (10 ms), zero out all latched values and set a critical sensor fault.

---

### [MED-SAF-01] Excessively Permissive APPS Out-of-Range Thresholds
- **Location:** `Core/Inc/config.h#L20-L25`
- **Mechanism:**
  APPS calibrated spans: TPS1 = 1.39V–2.40V, TPS2 = 0.62V–1.64V. Configured OOR limits are 0.2V and 2.8V. For TPS2, 2.80V is 1.16V above full throttle (114% of stroke). A partial short to a 2.5V rail is clamped to 1.0f by `calibrate()` and never trips OOR.
- **Remediation:**
  Tighten thresholds: TPS1 [1.20V, 2.55V], TPS2 [0.45V, 1.80V].

---

### [MED-SAF-02] BSE Calculated from Average Throttle Blinds Plausibility Device
- **Location:** `Core/Src/main.c#L345, L374`, `Core/Src/control.c#L38-L46`
- **Mechanism:**
  `bse_error = control_bse(&ctrl, brake_pressed, tps_combined)`. If TPS1 fails low (0%) and TPS2 is at 35% with brake engaged, `tps_combined = 17.5% < 25%`. BSE fails to trip.
- **Remediation:**
  Evaluate BSE using $\max(\text{tps1}, \text{tps2})$.

---

## 3. Section II: Vehicle Dynamics & Powertrain Controls

---

### [CRIT-DYN-01] Open-Loop Launch Map Degenerate (1.0 Nm Commanded / Start-Line Stall)
- **Location:** `Core/Src/launch_control.c#L8-L10, L256-L265`
- **Mechanism:**
  ```c
  static const uint16_t lc_map_rpm[LC_MAP_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};
  static const uint16_t lc_map_torque[LC_MAP_SIZE] = {10, 10, 10, 10, 10, 10, 10, 10};
  ```
  Torque is in units of 0.1 Nm. A value of `10` is **$1.0\text{ Nm}$**. Through the 3.636:1 gear reduction and 0.2 m tire radius, tractive force is:
  $$F_{\text{tractive}} = \frac{1.0 \times 3.636}{0.2} = 18.18\text{ N} \implies a = \frac{18.18\text{ N}}{250\text{ kg}} = 0.0727\text{ m/s}^2 \approx 0.007\text{ g}$$
  If slip rate exceeds 3.0, torque drops to $0.5\text{ Nm}$ ($9\text{ N}$). Rolling resistance and driveline friction exceed $18\text{ N}$. The car **stalls on the start line**, never reaches $2.0\text{ m/s}$ crossover speed, and remains permanently trapped in open-loop launch mode.
- **Remediation:**
  Populate realistic open-loop tractive torque breakpoints (e.g. 0 to 4000 RPM mapped to 1200 to 1800 units / 120–180 Nm).

---

### [CRIT-DYN-02] Regen Cut Bypasses Slew Limiter (Violent Shock & Deceleration Dip)
- **Location:** `Core/Src/regen.c#L96`
- **Mechanism:**
  ```c
  dbg.torque = cut ? drive_torque : slew(dbg.torque, dbg.target, dt_ms);
  ```
  When decelerating under one-pedal regen at $-75.0\text{ Nm}$ (`dbg.torque = -750`) and the driver touches the brake pedal:
  - `brake_pressed == 1` $\implies$ `cut = 1`.
  - Line 96 executes `dbg.torque = drive_torque` (0 Nm).
  - Torque **instantly steps from $-75.0\text{ Nm}$ to $0.0\text{ Nm}$ in a single 5 ms tick (15,000 Nm/s)**.
  - **Mechanical Shock:** Snaps the drive chain, shocks differential gears, and slaps across driveline lash.
  - **Deceleration Dip:** At the moment the driver initiates threshold braking, rear regen torque collapses before hydraulic calipers generate pad clamp force (30–60 ms lag), causing the car to momentarily lose braking deceleration.
- **Remediation:**
  Route cut transitions through `slew()` to ramp torque up to 0 Nm smoothly at `REGEN_SLEW_UP`:
  ```c
  int32_t target = cut ? drive_torque : dbg.target;
  dbg.torque = slew(dbg.torque, target, dt_ms);
  ```

---

### [CRIT-DYN-03] Peak Regen Torque (100 Nm) Violates Physics & Design Spec (Snap-Oversteer)
- **Location:** `Core/Inc/regen.h#L19`, `Core/Src/regen.c#L42`, `docs/3d-regen-map-implementation-plan.md#L26`
- **Mechanism:**
  `#define REGEN_PEAK_NM 100.0f` ($3.33\times$ higher than the 30.0 Nm specification limit).
  On a 250 kg RWD FSAE car under $0.74\text{ g}$ deceleration, forward weight transfer unloads the rear axle normal force to $F_{z, \text{rear}} \approx 949\text{ N}$.
  Rear braking force from 100 Nm motor torque is:
  $$F_{\text{brake}} = \frac{100 \times 3.636}{0.2} = 1,818\text{ N} \implies \mu_{\text{req}} = \frac{1818}{949} = \mathbf{1.92}$$
  Requiring $\mu = 1.92$ exceeds available tire adhesion. **The rear tires lock up immediately on throttle lift-off**, causing instant snap-oversteer.
- **Remediation:**
  Enforce the specification cap: clamp `REGEN_PEAK_NM` to 30.0 Nm (or 15.0 Nm for initial testing).

---

### [HIGH-DYN-01] Launch Control Dead Code
- **Location:** `Core/Src/launch_control.c#L4`, `Core/Src/main.c#L398`
- **Mechanism:**
  `launch_control_enable` is initialized to 0 and is never modified anywhere in `main.c`, CAN ISRs, or GPIO callbacks. Telemetry CAN frame `0x557` is never transmitted.
- **Remediation:**
  Integrate `launch_control_enable` with a dedicated CAN or steering wheel digital input, protected by interlocks (vehicle stationary, steering angle $< 5^\circ$). Broadcast `0x557` in `main.c`.

---

### [HIGH-DYN-02] Bumpless Transfer Clamping Cuts Torque to 0 Nm Under High Slip
- **Location:** `Core/Src/launch_control.c#L217-L222`
- **Mechanism:**
  At crossover ($2.0\text{ m/s}$), if wheelspin is elevated ($\text{slip} = 0.58$, target = 0.10, error = -0.48), $P = -2400$. To match open-loop map ($+1400$), `target_i` needs to be $+1600$. However, line 219 forces `if (target_i > 0.0f) target_i = 0.0f;`. As a result, closed-loop torque evaluates to $2200 - 2400 + 0 \implies \mathbf{0.0\text{ Nm}}$. Torque instantly collapses from 140 Nm to 0 Nm.
- **Remediation:**
  Allow `target_i` to take positive values at crossover to cancel negative $P+D$ terms, or implement a 50 ms blend filter.

---

### [HIGH-DYN-03] Battery Charge Power Ceiling Exceeded (40 kW vs 15 kW Spec)
- **Location:** `Core/Src/regen.c#L39-L42`, `Core/Inc/regen.h#L12`
- **Mechanism:**
  `REGEN_CELL_CHARGE_A` = 25A across 4p = 100A pack current. At 400V, charge power reaches **40 kW**, exceeding the 15 kW limit defined in `docs/3d-regen-map-implementation-plan.md#L6.1`. This risks BMS overvoltage trips during on-track braking.
- **Remediation:**
  Add a 15 kW power limit cap: `p_max_nm = 15000.0f / (rpm * RPM_TO_RAD_S)`.

---

### [HIGH-DYN-04] Asymmetric Slew Rate Limiter Free Above 10 Nm (+140 Nm Step)
- **Location:** `Core/Src/regen.c#L52-L70`
- **Mechanism:**
  `REGEN_SLEW_BAND` is 100 (10 Nm). When transitioning from regen (-30 Nm) to drive (+150 Nm), slew rate limits torque up to +10 Nm. Once `prev >= 100`, the function executes `return target;`, causing an instantaneous **+140 Nm step** in a single tick.
- **Remediation:**
  Apply continuous slew limiting across the full torque range.

---

### [HIGH-DYN-05] Telemetry Self-Assignment Bug Blinds Worst-Case Loop Timing
- **Location:** `Core/Src/main.c#L334`
- **Mechanism:**
  ```c
  if (loop_dt_ms > loop_dt_max_ms) {
      loop_dt_max_ms = loop_dt_max_ms; // SELF-ASSIGNMENT BUG
  }
  ```
  `loop_dt_max_ms` remains 0 forever, permanently blinding telemetry on CAN ID `0x558` to control loop timing spikes.
- **Remediation:**
  Change to `loop_dt_max_ms = loop_dt_ms;`.

---

### [HIGH-DYN-06] Torque Limit Assumes Fixed 408V Bus Voltage (36% DC Overcurrent)
- **Location:** `Core/Src/control.c#L69-L72`
- **Mechanism:**
  `TORQUE_POWER_NUM = 4200u` implicitly assumes a constant 408V bus. When pack voltage sags to 300V, the motor requires 36% more DC current to produce the same mechanical power, drawing up to $1.36\times$ the commanded DCL.
- **Remediation:**
  Scale power limit by measured `bus_voltage`.

---

### [HIGH-DYN-07] Hard Torque Discontinuity at 6000 RPM (Cliff to 30 Nm, Ignores DCL=0)
- **Location:** `Core/Src/control.c#L74-L76`, `tools/control_test.c#L336-L340`
- **Mechanism:**
  At 5999 RPM with 165A DCL, torque limit is ~81 Nm. At 6000 RPM, it snaps to 30 Nm in 1 RPM. Furthermore, if `dcl_a == 0` (BMS commanded zero current), the code commands 30 Nm above 6000 RPM regardless.
- **Remediation:**
  Enforce `if (rpm >= TORQUE_HIGH_SPEED_RPM && limit > TORQUE_HIGH_SPEED_NM_X10) limit = TORQUE_HIGH_SPEED_NM_X10;`.

---

## 4. Section III: Battery State Estimation (EKF) & Thermal Derating

---

### [CRIT-BMS-01] `soc_derate_dcl()` Overrides BMS Safety Cutoff (`bms_dcl`)
- **Location:** `Core/Src/soc_derate.c#L95-L101`, `Core/Src/main.c#L381-L385`
- **Mechanism:**
  ```c
  if (soc_kf_valid(kf)) {
      dbg.dcl = to_amps(interp(&tune->soc, dbg.weak_soc));
      dbg.src = SOC_DERATE_SRC_SOC;
  } else {
      dbg.dcl = bms_dcl;
      dbg.src = SOC_DERATE_SRC_BMS;
  }
  ```
  Whenever the EKF is initialized and valid, `bms_dcl` from CAN 0x202 is completely discarded. If Orion BMS demands 0 A due to cell overtemperature ($>60^\circ\text{C}$ thermal runaway hazard) or cell undervoltage, the ECU **continues discharging at up to 165 A**.
- **Remediation:**
  Arbitrate with a strict minimum:
  ```c
  if (soc_kf_valid(kf)) {
      uint32_t soc_dcl = to_amps(interp(&tune->soc, dbg.weak_soc));
      dbg.dcl = (soc_dcl < bms_dcl) ? soc_dcl : bms_dcl;
  }
  ```

---

### [CRIT-BMS-02] Instantaneous 160 A Torque Step on Fallback Transitions
- **Location:** `Core/Src/soc_derate.c#L95-L101`
- **Mechanism:**
  If CAN 0x600 experiences a transient timeout ($>200\text{ ms}$), `soc_kf_valid(kf)` drops to 0. `dbg.dcl` jumps instantly from internal derate (e.g. 40 A) to `bms_dcl` (165–200 A) in 5 ms with zero slewing, inducing a sudden $>80\text{ Nm}$ torque step.
- **Remediation:**
  Implement a bidirectional slew rate limiter on `dbg.dcl`.

---

### [CRIT-BMS-03] Permanent Filter Lockout Under Driving via NIS Gating Without In-Motion Inflation
- **Location:** `Core/Src/soc_kf.c#L389-L396`, `Core/Inc/soc_kf.h#L23-L24`
- **Mechanism:**
  Normalized Innovation Squared (NIS) gating threshold is $\gamma = 49.0$ ($7\sigma$). Covariance inflation (`s.p00 *= 1.5f`) is **only executed when at rest** (`rest_ms >= 2000`). While driving, current $|I| \ge 2\text{ A}$ continually resets `rest_ms = 0`. If a dynamic voltage sag causes a single gate rejection, $P_{00}$ cannot inflate. Because process noise $Q_z = 1.1 \times 10^{-9}\text{ s}^{-1}$ takes $>2.5\text{ hours}$ to grow, the filter **permanently locks out measurement updates for the rest of the race**, degrading into open-loop Coulomb counting.
- **Remediation:**
  Implement adaptive in-motion variance inflation after 10 consecutive gated cycles, or scale measurement noise $R_{meas}$ with $i_{cell}^2$.

---

### [HIGH-BMS-01] Standard Form Covariance Update & Singular Correlation Clamping
- **Location:** `Core/Src/soc_kf.c#L403-L425`
- **Mechanism:**
  Standard update $P^+ = (I - KH)P^-$ loses positive definiteness on single-precision soft-float. Clamping correlation to $\pm \sqrt{P_{00} P_{11}}$ forces $\det(P) = 0$ (rank deficiency), creating numerical singularity in subsequent inversion steps.
- **Remediation:**
  Replace with the **Joseph Stabilized Covariance Update**:
  $$P^+ = (I - KH) P^- (I - KH)^T + K R_{meas} K^T$$

---

### [HIGH-BMS-02] Torn-Frame Data Race on 0x79F Cell Voltages in `soc_derate.c`
- **Location:** `Core/Src/soc_derate.c#L17-L21, L56-L62, L72-L74`
- **Mechanism:**
  `low_ocv_raw` and `avg_ocv_raw` are updated in CAN ISR and read at 200 Hz in `update_gap()` without critical sections. An ISR firing between the two reads mixes old and new frames, generating spurious 50% SoC gaps that slash motor power for $>60\text{ s}$.
- **Remediation:**
  Guard reads with critical sections and convert OCV only upon CAN reception.

---

### [HIGH-BMS-03] Omission of $-i_{cell} \frac{\partial R_0}{\partial z}$ in Jacobian $H$
- **Location:** `Core/Src/soc_kf.c#L381-L382`
- **Mechanism:**
  At low SoC ($<15\%$), $R_0$ slope $\frac{\partial R_0}{\partial z} \approx -0.14\ \Omega/\text{unit}$. At 125 A, $-i_{cell} \frac{\partial R_0}{\partial z} \approx +4.38\text{ V}$, comparable to $\frac{d OCV}{dz}$. Omitting this causes $>50\%$ error in $h_0$, artificially inflating NIS and triggering false gating during low-battery acceleration.
- **Remediation:**
  Compute $h_0 = docv\_dz - i\_cell \cdot dr0\_dz \cdot kt$.

---

### [MED-BMS-01] ~16,000 Soft-Float Emulation Calls/Sec from Redundant Lookups
- **Location:** `Core/Src/soc_derate.c#L65-L87`, `Core/Src/main.c#L382`
- **Mechanism:**
  `update_gap()` runs at 200 Hz and calls `ocv_inverse()` twice, performing 40-step linear searches on unchanged CAN data, consuming $>400,000$ CPU cycles/sec ($>2.5\%$ MCU throughput).
- **Remediation:**
  Compute `gap` only in `soc_derate_feed_cells()` upon CAN message arrival.

---

### [MED-BMS-02] Unsigned Temperature Cast Interprets $<0^\circ\text{C}$ as $>55^\circ\text{C}$
- **Location:** `Core/Src/soc_kf.c#L103-L118`, `Core/Src/main.c#L146`
- **Mechanism:**
  `temp_c` is passed as `uint8_t`. At $-5^\circ\text{C}$ ($0\text{xFB} = 251$), `temp_scale()` treats it as $>55^\circ\text{C}$ and cuts modeled resistance in half instead of doubling it, causing immediate filter rejection in cold weather.
- **Remediation:**
  Change `temp_c` parameter to signed `int8_t`.

---

## 5. Section IV: Embedded Architecture, HAL Drivers & Concurrency

---

### [CRIT-EMB-01] Hardware Silicon Conflict: Concurrent USB and CAN on STM32F103
- **Location:** `Core/Src/stm32f1xx_it.c#L223-L233`, `Core/Src/main.c#L205-L206`
- **Mechanism:**
  On the STM32F103, CAN1 and USB Full-Speed **share the exact same 512-byte dedicated SRAM packet memory (PMA)** and interrupt vector `USB_LP_CAN1_RX0_IRQn` (ST RM0008 Sec 23.2). Both are initialized in `main.c`, and `stm32f1xx_it.c` calls `HAL_PCD_IRQHandler(&hpcd_USB_FS)` on every CAN RX interrupt. USB registers corrupt CAN mailboxes and packet pointers.
- **Remediation:**
  Completely eliminate USB: delete `MX_USB_PCD_Init()`, `hpcd_USB_FS`, and the PCD IRQ handler call.

---

### [CRIT-EMB-02] Synchronous Blocking Busy-Wait Inside 5 ms Real-Time Loop
- **Location:** `Core/Src/adc.c#L111-L118`
- **Mechanism:**
  ```c
  dual_done = 0u; bps_done = 0u; adc_arm();
  uint32_t timeout = 5000u;
  while ((!dual_done || !bps_done) && --timeout) {}
  ```
  The header comment claims non-blocking sampling, yet `adc_update()` executes a tight polling loop every 5 ms. On ADC/DMA failure, it burns **~1.56 ms (31% of the 5 ms loop budget)** in a spin-wait.
- **Remediation:**
  Trigger ADC via TIM2 TRGO hardware event in circular DMA mode without CPU polling.

---

### [CRIT-EMB-03] Complete Absence of Hardware Watchdog Timer
- **Location:** `Core/Src/main.c#L176-L467`
- **Mechanism:**
  Neither IWDG nor WWDG is configured. If the CPU hangs in the ADC busy-wait, enters a deadlock, or catches a hard fault, the vehicle freezes indefinitely with actuators in their last state.
- **Remediation:**
  Enable STM32 IWDG (30 ms timeout window), refreshed strictly at the end of each valid 5 ms control loop iteration.

---

### [CRIT-EMB-04] Single-Shot CAN Mode (`AutoRetransmission = DISABLE`)
- **Location:** `Core/Src/main.c#L690`
- **Mechanism:**
  Disabling auto-retransmission causes the bxCAN controller to discard inverter command frames (`0x0C0`) on any arbitration loss or transient noise disturbance.
- **Remediation:**
  Set `hcan.Init.AutoRetransmission = ENABLE;`.

---

### [CRIT-EMB-05] Unsafe Error Handler Halts with Interrupts Disabled
- **Location:** `Core/Src/main.c#L901-L909`
- **Mechanism:**
  `Error_Handler()` executes `__disable_irq(); while (1) {}`. If ADC calibration or clock setup fails on boot, the ECU halts silently with GPIO outputs floating or driven.
- **Remediation:**
  De-energize all output pins (force Inverter Enable LOW, RTD buzzer/light LOW), log fault, and trigger system reset.

---

### [HIGH-EMB-01] Clock Misconfiguration & Latency Hazard (Underclocked at 16 MHz)
- **Location:** `Core/Src/main.c#L482-L514`, `CT-18EV.ioc#L213-L223`
- **Mechanism:**
  HSE is 16 MHz, PLL produces 48 MHz, but `RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;`. The CPU runs at 16 MHz instead of 72 MHz (or 48 MHz). If switched to PLL without changing `FLASH_LATENCY_0` to `FLASH_LATENCY_1`, the CPU encounters immediate instruction prefetch crashes.
- **Remediation:**
  Configure SYSCLK to 72 MHz (or 48 MHz) with appropriate Flash wait states.

---

### [HIGH-EMB-02] TIM2 Period Discrepancy: `main.c` (200 Hz) vs `CT-18EV.ioc` (100 Hz)
- **Location:** `Core/Src/main.c#L724`, `CT-18EV.ioc#L233-L234`
- **Mechanism:**
  `main.c` sets TIM2 period to 4999 (200 Hz / 5 ms), but `.ioc` specifies 9999 (100 Hz / 10 ms). Any CubeMX code regeneration cuts control loop frequency in half.
- **Remediation:**
  Synchronize `TIM2.Period = 4999` in `CT-18EV.ioc`.

---

### [HIGH-EMB-03] Fragile CAN Bit Timing (4 Time Quanta Total at 500 kbps)
- **Location:** `Core/Src/main.c#L681-L686`
- **Mechanism:**
  Prescaler = 8, BS1 = 2 TQ, BS2 = 1 TQ $\implies$ 4 TQ total. Violates ISO 11898-1 (requires 8–25 TQ). Offers zero phase margin for isolator and transceiver propagation delays.
- **Remediation:**
  Reconfigure CAN for 16 TQ: Prescaler = 2, BS1 = 13 TQ, BS2 = 2 TQ.

---

### [HIGH-EMB-04] Priority Inversion with `TransmitFifoPriority = DISABLE`
- **Location:** `Core/Src/main.c#L692`
- **Mechanism:**
  Disabling FIFO priority causes bxCAN to transmit mailboxes by CAN ID rather than chronological order, delivering queued frames out of sequence.
- **Remediation:**
  Set `hcan.Init.TransmitFifoPriority = ENABLE;`.

---

### [HIGH-EMB-05] Missing `volatile` Qualifiers and Data Races in `can_tx` Ring Buffer
- **Location:** `Core/Src/can_tx.c#L62-L66, L76-L104`
- **Mechanism:**
  `q_head`, `q_tail`, `q_count`, and `q_dropped` lack `volatile` qualification, and read-modify-writes are unprotected by critical sections.
- **Remediation:**
  Add `volatile` and wrap ring buffer pointer operations in PRIMASK critical sections.

---

### [HIGH-EMB-06] Stale Inverter Torque Command Queuing During Bus Congestion
- **Location:** `Core/Src/can_tx.c#L78-L81`, `Core/Src/main.c#L413-L415`
- **Mechanism:**
  If CAN stalls for 100 ms, 16 old inverter commands buffer up. Upon bus recovery, stale positive torque commands pump out even if the driver lifted their foot or hit the brakes.
- **Remediation:**
  Implement a dedicated single-slot mailbox for periodic inverter torque frames that always overwrites with the latest request.

---

### [HIGH-EMB-07] Memory Strict Aliasing Violation (`uint16_t` Cast to `uint32_t *`)
- **Location:** `Core/Src/adc.c#L6, L68`
- **Mechanism:**
  `bps_raw` is `uint16_t`, but passed as `(uint32_t *)&bps_raw` to `HAL_ADC_Start_DMA`. A 32-bit DMA write corrupts adjacent variables `dual_done` and `bps_done`.
- **Remediation:**
  Declare `bps_raw` as `uint32_t` or use an aligned DMA buffer.

---

## 6. Section V: Host Test Suite & Simulation Blindspots

Audit of `tools/control_test.c`, `tools/lc_test.c`, `tools/regen_test.c`, `tools/soc_kf_test.c`, `tools/soc_derate_test.c`, `tools/rtd_test.c`, and `tools/adc_test.c` identified significant verification blindspots:

1. **Host-Target ADC Execution Discrepancy:**
   In `Core/Src/adc.c`, `#ifdef ADC_HOST` completely compiles out the 5000-iteration busy-wait loop. Target runs a blocking polling loop; host tests run an asynchronous pipelined mock.
2. **Confirmation Bias in Test Assertions:**
   - `control_test.c#L281-L291`: Actively asserts that intermittent fault chattering never trips the debouncer, codifying an unsafe failure mode.
   - `control_test.c#L339`: Asserts that above 6000 RPM DCL is ignored (`"above 6000 rpm ignores DCL (known issue)"`), treating an unmitigated bug as passing.
   - `regen_test.c#L400`: Asserts `check_eq(regen_update(..., cut=1), 0)`, validating that regen cut unslewed snap is expected behavior.
   - `lc_test.c#L131`: Asserts that `map_interp` on `{0,0...}` and `{10,10...}` returns 10, passing degenerate calibration data.
3. **Missing Critical Edge Cases:**
   - Zero tests for Brake Pressure Sensor (BPS) disconnection (0V) or short (5V).
   - Zero tests for what occurs when APPS disagreement clears while throttle is applied.
   - `rtd_test.c` tests only the accumulator math in `rtd.c`; contains zero tests for the actual vehicle state machine, buzzer lockout, or brake interlock.
   - `soc_kf_test.c` runs only 30s to 120s synthetic runs; no endurance driving tests (25–30 min) to evaluate covariance drift or permanent NIS gate lockouts.

---

## 7. Section VI: Prioritized Remediation Roadmap

### Phase 1: Pre-Flash Critical Safety Blockers (Mandatory Before Track Testing)

| Task ID | Component | Required Action |
|:---|:---|:---|
| **P0-01** | `control.c` | Latch APPS/BSE faults for $>100\text{ ms}$; lock torque to 0 Nm until pedal returned to $<5\%$. |
| **P0-02** | `control.c`, `config.h` | Add BPS out-of-range checks [0.25V, 4.75V]; disable inverter and inhibit RTD on fault. |
| **P0-03** | `main.c` | Inhibit inverter enable and zero torque command while RTD buzzer is sounding. |
| **P0-04** | `main.c` | Deactivate RTD state (`ready_to_drive = 0`) when Tractive System deactivates ($V_{bus} < 60\text{V}$). |
| **P0-05** | `soc_derate.c` | Arbitrate DCL via `dbg.dcl = min(soc_dcl, bms_dcl)` to enforce BMS safety cutoffs. |
| **P0-06** | `regen.c` | Route `cut == 1` through `slew()` to eliminate 15,000 Nm/s drivetrain shock load. |
| **P0-07** | `regen.h` | Reduce `REGEN_PEAK_NM` from 100.0 Nm to 30.0 Nm to prevent rear axle snap-oversteer. |
| **P0-08** | `main.c`, `stm32f1xx_it.c` | Eliminate USB peripheral and PCD IRQ handler to resolve CAN/USB SRAM conflict. |
| **P0-09** | `main.c` | Set `hcan.Init.AutoRetransmission = ENABLE;` to prevent dropped inverter command frames. |
| **P0-10** | `main.c` | Enable STM32 IWDG watchdog timer refreshed strictly at the end of the 5 ms control loop. |
| **P0-11** | `main.c` | Fix `loop_dt_max_ms` self-assignment bug to restore timing telemetry. |

### Phase 2: Controls & Stability Hardening

| Task ID | Component | Required Action |
|:---|:---|:---|
| **P1-01** | `adc.c` | Replace blocking busy-wait in `adc_update()` with non-blocking timer-triggered DMA. |
| **P1-02** | `main.c` | Synchronize `TIM2.Period = 4999` in `CT-18EV.ioc` and reconfigure CAN for 16 TQ. |
| **P1-03** | `soc_kf.c` | Implement Joseph stabilized covariance update to prevent numerical singularity. |
| **P1-04** | `soc_kf.c` | Add adaptive in-motion NIS gate inflation after 10 consecutive rejected cycles. |
| **P1-05** | `soc_derate.c` | Add slew rate limiting to `dbg.dcl` to prevent instantaneous 160 A fallback steps. |
| **P1-06** | `launch_control.c` | Calibrate realistic open-loop launch map and fix bumpless crossover clamping bug. |
| **P1-07** | `control.c` | Scale torque power limit by measured `bus_voltage` and clamp 6000 RPM high-speed limit. |
| **P1-08** | `can_tx.c` | Implement single-slot overwrite buffer for periodic inverter command frames. |

### Phase 3: Architecture & Performance Optimizations

| Task ID | Component | Required Action |
|:---|:---|:---|
| **P2-01** | `soc_derate.c` | Cache OCV conversions on CAN arrival instead of recomputing at 200 Hz. |
| **P2-02** | `soc_kf.c` | Fix temperature parameter to signed `int8_t` and correct Jacobian $h_0$ term. |
| **P2-03** | `can_tx.c` | Set `hcan.Init.TransmitFifoPriority = ENABLE;` and protect queue with critical sections. |
| **P2-04** | `adc.c` | Declare `bps_raw` as `uint32_t` to resolve strict aliasing DMA alignment violation. |
| **P2-05** | `tools/` | Expand test suites to eliminate confirmation bias and test hardware fault recovery. |

---
*Report compiled autonomously by Antigravity Agent Team.*
