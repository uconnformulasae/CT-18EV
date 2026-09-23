/**
 * @file    config.h
 * @brief   Vehicle tuning constants and CAN identifiers.
 *
 * Constants owned by a single subsystem live with it instead
 * (launch_control.h, soc_kf.h, regen.h).
 */

#ifndef CONFIG_H
#define CONFIG_H

/* --- ADC scaling --- */

#define ADC_FULL_SCALE_COUNTS 4095.0f

#define ADC_TPS_VREF_V      3.3f
#define ADC_TPS_V_PER_COUNT (ADC_TPS_VREF_V / ADC_FULL_SCALE_COUNTS)

/* The brake pressure sensor is divided down from 5 V. */
#define ADC_BPS_VREF_V      5.0f
#define ADC_BPS_V_PER_COUNT (ADC_BPS_VREF_V / ADC_FULL_SCALE_COUNTS)

/* --- Throttle position sensors (APPS) --- */

/* Measured pedal endpoints, volts at the ADC pin. Nominal is 1.6564/2.2905 for
 * TPS1 and 0.8153/1.4364 for TPS2. To recalibrate from the bus, take 0x555
 * byte 2 (byte 3 for APPS2), divide by 255 and multiply by 3.3. */
#define TPS1_0PER   1.38f
#define TPS1_100PER 2.34f

#define TPS2_0PER   0.62f
#define TPS2_100PER 1.64f

/* Out-of-range trip points, volts */
#define TPS1_FAULT_LOW  0.2f
#define TPS1_FAULT_HIGH 2.8f

#define TPS2_FAULT_LOW  0.2f
#define TPS2_FAULT_HIGH 2.8f

/* APPS plausibility: (FSAE T.4.2.4). */
#define APPS_TRIP_PERCENT 0.70f

/* Weight given to the previous TPS sample. 0.0f disables the filter. */
#define TPS_IIR_RATIO 0.0f

/* --- Brake pressure (BPS) and brake plausibility (BSE) --- */

#define BPS_SETPOINT_V 0.500f

/* BSE latches above BSE_TRIP_TPS with the brakes on, and only clears once the
 * pedal drops below BSE_CLEAR_TPS. */
#define BSE_TRIP_TPS  0.10f
#define BSE_CLEAR_TPS 0.05f

/* --- Torque map --- */

/* Pedal dead band: below LOW is 0%, above HIGH is 100%. */
#define TMAP_DEADBAND_LOW  0.1f
#define TMAP_DEADBAND_HIGH 0.9f

/* Inverter torque ceiling, Nm x10. */
#define TORQUE_LIMIT_NM_X10 2200u

/* Ceiling from the DC current limit:
 *   (TORQUE_POWER_NUM * current_limit) / max(DIV_MIN, rpm * RPM_TO_DIV)
 * The floor on the divisor keeps the ceiling finite near zero speed. */
#define TORQUE_POWER_NUM     4200u
#define TORQUE_SPEED_DIV_MIN 230.4f
#define TORQUE_RPM_TO_DIV    0.1076f

#define TORQUE_HIGH_SPEED_RPM    6000u
#define TORQUE_HIGH_SPEED_NM_X10 300.0f

/* --- Ready to drive --- */

/* Button debounce lives in rtd.h, in milliseconds. */

/* TIM3 ticks, ~10 ms each. */
#define RTD_BUZZER_TICKS       25
#define RTD_BUZZER_COUNTER_MAX 100

/* TIM3 ticks since the inverter last reported its ready pin high. Past
 * RTD_TIMEOUT_TICKS the car drops out of ready-to-drive. */
#define RTD_TIMEOUT_TICKS 20
#define RTD_TIMEOUT_MAX   200
#define RTD_TIMEOUT_INIT  199

/* --- Inverter disable debounce --- */

/* Faults are held for a few TIM2 ticks before the inverter is commanded off,
 * so one noisy sample cannot cut the drive. */
#define DISABLE_DEBOUNCE_TRIP 5
#define DISABLE_DEBOUNCE_MAX  100
#define DISABLE_DEBOUNCE_INIT 999

/* Below both of these the car is idle and the inverter is released. */
#define IDLE_TORQUE_NM_X10 5
#define IDLE_SPEED_RPM     500u

/* --- BMS --- */

/* 10 missed 0x600 frames at 50 Hz. */
#define BMS_TIMEOUT_MS 200u

/* Orion reports pack SoC in 0.5 %/bit */
#define BMS_SOC_PCT_PER_BIT     0.5f
#define BMS_SOC_RAW_TO_PCT(raw) ((uint8_t)(((uint16_t)(raw) + 1u) / 2u))

/* Used until the BMS and inverter have been heard from. */
#define CURRENT_LIMIT_DEFAULT_A 125u
#define BUS_VOLTAGE_DEFAULT_V   396u

/* --- CAN identifiers --- */

#define CAN_ID_RX_INVERTER_STATE 0x0AA /* lockout, enable, ready pin  */
#define CAN_ID_RX_MOTOR_SPEED    0x0A5 /* int16 RPM                   */
#define CAN_ID_RX_BMS_DCL        0x202 /* discharge current limit     */
#define CAN_ID_RX_BMS_STATUS     0x600 /* SoC, pack voltage, current  */

#define CAN_ID_TX_INVERTER_CMD 0x0C0 /* torque command, 8 bytes     */
#define CAN_ID_TX_DEBUG        0x555 /* pedal + fault telemetry     */
#define CAN_ID_TX_RTD          0x556 /* ready-to-drive flag, 1 byte */

/* Accept exactly one standard identifier. */
#define CAN_STID_MASK_HIGH 0xFFE0
#define CAN_STID_MASK_LOW  0x0004

#define CAN_HEARTBEAT_MASK 0x0F

/* Transmit queueing and backpressure live in can_tx.h. */

#endif /* CONFIG_H */
