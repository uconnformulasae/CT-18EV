#ifndef CONFIG_H
#define CONFIG_H

/* ADC scaling */
#define ADC_FULL_SCALE_COUNTS 4095.0f

#define ADC_TPS_VREF_V      3.3f
#define ADC_TPS_V_PER_COUNT (ADC_TPS_VREF_V / ADC_FULL_SCALE_COUNTS)

#define ADC_BPS_VREF_V      5.0f
#define ADC_BPS_V_PER_COUNT (ADC_BPS_VREF_V / ADC_FULL_SCALE_COUNTS)

/* APPS calibration (V at ADC pin) */
#define TPS1_0PER   1.38f
#define TPS1_100PER 2.34f

#define TPS2_0PER   0.62f
#define TPS2_100PER 1.64f

/* APPS out-of-range thresholds (V) */
#define TPS1_FAULT_LOW  0.2f
#define TPS1_FAULT_HIGH 2.8f

#define TPS2_FAULT_LOW  0.2f
#define TPS2_FAULT_HIGH 2.8f

/* Max APPS disagreement (FSAE T.4.2.4) */
#define APPS_TRIP_PERCENT 0.10f

/* TPS filter weight (0 = disabled) */
#define TPS_IIR_RATIO 0.0f

/* Brake pressure (BPS) and brake plausibility (BSE) */
#define BPS_SETPOINT_V 0.700f

#define BSE_TRIP_TPS  0.25f
#define BSE_CLEAR_TPS 0.05f

/* Regen (one-pedal) enable (1 = enabled, 0 = disabled) */
#ifndef REGEN_ENABLE
#define REGEN_ENABLE 1
#endif

/* Torque map */
#define TMAP_DEADBAND_LOW  0.05f
#define TMAP_REGEN_END     0.35f
#if REGEN_ENABLE
#define TMAP_DRIVE_START   0.39f
#define TMAP_DEADBAND_HIGH 0.95f
#else
#define TMAP_DRIVE_START   TMAP_DEADBAND_LOW
#define TMAP_DEADBAND_HIGH 0.95f
#endif

/* Max torque limit (0.1 Nm) */
#define TORQUE_LIMIT_NM_X10 2200u

/* DC current limit to torque conversion */
#define TORQUE_POWER_NUM     4200u
#define TORQUE_SPEED_DIV_MIN 230.4f
#define TORQUE_RPM_TO_DIV    0.1076f

#define TORQUE_HIGH_SPEED_RPM    6000u
#define TORQUE_HIGH_SPEED_NM_X10 300.0f

/* Ready to drive */
#define RTD_BUZZER_TICKS       12
#define RTD_BUZZER_COUNTER_MAX 100

/* Inverter timeout (TIM3 ticks) */
#define RTD_TIMEOUT_TICKS 20
#define RTD_TIMEOUT_MAX   200
#define RTD_TIMEOUT_INIT  199

/* Inverter disable debounce (TIM2 ticks, 50 ms at 200 Hz) */
#define DISABLE_DEBOUNCE_TRIP 10
#define DISABLE_DEBOUNCE_MAX  100
#define DISABLE_DEBOUNCE_INIT 999

#define MOTOR_SPEED_TIMEOUT_MS 200u

/* Orion SoC: 0.5%/bit */
#define BMS_SOC_PCT_PER_BIT 0.5f

/* Defaults before BMS/inverter comms */
#define CURRENT_LIMIT_DEFAULT_A 125u
#define BUS_VOLTAGE_DEFAULT_V   396u

/* CAN IDs */
#define CAN_ID_RX_INVERTER_STATE 0x0AA
#define CAN_ID_RX_MOTOR_SPEED    0x0A5
#define CAN_ID_RX_BMS_DCL        0x202
#define CAN_ID_RX_BMS_STATUS     0x600

#define CAN_ID_TX_INVERTER_CMD 0x0C0
#define CAN_ID_TX_DEBUG        0x555
#define CAN_ID_TX_RTD          0x556

#define CAN_STID_MASK_HIGH 0xFFE0
#define CAN_STID_MASK_LOW  0x0004

#define CAN_HEARTBEAT_MASK 0x0F

#endif /* CONFIG_H */
