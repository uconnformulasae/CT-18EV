#include "app.h"
#include "stm32f1xx_hal.h"

extern "C" {
#include "config.h"
#include "pinout.h"
#include "adc.h"
#include "can_tx.h"
#include "rtd.h"
#include "launch_control.h"
#include "regen.h"
#include "bms_limits.h"
#include "control.h"
#include "version.h"
}

#include <algorithm>

volatile uint32_t motor_speed = 0;
volatile uint32_t motor_speed_tick = 0;
volatile uint8_t inverter_enabled = 0;
volatile uint8_t inverter_lockout = 1;
volatile uint8_t control_ready = 0;
volatile uint8_t print_ready = 0;
volatile bool ready_to_drive = 0;
volatile uint16_t rtd_timeout = RTD_TIMEOUT_INIT;
volatile uint8_t rtd_buzzer_counter = 0;

volatile uint8_t inv_message = 0;
volatile uint16_t ext_frame_count = 0;

/* Worst control loop period (ms) */
volatile uint32_t loop_dt_max_ms = 0;

static uint8_t lc_was_enabled = 0;
static control_t ctrl;

void HAL_CAN_RxFifo0MsgPendingCallback_Impl(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    rx_header.StdId = 0;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
		return;
	}

	if (rx_header.IDE != CAN_ID_STD) {
		ext_frame_count += 1;
		return;
	}

	if (rx_header.StdId == CAN_ID_RX_INVERTER_STATE) {
		inverter_enabled = rx_data[6] & 0x01;
		inverter_lockout = (rx_data[6] >> 7) & 0x01;
		uint8_t ready_pin_state = (rx_data[3] >> 1) & 0x01;
		inv_message = rx_data[6];
		if (ready_pin_state) {
			rtd_timeout = 0;
		}
	} else if (rx_header.StdId == CAN_ID_RX_MOTOR_SPEED) {
		int16_t motor_speed_raw = (int16_t) (uint16_t) (rx_data[3] << 8
				| rx_data[2]);
		motor_speed = (motor_speed_raw > 0) ? (uint32_t) motor_speed_raw : 0u;
		motor_speed_tick = HAL_GetTick();
	} else if (rx_header.StdId == CAN_ID_RX_BMS_LIMITS) {
		bms_limits_feed(rx_data, static_cast<float>(rx_header.DLC), HAL_GetTick());
	} else if (rx_header.StdId == LC_AIM_WHEEL_SPEED_CAN_ID) {
		uint16_t fl_speed = rx_data[1] << 8 | rx_data[0];
		lc_feed_wheel_speed(fl_speed);
	}
}

void HAL_TIM_PeriodElapsedCallback_Impl(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		control_ready = 1;
	} else if (htim->Instance == TIM3) {
		print_ready = 1;
		if (ready_to_drive && rtd_buzzer_counter < RTD_BUZZER_COUNTER_MAX) {
			rtd_buzzer_counter += 1;
		}
		rtd_timeout += 1;
        
        // copy once to prevent torn reads
        const uint16_t rtd_timeout_copy = rtd_timeout;

        // note: should probably use constexpr instead of DEFINEs
		rtd_timeout = std::min(rtd_timeout_copy, static_cast<uint16_t>(RTD_TIMEOUT_MAX));
	}
}

void configure_can(CAN_HandleTypeDef* hcan) {
    /* Bank 1 held 0x0B1 (dynamic torque limit); nothing decodes it. */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 2; /* Inverter state */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_INVERTER_STATE << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 3; /* Motor speed */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_MOTOR_SPEED << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 4; /* BMS power limits */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_BMS_LIMITS << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 6; /* Front wheel speed */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = LC_AIM_WHEEL_SPEED_CAN_ID << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(hcan, &canfilterconfig);

	HAL_CAN_Start(hcan);

	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
}

void app_init(CAN_HandleTypeDef* hcan) {
	configure_can(hcan);
	lc_init();
	adc_init();
	rtd_init();
	can_tx_init();

    #if REGEN_ENABLE
    regen_init();
    #endif

    bms_limits_init();
    control_init(&ctrl);
}

static uint8_t heartbeat_counter = 0;
static uint32_t last_loop_tick = 0;
static uint32_t version_tick = 0;
static uint32_t bps_tick = 0;

void app_start() {
    heartbeat_counter = 0;
    last_loop_tick = HAL_GetTick();
    version_tick = last_loop_tick - VERSION_TX_MS;
    bps_tick = last_loop_tick;
}

control_data_t controls_update() {
    control_data_t controls;

    adc_update();
    controls.adcs.tps1_adc = adc_tps1();
    controls.adcs.tps2_adc = adc_tps2();
    controls.adcs.bps_adc = adc_bps();

    const control_throttle_t throttle = control_throttle(&ctrl,
        static_cast<float>(controls.adcs.tps1_adc * ADC_TPS_V_PER_COUNT),
        static_cast<float>(controls.adcs.tps2_adc * ADC_TPS_V_PER_COUNT)
    );
    controls.throttle.tps = throttle.tps;

    controls.brakes.bps = static_cast<float>(controls.adcs.bps_adc * ADC_BPS_V_PER_COUNT);
    controls.brakes = control_brakes(controls.brakes.bps);

    return controls;
}

void app_step(IWDG_HandleTypeDef* hiwdg, uint8_t reset_cause) {
    uint8_t tx_data[8];
	int32_t torque_request = 0;
	
	uint8_t bse_error = 0;
	uint8_t should_disable_inverter = 0;

	uint8_t rtd_raw = 0;
	last_loop_tick = HAL_GetTick();
	version_tick = last_loop_tick - VERSION_TX_MS;
	bps_tick = last_loop_tick;

    can_tx_pump();

    if (!control_ready) {
        return;
    }

    control_ready = 0;

    const uint32_t loop_tick = HAL_GetTick();
    const uint32_t loop_dt_ms = loop_tick - last_loop_tick;
    
    last_loop_tick = loop_tick;

    if (loop_dt_ms > loop_dt_max_ms) {
        loop_dt_max_ms = loop_dt_ms;
    }

    control_data_t controls = controls_update();

    rtd_raw = (HAL_GPIO_ReadPin(PIN_RTD_BUTTON_PORT, PIN_RTD_BUTTON)
            == PIN_RTD_BUTTON_ACTIVE);
    rtd_raw &= controls.brakes.brake_pressed;

    if (rtd_debounce_update(rtd_raw, loop_dt_ms)) {
        ready_to_drive = 1;
    }

    ready_to_drive &= rtd_timeout < RTD_TIMEOUT_TICKS;

    HAL_GPIO_WritePin(PIN_RTD_LIGHT_PORT, PIN_RTD_LIGHT, ready_to_drive ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (ready_to_drive == 0) {
        rtd_buzzer_counter = 0;
    } else {
        if (rtd_buzzer_counter < RTD_BUZZER_TICKS) {
            HAL_GPIO_WritePin(PIN_RTD_BUZZER_PORT, PIN_RTD_BUZZER,
                    GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(PIN_RTD_BUZZER_PORT, PIN_RTD_BUZZER,
                    GPIO_PIN_RESET);
        }
    }

    bse_error = control_bse(&ctrl, controls.brakes.brake_pressed, controls.throttle.tps);

    const uint8_t fault = controls.throttle.tps1_oor || controls.throttle.tps2_oor
            || controls.throttle.tps_disagree || bse_error || controls.brakes.bps_oor;
    should_disable_inverter = control_fault_debounce(&ctrl, fault,
            loop_dt_ms) || !ready_to_drive;

    bms_limits_t lim;
    bms_limits_get(loop_tick, &lim);

    torque_request = control_torque_limit(control_torque_map(controls.throttle.tps),
            motor_speed, lim.dpl_w);

#if REGEN_ENABLE
    const uint8_t speed_stale = (int32_t) (loop_tick - motor_speed_tick)
            > (int32_t) MOTOR_SPEED_TIMEOUT_MS;
    const uint8_t regen_cut = brake_pressed || fault
            || should_disable_inverter || speed_stale
            || launch_control_enable;
    torque_request = regen_update(torque_request, motor_speed, tps_combined,
            &lim, regen_cut, loop_dt_ms);
#endif

    lc_feed_tick(HAL_GetTick());
    if (launch_control_enable) {
        torque_request = lc_update(torque_request, motor_speed,
                controls.throttle.tps);
        lc_was_enabled = 1;
    } else if (lc_was_enabled) {
        lc_init();
        lc_was_enabled = 0;
    }

    if (should_disable_inverter) {
        torque_request = 0;
    }

    control_pack_inverter(tx_data, torque_request, inverter_lockout,
            should_disable_inverter, heartbeat_counter);
    can_tx_send(CAN_ID_TX_INVERTER_CMD, tx_data, 8);
    heartbeat_counter += 1;
    heartbeat_counter = heartbeat_counter & CAN_HEARTBEAT_MASK;

    if (loop_tick - version_tick >= VERSION_TX_MS) {
        version_pack(tx_data);
        can_tx_send(VERSION_CAN_ID, tx_data, VERSION_DLC);
        version_tick = loop_tick;
    }

    if (loop_tick - bps_tick >= BPS_TX_MS) {
        control_pack_bps(tx_data, control_bps_psi(controls.brakes.bps), static_cast<uint16_t>(controls.adcs.bps_adc),
                controls.brakes.brake_pressed, controls.brakes.bps_oor);
        can_tx_send(CAN_ID_TX_BPS, tx_data, CAN_BPS_DLC);
        bps_tick = loop_tick;
    }

    if (print_ready) {
        control_pack_debug(
            tx_data, &controls, 
            rtd_timeout, 
            inverter_lockout, 
            inverter_enabled, 
            bse_error, 
            ready_to_drive, 
            should_disable_inverter
        );

        can_tx_send(CAN_ID_TX_DEBUG, tx_data, 8);

        tx_data[0] = (ready_to_drive) & 0x01;
        tx_data[1] = reset_cause;

        can_tx_send(CAN_ID_TX_RTD, tx_data, 2);

        for (uint32_t i = 0; i < 7; i++) {
            tx_data[i] = 0;
        }

        // copy once to prevent torn reads
        const uint32_t loop_dt_max_ms_copy=loop_dt_max_ms;
        tx_data[7] = static_cast<uint8_t>(std::min(loop_dt_max_ms_copy, static_cast<uint32_t>(255)));
        
        can_tx_send(CAN_ID_TX_LOOP, tx_data, 8);

        control_pack_limits(tx_data,
                control_torque_limit(1.0f, motor_speed, lim.dpl_w),
                static_cast<int32_t>(10.0f * regen_get_debug()->max_nm), &lim);
        can_tx_send(CAN_ID_TX_LIMITS, tx_data, 8);

        #if REGEN_ENABLE
        regen_pack_debug(tx_data);
        can_tx_send(REGEN_DEBUG_CAN_ID, tx_data, REGEN_DEBUG_DLC);
        #endif

        print_ready = 0;
    }

    /* Only a finished control step feeds the dog */
    HAL_IWDG_Refresh(hiwdg);
}
