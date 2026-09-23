/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

#include "config.h"
#include "pinout.h"
#include "adc.h"
#include "can_tx.h"
#include "rtd.h"
#include "launch_control.h"
#include "regen.h"
#include "soc_kf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Integer on purpose: the ISRs below run on a Cortex-M3 with no FPU, where
 * fmin()/fminf() are soft-float calls. CAP is the unsigned case. */
#define CAP(v, hi)       (((v) > (hi)) ? (hi) : (v))
#define CLAMP(v, lo, hi) (((v) < (lo)) ? (lo) : CAP(v, hi))

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

static uint32_t torque_limit = TORQUE_LIMIT_NM_X10;
volatile uint32_t motor_speed = 0;
volatile uint32_t current_limit = CURRENT_LIMIT_DEFAULT_A;
volatile uint8_t soc = 0;
volatile uint8_t inverter_enabled = 0;
volatile uint8_t inverter_lockout = 1;
volatile uint8_t control_ready = 0;
volatile uint8_t print_ready = 0;
volatile uint8_t ready_to_drive = 0;
uint8_t tps1_oor = 0;
uint8_t tps2_oor = 0;
uint8_t tps_dist_error = 0;
volatile uint16_t rtd_timeout = RTD_TIMEOUT_INIT;
volatile uint8_t rtd_buzzer_counter = 0;
volatile uint8_t start_disable_debounce = 1;
volatile uint16_t disable_debounce = DISABLE_DEBOUNCE_INIT;

volatile uint8_t soc_valid = 0;
volatile uint32_t soc_last_tick = 0;

/* Written but never read by the firmware: watch these in the debugger. */
volatile uint32_t bus_voltage = BUS_VOLTAGE_DEFAULT_V;
volatile uint8_t inv_message = 0;
volatile uint16_t ext_frame_count = 0;

/* Worst control period seen, ms. Reported in byte 7 of SOC_KF_CAN_ID_STATE. */
volatile uint32_t loop_dt_max_ms = 0;

// Tracks the launch control enable edge so lc_init() runs once per disable.
static uint8_t lc_was_enabled = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	RxHeader.StdId = 0;
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		return;
	}
	// HAL only writes StdId for standard frames; an extended frame would be
	// dispatched under the previous frame's StdId with the new payload.
	if (RxHeader.IDE != CAN_ID_STD) {
		ext_frame_count++;
		return;
	}

	if (RxHeader.StdId == CAN_ID_RX_INVERTER_STATE) {
		inverter_enabled = RxData[6] & 0x01;
		inverter_lockout = (RxData[6] >> 7) & 0x01;
		uint8_t ready_pin_state = (RxData[3] >> 1) & 0x01;
		inv_message = RxData[6];
		if (ready_pin_state) {
			rtd_timeout = 0;
		}
	} else if (RxHeader.StdId == CAN_ID_RX_MOTOR_SPEED) {
		// signed int16 RPM; reverse rotation clamps to 0
		int16_t motor_speed_raw = (int16_t) (uint16_t) (RxData[3] << 8
				| RxData[2]);
		motor_speed = (motor_speed_raw > 0) ? (uint32_t) motor_speed_raw : 0u;
	} else if (RxHeader.StdId == CAN_ID_RX_BMS_DCL) {
		uint16_t dcl_raw = (uint16_t) (RxData[1] << 8 | RxData[0]);
		current_limit = (dcl_raw > 3u) ? (uint32_t) (dcl_raw - 3u) : 0u;
	} else if (RxHeader.StdId == CAN_ID_RX_BMS_STATUS) {
		soc = BMS_SOC_RAW_TO_PCT(RxData[1]);
		soc_valid = 1;
		soc_last_tick = HAL_GetTick();
		bus_voltage = (RxData[5] << 8 | RxData[4]);
		// b2 temp (1C/bit, unsigned), b6-7 current (int16, 0.1A/bit).
		soc_kf_feed_bms((int16_t) (uint16_t) (RxData[7] << 8 | RxData[6]),
				(uint16_t) (RxData[5] << 8 | RxData[4]), RxData[2], RxData[1],
				HAL_GetTick());
	}
	// AiM EVO5 front wheel speed broadcast
	else if (RxHeader.StdId == LC_AIM_WHEEL_SPEED_CAN_ID) {
		// Front left km/h x10, little endian. Bytes 2-3 carry a right front
		// channel that this car has no sensor for, so they are not read.
		uint16_t fl_speed = RxData[1] << 8 | RxData[0];
		lc_feed_wheel_speed(fl_speed);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		control_ready = 1;
		if (start_disable_debounce) {
			disable_debounce += 1;
			disable_debounce = CAP(disable_debounce, DISABLE_DEBOUNCE_MAX);
		} else {
			disable_debounce = 0;
		}
	} else if (htim->Instance == TIM3) {
		print_ready = 1;
		if (ready_to_drive && rtd_buzzer_counter < RTD_BUZZER_COUNTER_MAX) {
			rtd_buzzer_counter += 1;
		}
		rtd_timeout += 1;
		rtd_timeout = CAP(rtd_timeout, RTD_TIMEOUT_MAX);
	}
}

/* Raw pedal travel -> 0.0f..1.0f across the calibrated dead band. */
static float tmap_lut(float tps) {
	return (fmaxf(TMAP_DEADBAND_LOW, fminf(tps, TMAP_DEADBAND_HIGH))
			- TMAP_DEADBAND_LOW)
			* (1.0f / (TMAP_DEADBAND_HIGH - TMAP_DEADBAND_LOW));
}

/* Mapped pedal position -> torque request in Nm x10, capped by what the DC
 * current limit allows at the current motor speed. */
static int32_t torque_lut(float tps) {
	const float power_limit = (float) (TORQUE_POWER_NUM * current_limit)
			/ fmaxf(TORQUE_SPEED_DIV_MIN,
					(float) motor_speed * TORQUE_RPM_TO_DIV);

	float torque_limit_local = fminf((float) torque_limit, power_limit);

	if (motor_speed >= TORQUE_HIGH_SPEED_RPM) {
		torque_limit_local = TORQUE_HIGH_SPEED_NM_X10;
	}

	return (int32_t) (tps * torque_limit_local);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	for (uint32_t i = 0; i < sizeof(RxData); i++) {
		RxData[i] = 0;
	}
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
	uint8_t TxData[8];
	/* Bank 1 held 0x0B1 (dynamic torque limit); nothing decodes it. */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 2; // inverter status
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_INVERTER_STATE << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 3; // motor speed
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_MOTOR_SPEED << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 4; // BMS discharge current limit
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_BMS_DCL << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 5; // BMS SoC, pack voltage and current
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_BMS_STATUS << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 6; // AiM front wheel speed
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = LC_AIM_WHEEL_SPEED_CAN_ID << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	HAL_CAN_Start(&hcan);

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

	// Initialize launch control system
	lc_init();

	// Pedal and brake sampling (starts the first conversion)
	adc_init();

	// Ready-to-drive debounce
	rtd_init();

	// CAN transmit queue
	can_tx_init();

	// Regen braking
	regen_init();

	// SoC Kalman filter (telemetry only - never gates the control path)
	soc_kf_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	float tps1 = 0;
	float tps2 = 0;
	float tps_combined = 0;
	float bps = 0;
	int32_t torque_request = 0;
	uint8_t heartbeat_counter = 0;
	uint32_t tps1_adc = 0;
	uint32_t tps2_adc = 0;
	uint32_t bps_adc = 0;
	uint8_t brake_pressed = 0;
	uint8_t bse_error = 0;
	uint8_t should_disable_inverter = 0;

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	float tps1_avg = 0;
	float tps2_avg = 0;

	uint8_t rtd_raw = 0;
	uint32_t last_loop_tick = HAL_GetTick();

	while (1) {
		/* Drains every pass, independent of the control tick. */
		can_tx_pump();

		/* Control law runs on the TIM2 tick, not free-running: the slip
		 * derivative and the debounces need a bounded, known period. */
		if (!control_ready) {
			continue;
		}
		control_ready = 0;

		const uint32_t loop_tick = HAL_GetTick();
		const uint32_t loop_dt_ms = loop_tick - last_loop_tick; /* wrap-safe */
		last_loop_tick = loop_tick;
		if (loop_dt_ms > loop_dt_max_ms) {
			loop_dt_max_ms = loop_dt_ms;
		}

		/* Non-blocking: latches last tick's conversion, starts the next. */
		adc_update();
		tps1_adc = adc_tps1();
		tps2_adc = adc_tps2();
		bps_adc = adc_bps();

		// Throttle Position Potentiometer 1 Acquire and Calculate
		float tps1_v = (float) tps1_adc * ADC_TPS_V_PER_COUNT;
		tps1 = (tps1_v - TPS1_0PER) / (TPS1_100PER - TPS1_0PER);
		tps1 = fmaxf(0.0f, fminf(tps1, 1.0f));
		tps1_avg =
				(tps1_avg == 0) ?
						tps1 :
						tps1_avg * TPS_IIR_RATIO
								+ tps1 * (1.0f - TPS_IIR_RATIO);
		tps1 = tps1_avg;

		// Throttle Position Potentiometer 2 Acquire and Calculate
		float tps2_v = (float) tps2_adc * ADC_TPS_V_PER_COUNT;
		tps2 = (tps2_v - TPS2_0PER) / (TPS2_100PER - TPS2_0PER);
		tps2 = fmaxf(0.0f, fminf(tps2, 1.0f));
		tps2_avg =
				(tps2_avg == 0) ?
						tps2 :
						tps2_avg * TPS_IIR_RATIO
								+ tps2 * (1.0f - TPS_IIR_RATIO);
		tps2 = tps2_avg;

		// TPS and Torque request calculate
		tps_combined = (tps1 + tps2) / 2;
		torque_request = torque_lut(tmap_lut(tps_combined));

		// Regen braking on a closed throttle
		uint8_t bms_fresh = soc_valid
				&& ((HAL_GetTick() - soc_last_tick) < BMS_TIMEOUT_MS);
		torque_request = regen_update(torque_request, motor_speed, tps_combined,
				brake_pressed, soc, bms_fresh);

		/* Launch control. The tick goes in every pass, enabled or not, so the
		 * CAN RX ISR timestamps the wheel speed against a current clock and
		 * the sensor-timeout check is meaningful the moment LC is enabled. */
		lc_feed_tick(HAL_GetTick());
		if (launch_control_enable) {
			torque_request = lc_update(torque_request, motor_speed,
					tps_combined);
			lc_was_enabled = 1;
		} else if (lc_was_enabled) {
			// Reset once on the enable -> disable edge, not every pass: a reset
			// per pass clears the debug struct that telemetry reads.
			lc_init();
			lc_was_enabled = 0;
		}

		// Brake Pressure Acquire and Calculate
		bps = (float) bps_adc * ADC_BPS_V_PER_COUNT;
		brake_pressed = bps > BPS_SETPOINT_V;

		// Ready to Drive button poll
		// Active low: see PIN_RTD_BUTTON_ACTIVE in pinout.h.
		rtd_raw = (HAL_GPIO_ReadPin(PIN_RTD_BUTTON_PORT, PIN_RTD_BUTTON)
				== PIN_RTD_BUTTON_ACTIVE);
		rtd_raw &= brake_pressed;

		if (rtd_debounce_update(rtd_raw, loop_dt_ms)) {
			ready_to_drive = 1;
		}

		// Drop out of ready-to-drive if the inverter stops reporting ready
		ready_to_drive &= rtd_timeout < RTD_TIMEOUT_TICKS;

		// Ready to Drive dashboard light
		HAL_GPIO_WritePin(PIN_RTD_LIGHT_PORT, PIN_RTD_LIGHT, ready_to_drive);

		// Ready to drive Buzzer
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

		// Error States
		tps1_oor = tps1_v < TPS1_FAULT_LOW || tps1_v > TPS1_FAULT_HIGH;
		tps2_oor = tps2_v < TPS2_FAULT_LOW || tps2_v > TPS2_FAULT_HIGH;
		tps_dist_error = fabsf(tps1 - tps2) > APPS_TRIP_PERCENT;

		if (!bse_error) {
			bse_error = brake_pressed && tps_combined >= BSE_TRIP_TPS;
		} else {
			bse_error = tps_combined >= BSE_CLEAR_TPS;
		}

		// Disable Inverter if any errors present
		start_disable_debounce = tps1_oor || tps2_oor || tps_dist_error
				|| bse_error
				|| (torque_request < IDLE_TORQUE_NM_X10
						&& motor_speed < IDLE_SPEED_RPM);
		should_disable_inverter = (disable_debounce > DISABLE_DEBOUNCE_TRIP)
				|| !ready_to_drive;

		if (should_disable_inverter) {
			torque_request = 0;
		}

		if (inverter_lockout == 1) {
			TxData[0] = torque_request & 0xFF;      // Torque Command lo
			TxData[1] = torque_request >> 8 & 0xFF; // Torque Command hi
			TxData[2] = 0x00;                       // Speed Command lo
			TxData[3] = 0x00;                       // Speed Command hi
			TxData[4] = 0x01; // Direction: Reverse = 0x00 | Forward = 0x01;
			// 5[0] = Inv enable | 5[1] = Discharge enable | counter
			TxData[5] = 0x00 | 0x02 | (heartbeat_counter << 4);
			TxData[6] = 0x00; // Torque limit lo, 0 = EEprom limit
			TxData[7] = 0x00; // Torque limit hi, 0 = EEprom limit

			can_tx_send(CAN_ID_TX_INVERTER_CMD, TxData, 8);
			heartbeat_counter += 1;
			heartbeat_counter = heartbeat_counter & CAN_HEARTBEAT_MASK;
		} else {
			TxData[0] = torque_request & 0xFF;      // Torque Command lo
			TxData[1] = torque_request >> 8 & 0xFF; // Torque Command hi
			TxData[2] = 0x00;                       // Speed Command lo
			TxData[3] = 0x00;                       // Speed Command hi
			TxData[4] = 0x01; // Direction: Reverse = 0x00 | Forward = 0x01;
			// 5[0] = Inv enable | 5[1] = Discharge enable | counter
			TxData[5] = (~should_disable_inverter & 0x01) | 0x02
					| (heartbeat_counter << 4);
			TxData[6] = 0x00; // Torque limit lo, 0 = EEprom limit
			TxData[7] = 0x00; // Torque limit hi, 0 = EEprom limit

			can_tx_send(CAN_ID_TX_INVERTER_CMD, TxData, 8);
			heartbeat_counter += 1;
			heartbeat_counter = heartbeat_counter & CAN_HEARTBEAT_MASK;
		}

		if (print_ready) { // Every ~100(?) ms
			TxData[0] = rtd_timeout & 0xFF;
			TxData[1] = (bps_adc >> 4) & 0xFF;
			TxData[2] = (tps1_adc >> 4) & 0xFF;
			TxData[3] = (tps2_adc >> 4) & 0xFF;
			TxData[4] = (inverter_lockout << 7) | (inverter_enabled << 6)
					| (tps_dist_error << 5) | (tps2_oor << 4) | (tps1_oor << 3)
					| (brake_pressed << 2) | (ready_to_drive << 1)
					| should_disable_inverter;
			TxData[5] = (int) (tps1 * 100) & 0xff;
			TxData[6] = (int) (tps2 * 100) & 0xff;
			TxData[7] = (int) (tmap_lut(tps_combined) * 100) & 0xFF;

			can_tx_send(CAN_ID_TX_DEBUG, TxData, 8);

			TxData[0] = (ready_to_drive) & 0x01;

			can_tx_send(CAN_ID_TX_RTD, TxData, 1);

			soc_kf_update(HAL_GetTick());
			{
				uint8_t kfData[8];
				soc_kf_pack_state(kfData);
				/* 0x558 byte 7: worst control loop period in ms, saturating at
				 * 255. soc_kf_pack_state() owns bytes 0-6. */
				kfData[7] = (uint8_t)CAP(loop_dt_max_ms, 255u);
				can_tx_send(SOC_KF_CAN_ID_STATE, kfData, 8);
			}

			print_ready = 0;
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 14399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* PB14 (RTD button) is not in the .ioc, so CubeMX does not generate a
   * config for it. It was configured up to 50cc7c8 only because main.c had
   * drifted out of sync with the project file; the regeneration in c59aed0
   * synced them and silently dropped it, leaving the pin in its reset state
   * (floating input). Configured here, inside a USER CODE block, so a future
   * regeneration cannot drop it again. Pull direction comes from pinout.h so
   * it stays matched to PIN_RTD_BUTTON_ACTIVE. */
  GPIO_InitStruct.Pin = PIN_RTD_BUTTON;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = PIN_RTD_BUTTON_PULL;
  HAL_GPIO_Init(PIN_RTD_BUTTON_PORT, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
