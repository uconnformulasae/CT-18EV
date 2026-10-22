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
#include "soc_derate.h"
#include "control.h"
#include "version.h"
#include "inv_tune.h"
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

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CAP(v, hi)       (((v) > (hi)) ? (hi) : (v))
#define CLAMP(v, lo, hi) (((v) < (lo)) ? (lo) : CAP(v, hi))

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

volatile uint32_t motor_speed = 0;
volatile uint32_t motor_speed_tick = 0;
volatile uint32_t current_limit = CURRENT_LIMIT_DEFAULT_A;
volatile uint32_t bms_dcl = CURRENT_LIMIT_DEFAULT_A;
volatile uint8_t inverter_enabled = 0;
volatile uint8_t inverter_lockout = 1;
volatile uint8_t control_ready = 0;
volatile uint8_t print_ready = 0;
volatile uint8_t ready_to_drive = 0;
volatile uint16_t rtd_timeout = RTD_TIMEOUT_INIT;
volatile uint8_t rtd_buzzer_counter = 0;

volatile uint32_t bus_voltage = BUS_VOLTAGE_DEFAULT_V;
volatile uint8_t inv_message = 0;
volatile uint16_t ext_frame_count = 0;

/* Worst control loop period (ms) */
volatile uint32_t loop_dt_max_ms = 0;

static uint8_t lc_was_enabled = 0;
static control_t ctrl;
static uint8_t reset_cause = 0;

static uint8_t read_reset_cause(void) {
	uint8_t cause = 0;
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
		cause |= RESET_CAUSE_PIN;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
		cause |= RESET_CAUSE_POR;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
		cause |= RESET_CAUSE_SOFT;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		cause |= RESET_CAUSE_IWDG;
	}
	/* Flags are sticky until cleared, so the next reset reads fresh */
	__HAL_RCC_CLEAR_RESET_FLAGS();
	return cause;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	RxHeader.StdId = 0;
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		return;
	}
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
		inv_tune_feed_state(HAL_GetTick());
	} else if (RxHeader.StdId == CAN_ID_RX_INVERTER_PARAM) {
		inv_tune_feed_response(RxData);
	} else if (RxHeader.StdId == CAN_ID_RX_MOTOR_SPEED) {
		int16_t motor_speed_raw = (int16_t) (uint16_t) (RxData[3] << 8
				| RxData[2]);
		motor_speed = (motor_speed_raw > 0) ? (uint32_t) motor_speed_raw : 0u;
		motor_speed_tick = HAL_GetTick();
	} else if (RxHeader.StdId == CAN_ID_RX_BMS_DCL) {
		uint16_t dcl_raw = (uint16_t) (RxData[1] << 8 | RxData[0]);
		bms_dcl = (dcl_raw > 3u) ? (uint32_t) (dcl_raw - 3u) : 0u;
	} else if (RxHeader.StdId == CAN_ID_RX_BMS_STATUS) {
		bus_voltage = (RxData[5] << 8 | RxData[4]);
		soc_kf_feed_bms((int16_t) (uint16_t) (RxData[7] << 8 | RxData[6]),
				(uint16_t) (RxData[5] << 8 | RxData[4]), RxData[2], RxData[1],
				HAL_GetTick());
	} else if (RxHeader.StdId == CAN_ID_RX_BMS_CELL) {
		soc_derate_feed_cells((uint16_t) (RxData[5] << 8 | RxData[4]),
				(uint16_t) (RxData[7] << 8 | RxData[6]), HAL_GetTick());
	} else if (RxHeader.StdId == LC_AIM_WHEEL_SPEED_CAN_ID) {
		uint16_t fl_speed = RxData[1] << 8 | RxData[0];
		lc_feed_wheel_speed(fl_speed);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		control_ready = 1;
	} else if (htim->Instance == TIM3) {
		print_ready = 1;
		if (ready_to_drive && rtd_buzzer_counter < RTD_BUZZER_COUNTER_MAX) {
			rtd_buzzer_counter += 1;
		}
		rtd_timeout += 1;
		rtd_timeout = CAP(rtd_timeout, RTD_TIMEOUT_MAX);
	}
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
	reset_cause = read_reset_cause();
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
	uint8_t TxData[8];
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
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 3; /* Motor speed */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_MOTOR_SPEED << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 4; /* BMS current limit */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_BMS_DCL << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 5; /* BMS status */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_BMS_STATUS << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 6; /* Front wheel speed */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = LC_AIM_WHEEL_SPEED_CAN_ID << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 7; /* BMS low cell */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_BMS_CELL << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = CAN_STID_MASK_HIGH;
	canfilterconfig.FilterMaskIdLow = CAN_STID_MASK_LOW;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 8; /* Inverter parameter response */
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = CAN_ID_RX_INVERTER_PARAM << 5;
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

	lc_init();
	adc_init();
	rtd_init();
	can_tx_init();
#if REGEN_ENABLE
	regen_init();
#endif
	soc_kf_init();
	soc_derate_init();
	control_init(&ctrl);
	inv_tune_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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

	/* Started last so init time doesn't count against the timeout */
	MX_IWDG_Init();

	uint8_t rtd_raw = 0;
	uint32_t last_loop_tick = HAL_GetTick();
	uint32_t version_tick = last_loop_tick - VERSION_TX_MS;
	uint32_t bps_tick = last_loop_tick;
	uint32_t pedals_tick = last_loop_tick;
	uint32_t inv_tune_tick = last_loop_tick;

	while (1) {
		can_tx_pump();

		if (!control_ready) {
			continue;
		}
		control_ready = 0;

		const uint32_t loop_tick = HAL_GetTick();
		const uint32_t loop_dt_ms = loop_tick - last_loop_tick;
		last_loop_tick = loop_tick;
		if (loop_dt_ms > loop_dt_max_ms) {
			loop_dt_max_ms = loop_dt_ms;
		}

		adc_update();
		tps1_adc = adc_tps1();
		tps2_adc = adc_tps2();
		bps_adc = adc_bps();

		const control_pedals_t pedals = control_pedals(&ctrl,
				(float) tps1_adc * ADC_TPS_V_PER_COUNT,
				(float) tps2_adc * ADC_TPS_V_PER_COUNT);
		tps_combined = pedals.tps;

		bps = (float) bps_adc * ADC_BPS_V_PER_COUNT;
		/* A shorted sensor must not count as braking for the RTD interlock */
		const uint8_t bps_oor = control_bps_oor(bps);
		brake_pressed = bps > BPS_SETPOINT_V && !bps_oor;

		rtd_raw = (HAL_GPIO_ReadPin(PIN_RTD_BUTTON_PORT, PIN_RTD_BUTTON)
				== PIN_RTD_BUTTON_ACTIVE);
		rtd_raw &= brake_pressed;

		if (rtd_debounce_update(rtd_raw, loop_dt_ms)) {
			ready_to_drive = 1;
		}

		ready_to_drive &= rtd_timeout < RTD_TIMEOUT_TICKS;

		HAL_GPIO_WritePin(PIN_RTD_LIGHT_PORT, PIN_RTD_LIGHT, ready_to_drive);

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

		bse_error = control_bse(&ctrl, brake_pressed, tps_combined);

		const uint8_t fault = pedals.tps1_oor || pedals.tps2_oor
				|| pedals.tps_disagree || bse_error || bps_oor;
		should_disable_inverter = control_fault_debounce(&ctrl, fault,
				loop_dt_ms) || !ready_to_drive;

		const soc_kf_debug_t *kf = soc_kf_get_debug();
		current_limit = soc_derate_dcl(kf, bms_dcl, loop_tick, loop_dt_ms);

		torque_request = control_torque_limit(control_torque_map(tps_combined),
				motor_speed, current_limit);

#if REGEN_ENABLE
		const uint8_t speed_stale = (int32_t) (loop_tick - motor_speed_tick)
				> (int32_t) MOTOR_SPEED_TIMEOUT_MS;
		const uint8_t regen_cut = brake_pressed || fault
				|| should_disable_inverter || speed_stale
				|| launch_control_enable;
		torque_request = regen_update(torque_request, motor_speed, tps_combined,
				kf, regen_cut, loop_dt_ms);
#endif

		lc_feed_tick(HAL_GetTick());
		if (launch_control_enable) {
			torque_request = lc_update(torque_request, motor_speed,
					tps_combined);
			lc_was_enabled = 1;
		} else if (lc_was_enabled) {
			lc_init();
			lc_was_enabled = 0;
		}

		if (should_disable_inverter) {
			torque_request = 0;
		}

		control_pack_inverter(TxData, torque_request, inverter_lockout,
				should_disable_inverter, heartbeat_counter);
		can_tx_send_command(CAN_ID_TX_INVERTER_CMD, TxData, 8);
		heartbeat_counter += 1;
		heartbeat_counter = heartbeat_counter & CAN_HEARTBEAT_MASK;

		if (inv_tune_update(loop_tick, inverter_enabled, TxData)) {
			can_tx_send(CAN_ID_TX_INVERTER_PARAM, TxData, 8);
		}

		/* Telemetry gives way so the inverter command is never dropped */
		if (loop_tick - inv_tune_tick >= INV_TUNE_TX_MS
				&& can_tx_depth() < CAN_TX_QUEUE_LEN / 2) {
			uint16_t tune_id;
			const uint8_t tune_dlc = inv_tune_pack_next(&tune_id, TxData);
			if (tune_dlc) {
				can_tx_send(tune_id, TxData, tune_dlc);
			}
			inv_tune_tick = loop_tick;
		}

		if (loop_tick - version_tick >= VERSION_TX_MS) {
			version_pack(TxData);
			can_tx_send(VERSION_CAN_ID, TxData, VERSION_DLC);
			version_tick = loop_tick;
		}

		if (loop_tick - bps_tick >= BPS_TX_MS) {
			control_pack_bps(TxData, control_bps_psi(bps), (uint16_t) bps_adc,
					brake_pressed, bps_oor);
			can_tx_send(CAN_ID_TX_BPS, TxData, CAN_BPS_DLC);
			bps_tick = loop_tick;
		}

		if (loop_tick - pedals_tick >= PEDALS_TX_MS) {
			control_pack_pedals(TxData, &pedals, (uint16_t) tps1_adc,
					(uint16_t) tps2_adc);
			can_tx_send(CAN_ID_TX_PEDALS, TxData, CAN_PEDALS_DLC);
			pedals_tick = loop_tick;
		}

		if (print_ready) {
			const control_status_t status = {
				.rtd_timeout = (uint8_t) (rtd_timeout & 0xFF),
				.bps_adc = bps_adc,
				.tps1_adc = tps1_adc,
				.tps2_adc = tps2_adc,
				.pedals = pedals,
				.bse = bse_error,
				.ready_to_drive = ready_to_drive,
				.disable = should_disable_inverter,
				.inverter_enabled = inverter_enabled,
				.inverter_lockout = inverter_lockout,
			};
			control_pack_debug(TxData, &status);
			can_tx_send(CAN_ID_TX_DEBUG, TxData, 8);

			TxData[0] = (ready_to_drive) & 0x01;
			TxData[1] = reset_cause;

			can_tx_send(CAN_ID_TX_RTD, TxData, 2);

			soc_kf_update(HAL_GetTick());
			{
				uint8_t kfData[8];
				soc_kf_pack_state(kfData);
				kfData[7] = (uint8_t)CAP(loop_dt_max_ms, 255u);
				can_tx_send(SOC_KF_CAN_ID_STATE, kfData, 8);
			}

			soc_derate_pack_debug(TxData);
			can_tx_send(SOC_DERATE_CAN_ID, TxData, 8);

#if REGEN_ENABLE
			regen_pack_debug(TxData);
			can_tx_send(REGEN_DEBUG_CAN_ID, TxData, REGEN_DEBUG_DLC);
#endif

			print_ready = 0;
		}

		/* Only a finished control step feeds the dog */
		HAL_IWDG_Refresh(&hiwdg);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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
	/* Retry a frame that loses arbitration. 0x0C0 loses to every fast
	 * inverter frame, and without this it was dropped instead of sent late.
	 * Still in init mode here, HAL_CAN_Start leaves it. */
	hcan.Init.AutoRetransmission = ENABLE;
	CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_NART);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */
	/* LSI 40 kHz / 32 * 125 = 100 ms, 67-133 ms over LSI tolerance */
	/* Hold the dog while the debugger has the core halted */
	__HAL_DBGMCU_FREEZE_IWDG();
  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 125;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim2.Init.Period = 4999;
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
  /* RTD button (PB14) configured here to persist across CubeMX generation */
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
	__disable_irq();
	/* Inverter enable is a CAN bit, so going quiet lets its command timeout drop it */
	HAL_GPIO_WritePin(PIN_RTD_LIGHT_PORT, PIN_RTD_LIGHT, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PIN_RTD_BUZZER_PORT, PIN_RTD_BUZZER, GPIO_PIN_RESET);
	/* Retry from scratch, reported as RESET_CAUSE_SOFT on the way back up */
	NVIC_SystemReset();
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
