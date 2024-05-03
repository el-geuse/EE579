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
#include "sensors.h"
#include "motortest.h"
#include <stdlib.h>

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

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern uint16_t clear, red, green, blue, apdsDistance;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

// Possible states for the car to be in.
enum MACHINE_STATES {
	  MOVESIX,
	  READJUST_SIX,
	  SEARCH,
	  FAR_APPROACH,
	  CLOSE_APPROACH,
	  COLOURCHECK,
	  SHIMMY,
	  TEST,
};

enum DRIVE_DIRECTION {
	  STOP,
	  FORWARDS,
	  SLOW_FORWARDS,
	  BACKWARDS,
	  SLOW_BACKWARDS,
};

enum WHEEL_DIRECTION {
	  STRAIGHT,
	  LEFT,
	  RIGHT,
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  sensorsAdcInit(&hadc1);
  sensorsI2CInit(&hi2c2);
  motorTimInit(&htim3);
  smackInit(&htim4);
  initAPDS(&hi2c2);
  stopMotor();

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); //PA12 PA12 PA12 PA12
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  uint16_t lidarDistance, irLeftDistance, irRightDistance, prevLidarDistance = 0;

  // Preference of which wall to prefer assuming things go wrong.
  enum WALL_STATE{
	L,
	R,
  };

  enum WALL_STATE wall = L;
  enum WALL_STATE snaking_flag = L;

  enum MACHINE_STATES state = TEST;

  uint32_t start_time = HAL_GetTick();
  uint32_t adjust_start_time;
  // Assuming 6 seconds is needed to travel the 6m.
  uint32_t duration_ms = 5000; // 6 seconds
  uint32_t adjust_duration_ms = 200;

  int hold_flag = 0;
  int left_ir_flag, right_ir_flag;


  enum DRIVE_DIRECTION prev_move = STOP;
  enum WHEEL_DIRECTION prev_turn = STRAIGHT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  calibrate();
//  TIM3->CCR4 = percentageToTIM3(100);

  while (1) {
	  // Beginning control loop
	  switch (state) {



	  // Moving 6 metres to approach the starting spot.
	  case MOVESIX:
		  // Checking if 6m worth of time has elapsed.
		  if ((HAL_GetTick() - start_time) > duration_ms) {
			  // Move to next state
			  state = SEARCH;
			  break;
		  }

		  lidarDistance = getLidarDistance();
		  // Ensuring:
		  // 1. There's more than 1s remaining, making sure the lidar doesn't pick up the skittle.
		  // 2. The lidar sees something within 50cm, presumably (and hopefully) a wall.
		  if (((HAL_GetTick() - start_time) < (duration_ms - 1000)) && lidarDistance < 50) {
			  // Adjust
			  state = READJUST_SIX;
			  adjust_start_time = HAL_GetTick();

			  move(SLOW_FORWARDS, &prev_move);
			  break;
		  }

		  // If nothing wrong, ensure moving forwards
		  move(FORWARDS, &prev_move);

		  break;



	  // Readjusting the initial 6m approach.
	  case READJUST_SIX:

		  // Ternary statements stop the IR sensors from reporting 0, and change those 0s to 999.
		  // The IR sensors still seem to fire off random readings at further distances, implementing some
		  // sort of averaging would be good.
		  // LEFT AND RIGHT ARE THE WRONG WAY ROUND.
		  irLeftDistance = getIrLeftDistance();
		  irRightDistance = getIrRightDistance();

		  if (irLeftDistance > irRightDistance) {
			  turn(RIGHT, &prev_turn);
		  }

		  else if (irLeftDistance < irRightDistance){
			  turn(LEFT, &prev_turn);
		  }

		  else{
			  // Something is up / IRs not in range
			  turn(STRAIGHT, &prev_turn);
		  }

		  while ((HAL_GetTick() - adjust_start_time) < adjust_duration_ms) {}

		  // Update forward duration with the readjustment
		  // duration_ms = duration_ms + adjust_duration_ms;

		  turn(STRAIGHT, &prev_turn);
		  state = MOVESIX;
		  break;



	  // Looking for the skittles. Following a curve to hopefully find them.
	  case SEARCH:
		  lidarDistance = getLidarDistance();

		  if (lidarDistance < 100) {
			  state = FAR_APPROACH;
			  break;
		  }

		  // if nothing in view (taking into account the side we're on)
		  if (wall == L) {
			  turn(RIGHT, &prev_turn);
			  move(SLOW_FORWARDS, &prev_move);
		  } else {
			  turn(LEFT, &prev_turn);
			  move(SLOW_FORWARDS, &prev_move);
		  }

		  break;



	  // Basic method to approach the skittle.
	  // Potential upgrades needed to ensure car doesn't veer off.
	  case FAR_APPROACH:
		  lidarDistance = getLidarDistance();

		  // When the skittle is close enough.
		  if (lidarDistance < 40) {
			  state = CLOSE_APPROACH;
			  turn(STRAIGHT, &prev_turn);
			  move(STOP, &prev_move);

			  // Resetting state.
			  hold_flag = 0;
			  prevLidarDistance = 0;
			  break;
		  }

		  // Ensuring no break
		  if (prevLidarDistance == 0) {prevLidarDistance = lidarDistance;}

		  // A short check to ensure the object hasn't been lost / things move too fast.
		  if (lidarDistance < prevLidarDistance) {prevLidarDistance = lidarDistance;}

		  // Starting snaking
		  if (snaking_flag == L){
			  // Checking to see if the snaking has passed the left side of skittle.
			  if (abs(prevLidarDistance - lidarDistance) > 15) {
				  if (!hold_flag) {
					  // Car has passed the skittle.
					  hold_flag = 1;
					  snaking_flag = R;
//					  TIM3->CCR4 = percentageToTIM3(0);
				  }
			  } else {
				  if (hold_flag) {
					  // Car is back on track
					  hold_flag = 0;
//					  TIM3->CCR4 = percentageToTIM3(100);
				  }
				  // Update previous distance only if not holding.
				  // This keeps a memory of the last skittle location
				  prevLidarDistance = hold_flag ? prevLidarDistance : lidarDistance;
			  }

			  turn(LEFT, &prev_turn);
			  move(SLOW_FORWARDS, &prev_move);

		  } else {
			  // Checking to see if the snaking has passed the left side of skittle.
			  if (abs(prevLidarDistance - lidarDistance) > 15) {
				  if (!hold_flag) {
					  // Car has passed the skittle.
					  hold_flag = 1;
					  snaking_flag = L;
//					  TIM3->CCR4 = percentageToTIM3(0);
				  }
			  } else {
				  if (hold_flag) {
					  // Car is back on track
					  hold_flag = 0;
//					  TIM3->CCR4 = percentageToTIM3(100);
				  }
				  // Update previous distance only if not holding.
				  // This keeps a memory of the last skittle location
				  prevLidarDistance = hold_flag ? prevLidarDistance : lidarDistance;
			  }

			  turn(RIGHT, &prev_turn);
			  move(SLOW_FORWARDS, &prev_move);
		  }

		  break;



	  // Approach using the ir sensors to help out as well.
	  // REMINDER LEFT AND RIGHT ARE WRONG WAY ROUND.
	  case CLOSE_APPROACH:

		  lidarDistance = getLidarDistance();

		  // When the skittle is close enough.
		  if (lidarDistance < 10) {
			  refreshAPDSData();
			  if (apdsDistance > 8){
				  state = TEST;
				  turn(STRAIGHT, &prev_turn);
				  move(STOP, &prev_move);
				  break;
			  }
		  }

		  move(SLOW_FORWARDS, &prev_move);

		  irLeftDistance = getIrLeftDistance();
		  irRightDistance = getIrRightDistance();

		  left_ir_flag = (irLeftDistance < 100) ? 1 : 0;
		  right_ir_flag = (irRightDistance  < 100) ? 1 : 0;

		  // remember they are opposite.
		  if (left_ir_flag && !right_ir_flag) {turn(RIGHT, &prev_turn);TIM3->CCR4 = percentageToTIM3(100);}
		  if (!left_ir_flag && right_ir_flag) {turn(LEFT, &prev_turn);TIM3->CCR4 = percentageToTIM3(0);}
		  if (!left_ir_flag && !right_ir_flag) {turn(STRAIGHT, &prev_turn);}
		  if (left_ir_flag && right_ir_flag) {turn(STRAIGHT, &prev_turn);}
	  	  break;



	  // Checking colour of object in front of car and moving to appropriate state.
	  // No method of verifying there is something in front of the car (except maybe with the apds?)
	  case COLOURCHECK:
		  TIM3->CCR4 = percentageToTIM3(100);
		  HAL_Delay(200);

		  // Check colour data
		  refreshAPDSData();

		  if ((red < 200) && (green < 100) && (blue < 100)) {smack();}

		  TIM3->CCR4 = percentageToTIM3(0);
		  state = SHIMMY;

		  break;



	  // A three-point-turn manoeuvre that hopefully gets the car out of stuck spots /.
	  // away from white skittles.
	  case SHIMMY:

		  if (wall == L) {
			  // Means the right wall has just been hit
			  turn(RIGHT, &prev_turn);
			  HAL_Delay(300);
			  move(SLOW_BACKWARDS, &prev_move);
		  } else {
			  // Same but for left wall
			  turn(LEFT, &prev_turn);
			  HAL_Delay(300);
			  move(SLOW_BACKWARDS, &prev_move);
		  }

		  HAL_Delay(3000);

		  move(STOP, &prev_move);
		  turn(STRAIGHT, &prev_turn);

		  HAL_Delay(1000);
		  // Switching sides
		  wall = 1 - wall;
		  state = SEARCH;
		  break;



	  case TEST:
		  move(SLOW_FORWARDS, &prev_move);
		  break;



	  }
  }
//	  lidarDistance = getLidarDistance();
//	  irLeftDistance = getIrLeftDistance();
//	  irRightDistance = getIrRightDistance();
//
//	  refreshAPDSData();
//	  nearDistance = apdsDistance;
//	  TIM3->CCR4 = percentageToTIM3(distanceToPercentage(lidarDistance));
//
//	  //HAL_Delay(5000);  // Delay for 5 second
//	  //calibrate();
//	  //HAL_Delay(5000);  // Delay for 5 second
//	  //straighten();
//
//	  if (lidarDistance < 30)
//	  {
//		  HAL_Delay(5000);
//		  smack();
//	  }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// Checks previous and issued command, to not have constant sending of turning commands.
void turn(enum WHEEL_DIRECTION turn_dir, enum WHEEL_DIRECTION *prev_turn) {
    if (*prev_turn != turn_dir) {
        *prev_turn = turn_dir;

        switch (turn_dir) {
        case LEFT:
        	turnLeft();
        	break;
        case RIGHT:
        	turnRight();
        	break;
        case STRAIGHT:
        	straighten();
        	break;
        }
    }
}

// As above but for movement.
void move(enum DRIVE_DIRECTION move_dir, enum DRIVE_DIRECTION *prev_move) {
	if (*prev_move != move_dir) {
		*prev_move = move_dir;
		switch(move_dir) {
		case STOP:
			stopMotor();
			break;
		case FORWARDS:
			moveForwards(55);
			break;
		case SLOW_FORWARDS:
			moveForwards(30);
			break;
		case BACKWARDS:
			moveBackwards(55);
			break;
		case SLOW_BACKWARDS:
			moveBackwards(45);
			break;
		}
	}
}
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 2;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A0A7FB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 169;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 169;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
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
  while (1)
  {
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
