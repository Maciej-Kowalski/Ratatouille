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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "bmi160_wrapper.h"

#include "BMI270_SensorAPI-master/bmi270.h"
#include "BMI270_SensorAPI-master/common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
BMI160_t imu_t;
int8_t countertemp = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*! Macros to select the sensors                   */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* Analog */
uint16_t mic = 0;
uint16_t mic2 = 0;
uint16_t rawValues[2];
/* Analog END */

/* USB */
uint8_t message[] = "USB VCP Test\n";
uint8_t flag = 0;
char msg[50];

uint16_t a = 7;
uint16_t b = 9;
uint8_t USB_buffer[32];
size_t packet_length;
/* USB END */

/* Mouse */
int delay = 5;
int8_t x = 0;
int8_t y = 0;
/* Mouse END */

/* IMU */
float g_f32[3] = {0.0f, 0.0f, 0.0f};
float a_f32[3] = {0.0f, 0.0f, 0.0f};
float g = 9.81f;
/* IMU END */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void prepare_data_packet_both(float g_f32[3], float a_f32[3], uint16_t var1, uint16_t var2, uint8_t *buffer, size_t *length);
void prepare_data_packet_IMU(float g_f32[3], float a_f32[3], uint8_t *buffer, size_t *length);

/*unused/legacy */
void send_packet_joystick(int8_t x, int8_t y);
void move_square(int delay);

static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2)
  {
	////this updates the X and Y axes of my joystick
	  for(uint8_t i = 0; i<hadc1.Init.NbrOfConversion; i++){
		  mic = (uint16_t) rawValues[0];
		  mic2 = mic;
	  }
	  flag = 1;
	 ////This allows me to write to the serial monitor similar to the serial monitor on Arduino
	 ////Note STM32CUBEIDE does not have a serial monitor. You will need to download PuTTY to view
	 ////your serial output.
	  /*HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
	  sprintf(msg, "X axis: %hu, Y axis: %hu \r\n", Xaxis, Yaxis);
	  status = CDC_Transmit_FS(msg, strlen((char*)msg));
	  if (status == USBD_OK) {
	  	  HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin); // Example: Toggle LED on success
	  }*/
  }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int8_t rslt;
	uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
	struct bmi2_dev bmi;
	struct bmi2_sens_data sensor_data = { { 0 } };
	float acc_x = 0, acc_y = 0, acc_z = 0;
	float gyr_x = 0, gyr_y = 0, gyr_z = 0;
	struct bmi2_sens_config config;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USB_Device_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ////This begins the process of storing our ADC readings into the DMA. The DMA can be thought of a temporary storage location.
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 2);
    ////This begins our timer 2
  HAL_TIM_Base_Start_IT(&htim2);

//  while (BMI160_init(imu_t) == 1); // waits for IMU to be ready
//
//  if (imu_t.INIT_OK_i8 != TRUE){
//	  //HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, SET);
//  }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW3, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*                                                                          ********************************************************************* */
  printf("Before init");
  while (BMI160_init(imu_t) == 1); // waits for IMU to be ready
  rslt = bmi2_interface_init(&bmi, BMI2_I2C_INTF);
  bmi2_error_codes_print_result(rslt);
  rslt = bmi270_init(&bmi);
  bmi2_error_codes_print_result(rslt);
  rslt = set_accel_gyro_config(&bmi);
  bmi2_error_codes_print_result(rslt);
  rslt = bmi2_sensor_enable(sensor_list, 2, &bmi);
  bmi2_error_codes_print_result(rslt);
  config.type = BMI2_ACCEL;

                  /* Get the accel configurations. */
                  rslt = bmi2_get_sensor_config(&config, 1, &bmi);
                  bmi2_error_codes_print_result(rslt);

                  printf(
                      "\nData set, Accel Range, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyro_DPS_X, Gyro_DPS_Y, Gyro_DPS_Z\n\n");
  printf("After init");

  if (imu_t.INIT_OK_i8 != TRUE){
	  printf("Stuck");
	  //HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, SET);
  }
  printf("Now entering while...");
  while (1)
  {
	  bmi160ReadAccelGyro(&imu_t);
	  //printf("Whileing!");
	  rslt = bmi2_get_sensor_data(&sensor_data, &bmi);
	  bmi2_error_codes_print_result(rslt);
//	  printf("After get sensor...");
	  if (flag==1){
		  /*a_f32[0] = 1;
		  a_f32[1] = 2;
		  a_f32[2] = 3;

		  g_f32[0] = 4;
		  g_f32[1] = 5;
		  g_f32[2] = 6;*/
		  // Normal (old) orientation
//		  a_f32[0] = imu_t.BMI160_Ax_f32*g;
//		  a_f32[1] = imu_t.BMI160_Ay_f32*g;
//		  a_f32[2] = imu_t.BMI160_Az_f32*g;
//
//		  g_f32[0] = imu_t.BMI160_Gx_f32*0.0174533;
//		  g_f32[1] = imu_t.BMI160_Gy_f32*0.0174533;
//		  g_f32[2] = imu_t.BMI160_Gz_f32*0.0174533;

//		  // head orientation
//		  a_f32[1] = imu_t.BMI160_Ax_f32*g;
//		  a_f32[2] = imu_t.BMI160_Ay_f32*g;
//		  a_f32[0] = imu_t.BMI160_Az_f32*g;
//
//		  g_f32[1] = imu_t.BMI160_Gx_f32*0.0174533;
//		  g_f32[2] = imu_t.BMI160_Gy_f32*0.0174533;
//		  g_f32[0] = imu_t.BMI160_Gz_f32*0.0174533;
//
//		  prepare_data_packet_IMU(g_f32,a_f32,USB_buffer,&packet_length);
//		  CDC_Transmit_FS(USB_buffer, packet_length);
//		  BSP_LED_Toggle(LED_GREEN);
		  //HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		  //snprintf(msg, sizeof(msg), "a: %.2f, g: %.2f", a_f32[0], g_f32[0]);
		  //CDC_Transmit_FS((uint8_t *)msg, strlen(msg));

		  // BMI270
		  /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
		  a_f32[1] = lsb_to_mps2(sensor_data.acc.x, (float)2, bmi.resolution);
		  a_f32[2] = lsb_to_mps2(sensor_data.acc.y, (float)2, bmi.resolution);
		  a_f32[0] = lsb_to_mps2(sensor_data.acc.z, (float)2, bmi.resolution);

		  /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
		  g_f32[1] = lsb_to_dps(sensor_data.gyr.x, (float)2000, bmi.resolution)*0.0174533;
		  g_f32[2] = lsb_to_dps(sensor_data.gyr.y, (float)2000, bmi.resolution)*0.0174533;
		  g_f32[0] = lsb_to_dps(sensor_data.gyr.z, (float)2000, bmi.resolution)*0.0174533;

		  prepare_data_packet_IMU(g_f32,a_f32,USB_buffer,&packet_length);
		  		  CDC_Transmit_FS(USB_buffer, packet_length);
		  		  BSP_LED_Toggle(LED_GREEN);
//
//		  printf("%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f\r\n",
//				 acc_x,
//				 acc_y,
//				 acc_z,
//				 gyr_x,
//				 gyr_y,
//				 gyr_z);
//
//		  countertemp++;
//		  printf("%d\r\n",countertemp);


		  flag = 0;
	  }

	  /*HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
	  status = CDC_Transmit_FS(message, strlen((char*)message));
	  if (status == USBD_OK) {
	      HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin); // Example: Toggle LED on success
	  }
	  HAL_Delay(delay);*/


  /*                                                                          ********************************************************************* */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLLSAI1.PLLN = 6;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_USBCLK|RCC_PLLSAI1_ADCCLK;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 10;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void prepare_data_packet_both(float g_f32[3], float a_f32[3], uint16_t var1, uint16_t var2, uint8_t *buffer, size_t *length) {
    // Define the start byte
    buffer[0] = (uint8_t)-128;

    memcpy(&buffer[1], g_f32, sizeof(float) * 3);
    memcpy(&buffer[1 + sizeof(float) * 3], a_f32, sizeof(float) * 3);
    memcpy(&buffer[1 + 2 * sizeof(float) * 3], &var1, sizeof(uint16_t));
    memcpy(&buffer[1 + 2 * sizeof(float) * 3 + sizeof(uint16_t)], &var2, sizeof(uint16_t));
    // Copy the int8_t variables into the buffer
    /*buffer[1 + 2 * sizeof(float) * 3 + sizeof(uint16_t)] = (uint16_t)var1;
    buffer[1 + 2 * sizeof(float) * 3 + sizeof(uint16_t)] = (uint16_t)var2;*/

    // Set the total length of the packet
    *length = 1 + 2 * sizeof(float) * 3 + 2*sizeof(uint16_t);
}
void prepare_data_packet_IMU(float g_f32[3], float a_f32[3], uint8_t *buffer, size_t *length) {
    // Define the start byte
    buffer[0] = (uint8_t)-128;
    memcpy(&buffer[1], g_f32, sizeof(float) * 3);
    memcpy(&buffer[1 + sizeof(float) * 3], a_f32, sizeof(float) * 3);

    // Set the total length of the packet
    *length = 1 + 2 * sizeof(float) * 3;
}
void send_packet_joystick(int8_t value1, int8_t value2) {
    uint8_t packet[3];
    packet[0] = (uint8_t)0x80;             // Start byte (-128 or 0x80 in unsigned)
    packet[1] = (uint8_t)value1;           // First data byte (cast int8_t to uint8_t)
    packet[2] = (uint8_t)value2;           // Second data byte (cast int8_t to uint8_t)
    //packet[3] = (uint8_t)(value1 + value2); // Checksum (optional)

    // Send the packet over USB VCP
    CDC_Transmit_FS(packet, sizeof(packet));
}
void move_square(int delay){
	x = 0;
	y = 15;
	send_packet_joystick(x,y);
	HAL_Delay(delay);

	x = 15;
	y = 0;
	send_packet_joystick(x,y);
	HAL_Delay(delay);

	x = 0;
	y = -15;
	send_packet_joystick(x,y);
	HAL_Delay(delay);

	x = -15;
	y = 0;
	send_packet_joystick(x,y);
	HAL_Delay(delay);
}

static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_set_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_800HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi270_set_sensor_config(config, 2, bmi);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

//void bmi2_error_codes_print_result(int8_t rslt)
//{
//    switch (rslt)
//    {
//        case BMI2_OK:
//
//            /* Do nothing */
//            break;
//
//        case BMI2_W_FIFO_EMPTY:
//            printf("Warning [%d] : FIFO empty\r\n", rslt);
//            break;
//        case BMI2_W_PARTIAL_READ:
//            printf("Warning [%d] : FIFO partial read\r\n", rslt);
//            break;
//        case BMI2_E_NULL_PTR:
//            printf(
//                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_COM_FAIL:
//            printf(
//                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_DEV_NOT_FOUND:
//            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
//                   rslt);
//            break;
//
//        case BMI2_E_INVALID_SENSOR:
//            printf(
//                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_SELF_TEST_FAIL:
//            printf(
//                "Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is " "not satisfied\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_INVALID_INT_PIN:
//            printf(
//                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_OUT_OF_RANGE:
//            printf(
//                "Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from " "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_ACC_INVALID_CFG:
//            printf(
//                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x40\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_GYRO_INVALID_CFG:
//            printf(
//                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x42\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_ACC_GYR_INVALID_CFG:
//            printf(
//                "Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro" " configuration registers which could be one among range, BW or filter performance in reg address 0x40 " "and 0x42\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_CONFIG_LOAD:
//            printf(
//                "Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration " "into the sensor\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_INVALID_PAGE:
//            printf(
//                "Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration " "from selected page\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_SET_APS_FAIL:
//            printf(
//                "Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration " "register\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_AUX_INVALID_CFG:
//            printf(
//                "Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not " "enabled properly\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_AUX_BUSY:
//            printf(
//                "Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring" " the AUX\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_REMAP_ERROR:
//            printf(
//                "Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes " "after change in axis position\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
//            printf(
//                "Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status " "fails\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_SELF_TEST_NOT_DONE:
//            printf(
//                "Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not " "completed\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_INVALID_INPUT:
//            printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
//            break;
//
//        case BMI2_E_INVALID_STATUS:
//            printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
//            break;
//
//        case BMI2_E_CRT_ERROR:
//            printf("Error [%d] : CRT error. It occurs when the CRT test has failed\r\n", rslt);
//            break;
//
//        case BMI2_E_ST_ALREADY_RUNNING:
//            printf(
//                "Error [%d] : Self-test already running error. It occurs when the self-test is already running and " "another has been initiated\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
//            printf(
//                "Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong " "address location\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_DL_ERROR:
//            printf(
//                "Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_PRECON_ERROR:
//            printf(
//                "Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not " "completed\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_ABORT_ERROR:
//            printf("Error [%d] : Abort error. It occurs when the device was shaken during CRT test\r\n", rslt);
//            break;
//
//        case BMI2_E_WRITE_CYCLE_ONGOING:
//            printf(
//                "Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another " "has been initiated\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_ST_NOT_RUNING:
//            printf(
//                "Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's " "running\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_DATA_RDY_INT_FAILED:
//            printf(
//                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
//                rslt);
//            break;
//
//        case BMI2_E_INVALID_FOC_POSITION:
//            printf(
//                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
//                rslt);
//            break;
//
//        default:
//            printf("Error [%d] : Unknown error code\r\n", rslt);
//            break;
//    }
//}
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
