/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  PERI_I2C_SEND              = 0x00U,
  PERI_I2C_SEND_WAIT         = 0x01U,
  PERI_I2C_REC               = 0x02U,
  PERI_I2C_REC_WAIT          = 0x03U,
  PERI_RAND_CALC             = 0x04U,
  PERI_ADC1					 = 0x05U
} PERI_STATE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDR 0x40 << 1
#define I2C_REG_TEMPERATURE  0xE3
#define I2C_REG_HUMIDITY 0xE5
#define I2C_TIME_DELAY 1000

static char str_buf[40];
static int str_len;

static PERI_STATE peri_state;

HAL_StatusTypeDef ret_stat;

uint16_t raw_value;
float tmp_c, bifur_x, bifur_x_next, bifur_r;

uint8_t i2c_sent = I2C_REG_TEMPERATURE;
uint8_t i2c_rcv[4];

#define ADC_LENGTH 64
uint16_t adc_value[ADC_LENGTH];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef CrcCheck(uint16_t value, uint8_t crc)
{
	  uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	  uint32_t msb = 0x800000;
	  uint32_t result = (uint32_t)value << 8; // Pad with zeros as specified in spec

	  while (msb != 0x80) {
	    // Check if msb of current value is 1 and apply XOR mask
		if( result & msb ) result = result ^ polynom;
	    // Shift by one
	    msb >>= 1;
	    polynom >>= 1;
	  }

	  if( result == crc ) return HAL_OK;
	  else return HAL_ERROR;
}

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	hi2c1.State = HAL_I2C_STATE_READY;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
	hi2c1.State = HAL_I2C_STATE_READY;
}

void DMATransferComplete(DMA_HandleTypeDef *hdma)
{
	huart3.Instance->CR3 &= ~USART_CR3_DMAT;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// interrupt when ADC buffer is full, do nothing since it is in circular mode
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6){
		HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
		str_len = 0;
		switch(peri_state){
		case PERI_I2C_SEND:
			  ret_stat = HAL_I2C_Master_Transmit_DMA(&hi2c1, I2C_ADDR, &i2c_sent, 1);
			  if( ret_stat == HAL_OK ) peri_state = PERI_I2C_SEND_WAIT;
			  else{
				  str_len = sprintf((char*)str_buf, ".. I2C DMA Write Fail!\r\n");
				  peri_state = PERI_RAND_CALC;	// failed, will not proceed other I2C stuff
			  }
			break;
		case PERI_I2C_SEND_WAIT:
			if(hi2c1.State != HAL_I2C_STATE_READY){
				str_len = sprintf((char*)str_buf, ".. I2C is not ready!\r\n");
			    peri_state = PERI_RAND_CALC;	// failed, will not proceed other I2C stuff
			}
			else peri_state = PERI_I2C_REC;
			break;
		case PERI_I2C_REC:
			ret_stat = HAL_I2C_Master_Receive_DMA( &hi2c1, I2C_ADDR, i2c_rcv, 4 );
		    if( ret_stat == HAL_OK ) peri_state = PERI_I2C_REC_WAIT;
		    else{
		    	str_len = sprintf((char*)str_buf, ".. I2C DMA Read Fail!\r\n");
		    	peri_state = PERI_RAND_CALC;	// failed, will not proceed other I2C stuff
		    }
			break;
		case PERI_I2C_REC_WAIT:
			if(hi2c1.State != HAL_I2C_STATE_READY){
				str_len = sprintf((char*)str_buf, ".. I2C is not ready!\r\n");
				peri_state = PERI_RAND_CALC;	// failed, will not proceed other I2C stuff
			}
			else{
				peri_state = PERI_RAND_CALC;
				raw_value = ((uint16_t) i2c_rcv[0] << 8) | (uint16_t) i2c_rcv[1];
				if( CrcCheck(raw_value, i2c_rcv[2]) == HAL_OK ){
					tmp_c = (raw_value*175.72/65536.0)-46.85;
					tmp_c *= 100;
					str_len = sprintf(str_buf, "Temperature: %d.%02d C\r\n", ((int)tmp_c/100), ((int)tmp_c%100));
				}
				else{
					str_len = sprintf((char*)str_buf, ".. I2C checksum fail!\r\n");
					peri_state = PERI_RAND_CALC;	// failed, will not proceed other I2C stuff
				}
			}
			break;
		case PERI_RAND_CALC:
			bifur_x_next = bifur_r*bifur_x*(1-bifur_x);

			tmp_c = bifur_x_next;
			tmp_c *= 100;
			str_len = sprintf(str_buf, "Bifurcation: %3d\r\n", (int)tmp_c);

			bifur_x = bifur_x_next;

			peri_state = PERI_ADC1;
			break;
		case PERI_ADC1:
			tmp_c = adc_value[0]/4096.0*3.3;
			tmp_c *= 100;
			str_len = sprintf(str_buf, "Temperature: %d.%02dv\r\n", ((int)tmp_c/100), ((int)tmp_c%100));

			peri_state = PERI_I2C_SEND;
			break;
		default:
			str_len = sprintf((char*)str_buf, ".. ERROR STATE!\r\n");
			break;
		}
		if( str_len > 0 ){
			huart3.Instance->CR3 |= USART_CR3_DMAT;
			HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)str_buf,
					(uint32_t)&huart3.Instance->DR, str_len);
		}

	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_DMA_RegisterCallback(&hdma_usart3_tx, HAL_DMA_XFER_CPLT_CB_ID, &DMATransferComplete);

  HAL_TIM_Base_Start_IT(&htim6);

  bifur_r = 3.86;
  bifur_x = 0.43;

  peri_state = PERI_I2C_SEND;

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, ADC_LENGTH);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_GPIO_TogglePin( GPIOB, LD1_Pin );
	  HAL_Delay(20);

	  // I2C DMA method
/*	  if(hi2c1.State == HAL_I2C_STATE_READY){
		  ret_stat = HAL_I2C_Master_Transmit_DMA(&hi2c1, I2C_ADDR, &i2c_sent, 1);
		  if( ret_stat != HAL_OK ){
			str_len = sprintf((char*)str_buf, "I2C Write Error!\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t*)str_buf, str_len, 100);
		  }
		  else{
			  while(hi2c1.State != HAL_I2C_STATE_READY) {}
			  ret_stat = HAL_I2C_Master_Receive_DMA( &hi2c1, I2C_ADDR, i2c_rcv, 4 );
			  while(hi2c1.State != HAL_I2C_STATE_READY) {}
			  raw_value = ((uint16_t) i2c_rcv[0] << 8) | (uint16_t) i2c_rcv[1];
			  if( CrcCheck(raw_value, i2c_rcv[2]) == HAL_OK ){
				  tmp_c = (raw_value*175.72/65536.0)-46.85;
				  tmp_c *= 100;
				  str_len = sprintf(str_buf, "Temperature: %d.%02d C\r\n", ((int)tmp_c/100), ((int)tmp_c%100));
				  HAL_UART_Transmit(&huart3, (uint8_t*)str_buf, str_len, 100);
			  }
		  }
	  }
	  else{
		  str_len = sprintf((char*)str_buf, "I2C State Busy.\r\n");
		  HAL_UART_Transmit(&huart3, (uint8_t*)str_buf, str_len, 100);

	  }

	  bifur_x_next = bifur_r*bifur_x*(1-bifur_x);

	  tmp_c = bifur_x_next;
	  tmp_c *= 100;
	  str_len = sprintf(str_buf, "Bifurcation: %3d\r\n", (int)tmp_c);
	  huart3.Instance->CR3 |= USART_CR3_DMAT;
	  HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)str_buf,
			  (uint32_t)&huart3.Instance->DR, str_len);

	  bifur_x = bifur_x_next;

	  HAL_Delay(1000);*/
/*
	  // I2C Polling method

	  i2c_sent = I2C_REG_TEMPERATURE;
	  ret_stat = HAL_I2C_Master_Transmit( &hi2c1, I2C_ADDR, &i2c_sent, 1, I2C_TIME_DELAY );
	  if( ret_stat != HAL_OK ){
		str_len = sprintf((char*)str_buf, "I2C Write Error!\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)str_buf, str_len, 100);
	  }
	  else{
		  ret_stat = HAL_I2C_Master_Receive( &hi2c1, I2C_ADDR, i2c_rcv, 4, I2C_TIME_DELAY );
		  if( ret_stat != HAL_OK ){
			  str_len = sprintf((char*)str_buf, "I2C Read Error!\r\n");
			  HAL_UART_Transmit(&huart3, (uint8_t*)str_buf, str_len, I2C_TIME_DELAY );
		  }
		  else{
			  raw_value = ((uint16_t) i2c_rcv[0] << 8) | (uint16_t) i2c_rcv[1];
			  if( CrcCheck(raw_value, i2c_rcv[2]) == HAL_OK ){
				  tmp_c = (raw_value*175.72/65536.0)-46.85;
				  tmp_c *= 100;
				  str_len = sprintf(str_buf, "Temperature: %d.%02d C\r\n", ((int)tmp_c/100), ((int)tmp_c%100));
				  HAL_UART_Transmit(&huart3, (uint8_t*)str_buf, str_len, 100);
			  }
		  }
	  }
	  HAL_Delay(1000);*/
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16800-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
