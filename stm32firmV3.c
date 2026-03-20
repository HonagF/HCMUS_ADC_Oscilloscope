/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <math.h>
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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define SAMPLES_PER_CH    1024   // 512 mẫu mỗi kênh
#define TOTAL_SAMPLES	(SAMPLES_PER_CH * 2)
typedef struct __attribute__((packed)){
	uint8_t header[2]; //0xAA, 0xBB
	uint16_t ch1[SAMPLES_PER_CH]; //1024 bytes
	uint16_t ch2[SAMPLES_PER_CH]; //1024 bytes
	uint8_t padding[2]; //chống Hardfault
	float vpp1, vpp2; //8 bytes
	float freq1, freq2; //8 bytes
	float vavg1, vavg2;
	float vrms1, vrms2;
	float vamp1, vamp2;
	uint16_t trigger_idx; //2 bytes
	// --- KHỐI ĐIỀU KHIỂN (nút bấm, ngoại vi)---
	uint8_t hold_flag; //0: run, 1: hold
	uint8_t ch_mode; // Bitmask -> 1: CH1, 2:CH2, 3:DUAL (0:tắt hết)
	float y_scale1; //Zoom CH1
	float y_scale2; //Zoom CH2
	int16_t y_offset1; //dịch theo trục y CH1
	int16_t y_offset2; //dịch theo trục y CH2
	// --- KHỐI OFFSET X/Y ---
	uint8_t offset_axis; //0: Đang chọn trục y, 1: Đang chọn trục x
	int16_t x_offset1; //dịch theo trục x CH1
	int16_t x_offset2; //dịch theo trục x CH2
	uint8_t reset_flag;

	uint8_t footer[2]; //0xCC, 0xDD
} Packet_t;

Packet_t tx_packet;
#define PACKET_SIZE sizeof(Packet_t) //Tự động tính kích thước (khoảng 2070 bytes)

uint16_t adc_buffer[SAMPLES_PER_CH * 4]; // Buffer Ping-Pong (2048 mẫu)
volatile uint8_t data_ready_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Process_Signal(uint16_t* raw_data, float* vpp, float* freq, float* vavg, float* vrms, float* vamp);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Chốt chân CS ở mức CAO
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Chuẩn bị Header & Footer tĩnh
  tx_packet.header[0] = 0xAA;
  tx_packet.header[1] = 0xBB;
  tx_packet.footer[0] = 0xCC;
  tx_packet.footer[1] = 0xDD;
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  // Tổng số dữ liệu DMA cần lấy là TOTAL_SAMPLES * 2 (cho Ping-Pong)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, TOTAL_SAMPLES * 2);
  HAL_TIM_Base_Start(&htim3);
  uint8_t is_holding = 0;
  uint32_t last_btn_tick = 0;
  tx_packet.y_scale1 = 1.0f;
  tx_packet.y_scale2 = 1.0f;
  tx_packet.y_offset1 = 0;
  tx_packet.y_offset2 = 0;
  tx_packet.offset_axis = 0;
  tx_packet.x_offset1 = 0;
  tx_packet.x_offset2 = 0;


  //Trạng thái cũ của 3 chân CLK (Encoder)
  uint8_t last_clk1 = 1;
  uint8_t last_clk2 = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t current_tick = HAL_GetTick(); // Lấy thời gian hiện tại của hệ thống
	  //--- 1. ĐỌC NÚT NHẤN CHỌN CHANNEL ---
	  // Đọc chân PC4 (CH1). Nút lún xuống (LOW) -> gán bằng 1, nhả ra (HIGH) -> 0
	  uint8_t ch1_en = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET) ? 1:0;
	  // Đọc chân PC5 (CH2). Nút lún xuống (LOW) -> gán bằng 2, nhả ra (HIGH) -> 0
	  uint8_t ch2_en = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) ? 2:0;
	  // Dùng phép toán OR bit: 1|0 = 1 (CH1), 0|2 = 2 (CH2), 1|2 = 3 (Dual)
	  tx_packet.ch_mode = ch1_en | ch2_en;

	  //--- 2. ĐỌC NÚT NHẤN NHẢ
	  //Chỉ nhận data từ nút bấm nếu đã qua 200ms kể từ lần bấm trước đó (chống rung)
	  if(current_tick - last_btn_tick >200){
		  // --- HOLD (PC0) ---
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET){
			  is_holding = !is_holding;
			  tx_packet.hold_flag = is_holding;
			  last_btn_tick = current_tick; //Reset bộ đếm
		  }
		  //--- Autoset (PC1) ---
		  else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET){
			  tx_packet.y_scale1 = 1.0f; //Trả zoom về 1x
			  tx_packet.y_scale2 = 1.0f; //Trả zoom về 1x
			  tx_packet.y_offset1 = 0; //Đưa sóng về giữa
			  tx_packet.y_offset2 = 0; //Đưa sóng về giữa
			  tx_packet.x_offset1 = 0; //Đưa sóng về giữa
			  tx_packet.x_offset2 = 0; //Đưa sóng về giữa
			  tx_packet.offset_axis = 0;
			  tx_packet.reset_flag = 1;
			  last_btn_tick = current_tick; //Reset bộ đếm
		  }

		  //--- Chọn trục x/y (PC2) ---
		  else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET){
			  tx_packet.offset_axis = !tx_packet.offset_axis;
			  last_btn_tick = current_tick; //Reset bộ đếm
		  }
	  }
	  //--- 3. Đọc encoder --- (không chờ, đọc liên tục)

	  //ENC1: Zoom sóng
	  uint8_t clk1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	  if (clk1==0 && last_clk1 == 1){
		  float delta = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) != clk1) ? 0.2f : -0.2f;
		  if(ch1_en) {
			  tx_packet.y_scale1 += delta;
			  if (tx_packet.y_scale1 < 0.2f) tx_packet.y_scale1 = 0.2f;
			  if (tx_packet.y_scale1 > 5.0f) tx_packet.y_scale1 = 5.0f;
		  }
		  if(ch2_en) {
			  tx_packet.y_scale2 += delta;
			  if (tx_packet.y_scale2 < 0.2f) tx_packet.y_scale2 = 0.2f;
			  if (tx_packet.y_scale2 > 5.0f) tx_packet.y_scale2 = 5.0f;
		  }
	  }
	  last_clk1 = clk1;
	  //ENC2: x/y offset
	  	  uint8_t clk2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	  	  if (clk2==0 && last_clk2 == 1){
	  		  int delta_y = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) != clk2) ? -10 : 10;
	  		  int delta_x = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) != clk2) ? -5 : 5;

	            if (tx_packet.offset_axis == 0) {
	                // --- MODE 0: DỊCH Y ---
	                if(ch1_en) {
	                    tx_packet.y_offset1 += delta_y;
	                    if (tx_packet.y_offset1 > 200) tx_packet.y_offset1 = 200;
	                    if (tx_packet.y_offset1 < -200) tx_packet.y_offset1 = -200;
	                }
	                if (ch2_en) {
	                    tx_packet.y_offset2 += delta_y;
	                    if (tx_packet.y_offset2 > 200) tx_packet.y_offset2 = 200;
	                    if (tx_packet.y_offset2 < -200) tx_packet.y_offset2 = -200;
	                }
	            } else {
	                // --- MODE 1: DỊCH X ---
	                if(ch1_en) {
	                    tx_packet.x_offset1 += delta_x;
	                    if (tx_packet.x_offset1 > 250) tx_packet.x_offset1 = 250;
	                    if (tx_packet.x_offset1 < -250) tx_packet.x_offset1 = -250;
	                }
	                if (ch2_en) {
	                    tx_packet.x_offset2 += delta_x;
	                    if (tx_packet.x_offset2 > 250) tx_packet.x_offset2 = 250;
	                    if (tx_packet.x_offset2 < -250) tx_packet.x_offset2 = -250;
	                }
	            }
	  	  }
	  	  last_clk2 = clk2;



	  // --- 4. Xử lý ADC và truyền packet qua SPI ---
      if (data_ready_flag != 0) {
          // Chỉ gửi khi đường SPI đang rảnh
    	  if (data_ready_flag != 0) {

    	            // BƯỚC 1: CHỈ CẬP NHẬT DỮ LIỆU SÓNG KHI KHÔNG BỊ "HOLD"
    	            if (is_holding == 0) {
    	                uint16_t* current_buffer = (data_ready_flag == 1) ? &adc_buffer[0] : &adc_buffer[SAMPLES_PER_CH*2];

    	                // Tách dữ liệu CH1 và CH2 xen kẽ
    	                for(int i = 0; i < SAMPLES_PER_CH; i++) {
    	                    tx_packet.ch1[i] = current_buffer[i*2];
    	                    tx_packet.ch2[i] = current_buffer[i*2+1];
    	                }

    	                // Tính toán các thông số và tìm Trigger

    	                if (tx_packet.ch_mode == 2){
        	                tx_packet.trigger_idx = Process_Signal(tx_packet.ch2, &tx_packet.vpp2, &tx_packet.freq2, &tx_packet.vavg2, &tx_packet.vrms2, &tx_packet.vamp2);
    	                }
    	                else {
        	                tx_packet.trigger_idx = Process_Signal(tx_packet.ch1, &tx_packet.vpp1, &tx_packet.freq1, &tx_packet.vavg1, &tx_packet.vrms1, &tx_packet.vamp1);
        	                Process_Signal(tx_packet.ch2, &tx_packet.vpp2, &tx_packet.freq2, &tx_packet.vavg2, &tx_packet.vrms2, &tx_packet.vamp2);

    	                }

    	            }

    	            // BƯỚC 2: XÓA CỜ DMA (Luôn thực hiện dù có Hold hay không)
    	            data_ready_flag = 0;

    	            // BƯỚC 3: BẮN GÓI TIN SANG ESP32
    	            // Phải đặt ngoài"if (is_holding == 0)" để dù sóng có đứng im,
    	            // STM32 vẫn liên tục gửi các cờ hiệu (Cursor, CH Mode, Hold) sang cho màn hình.
    	            if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) {
    	                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Kéo CS xuống LOW
    	                HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&tx_packet, PACKET_SIZE);
    	                tx_packet.reset_flag = 0;

    	            }
    	        }
      }
      HAL_Delay(5);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC4
                           PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin PA6 */
  GPIO_InitStruct.Pin = SPI_CS_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Ngắt khi đầy nửa buffer (Ping)
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        data_ready_flag = 1;
    }
}

// Ngắt khi đầy toàn bộ buffer (Pong)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        data_ready_flag = 2;
    }
}

// Ngắt khi truyền xong SPI để kéo chân CS lên
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if(hspi->Instance == SPI1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
}


uint16_t Process_Signal(uint16_t* raw_data, float* vpp, float* freq, float* vavg, float* vrms, float* vamp) {
    uint16_t max = 0, min = 4095;
    uint32_t sum = 0;
    uint64_t sum_sq =0;
    // 1. Tìm Max, Min, tổng và tổng bình phương
    for(int i=0;i<SAMPLES_PER_CH;i++){
    	uint16_t val = raw_data[i];
    	if(val>max) max = val;
    	if(val<min) min = val;
    	sum+=val;
    	sum_sq += (uint32_t)val *val; //tổng bình phương
    }
    //Tính toán các giá trị điện áp
    float adc_to_volt = 3.3f/4095.0f;

    *vpp = (float)(max - min)*adc_to_volt;
    *vamp = *vpp/2.0f;

    float avg_raw = (float)sum/SAMPLES_PER_CH;
    *vavg = avg_raw * adc_to_volt;

    float rms_raw = sqrtf((float)sum_sq/SAMPLES_PER_CH);
    *vrms = rms_raw * adc_to_volt;

    int16_t trigger_level = 2048;
    int16_t hysteresis = 40; //đệm để lọc nhiễu

    // 2. Tính Tần số (Zero-Crossing)

    // Tìm trigger rising edge
    // Tìm từ mẫu 400
    for(int i=400; i<SAMPLES_PER_CH - 400; i++) {
        if(raw_data[i-2] < (trigger_level - hysteresis) && raw_data[i] > (trigger_level + hysteresis)) {
        	return (uint16_t)i;
        }
        return 400;
    }
    int first_cross = -1, second_cross = -1;
    if(second_cross != -1) {
        // Tần số lấy mẫu 100kHz = 100,000 Hz
        *freq = 100000.0f / (float)(second_cross - first_cross);
    } else {
        *freq = 0;
    }
    return (uint16_t)first_cross;
}
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
#ifdef USE_FULL_ASSERT
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
