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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Set số lượng mẫu thu thập cho mỗi khung hình là 1024 mẫu/kênh
#define SAMPLES_PER_CH    1024
// Tổng số mẫu DMA cần lấy = Kênh 1 + Kênh 2
#define TOTAL_SAMPLES	(SAMPLES_PER_CH * 2)

// Khai báo struct đóng gói dữ liệu truyền qua SPI.
// dùng __attribute__((packed)) để vô hiệu hóa tính năng tự động đệm (padding) của GCC.
typedef struct __attribute__((packed)){
	uint8_t header[2]; // Byte header: 0xAA, 0xBB để ESP32 nhận diện điểm bắt đầu gói tin
	uint16_t ch1[SAMPLES_PER_CH]; // Dữ liệu thô ADC Kênh 1 (1024 mẫu * 2 byte = 2048 bytes)
	uint16_t ch2[SAMPLES_PER_CH]; // Dữ liệu thô ADC Kênh 2 (1024 mẫu * 2 byte = 2048 bytes)
	uint8_t padding[2]; // 2 byte padding thủ công để chống lỗi Hardfault do Misaligned memory access trên vi điều khiển ARM

	// Các thông số điện áp & thời gian đo được
	float vpp1, vpp2; // Điện áp đỉnh-đỉnh (Peak-to-Peak)
	float freq1, freq2; // Tần số (Frequency)
	float vavg1, vavg2; // Điện áp trung bình (Average/DC Offset)
	float vrms1, vrms2; // Điện áp hiệu dụng thực (True RMS)
	float vamp1, vamp2; // Biên độ sóng (Amplitude)
	uint16_t trigger_idx1; // Vị trí điểm kích hoạt (Trigger Point) trong mảng để ESP32 làm mốc vẽ sóng
	uint16_t trigger_idx2;

	// --- KHỐI ĐIỀU KHIỂN (Giao tiếp với giao diện LCD) ---
	uint8_t hold_flag; // Cờ dừng màn hình (0: Đang chạy, 1: Giữ nguyên khung hình)
	uint8_t ch_mode;   // Cờ bitmask chọn kênh hiển thị (1: CH1, 2: CH2, 3: Bật cả 2 kênh)
	float y_scale1;    // Hệ số phóng to/thu nhỏ trục Y của CH1 (điều khiển bởi Encoder 1)
	float y_scale2;    // Hệ số phóng to/thu nhỏ trục Y của CH2 (điều khiển bởi Encoder 1)
	float x_scale1;	   // Hệ số thu phóng trục X của CH1
	float x_scale2;    // Hệ số thu phóng trục Y của CH2
	int16_t y_offset1; // Số pixel dịch chuyển sóng lên/xuống của CH1
	int16_t y_offset2; // Số pixel dịch chuyển sóng lên/xuống của CH2

	// --- KHỐI OFFSET X/Y ---
	uint8_t offset_axis; // Trạng thái của nút nhấn: 0 đang chọn chỉnh Y, 1 đang chọn chỉnh X
	int16_t x_offset1; // Số pixel dịch chuyển trục thời gian (X) cho CH1
	int16_t x_offset2; // Số pixel dịch chuyển trục thời gian (X) cho CH2
	uint8_t reset_flag; // Cờ reset

	uint8_t footer[2]; // Byte footer: 0xCC, 0xDD
} Packet_t;

Packet_t tx_packet; // Khởi tạo biến toàn cục chứa gói tin
#define PACKET_SIZE sizeof(Packet_t)

// Mảng đệm Ping-Pong cho ADC DMA. Kích thước = Số mẫu * 2 kênh * 2 (Ping và Pong). Tổng = 4096 phần tử.
// Sử dụng Ping-Pong buffer giúp CPU có thể xử lý nửa mảng này trong khi phần cứng DMA đang tự động điền dữ liệu vào nửa mảng kia, đảm bảo Real-time.
volatile uint8_t adc1_ready = 0;
volatile uint8_t adc2_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
// Hàm xử lý tín hiệu: Tìm Trigger, tính Vrms, Vpp, Freq
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  // Khởi tạo ban đầu: Chốt chân Chip Select (CS) của SPI ở mức CAO (Không truyền)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Set cứng các byte Header và Footer để định dạng gói tin giao thức
  tx_packet.header[0] = 0xAA;
  tx_packet.header[1] = 0xBB;
  tx_packet.footer[0] = 0xCC;
  tx_packet.footer[1] = 0xDD;



  // Calib ADC để loại bỏ sai số nội vi trước khi chạy thực tế
//  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  // Kích hoạt bộ chuyển đổi ADC chạy ở chế độ DMA Circular
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)tx_packet.ch1, SAMPLES_PER_CH);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)tx_packet.ch2, SAMPLES_PER_CH);

  // Khởi động Timer 3. Timer 3 được cấu hình làm TRGO (Trigger Output) tạo xung đập nhịp 100kHz để kích ADC lấy mẫu.
  HAL_TIM_Base_Start(&htim3);

  // Khởi tạo các biến quản lý trạng thái
  uint8_t is_holding = 0;
  uint32_t last_btn_tick = 0;
  tx_packet.y_scale1 = 1.0f;
  tx_packet.y_scale2 = 1.0f;
  tx_packet.x_scale1 = 1.0f;
  tx_packet.x_scale2 = 1.0f;
  tx_packet.y_offset1 = 0;
  tx_packet.y_offset2 = 0;
  tx_packet.offset_axis = 0;
  tx_packet.x_offset1 = 0;
  tx_packet.x_offset2 = 0;

  // Biến lưu trạng thái của 2 chân CLK Encoder để so sánh cạnh xuống
  uint8_t last_clk1 = 1;
  uint8_t last_clk2 = 1;
  // Khởi tạo 2 biến dùng để chống dội phím (Debounce) cho Encoder
  uint32_t last_enc1_tick = 0;
  uint32_t last_enc2_tick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t current_tick = HAL_GetTick(); // Lấy thời gian ms hiện tại của hệ thống

	  //--- 1. ĐỌC CÔNG TẮC CHỌN KÊNH ---
	  // Kiểm tra chân PC4 và PC5
	  uint8_t ch1_en = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET) ? 1:0;
	  uint8_t ch2_en = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) ? 2:0;
	  // Dùng phép toán OR Bitmask để gộp chung cờ hiển thị (1: bật CH1, 2: bật CH2, 3: bật cả 2)
	  tx_packet.ch_mode = ch1_en | ch2_en;

	  //--- 2. ĐỌC NÚT NHẤN (Có Debounce 200ms) ---
	  // Giải thuật chống rung: Chỉ chấp nhận lần bấm tiếp theo nếu khoảng cách giữa 2 lần bấm cách nhau > 200ms.
	  if(current_tick - last_btn_tick > 200){

		  // Nút Dừng hình / Chạy tiếp (Hold)
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET){
			  is_holding = !is_holding;
			  tx_packet.hold_flag = is_holding;
			  last_btn_tick = current_tick; // Cập nhật mốc thời gian chốt nút
		  }
		  // Nút Autoset (Trả mọi thông số về default)
		  else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET){
			  tx_packet.x_scale1 = 1.0f;
			  tx_packet.x_scale2 = 1.0f;
			  tx_packet.y_scale1 = 1.0f; // Trả hệ số zoom trục Y về 1x
			  tx_packet.y_scale2 = 1.0f;
			  tx_packet.y_offset1 = 0;   // Trả vị trí gốc Y về giữa màn hình
			  tx_packet.y_offset2 = 0;
			  tx_packet.x_offset1 = 0;   // Trả gốc thời gian X về ban đầu
			  tx_packet.x_offset2 = 0;
			  tx_packet.offset_axis = 0;
			  tx_packet.reset_flag = 1;  // Bật cờ để ra lệnh cho ESP32 clear màn hình
			  last_btn_tick = current_tick;
		  }
		  // Nút Chuyển chế độ điều khiển trục (X hoặc Y)
		  else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET){
			  tx_packet.offset_axis = !tx_packet.offset_axis; // Đảo trạng thái 0 <-> 1
			  last_btn_tick = current_tick;
		  }
	  }

	  //--- 3. ĐỌC ROTARY ENCODER ---
	  	  // ENCODER 1: Dùng để phóng to/thu nhỏ sóng (Scale Y/X)
	  	  uint8_t clk1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

	  	  // Phát hiện cạnh xuống và đảm bảo khoảng cách giữa 2 lần nhận tín hiệu tối thiểu là 50ms (chống dội)
	  	  if (clk1 == 0 && last_clk1 == 1 && (current_tick - last_enc1_tick > 50)){

	  		  // Nếu chân DT khác mức logic của CLK -> Quay cùng chiều kim đồng hồ (Tăng). Ngược lại là giảm.
	  		  float delta = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) != clk1) ? 0.2f : -0.2f;
	  		  if (tx_packet.offset_axis == 0){
	  			 if (ch1_en) {
	  				 tx_packet.y_scale1 += delta;
	  				 if (tx_packet.y_scale1 < 0.2f) tx_packet.y_scale1 = 0.2f;
	  				 if (tx_packet.y_scale1 > 5.0f) tx_packet.y_scale1 = 5.0f;
	  			 }
	  			 if (ch2_en) {
	  				tx_packet.y_scale2 += delta;
	  				if (tx_packet.y_scale2 < 0.2f) tx_packet.y_scale2 = 0.2f;
	  				if (tx_packet.y_scale2 > 5.0f) tx_packet.y_scale2 = 5.0f;
	  			 }
	  		  } else {
	  			if (ch1_en) {
	  				tx_packet.x_scale1 += delta;
	  				if (tx_packet.x_scale1 < 0.8f) tx_packet.x_scale1 = 0.8f;
	  				if (tx_packet.x_scale1 > 5.0f) tx_packet.x_scale1 = 5.0f;
	  			}
	  			if (ch2_en) {
	  				tx_packet.x_scale2 += delta;
	  				if (tx_packet.x_scale2 < 0.8f) tx_packet.x_scale2 = 0.8f;
	  				if (tx_packet.x_scale2 > 5.0f) tx_packet.x_scale2 = 5.0f;
	  			}
	  		  }
	  		  last_enc1_tick = current_tick; // Chốt thời gian lặp để block các tín hiệu nhiễu tiếp theo
	  	  }
	  	  last_clk1 = clk1;

	  	  // ENCODER 2: Dùng để dịch chuyển vị trí sóng theo trục ngang (X) hoặc dọc (Y)
	  	  uint8_t clk2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);

	  	  if (clk2 == 0 && last_clk2 == 1 && (current_tick - last_enc2_tick > 50)){

	  	  		int delta_y = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) != clk2) ? -10 : 10;
	  	  		int delta_x = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) != clk2) ? -5 : 5;

	  	        if (tx_packet.offset_axis == 0) {
	  	            // --- CHẾ ĐỘ 0: ĐANG CHỌN DỊCH THEO TRỤC Y ---
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
	  	            // --- CHẾ ĐỘ 1: ĐANG CHỌN DỊCH THEO TRỤC X (THỜI GIAN) ---
	  	            if(ch1_en) {
	  	                tx_packet.x_offset1 += delta_x;
	  	                if (tx_packet.x_offset1 > 250) tx_packet.x_offset1 = 250;
	  	                if (tx_packet.x_offset1 < -75) tx_packet.x_offset1 = -75;
	  	            }
	  	            if (ch2_en) {
	  	                tx_packet.x_offset2 += delta_x;
	  	                if (tx_packet.x_offset2 > 250) tx_packet.x_offset2 = 250;
	  	                if (tx_packet.x_offset2 < -75) tx_packet.x_offset2 = -75;
	  	            }
	  	        }
	  	        last_enc2_tick = current_tick;
	  	  }
	  	  last_clk2 = clk2;
	  // --- 4. TÁCH DỮ LIỆU DMA, XỬ LÝ DSP VÀ TRUYỀN SPI ---
	  // Cờ data_ready_flag được kích bởi các hàm callback ngắt của DMA khi nó điền xong Ping hoặc Pong
      if (adc1_ready == 1 && adc2_ready == 1) {

    	  // BƯỚC 1: CẬP NHẬT VÀ XỬ LÝ SÓNG (Chỉ chạy khi người dùng KHÔNG bấm Hold)
    	  if (is_holding == 0) {
    	      tx_packet.trigger_idx1 = Process_Signal(tx_packet.ch1, &tx_packet.vpp1, &tx_packet.freq1, &tx_packet.vavg1, &tx_packet.vrms1, &tx_packet.vamp1);
        	  tx_packet.trigger_idx2 = Process_Signal(tx_packet.ch2, &tx_packet.vpp2, &tx_packet.freq2, &tx_packet.vavg2, &tx_packet.vrms2, &tx_packet.vamp2);
    	  }

    	  // BƯỚC 2: XÓA CỜ NGẮT DMA (Phải thực hiện dù có bấm Hold hay không để hệ thống không bị treo)
    	  adc1_ready = 0;
    	  adc2_ready = 0;

    	  // BƯỚC 3: BẮN GÓI TIN SANG ESP32 (Giao tiếp SPI bằng DMA)
    	  // Lưu ý: Quá trình truyền này được đặt ngoài khối "is_holding" để ESP32 vẫn liên tục nhận được cập nhật về thao tác nút bấm, cursor, zoom ngay cả khi màn hình đang Hold.
    	  if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) { // Đảm bảo đường SPI đang không bận
    	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Kéo chân CS xuống LOW để chọn chip Slave (ESP32)
    	      // Sử dụng DMA để đẩy gói tin ~2KB đi. CPU không phải chờ truyền xong.
    	      HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&tx_packet, PACKET_SIZE);
    	  }
      }
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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

  /*Configure GPIO pins : PC0 PC1 PC2 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin PA6 */
  GPIO_InitStruct.Pin = SPI_CS_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Callback được gọi tự động khi DMA điền đầy NỬA SAU mảng adc_buffer (Pong)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
    	adc1_ready = 1; // Luồng DMA của ADC1 xong -> Bật flag kênh 1
    }
    if(hadc->Instance == ADC2) {
    	adc2_ready = 1; // Luồng DMA của ADC2 xong -> Bật flag kênh 2
    }
}

// Callback được gọi tự động sau khi DMA hoàn thành việc bắn toàn bộ gói SPI sang ESP32
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if(hspi->Instance == SPI1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);// Kéo chân CS lên mức CAO để chốt quá trình truyền và giải phóng bus

	    tx_packet.reset_flag = 0;

        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)tx_packet.ch1, SAMPLES_PER_CH);
        HAL_ADC_Start_DMA(&hadc2, (uint32_t*)tx_packet.ch2, SAMPLES_PER_CH);
    }
}

// Hàm DSP xử lý tín hiệu: Tìm Max, Min, RMS, Vavg, Tần số và tính toán điểm Trigger
uint16_t Process_Signal(uint16_t* raw_data, float* vpp, float* freq, float* vavg, float* vrms, float* vamp) {
    uint16_t max = 0, min = 4095;
    uint32_t sum = 0;
    uint64_t sum_sq = 0;

    // 1. Quét mảng để tìm Max, Min, cộng dồn tổng và tổng bình phương
    for(int i=0; i<SAMPLES_PER_CH; i++){
        uint16_t val = raw_data[i];
        if(val > max) max = val;
        if(val < min) min = val;
        sum += val;
        sum_sq += (uint64_t)val * val;
    }

    // 2. Chuyển đổi dữ liệu thô (0-4095) sang Volt
    float adc_to_volt = 3.3f/4095.0f;

    *vpp = (float)(max - min) * adc_to_volt;
    *vamp = *vpp / 2.0f;
    float avg_raw = (float)sum / SAMPLES_PER_CH;
    *vavg = avg_raw * adc_to_volt;
    *vrms = sqrtf((float)sum_sq / SAMPLES_PER_CH) * adc_to_volt;

    // 3. TÍNH TẦN SỐ (Bằng State Machine quét từ đầu mảng)
        int16_t trigger_level = (max + min) / 2;
        // Độ trễ 5% Vpp. Chống nhiễu nhưng không làm mất sóng.
        int16_t hysteresis = (max - min) * 5 / 100;
        if (hysteresis < 15) hysteresis = 15;
        if (hysteresis > 100) hysteresis = 100;

        int first_cross_freq = -1;
        int last_cross_freq = -1;
        int period_count = 0;

        // Khởi tạo trạng thái ban đầu cho máy đếm tần số
        uint8_t is_low = (raw_data[0] < trigger_level) ? 1 : 0;

        for(int i = 0; i < SAMPLES_PER_CH; i++) {
            // Nếu sóng vượt vạch trên -> Ghi nhận sườn lên
            if(is_low == 1 && raw_data[i] > (trigger_level + hysteresis)) {
                if (first_cross_freq == -1) {
                    first_cross_freq = i;
                } else {
                    last_cross_freq = i;
                    period_count++;
                }
                is_low = 0; // Khóa lại, chờ sóng xuống
            }
            // Nếu sóng rớt xuống vạch dưới -> Mở khóa chờ sườn lên tiếp theo
            else if(is_low == 0 && raw_data[i] < (trigger_level - hysteresis)) {
                is_low = 1;
            }
        }

        // 4. TÍNH TOÁN TẦN SỐ
        if(period_count > 0) {
            *freq = (100000.0f * period_count) / (float)(last_cross_freq - first_cross_freq);
        } else {
            *freq = 0;
        }


        // 5. TÌM ĐIỂM NEO VẼ SÓNG (TRIGGER) CŨNG BẰNG STATE MACHINE
        int display_trigger = 50; // Mặc định ở mẫu 400 để dành chỗ cho dịch trục X

        // Đánh giá trạng thái sóng ngay tại vị trí 400
        uint8_t trig_is_low = (raw_data[50] < trigger_level) ? 1 : 0;

        for(int i = 50; i < SAMPLES_PER_CH; i++) {
            // Chỉ cần sóng trườn qua vạch trên là chốt điểm neo, bất kể dốc thoai thoải cỡ nào
            if(trig_is_low == 1 && raw_data[i] > (trigger_level + hysteresis)) {
                display_trigger = i; // Đã tìm thấy điểm Trigger!
                break;               // Cắt vòng lặp ngay, lấy điểm đầu tiên làm mốc
            }
            else if(trig_is_low == 0 && raw_data[i] < (trigger_level - hysteresis)) {
                trig_is_low = 1;     // Sóng xuống thấp, mở khóa chờ cắt lên
            }
        }

        return (uint16_t)display_trigger;
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
