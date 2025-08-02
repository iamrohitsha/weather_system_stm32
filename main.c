/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// --- IMPORTANT: CONFIGURE YOUR DETAILS HERE ---
#define WIFI_SSID "MOTO_Rohit"
#define WIFI_PASS "rohit1234"
#define NODE_RED_SERVER_IP "10.172.213.162"
#define NODE_RED_SERVER_PORT "5000"
#define NODE_RED_ENDPOINT "/sensor"

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT_PORT GPIOF
#define DHT_PIN  GPIO_PIN_15

#define BMP_ADDRESS (0x76 << 1)
#define BMP_CHIP_ID_REG 0xD0
#define BMP_CTRL_MEAS_REG 0xF4
#define BMP_CONFIG_REG 0xF5
#define BMP_PRESS_MSB_REG 0xF7
#define BMP_CALIB_REG 0x88
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// BMP280 Calibration data
uint16_t dig_T1;
int16_t  dig_T2, dig_T3;
uint16_t dig_P1;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t  t_fine;

// Buffer for UART communication
char tx_buf[512];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
// Helper & Sensor Functions

float Read_UV_Voltage(void);

// DHT11 Sensor Functions
void MicroDelay(uint16_t us);
void DHT11_InitSequence(void);
int DHT11_ResponseCheck(void);
uint32_t DHT11_ReadByte(void);
float DHT_Temperature(void);
float DHT_Humidity(void);

// BMP280 Sensor Functions
void BMP280_Init(void);
void BMP280_Read_Calibration(void);
void BMP280_Read_Temp_Press(float *temperature, float *pressure);
float Calculate_Altitude(float pressure_hPa);

// ESP8266 Communication Functions
void ESP_SendCmd(const char *cmd, uint32_t delay_ms);
void ESP_Init(void);
void ESP_SendDataToNodeRED(float temp, float alt, float pres, float humi, float uv);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t hum_msb, hum_lsb, temp_msb, temp_lsb;
uint16_t checksum;
int dht_status = 0;
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  BMP280_Init();
  ESP_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    float bmp_temp, pressure, altitude, uv_voltage;
    char debug_msg[200];

    // 1. Read from BMP280 Sensor
    BMP280_Read_Temp_Press(&bmp_temp, &pressure);
    altitude = Calculate_Altitude(pressure);

    // 2. Read from DHT11 Sensor
    // This function now correctly reads both values at the same time.
    float dht_temper = DHT_Temperature();
    float dht_humi = DHT_Humidity();

    // 3. Read from UV Sensor
    uv_voltage = Read_UV_Voltage();

    // 5. Send the collected data to the server (Node-RED)

    ESP_SendDataToNodeRED(bmp_temp, altitude, pressure, dht_humi, uv_voltage);

    // 6. Wait before the next cycle
    HAL_Delay(10000); // Wait 10 seconds
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF15 */
  GPIO_InitStruct.Pin = DHT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MicroDelay(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void DHT11_InitSequence() {
    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin = DHT_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT_PORT, &gpioInit);

    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
    MicroDelay(30);

    gpioInit.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DHT_PORT, &gpioInit);
}

int DHT11_ResponseCheck() {
    MicroDelay(40);
    if (!HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)) {
        MicroDelay(80);
        dht_status = HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) ? 1 : 0;
    }
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN));
    return dht_status;
}

uint32_t DHT11_ReadByte() {
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++) {
        while (!HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN));
        MicroDelay(50);
        if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN))
            data |= (1 << (7 - i));
        while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN));
    }
    return data;
}

float DHT_Temperature() {
    DHT11_InitSequence();
    if (DHT11_ResponseCheck()) {
        hum_msb = DHT11_ReadByte();
        hum_lsb = DHT11_ReadByte();
        temp_msb = DHT11_ReadByte();
        temp_lsb = DHT11_ReadByte();
        checksum = DHT11_ReadByte();

        if (checksum == (hum_msb + hum_lsb + temp_msb + temp_lsb)){
      	  float temper = temp_msb + (temp_lsb/10.0f);
      	  return temper;
    }
        else {
        	float temper = 0;
        	  return temper;
        }

}
}
float DHT_Humidity(){
    if (checksum == (hum_msb + hum_lsb + temp_msb + temp_lsb)){
	  float humi = hum_msb + (hum_lsb/10.0f);
  	  return humi;
}
    else {
    	float humi = 0;
    	return humi;
    }
}
// --- UV Sensor Function ---
float Read_UV_Voltage(void) {
    uint32_t raw_value = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        raw_value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    // Convert 12-bit ADC value to voltage (assuming Vref = 3.3V)
    return (float)raw_value * (3.3f / 4095.0f);
}

// --- BMP280 Functions ---
int32_t BMP280_Compensate_Temperature(int32_t adc_T) {
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

uint32_t BMP280_Compensate_Pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;
}

void BMP280_Read_Calibration(void) {
    uint8_t calib_data[24];
    HAL_I2C_Mem_Read(&hi2c1, BMP_ADDRESS, BMP_CALIB_REG, 1, calib_data, 24, HAL_MAX_DELAY);
    dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dig_T3 = (calib_data[5] << 8) | calib_data[4];
    dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dig_P9 = (calib_data[23] << 8) | calib_data[22];
}

void BMP280_Init(void) {
    uint8_t chip_id = 0;
    HAL_I2C_Mem_Read(&hi2c1, BMP_ADDRESS, BMP_CHIP_ID_REG, 1, &chip_id, 1, HAL_MAX_DELAY);
    if (chip_id == 0x58) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"BMP280 Found.\r\n", strlen("BMP280 Found.\r\n"), HAL_MAX_DELAY);
        uint8_t config_data[2];
        // Set forced mode, 1x oversampling for temp and press
        config_data[0] = 0x27;
        HAL_I2C_Mem_Write(&hi2c1, BMP_ADDRESS, BMP_CTRL_MEAS_REG, 1, &config_data[0], 1, HAL_MAX_DELAY);
        // Set standby time, filter off
        config_data[0] = 0xA0;
        HAL_I2C_Mem_Write(&hi2c1, BMP_ADDRESS, BMP_CONFIG_REG, 1, &config_data[0], 1, HAL_MAX_DELAY);
        BMP280_Read_Calibration();
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t*)"BMP280 Not Found!\r\n", strlen("BMP280 Not Found!\r\n"), HAL_MAX_DELAY);
    }
}

void BMP280_Read_Temp_Press(float *temperature, float *pressure) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, BMP_ADDRESS, BMP_PRESS_MSB_REG, 1, data, 6, HAL_MAX_DELAY);
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    *temperature = (float)BMP280_Compensate_Temperature(adc_T) / 100.0f;
    *pressure = (float)BMP280_Compensate_Pressure(adc_P) / 256.0f / 100.0f; // Convert to hPa
}

float Calculate_Altitude(float pressure_hPa) {
    return 44330.0 * (1.0 - pow((pressure_hPa / 1013.25), 0.1903));
}


// --- ESP8266 Functions ---
void ESP_SendCmd(const char *cmd, uint32_t delay_ms) {
    sprintf(tx_buf, "%s\r\n", cmd);
    HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, strlen(tx_buf), 1000);
    HAL_Delay(delay_ms);
}

void ESP_Init(void) {
    ESP_SendCmd("AT", 1000); // Check if ESP is responding
    ESP_SendCmd("AT+CWMODE=1", 1000); // Set Wi-Fi mode to Station (STA)
    // Connect to the Wi-Fi network
    char wifi_connect_cmd[100];
    sprintf(wifi_connect_cmd, "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASS);
    ESP_SendCmd(wifi_connect_cmd, 5000);
    ESP_SendCmd("AT+CIPMUX=0", 1000); // Configure for single connection mode
}

void ESP_SendDataToNodeRED(float temp, float alt, float pres, float humi, float uv) {
    char http_cmd[256];
    char json_data[200];

    // 1. Create the JSON payload with the sensor values.
    //    Example payload: {"temperature":30.50,"altitude":200.00,"pressure":1013.25}
    sprintf(json_data, "{\"temperature\":%.2f,\"altitude\":%.2f,\"pressure\":%.2f,\"humidity\":%.2f,\"uv_index\":%.2f}", temp, alt, pres, humi, uv);

    // 2. Establish a TCP connection to the Node-RED server.
    sprintf(http_cmd, "AT+CIPSTART=\"TCP\",\"%s\",%s", NODE_RED_SERVER_IP, NODE_RED_SERVER_PORT);
    ESP_SendCmd(http_cmd, 2000);

    // 3. Prepare the full HTTP POST request.
    //    This includes the request line, headers, and the JSON data.
    //    The link being targeted is http://[NODE_RED_SERVER_IP]:[NODE_RED_SERVER_PORT][NODE_RED_ENDPOINT]
    int len = strlen(json_data);
    sprintf(http_cmd, "POST %s HTTP/1.1\r\nHost: %s:%s\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n%s",
            NODE_RED_ENDPOINT, NODE_RED_SERVER_IP, NODE_RED_SERVER_PORT, len, json_data);
    int total_len = strlen(http_cmd);

    // 4. Send the command to the ESP8266 indicating the length of the data to be sent.
    sprintf(tx_buf, "AT+CIPSEND=%d", total_len);
    ESP_SendCmd(tx_buf, 1000);

    // 5. Send the actual HTTP request packet.
    HAL_UART_Transmit(&huart1, (uint8_t *)http_cmd, total_len, 2000);
    HAL_Delay(2000);

    // 6. Close the TCP connection.
    ESP_SendCmd("AT+CIPCLOSE", 1000);
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
