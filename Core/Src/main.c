/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  UART_RX_0 = 48,
  UART_RX_1,
  UART_RX_2,
  UART_RX_3,
  UART_RX_4,
  UART_RX_5,
  UART_RX_NONE = 0xFF,
}UARTCommandTypeDef;


// SHT85 defines
typedef enum {
  CMD_READ_SERIALNBR = 0x3780, // read serial number
  CMD_READ_STATUS    = 0xF32D, // read status register
  CMD_CLEAR_STATUS   = 0x3041, // clear status register
  CMD_HEATER_ENABLE  = 0x306D, // enabled heater
  CMD_HEATER_DISABLE = 0x3066, // disable heater
  CMD_SOFT_RESET     = 0x30A2, // soft reset
  CMD_MEAS_SINGLE_H  = 0x2400, // single meas., high repeatability
  CMD_MEAS_SINGLE_M  = 0x240B, // single meas., medium repeatability
  CMD_MEAS_SINGLE_L  = 0x2416, // single meas., low repeatability
  CMD_MEAS_PERI_05_H = 0x2032, // periodic meas. 0.5 mps, high repeatability
  CMD_MEAS_PERI_05_M = 0x2024, // periodic meas. 0.5 mps, medium repeatability
  CMD_MEAS_PERI_05_L = 0x202F, // periodic meas. 0.5 mps, low repeatability
  CMD_MEAS_PERI_1_H  = 0x2130, // periodic meas. 1 mps, high repeatability
  CMD_MEAS_PERI_1_M  = 0x2126, // periodic meas. 1 mps, medium repeatability
  CMD_MEAS_PERI_1_L  = 0x212D, // periodic meas. 1 mps, low repeatability
  CMD_MEAS_PERI_2_H  = 0x2236, // periodic meas. 2 mps, high repeatability
  CMD_MEAS_PERI_2_M  = 0x2220, // periodic meas. 2 mps, medium repeatability
  CMD_MEAS_PERI_2_L  = 0x222B, // periodic meas. 2 mps, low repeatability
  CMD_MEAS_PERI_4_H  = 0x2334, // periodic meas. 4 mps, high repeatability
  CMD_MEAS_PERI_4_M  = 0x2322, // periodic meas. 4 mps, medium repeatability
  CMD_MEAS_PERI_4_L  = 0x2329, // periodic meas. 4 mps, low repeatability
  CMD_MEAS_PERI_10_H = 0x2737, // periodic meas. 10 mps, high repeatability
  CMD_MEAS_PERI_10_M = 0x2721, // periodic meas. 10 mps, medium repeatability
  CMD_MEAS_PERI_10_L = 0x272A, // periodic meas. 10 mps, low repeatability
  CMD_FETCH_DATA     = 0xE000, // readout measurements for periodic mode
  CMD_BREAK          = 0x3093, // stop periodic measurement
} etCommands;

// Single Shot Measurement Repeatability
typedef enum {
  SINGLE_MEAS_LOW        = CMD_MEAS_SINGLE_L, // low repeatability
  SINGLE_MEAS_MEDIUM     = CMD_MEAS_SINGLE_M, // medium repeatability
  SINGLE_MEAS_HIGH       = CMD_MEAS_SINGLE_H  // high repeatability
} etSingleMeasureModes;

// Periodic Measurement Configurations
typedef enum {
  PERI_MEAS_LOW_05_HZ    = CMD_MEAS_PERI_05_L,
  PERI_MEAS_LOW_1_HZ     = CMD_MEAS_PERI_1_L,
  PERI_MEAS_LOW_2_HZ     = CMD_MEAS_PERI_2_L,
  PERI_MEAS_LOW_4_HZ     = CMD_MEAS_PERI_4_L,
  PERI_MEAS_LOW_10_HZ    = CMD_MEAS_PERI_10_L,
  PERI_MEAS_MEDIUM_05_HZ = CMD_MEAS_PERI_05_M,
  PERI_MEAS_MEDIUM_1_HZ  = CMD_MEAS_PERI_1_M,
  PERI_MEAS_MEDIUM_2_HZ  = CMD_MEAS_PERI_2_M,
  PERI_MEAS_MEDIUM_4_HZ  = CMD_MEAS_PERI_4_M,
  PERI_MEAS_MEDIUM_10_HZ = CMD_MEAS_PERI_10_M,
  PERI_MEAS_HIGH_05_HZ   = CMD_MEAS_PERI_05_H,
  PERI_MEAS_HIGH_1_HZ    = CMD_MEAS_PERI_1_H,
  PERI_MEAS_HIGH_2_HZ    = CMD_MEAS_PERI_2_H,
  PERI_MEAS_HIGH_4_HZ    = CMD_MEAS_PERI_4_H,
  PERI_MEAS_HIGH_10_HZ   = CMD_MEAS_PERI_10_H,
} etPeriodicMeasureModes;


// Error codes
typedef enum{
  NO_ERROR       = 0x00, // no error
  ACK_ERROR      = 0x01, // no acknowledgment error
  CHECKSUM_ERROR = 0x02, // checksum mismatch error
  TIMEOUT_ERROR  = 0x04, // timeout error
} etError;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE                     1

#define BLEUIO_NOT_READY_MSG	"\r\nBleuIO not ready yet.\r\n"

// Dongle commands
#define DONGLE_CMD_ATI "ATI\r\n"
#define DONGLE_CMD_AT_ADVSTART "AT+ADVSTART\r\n"
#define DONGLE_CMD_AT_ADVSTOP "AT+ADVSTOP\r\n"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define CRC_POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
#define I2C_ADDR        0x44

static float CalcTemperature(uint16_t rawValue);
static float CalcHumidity(uint16_t rawValue);
void WriteCommand(etCommands command);
void SHT85_ReadStatus(uint16_t* status);
void SHT85_ReadSerialNumber(uint32_t* serialNumber);
void SHT85_SingleMeasurment(float* temperature, float* humidity,
                               etSingleMeasureModes measureMode,
                               uint8_t timeout);

uint8_t revbuff[10]={0,};
uint32_t serialNumber;   // serial number
uint16_t SHt85_status;   // serial number
float    temperature;    // temperature
float    humidity;
HAL_StatusTypeDef err0r;



static char uart_tx_buf[250];
int uart_buf_len;
bool isBleuIOReady;
static UARTCommandTypeDef uartStatus;
static uint8_t aRxBuffer[RXBUFFERSIZE];
static int RX_value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void handleUartInput(UARTCommandTypeDef cmd);


void i2c_bus_scan(I2C_HandleTypeDef hi2c);


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
  MX_ETH_Init();
  MX_USART3_UART_Init();
 // MX_USB_HOST_Init();
  MX_I2C2_Init();


  /* USER CODE BEGIN 2 */
  isBleuIOReady = false;
  uartStatus = UART_RX_NONE;

  // Starts uart recieve interrupt mode
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, RXBUFFERSIZE);

  // Turns on all LEDs on start up
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay(1000);

  // Turns off all LEDs except Red
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  // scan i2c bus 2 (external)
  i2c_bus_scan(hi2c2);
	HAL_Delay(1000);

  // Sends welcome message to uart
  uart_buf_len = sprintf(uart_tx_buf, "\r\nWelcome to This Tutorial on STM32 BleuIO Example!\r\n, My name is Emil Lindblom\r\n Please Press 0 to run the ATI command\r\nPress 1 to start advertising\r\nPress 2 to stop advertising\r\nPress 3 to read temp & Humidity\r\n\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);



  //
  // SHT85 init
  //
  uint8_t sendbuff[2]={0,};
  sendbuff[0]=(uint8_t)(CMD_SOFT_RESET>>8);
  sendbuff[1]=(uint8_t)(CMD_SOFT_RESET& 0xFF);
  err0r=HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)(I2C_ADDR<<1),sendbuff,2,HAL_MAX_DELAY);

  HAL_Delay(200); // in ms

  // Raed and print SHT85 Serial Number
  SHT85_ReadSerialNumber(&serialNumber);
  HAL_Delay(200); // in ms
  uart_buf_len = sprintf(uart_tx_buf,  "serialNumber: 0x%08X \r\n", (int)serialNumber);
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, 100);
  HAL_Delay(200); // in ms



   WriteCommand(PERI_MEAS_HIGH_1_HZ); // Set SHT85 read frequency


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
   // MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    // Simple handler for uart input
    handleUartInput(uartStatus);


    HAL_Delay(1000); // in ms


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c2.Init.Timing = 0x10707DBC;
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &huart3)
	{
		RX_value = (int)aRxBuffer[0];
		uartStatus = UART_RX_NONE;

		switch(RX_value)
		{
			case UART_RX_0:
			{
				uartStatus = UART_RX_0;
				break;
			}
			case UART_RX_1:
			{
				uartStatus = UART_RX_1;
				break;
			}
			case UART_RX_2:
			{
				uartStatus = UART_RX_2;
				break;
			}

			case UART_RX_3:
			{
				uartStatus = UART_RX_3;
				break;
			}

			case UART_RX_4:
			{
				uartStatus = UART_RX_4;
				break;
			}

			case UART_RX_5:
			{
				uartStatus = UART_RX_5;
				break;
			}

			default:
			{
				uartStatus = UART_RX_NONE;
				break;
			}
		}
		// Resets uart recieve interrupt mode
		HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
	}
}


/**
  * @brief Simple uart input handler
  * @retval None
  */
void handleUartInput(UARTCommandTypeDef cmd)
{
	switch(cmd)
	{
		case UART_RX_0:
		{
			// 0
			uart_buf_len = sprintf(uart_tx_buf, "\r\n(0 pressed)\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			if(isBleuIOReady)
			{
				writeToDongle((uint8_t*)DONGLE_CMD_ATI);
			} else
			{
				uart_buf_len = sprintf(uart_tx_buf, BLEUIO_NOT_READY_MSG);
				HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			}
			uartStatus = UART_RX_NONE;
			break;
		}

		case UART_RX_1:
		{
			// 1
			uart_buf_len = sprintf(uart_tx_buf, "\r\n(1 pressed)\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			if(isBleuIOReady)
			{
				writeToDongle((uint8_t*)DONGLE_CMD_AT_ADVSTART);
			} else
			{
				uart_buf_len = sprintf(uart_tx_buf, BLEUIO_NOT_READY_MSG);
				HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			}
			uartStatus = UART_RX_NONE;
			break;
		}

		case UART_RX_2:
		{
			// 2
			uart_buf_len = sprintf(uart_tx_buf, "\r\n(2 pressed)\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			if(isBleuIOReady)
			{
				writeToDongle((uint8_t*)DONGLE_CMD_AT_ADVSTOP);
			} else
			{
				uart_buf_len = sprintf(uart_tx_buf, BLEUIO_NOT_READY_MSG);
				HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			}
			uartStatus = UART_RX_NONE;
			break;
		}

		case UART_RX_3:
		{
		    SHT85_SingleMeasurment(&temperature, &humidity, SINGLE_MEAS_HIGH, 50);
		    HAL_Delay(1000); // in ms

			 uart_buf_len = sprintf(uart_tx_buf, "\r\n temperature:%.2f \r\n humidity: %.2f \r\n Error: %i",temperature,humidity,  err0r);
			  HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			  uartStatus = UART_RX_NONE;
			  break;
	    }

		case UART_RX_4:
		{

			SHT85_ReadStatus(&SHt85_status);
			HAL_Delay(1000); // in ms

			uart_buf_len = sprintf(uart_tx_buf, "\r\n SHt85_status 0x%X \r\n Error: %i",SHt85_status,  err0r);
		    HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			uartStatus = UART_RX_NONE;
		    break;
		}

		case UART_RX_5:
		{

			SHT85_ReadSerialNumber(&serialNumber);
			HAL_Delay(1000); // in ms

			uart_buf_len = sprintf(uart_tx_buf, "\r\n SHt85 Serial Nr: 0x%4X \r\n Error: %i",serialNumber,  err0r);
		    HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			uartStatus = UART_RX_NONE;
			break;
		}




		case UART_RX_NONE:
		{
			break;
		}

		default:
		{
			uartStatus = UART_RX_NONE;
			break;
		}
	}
}

/**
  * @brief
  * @retval None
  */
void WriteCommand(etCommands command)
{
    uint8_t sendbuff[2]={0,};


    sendbuff[0]=(uint8_t)(command>>8);
    sendbuff[1]=(uint8_t)(command& 0xFF);

    err0r=  HAL_I2C_Master_Transmit(&hi2c2,(uint16_t)(I2C_ADDR<<1),sendbuff,2,HAL_MAX_DELAY);
    HAL_Delay(2); // in ms
}


/**
  * @brief The status register contains information on the operational status of the heater,
  *        the alert mode and on the execution
  *        status of the last command and the last write sequence.
  * @retval None
  */
void  SHT85_ReadStatus(uint16_t* status)
{
  uint8_t serialNumWords[10];


  *status =0;

    WriteCommand(CMD_READ_STATUS);

    err0r=HAL_I2C_Master_Receive(&hi2c2,(uint16_t)(I2C_ADDR<<1),serialNumWords, 6, HAL_MAX_DELAY);

    *status = (uint16_t)(serialNumWords[0] << 8) | serialNumWords[1];
}


/**
  * @brief
  * @retval None
  */
void  SHT85_ReadSerialNumber(uint32_t* serialNumber)
{
  uint8_t serialNumWords[10];
  uint16_t serialNumWordsHL[2]={0,};

    WriteCommand(CMD_READ_SERIALNBR);

    err0r=HAL_I2C_Master_Receive(&hi2c2,(uint16_t)(I2C_ADDR<<1),serialNumWords, 6, HAL_MAX_DELAY);

     serialNumWordsHL[0]= (uint16_t)(serialNumWords[0] << 8) | serialNumWords[1];
     serialNumWordsHL[1]=(uint16_t)(serialNumWords[2] << 8) | serialNumWords[3];
    *serialNumber = (serialNumWordsHL[0] << 16) | serialNumWordsHL[1];

}



/**
  * @brief Temperature conversion formula, result in °C
  * @retval None
  */
static float CalcTemperature(uint16_t rawValue)
{
  // calculate temperature [°C]
  // T = -45 + 175 * rawValue / (2^16-1)
  return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}



/**
  * @brief  Relative humidity conversion formula (result in %RH):
  * @retval None
  */
static float CalcHumidity(uint16_t rawValue)
{
  // calculate relative humidity [%RH]
  // RH = rawValue / (2^16-1) * 100
  return 100.0f * (float)rawValue / 65535.0f;
}


/**
  * @brief
  * @retval None
  */
void SHT85_SingleMeasurment(float* temperature, float* humidity,
                               etSingleMeasureModes measureMode,
                               uint8_t timeout)
{
  etError  error;           // error code
  uint16_t rawValueTemp;    // temperature raw value from sensor
  uint16_t rawValueHumi;    // humidity raw value from sensor
  uint8_t revbufff[10];

  *temperature = 0;
  *humidity = 0;

  WriteCommand((etCommands)measureMode);
  //  if(err0r!=HAL_OK)
  //      return;

  //System_DelayUs(100 000);
  HAL_Delay(1000); // in ms

  err0r= HAL_I2C_Master_Receive(&hi2c2,(uint16_t)(I2C_ADDR<<1),revbufff, 6, HAL_MAX_DELAY);
 // if(err0r!=HAL_OK)
  //      return;

    rawValueTemp=(uint16_t)(revbufff[0] << 8) | revbufff[1];
    rawValueHumi=(uint16_t)(revbufff[3] << 8) | revbufff[4];

    *temperature = CalcTemperature(rawValueTemp);
    *humidity = CalcHumidity(rawValueHumi);

}



/** i2c_bus_scan
  * @brief  Function scans the devices connected to i2c bus.
  * @param  argument: Not used
  * @retval None
  */
void i2c_bus_scan(I2C_HandleTypeDef hi2c )
{
	int uart__len;
	int i,nr;
	HAL_StatusTypeDef res;


	uart__len = sprintf(uart_tx_buf,  "\r\n I2C bus Scan: \r\n ");

	HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart__len, 100);


	// scan all i2c addresses
	for (i=1,nr=0; i<128; i++)
	{
	  	/*
	  	 * the HAL wants a left aligned i2c address
	  	 * &hi2c1 is the handle
	  	 * (uint16_t)(i<<1) is the i2c address left aligned
	  	 * retries 2
	  	 * timeout 2
	  	 */



	  	 res = HAL_I2C_IsDeviceReady(&hi2c, (uint16_t)(i<<1), 2, 2);

	  	 if (res != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
	  	 {
	  	 	 	 nr++;//
	  	 }

	  	 if (res == HAL_OK)
	  	 {
	  	 	 uart__len = sprintf(uart_tx_buf,  "0x%X \r\n", i);

	  		 HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart__len, 100);

	  	 }
	  }

	  uart__len = sprintf(uart_tx_buf,  "scans 0x%X \r\n", nr);

	  HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart__len, 100);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
