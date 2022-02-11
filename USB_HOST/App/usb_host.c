/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_cdc.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BLEUIO_READY	"\r\n[BleuIO Dongle Ready]\r\n"
#define RX_BUFF_SIZE   0x400  /* Max Received data 1KB */
uint8_t CDC_RX_Buffer[RX_BUFF_SIZE];
static uint32_t rx_size;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
void USBH_CDC_TransmitCallback(USBH_HandleTypeDef *phost)
{
  	if(phost == &hUsbHostFS)
  	{

  	}
}

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost)
{
  	if(phost == &hUsbHostFS)
  	{
  		// Handles the data recived from the USB CDC host, here just printing it out to UART
  		rx_size = USBH_CDC_GetLastReceivedDataSize(phost);
		HAL_UART_Transmit(&huart3, CDC_RX_Buffer, rx_size, HAL_MAX_DELAY);

		// Reset buffer and restart the callback function to receive more data
		memset(CDC_RX_Buffer,0,RX_BUFF_SIZE);
		USBH_CDC_Receive(phost, CDC_RX_Buffer, RX_BUFF_SIZE);
  	}

  	return;
}

/**
  * @brief Simple function that takes a string and transmit it to the dongle
  * @retval None
  */
void writeToDongle(uint8_t * cmd)
{
	USBH_CDC_Transmit(&hUsbHostFS, cmd, strlen((char *)cmd));
}
/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */

  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_CDC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */

  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostFS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  isBleuIOReady = false;

  // Turn on Red LED, turn off Green and Yellow LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
  // Check if BleuIO firmware is running
  // (idProduct:0x6001 = bootloader, idProduct:0x6002 = bleuio fw)
  if(phost->device.DevDesc.idProduct == 0x6002)
  {
	  isBleuIOReady = true;
	  // Sends message to uart that BleuIO is connected and ready
	  HAL_UART_Transmit(&huart3, (uint8_t*)BLEUIO_READY, strlen(BLEUIO_READY), HAL_MAX_DELAY);

	  // Turn on Green LED, turn off Yellow and Red LED
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

	  // Start receiving from usb
	  USBH_CDC_Receive(&hUsbHostFS, CDC_RX_Buffer, RX_BUFF_SIZE);
  }
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;
  isBleuIOReady = false;
  // Turn on Yellow LED, turn off Green and Red LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
