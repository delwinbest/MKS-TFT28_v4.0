/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "fatfs.h"

uint8_t retUSBH;    /* Return value for USBH */
char USBHPath[4];   /* USBH logical drive path */
FATFS USBHFatFS;    /* File system object for USBH logical drive */
FIL USBHFile;       /* File object for USBH */
uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
uint8_t retSPISD;    /* Return value for USER */
char SPISD_Path[4];   /* USER logical drive path */
//FATFS USERFatFS;    /* File system object for USER logical drive */
//FIL USERFile;       /* File object for USER */
/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the USBH driver ###########################*/
  retUSBH = FATFS_LinkDriver(&USBH_Driver, USBHPath);
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  retSPISD = FATFS_LinkDriver(&SPISD_Driver, SPISD_Path);    // SPI_SD CARD
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
void deviceSelect(dselect_t device)  {

	if (device == SPI_SDCARD) {
		HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_RESET);
	}
}

void deviceDeselect() {
	HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_SET);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
