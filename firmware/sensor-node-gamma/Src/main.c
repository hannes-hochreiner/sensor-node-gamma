
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "aes.h"
#include "crc.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_AES_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  uint32_t message_index = 0;
  rfm9x_t rfm98;
  RFM98Glue_Init(&rfm98);
  RFM9X_Init(&rfm98);
  uint8_t syncWord[] = {0x46, 0xA5, 0xE3};
  RFM9X_SetSyncWord(&rfm98, syncWord, 3);
  uint8_t power = 0x08;
  RFM9X_SetPower(&rfm98, &power);

  rfm9x_flags_t flags;
  RFM9X_GetFlags(&rfm98, &flags);

  rfm9x_mode_t setMode = RFM9X_MODE_TRANSMIT;
  RFM9X_SetMode(&rfm98, &setMode);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LL_I2C_Enable(I2C1);
    uint8_t address = 0xE0;

    uint8_t comWake[] = {0x35, 0x17};
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}
    LL_I2C_TransmitData8(I2C1, comWake[0]);
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}
    LL_I2C_TransmitData8(I2C1, comWake[1]);
    // LL_mDelay(1);

    uint8_t comId[] = {0xEF, 0xC8};
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}
    LL_I2C_TransmitData8(I2C1, comId[0]);
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}
    LL_I2C_TransmitData8(I2C1, comId[1]);

    uint8_t i2cSize = 3;
    uint8_t valuesId[i2cSize];
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, i2cSize, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    for (uint8_t cntr = 0; cntr < i2cSize; cntr++) {
      while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {}
      valuesId[cntr] = LL_I2C_ReceiveData8(I2C1);
    }

    uint8_t comMeas[] = {0x7C, 0xA2};
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}
    LL_I2C_TransmitData8(I2C1, comMeas[0]);
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}
    LL_I2C_TransmitData8(I2C1, comMeas[1]);
    
    i2cSize = 6;
    uint8_t values[i2cSize];
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, i2cSize, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    for (uint8_t cntr = 0; cntr < i2cSize; cntr++) {
      while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {}
      values[cntr] = LL_I2C_ReceiveData8(I2C1);
    }

    volatile uint32_t i2cStatus = LL_I2C_IsActiveFlag_BERR(I2C1);

    uint8_t comSleep[] = {0xB0, 0x98};
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}
    LL_I2C_TransmitData8(I2C1, comSleep[0]);
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {}
    LL_I2C_TransmitData8(I2C1, comSleep[1]);

    LL_I2C_Disable(I2C1);

    // check crc start
    LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_8B);
    LL_CRC_SetInitialData(CRC, 0xFF);
    LL_CRC_SetPolynomialCoef(CRC, 0x31);
    LL_CRC_ResetCRCCalculationUnit(CRC);
    LL_CRC_FeedData16(CRC, ((uint16_t)values[0] << 8) + values[1]);

    if (values[2] != LL_CRC_ReadData8(CRC)) {
      volatile uint8_t tmp = 0x00;
    }

    LL_CRC_ResetCRCCalculationUnit(CRC);
    LL_CRC_FeedData16(CRC, ((uint16_t)values[3] << 8) + values[4]);

    if (values[5] != LL_CRC_ReadData8(CRC)) {
      volatile uint8_t tmp = 0x00;
    }

    LL_CRC_ResetCRCCalculationUnit(CRC);
    LL_CRC_FeedData16(CRC, ((uint16_t)valuesId[0] << 8) + valuesId[1]);

    if (valuesId[2] != LL_CRC_ReadData8(CRC)) {
      volatile uint8_t tmp = 0x00;
    }
    // check crc end

    volatile double valTemp = 175 * ((double)(((uint16_t)values[0] << 8) + values[1]) / (1 << 16)) - 45;
    volatile double valHum = 100 * ((double)(((uint16_t)values[3] << 8) + values[4]) / (1 << 16));

    volatile message_0001_t msg = {
      .type = 0x0001,
      .mcu_id_1 = LL_GetUID_Word0(),
      .mcu_id_2 = LL_GetUID_Word1(),
      .mcu_id_3 = LL_GetUID_Word2(),
      .message_index = ++message_index,
      .sensor_id = (((uint16_t)valuesId[0] << 8) + valuesId[1]),
      .temperature = (float)valTemp,
      .humidity = (float)valHum,
      ._rng = HAL_GetTick() ^ (((uint16_t)values[1] << 8) + values[4])
    };

    // encrypt start
    // size_t dataSize = sizeof(message_0001_t);
    size_t dataSize = 32;
    uint8_t dataPlain[dataSize] __attribute__ ((aligned (32)));
    // uint8_t dataPlainNew[dataSize] __attribute__ ((aligned (32)));
    uint8_t dataCrypt[dataSize] __attribute__ ((aligned (32)));

    memcpy(dataPlain, &msg, dataSize);
    volatile HAL_StatusTypeDef statusCrypt = HAL_CRYP_Init(&hcryp);

    if (statusCrypt != HAL_OK) {
      volatile uint8_t tmp = 0;
    }

    statusCrypt = HAL_CRYP_AESECB_Encrypt(&hcryp, dataPlain, dataSize, dataCrypt, 1000);

    if (statusCrypt != HAL_OK) {
      volatile uint8_t tmp = 0;
    }

    HAL_CRYP_DeInit(&hcryp);
    // HAL_CRYP_Init(&hcryp);
    
    // statusCrypt = HAL_CRYP_AESECB_Decrypt(&hcryp, dataCrypt, dataSize, dataPlainNew, 1000);

    // if (statusCrypt != HAL_OK) {
    //   volatile uint8_t tmp = 0;
    // }
    // HAL_CRYP_DeInit(&hcryp);

    // volatile message_0001_t msgNew;
    // memcpy(&msgNew, dataPlainNew, dataSize);
    // encrypt end

    // uint8_t text[] = "Hello World!";
    // RFM9X_WriteMessage(&rfm98, text, 12);
    RFM9X_WriteMessage(&rfm98, values, 6);
    LL_mDelay(10);

    RFM9X_GetFlags(&rfm98, &flags);

    while(!(flags & RFM9X_FLAG_PACKET_SENT)) {
      LL_mDelay(1);
      RFM9X_GetFlags(&rfm98, &flags);
    }

    LL_mDelay(1000);
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

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_4, LL_RCC_PLL_DIV_2);

  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);

  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(32000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

  LL_SetSystemCoreClock(32000000);

  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
