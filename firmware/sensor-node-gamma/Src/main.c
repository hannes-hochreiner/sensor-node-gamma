
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
  MX_I2C1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_Init(&hi2c1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // LL_GPIO_ResetOutputPin(RFM_ENABLE_GPIO_Port, RFM_ENABLE_Pin);
    LL_GPIO_SetOutputPin(RFM_ENABLE_GPIO_Port, RFM_ENABLE_Pin);

    volatile HAL_StatusTypeDef status;
    status = HAL_I2C_IsDeviceReady(&hi2c1, 0xE0, 10, 1000);
    // status = HAL_SMBUS_IsDeviceReady(&hsmbus1, 0xE0, 10, 1000);

    // reset
    // uint8_t comReset[] = {0x06};
    // status = HAL_I2C_Master_Transmit(&hi2c1, 0x00, comReset, 1, 1000);

    // if (status != HAL_OK) {
    //   volatile uint32_t errorCode = HAL_I2C_GetError(&hi2c1);
    //   volatile int tmp = 5;
    // }
    // HAL_Delay(1);

    // uint8_t comSleep[] = {0xB0, 0x98};
    // status = HAL_I2C_Master_Transmit(&hi2c1, 0xE0, comSleep, 2, 1000);
    // HAL_Delay(1);

    uint8_t comWake[] = {0x35, 0x17};
    status = HAL_I2C_Master_Transmit(&hi2c1, 0xE0, comWake, 2, 1000);
    // HAL_Delay(1);
    LL_mDelay(1);

    uint8_t comMeas[] = {0x7C, 0xA2};
    status = HAL_I2C_Master_Transmit(&hi2c1, 0xE0, comMeas, 2, 1000);
    
    uint8_t values[6];
    status = HAL_I2C_Master_Receive(&hi2c1, 0xE0, values, 6, 1000);

    uint8_t comSleep[] = {0xB0, 0x98};
    status = HAL_I2C_Master_Transmit(&hi2c1, 0x70, comSleep, 2, 1000);

    volatile uint16_t valueTemp = ((uint16_t)values[0] << 8) + (uint16_t)values[1];
    volatile uint16_t valueHum = ((uint16_t)values[3] << 8) + (uint16_t)values[4];

    CRC_Config_Sensor();

    volatile uint32_t crcTempCalc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&values[0], 2);
    volatile uint32_t crcTempTrans = (uint32_t)values[2];

    volatile uint32_t crcHumCalc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&values[3], 2);
    volatile uint32_t crcHumTrans = (uint32_t)values[5];

    //
    // transmitter
    //
    LL_GPIO_ResetOutputPin(RFM_ENABLE_GPIO_Port, RFM_ENABLE_Pin);
    LL_mDelay(100);

    // properties to be set:
    uint8_t transRes;
    // PA_CONFIG (0x60): 0x00, 0x4D, 0x00, 0x19, 0x7D, 0xF3 => 9 dBm
    uint8_t comPaConfig[] = {0x11, 0x60, 0x00, 0x4D, 0x00, 0x19, 0x7D, 0xF3};
    TransmitterCommand(comPaConfig, 8, &transRes, 1);
    if (transRes != 0x80) { Error_Handler(); }
    // TX_FREQ (0x40): 0x19, 0xDE, 0x64, 0x08 => 434.005 MHz
    uint8_t comTxFreq[] = {0x11, 0x40, 0x19, 0xDE, 0x64, 0x08};
    TransmitterCommand(comTxFreq, 6, &transRes, 1);
    if (transRes != 0x80) { Error_Handler(); }
    // MODULATION_FSKDEV (0x20): 0x01, 0x04 => FSK, 5 kHz
    uint8_t comModulationFskdev[] = {0x11, 0x20, 0x01, 0x04};
    TransmitterCommand(comModulationFskdev, 4, &transRes, 1);
    if (transRes != 0x80) { Error_Handler(); }
    // BITRATE_CONFIG (0x31): 0x00, 0x30, 0x04 => 4.8 kb/s
    uint8_t comBitrateConfig[] = {0x11, 0x31, 0x00, 0x30, 0x04};
    TransmitterCommand(comBitrateConfig, 5, &transRes, 1);
    if (transRes != 0x80) { Error_Handler(); }

    // set fifo
    uint8_t comSetFifo[] = {0x66, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x46, 0xA5, 0xE3, 0x05, 0x48, 0x45, 0x4C, 0x4C, 0x4F, 0x00, 0x00};

    // add crc calculation
    CRC_Config_Transmitter();
    uint32_t crcTrans = HAL_CRC_Calculate(&hcrc, (uint32_t*)&comSetFifo[19], 6);

    comSetFifo[25] = (uint8_t)(crcTrans >> 8);
    comSetFifo[26] = (uint8_t)crcTrans;

    TransmitterCommand(comSetFifo, 27, &transRes, 1);
    if (transRes != 0x80) { Error_Handler(); }

    uint8_t comTxStart[] = {0x62, 0x00, 0x10, 0x00, 0x00, 0x00};
    uint8_t dataTxStart[2];
    TransmitterCommand(comTxStart, 6, dataTxStart, 2);
    if (dataTxStart[0] != 0x80) { Error_Handler(); }

    uint8_t comGetIntStatus[] = {0x64};
    uint8_t dataGetIntStatus[2];
    volatile uint8_t pkgSent = 0;

    while (pkgSent != 8) {
      LL_mDelay(50);
      dataGetIntStatus[1] = 0;
      TransmitterCommand(comGetIntStatus, 1, dataGetIntStatus, 2);
      if (dataGetIntStatus[0] != 0x80) { Error_Handler(); }
      pkgSent = dataGetIntStatus[1] & 0x08;
    }

    LL_GPIO_ResetOutputPin(RFM_ENABLE_GPIO_Port, RFM_ENABLE_Pin);
    LL_mDelay(500);
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
void CRC_Config_Sensor() {
  HAL_CRC_DeInit(&hcrc);
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 0x31;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
  hcrc.Init.InitValue = 0xFF;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  HAL_CRC_Init(&hcrc);
}

void CRC_Config_Transmitter() {
  HAL_CRC_DeInit(&hcrc);
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 0x1D0F;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 0x1D0F;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  HAL_CRC_Init(&hcrc);
}

void TransmitterCommand(uint8_t* comBuf, uint8_t comLen, uint8_t* resBuf, uint8_t resLen) {
  volatile HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Master_Transmit(&hi2c1, 0xE0, comBuf, comLen, 1000);

  if (status != HAL_OK) {
    volatile uint32_t error = HAL_I2C_GetError(&hi2c1);
    Error_Handler();
  }

  status = HAL_I2C_Master_Receive(&hi2c1, 0xE0, resBuf, resLen, 1000);

  for (uint8_t cntr = 0; cntr < 15 && status != HAL_OK; cntr++) {
    LL_mDelay(10);
    status = HAL_I2C_Master_Receive(&hi2c1, 0xE0, resBuf, resLen, 1000);
  }

  if (status != HAL_OK) {
    volatile uint32_t error = HAL_I2C_GetError(&hi2c1);
    Error_Handler();
  }
}
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
