
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

    status = HAL_CRCEx_Polynomial_Set(&hcrc, 0x31, CRC_POLYLENGTH_8B);

    volatile uint32_t crcTempCalc = HAL_CRC_Calculate(&hcrc, (uint32_t)&values, 2);
    volatile uint32_t crcTempTrans = (uint32_t)values[2];

    volatile uint32_t crcHumCalc = HAL_CRC_Calculate(&hcrc, (uint32_t)&values + 3, 2);
    volatile uint32_t crcHumTrans = (uint32_t)values[5];

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
