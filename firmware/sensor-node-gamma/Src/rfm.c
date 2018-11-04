#include "main.h"
#include "stm32l0xx_hal.h"
#include "spi.h"
#include "rfm.h"

void rfm_set_reset_pin() {
  LL_GPIO_ResetOutputPin(RFM_RESET_GPIO_Port, RFM_RESET_Pin);
}

void rfm_reset_reset_pin() {
  LL_GPIO_SetOutputPin(RFM_RESET_GPIO_Port, RFM_RESET_Pin);
}

void rfm_set_spi_nss_pin() {
  LL_GPIO_SetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
}

void rfm_reset_spi_nss_pin() {
  LL_GPIO_ResetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
}

void rfm_delay(uint8_t millisec) {
  LL_mDelay(millisec);
}

void rfm_spi_transfer(uint8_t* const data) {
  HAL_SPI_Transmit(&hspi1, data, 1, 1000);
  HAL_SPI_Receive(&hspi1, data, 1, 1000);
}
