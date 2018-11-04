#ifndef __rmf_h_
#define __rmf_h_

#include "rfm9x.h"

void rfm_set_reset_pin();
void rfm_reset_reset_pin();
void rfm_set_spi_nss_pin();
void rfm_reset_spi_nss_pin();
void rfm_delay(uint8_t millisec);
void rfm_spi_transfer(uint8_t* const data);

#endif
