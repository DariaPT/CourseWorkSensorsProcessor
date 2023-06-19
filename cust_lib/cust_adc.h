#ifndef __CUST_ADC_H__
#define __CUST_ADC_H__

#include "stm32f4xx.h"

void cust_adc_init(void);
u16 cust_adc_read(u8 channel);
u16 cust_adc_read_chan1(void);

#endif
