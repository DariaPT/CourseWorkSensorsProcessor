#include "cust_adc.h"

#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

// ALL BLOKING!!!

void cust_adc_init(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  ADC_DeInit();

  ADC_StructInit(&ADC_InitStructure);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0 , 1, ADC_SampleTime_144Cycles);

  ADC_Cmd(ADC1, ENABLE);

  ADC_SoftwareStartConv(ADC1);
}


u16 cust_adc_read(u8 channel)
{
	return ADC_GetConversionValue(ADC1);
}

u16 cust_adc_read_chan1(void)
{
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	    return ADC_GetConversionValue(ADC1);

	return 0;
}
