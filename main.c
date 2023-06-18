
#define HSE_VALUE ((uint32_t)8000000) /* STM32 discovery uses a 8Mhz external crystal */

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usb_dcd_int.h"
#include "stm32f4xx_adc.h"

#include <stdio.h>
//#include <stdlib.h>
#include <stdarg.h>

#define ADC_POLL_PERIOD_SEC 2

volatile uint32_t ticker;
volatile uint32_t secSincePowerOn = 0;

uint32_t lastAdcPollTimeSec = 0;
/*
 * The USB data must be 4 byte aligned if DMA is enabled. This macro handles
 * the alignment, if necessary (it's actually magic, but don't tell anyone).
 */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


void init();
void ColorfulRingOfDeath(void);

/*
 * Define prototypes for interrupt handlers here. The conditional "extern"
 * ensures the weak declarations from startup_stm32f4xx.c are overridden.
 */
#ifdef __cplusplus
 extern "C" {
#endif

void SysTick_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void OTG_FS_IRQHandler(void);
void OTG_FS_WKUP_IRQHandler(void);

#ifdef __cplusplus
}
#endif

//void adc_init(void)
//{
//	ADC_InitTypeDef  ADC_InitStructure;
//
//	/* PCLK2 is the APB2 clock */
//	/* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
//	//RCC_ADCCLKConfig(RCC_PCLK2_Div6);
//
//	/* Enable ADC1 clock so that we can talk to it */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//	/* Put everything back to power-on defaults */
//	ADC_DeInit();
//
//	/* Disable the scan conversion so we do one at a time */
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
//	/* Don't do continuous conversions - do them on demand */
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
//	/* Start conversion by software, not an external trigger */
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//	/* Conversions are 12 bit - put them in the lower 12 bits of the result */
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	/* Say how many channels would be used by the sequencer */
//	ADC_InitStructure.ADC_NbrOfChannel = 1;
//
//	/* Now do the setup */
//	ADC_Init(ADC1, &ADC_InitStructure);
//	/* Enable ADC1 */
//	ADC_Cmd(ADC1, ENABLE);
//
//	/* Enable ADC1 reset calibaration register */
//	ADC_ResetCalibration(ADC1);
//	/* Check the end of ADC1 reset calibration register */
//	while(ADC_GetResetCalibrationStatus(ADC1));
//	/* Start ADC1 calibaration */
//	ADC_StartCalibration(ADC1);
//	/* Check the end of ADC1 calibration */
//	while(ADC_GetCalibrationStatus(ADC1));
//}

// Почему-то линковщик ругается
 void usb_printf(char* format, ...)
 {
	 static char usbOutputBuf[512] = { 0 };

	 va_list args;
	 va_start(args, format);

	 vsnprintf(usbOutputBuf, sizeof(usbOutputBuf), format, args);

	 VCP_send_str(usbOutputBuf);
	 va_end(args);

   // ...
 }

 void usb_print_adc_value(u16 adcRawValue)
 {
//	 static char usbOutputBuf[512] = { 0 };
//	 //
//	 snprintf(usbOutputBuf,sizeof(usbOutputBuf), "ADC=%d\r\n", adcRawValue);
//
//	 VCP_send_str(usbOutputBuf);
 }

void adc_init(void)
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
//  ADC_GetCalibrationFactor(ADC1);
  ADC_Cmd(ADC1, ENABLE);

  /*
#define ADC_FLAG_AWD                               ((uint8_t)0x01)
#define ADC_FLAG_EOC                               ((uint8_t)0x02)
#define ADC_FLAG_JEOC                              ((uint8_t)0x04)
#define ADC_FLAG_JSTRT                             ((uint8_t)0x08)
#define ADC_FLAG_STRT                              ((uint8_t)0x10)
#define ADC_FLAG_OVR                               ((uint8_t)0x20)
*/
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_STRT));

  ADC_SoftwareStartConv(ADC1);
}


u16 read_adc_1(u8 channel)
{
//	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);
//	// Start the conversion
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//	// Wait until conversion completion
//	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	// Get the conversion value
	return ADC_GetConversionValue(ADC1);
}

u16 read_adc_chan1(void)
{
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	    return ADC_GetConversionValue(ADC1);

	return 0;
}


int main(void)
{
	/* Set up the system clocks */
	SystemInit();

	/* Initialize USB, IO, SysTick, and all those other things you do in the morning */
	init();

	while (1)
	{
		uint32_t timeSinceLastAdcPollSec = secSincePowerOn - lastAdcPollTimeSec;

		if(timeSinceLastAdcPollSec >= ADC_POLL_PERIOD_SEC)
		{
			//		u16 adcRawValue = read_adc_chan1();
			u16 adcRawValue = ticker % 100;
			usb_printf("1(h)=%d {%d};\r\n", adcRawValue, secSincePowerOn);

			lastAdcPollTimeSec = secSincePowerOn;
		}
	}

	return 0;
}


void init()
{
	/* STM32F4 discovery LEDs */
	GPIO_InitTypeDef LED_Config;

	/* Always remember to turn on the peripheral clock...  If not, you may be up till 3am debugging... */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	LED_Config.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	LED_Config.GPIO_Mode = GPIO_Mode_OUT;
	LED_Config.GPIO_OType = GPIO_OType_PP;
	LED_Config.GPIO_Speed = GPIO_Speed_25MHz;
	LED_Config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &LED_Config);



	/* Setup SysTick or CROD! */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		ColorfulRingOfDeath();
	}


	/* Setup USB */
	USBD_Init(&USB_OTG_dev,
	            USB_OTG_FS_CORE_ID,
	            &USR_desc,
	            &USBD_CDC_cb,
	            &USR_cb);

	// adc_init();

	return;
}

/*
 * Call this to indicate a failure.  Blinks the STM32F4 discovery LEDs
 * in sequence.  At 168Mhz, the blinking will be very fast - about 5 Hz.
 * Keep that in mind when debugging, knowing the clock speed might help
 * with debugging.
 */
void ColorfulRingOfDeath(void)
{
	uint16_t ring = 1;
	while (1)
	{
		uint32_t count = 0;
		while (count++ < 500000);

		GPIOD->BSRRH = (ring << 12);
		ring = ring << 1;
		if (ring >= 1<<4)
		{
			ring = 1;
		}
		GPIOD->BSRRL = (ring << 12);
	}
}

/*
 * Interrupt Handlers
 */
void SysTick_Handler(void)
{
	ticker++;
	secSincePowerOn = ticker / 1000;
}

void NMI_Handler(void)       {}
void HardFault_Handler(void) { ColorfulRingOfDeath(); }
void MemManage_Handler(void) { ColorfulRingOfDeath(); }
void BusFault_Handler(void)  { ColorfulRingOfDeath(); }
void UsageFault_Handler(void){ ColorfulRingOfDeath(); }
void SVC_Handler(void)       {}
void DebugMon_Handler(void)  {}
void PendSV_Handler(void)    {}

void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}
