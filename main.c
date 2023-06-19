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

#include "cust_lib/cust_adc.h"

#include <stdio.h>
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


static void cust_init();
static void set_error_indication(void);

/*
 * Define prototypes for interrupt handlers here. The conditional "extern"
 * ensures the weak declarations from startup_stm32f4xx.c are overridden.
 */

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


// Почему-то линковщик ругается
 void usb_printf(char* format, ...)
 {
	 static char usbOutputBuf[512] = { 0 };

	 va_list args;
	 va_start(args, format);

	 vsnprintf(usbOutputBuf, sizeof(usbOutputBuf), format, args);

	 VCP_send_str(usbOutputBuf);
	 va_end(args);
 }

int main(void)
{
	/* Set up the system clocks */
	SystemInit();

	cust_init();

	while (1)
	{
		uint32_t timeSinceLastAdcPollSec = secSincePowerOn - lastAdcPollTimeSec;

		if(timeSinceLastAdcPollSec >= ADC_POLL_PERIOD_SEC)
		{
			u16 adcRawValue = cust_adc_read_chan1();
//			u16 adcRawValue = ticker % 100;
			usb_printf("1(V)=%.1f {%d};\r\n", adcRawValue * 0.000806f, secSincePowerOn);

			lastAdcPollTimeSec = secSincePowerOn;
		}
	}

	return 0;
}


void cust_init()
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

	if (SysTick_Config(SystemCoreClock / 1000))
	{
		set_error_indication();
	}


	/* Setup USB */
	USBD_Init(&USB_OTG_dev,
	            USB_OTG_FS_CORE_ID,
	            &USR_desc,
	            &USBD_CDC_cb,
	            &USR_cb);

	cust_adc_init();

	return;
}

/*
 * Call this to indicate a failure.  Blinks the STM32F4 discovery LEDs
 * in sequence.  At 168Mhz, the blinking will be very fast - about 5 Hz.
 * Keep that in mind when debugging, knowing the clock speed might help
 * with debugging.
 */
static void set_error_indication(void)
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
void HardFault_Handler(void) { set_error_indication(); }
void MemManage_Handler(void) { set_error_indication(); }
void BusFault_Handler(void)  { set_error_indication(); }
void UsageFault_Handler(void){ set_error_indication(); }
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
