#define HSE_VALUE ((uint32_t)8000000) /* STM32 discovery uses a 8Mhz external crystal */

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"

#include "cust_lib/cust_adc.h"
#include "cust_lib/usb_printer.h"

#define ADC_POLL_PERIOD_SEC 2
#define CUST_PACKET_SYMBOL 0xFFFF

volatile uint32_t ticker;
volatile uint32_t secSincePowerOn = 0;

uint32_t lastAdcPollTimeSec = 0;

static void cust_init();
static void set_error_indication(void);


void SysTick_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);

struct CustPacket
{
	u16 prefix;
	u16 temperature;
	u16 humidity;
	u16 light;
	u16 postfix;
}
__attribute__((packed));


struct CustPacket custPacket = {
		.prefix = CUST_PACKET_SYMBOL,
		.temperature = 0,
		.humidity = 0,
		.light = 0,
		.postfix = CUST_PACKET_SYMBOL
};

int main(void)
{
	/* Set up the system clocks */
	SystemInit();

	cust_init();

	u8 Rh,RhDec,Temp,TempDec,ChkSum;
	double devPoint;
	char str[20];

	while (1)
	{
		uint8_t theByte;
		if (VCP_get_char(&theByte))
		{
//			DHT11Read(&Rh,&RhDec,&Temp,&TempDec,&ChkSum);
			//			devPoint=dewPointFast(Temp,Rh);
			custPacket.temperature = 666;
			custPacket.humidity = 666;
			custPacket.light = 666;

			usb_send_bytes((u8*)&custPacket, sizeof(custPacket));
		}

//		uint32_t timeSinceLastAdcPollSec = secSincePowerOn - lastAdcPollTimeSec;
//
//		if(timeSinceLastAdcPollSec >= ADC_POLL_PERIOD_SEC)
//		{
//			u16 adcRawValue = cust_adc_read_chan1();
//
////			DHT11Read(&Rh,&RhDec,&Temp,&TempDec,&ChkSum);
////			devPoint=dewPointFast(Temp,Rh);
//
////			sprintf(str, "Value= %dRh %d %dC %d %d %fDwP %fFah %fKel\r\n",Rh,RhDec,Temp,TempDec,ChkSum,devPoint,Fahrenheit(Temp),Kelvin(Temp));
//
////			usb_printer_printf("1(V)=%.1f;\r\n", adcRawValue * 0.000806f, secSincePowerOn);
//			usb_printer_printf("Value= %dRh %d %dC %d %d %fDwP %fFah %fKel\r\n", Rh,RhDec,Temp,TempDec,ChkSum,devPoint,Fahrenheit(Temp),Kelvin(Temp));
//
//			lastAdcPollTimeSec = secSincePowerOn;
//		}
	}
	return 0;
}


void cust_init()
{
	GPIO_InitTypeDef LED_Config;

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

	usb_printer_init();
	cust_adc_init();
	DHT11initTIM2();

	return;
}

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
