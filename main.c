#define HSE_VALUE ((uint32_t)8000000) /* STM32 discovery uses a 8Mhz external crystal */

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"

#include "cust_lib/cust_adc.h"
#include "cust_lib/usb_printer.h"

#include "math.h"

#define ADC_POLL_PERIOD_SEC 2
#define CUST_PACKET_SYMBOL 0xFFFF

/* Photocell constants */
#define PHT_UP_R 10000.0F
#define PHT_10LX_R 50000.0F
#define PHT_GAMMA 0.8F

/* Photocell vars */
uint32_t Pht_R;
float Pht_Div;
float Pht_Temp;
uint32_t Pht_Lux;


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
			DHT11Read(&Rh,&RhDec,&Temp,&TempDec,&ChkSum);
			devPoint=dewPointFast(Temp,Rh);

			u16 adcRawValue = cust_adc_read_chan1();
			Pht_R = ((PHT_UP_R)/((4095.0)/adcRawValue-1));

			Pht_Div = PHT_10LX_R/Pht_R;
			Pht_Temp = ((0.42*log(Pht_Div))/(PHT_GAMMA)) + 1;

			Pht_Lux = pow(10, Pht_Temp);

			custPacket.temperature = Temp;
			custPacket.humidity = Rh;
			custPacket.light = Pht_Lux;

//			usb_printer_printf("t=%d;h=%d;l=%d\r\n", Temp, Rh,Pht_Lux);
			usb_send_bytes((u8*)&custPacket, sizeof(custPacket));
		}

		uint32_t timeSinceLastAdcPollSec = secSincePowerOn - lastAdcPollTimeSec;
		if(timeSinceLastAdcPollSec >= ADC_POLL_PERIOD_SEC)
		{
			// Здесь можно добавить опрос по времени с последующим занесением в структуру.
			// Чтобы данные были готовы заранее
			lastAdcPollTimeSec = secSincePowerOn;
		}
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
