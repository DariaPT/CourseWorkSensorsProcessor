#include "usb_printer.h"

#include <stdio.h>
#include <stdarg.h>

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usb_dcd_int.h"

#include "stm32f4xx_exti.h"

/*
 * The USB data must be 4 byte aligned if DMA is enabled. This macro handles
 * the alignment, if necessary (it's actually magic, but don't tell anyone).
 */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

void OTG_FS_IRQHandler(void);
void OTG_FS_WKUP_IRQHandler(void);


// Почему-то линковщик ругается
void usb_send_bytes(u8 *sendBuf, u32 size)
{
	VCP_send_buffer(sendBuf, size);
}

void usb_printer_printf(char* format, ...)
{
	static char usbOutputBuf[512] = { 0 };

	va_list args;
	va_start(args, format);

	vsnprintf(usbOutputBuf, sizeof(usbOutputBuf), format, args);

	VCP_send_str(usbOutputBuf);
	va_end(args);
}

void usb_printer_init()
{
	USBD_Init(&USB_OTG_dev,
	            USB_OTG_FS_CORE_ID,
	            &USR_desc,
	            &USBD_CDC_cb,
	            &USR_cb);
}

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
