#ifndef __USB_PRINTER_H__
#define __USB_PRINTER_H__

#include "stm32f4xx.h"

void usb_printer_printf(char* format, ...);
void usb_printer_init();
void usb_send_bytes(u8 *sendBuf, u32 size);

#endif

