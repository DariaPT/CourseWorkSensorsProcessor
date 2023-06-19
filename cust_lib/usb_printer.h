#ifndef __USB_PRINTER_H__
#define __USB_PRINTER_H__

#include "stm32f4xx.h"

void usb_printer_printf(char* format, ...);
void usb_printer_init();

#endif

