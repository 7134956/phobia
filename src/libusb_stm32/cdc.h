#ifndef __CDC_H__
#define __CDC_H__

#include <stdint.h>

/*******************************************************************************
 * Настроить и запустить USB и VCP
 ******************************************************************************/
void USBCDC_startup();
int USBCDC_getc();
void USBCDC_putc(int c);

#endif /* __CDC_H__ */
