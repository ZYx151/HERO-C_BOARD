#ifndef BSP_USB_H
#define BSP_USB_H

#include "usb_typdef.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#include "CRC.h"
/* 在任务中持续进行 */
 void UsbReceiveData(void);
 
void UsbTransmit(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len);


#endif

