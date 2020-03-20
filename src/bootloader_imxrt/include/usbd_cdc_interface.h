#ifndef __USBD_CDC_INTERFACE_H_
#define __USBD_CDC_INTERFACE_H_

int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length);
int8_t CDC_Itf_Receive(uint8_t *Buf, uint32_t Len);

#endif //__USBD_CDC_INTERFACE_H_

