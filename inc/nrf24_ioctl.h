#ifndef __NRF24_IOCTL_H__
#define __NRF24_IOCTL_H__

#include <linux/ioctl.h>
#include "nrf24_mod.h"

#define NRF24_IOCTL_MAGIC           0x16

#define NRF24_IOCTL_SET_CHANNEL             _IOW(NRF24_IOCTL_MAGIC, 0, u8 *)
#define NRF24_IOCTL_GET_CHANNEL             _IOR(NRF24_IOCTL_MAGIC, 1, u8 *)

#define NRF24_IOCTL_SET_RX_PAYLOAD_SIZE     _IOW(NRF24_IOCTL_MAGIC, 2, u8 *)
#define NRF24_IOCTL_GET_RX_PAYLOAD_SIZE     _IOR(NRF24_IOCTL_MAGIC, 3, u8 *)

#define NRF24_IOCTL_SET_DATA_RATE           _IOW(NRF24_IOCTL_MAGIC, 4, u8 *)
#define NRF24_IOCTL_GET_DATA_RATE           _IOR(NRF24_IOCTL_MAGIC, 5, u8 *)

long nrf24_handle_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif // __NRF24_IOCTL_H__