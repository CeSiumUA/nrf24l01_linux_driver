#ifndef __NRF24_SYSFS_H__
#define __NRF24_SYSFS_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/kfifo.h>
#include <linux/list.h>

extern struct attribute *nrf24_pipe_attrs[];
extern struct attribute *nrf24_attrs[];

#endif // __NRF24_SYSFS_H__