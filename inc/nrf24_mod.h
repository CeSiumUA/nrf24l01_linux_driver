#ifndef __NRF24_MOD_H__
#define __NRF24_MOD_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/timer.h>
#include "nrf24_hal.h"

#define FIFO_SIZE			                            65536

struct nrf24_device_config_t {
    enum nrf24_crc_mode_t crc_mode;
    enum nrf24_air_data_rate_t data_rate;
    enum nrf24_address_width_t addr_width;
    enum nrf24_auto_retransmit_count_t auto_retransmit_count;
    enum nrf24_auto_retransmit_delay_t auto_retransmit_delay;
    enum nrf24_mode_t mode;
    enum nrf24_tx_power_t tx_power;
};

struct nrf24_pipe_config_t {
    u64 addr;
    ssize_t plw;
}

struct nrf24_pipe_t {
    int id;
    dev_t devt;
    struct device *dev;
    struct cdev cdev;
    struct nrf24_pipe_config_t config;

    DECLARE_KFIFO(rx_fifo, u8, FIFO_SIZE);
    struct mutex rx_fifo_lock;
    wait_queue_head_t read_wait_queue;
    wait_queue_head_t write_wait_queue;

    u32 sent;
    bool write_done;
};

struct nrf24_tx_data_t {
    struct nrf24_pipe *pipe;
    u8 size;
    u8 payload[NRF24_MAX_PAYLOAD_SIZE];
}

struct nrf24_device_t {
    u32 id;
    struct device dev;
    struct spi_device *spi;
    struct gpio_desc *ce;

    struct nrf24_device_config_t config;

    spinlock_t lock;

    struct work_struct isr_work;
    struct work_struct rx_work;

    STRUCT_KFIFO_REC_1(FIFO_SIZE)   tx_fifo;
    struct mutex tx_fifo_lock;
    struct task_struct *tx_task_struct;
    wait_queue_head_t tx_wait_queue;
    wait_queue_head_t rx_wait_queue;
    bool tx_done;
    bool tx_failed;

    struct timer_list rx_active_timer;
    bool rx_active;
}

#define to_nrf24_device(device)	                        container_of(device, struct nrf24_device, dev)

#endif // __NRF24_MOD_H__