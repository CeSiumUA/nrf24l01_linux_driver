#ifndef __NRF24_HAL_H__
#define __NRF24_HAL_H__

#ifdef __KERNEL__
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
#else
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#endif

#ifdef __KERNEL__
typedef struct spi_device nrf24_spi_t;
typedef enum nrf24_hal_status_t {
    HAL_OK = 0,
    HAL_ERROR,
} nrf24_hal_status_t;
typedef struct gpio_desc * nrf24_gpio_pin_t;

#define NRF24_HAL_SET_PIN_HIGH(gpio_pin)                    gpiod_set_value((gpio_pin), 1);
#define NRF24_HAL_SET_PIN_LOW(gpio_pin)                     gpiod_set_value((gpio_pin), 0);
#else
typedef SPI_HandleTypeDef nrf24_spi_t;
typedef GPIO_TypeDef nrf24_gpio_t;
typedef HAL_StatusTypeDef nrf24_hal_status_t;
typedef struct nrf24_gpio_pin_t {
    nrf24_gpio_t *port;
    uint16_t pin;
} nrf24_gpio_pin_t;

#define NRF24_HAL_SET_PIN_HIGH(gpio_pin)                    HAL_GPIO_WritePin((gpio_pin.port), (gpio_pin.pin), (GPIO_PIN_SET))
#define NRF24_HAL_SET_PIN_LOW(gpio_pin)                     HAL_GPIO_WritePin((gpio_pin.port), (gpio_pin.pin), (GPIO_PIN_RESET))
#endif

/*Commands*/
#define NRF24_CMD_R_REGISTER(reg)                (0x00 | (reg & 0x1F))
#define NRF24_CMD_W_REGISTER(reg)                (0x20 | (reg & 0x1F))
#define NRF24_CMD_R_RX_PAYLOAD                   (0x61)
#define NRF24_CMD_W_TX_PAYLOAD                   (0xA0)
#define NRF24_CMD_FLUSH_TX                       (0xE1)
#define NRF24_CMD_FLUSH_RX                       (0xE2)
#define NRF24_CMD_REUSE_TX_PL                    (0xE3)
#define NRF24_CMD_ACTIVATE                       (0x50)
#define NRF24_CMD_R_RX_PL_WID                    (0x60)
#define NRF24_CMD_W_ACK_PAYLOAD(pipe)            (0xA8 | (pipe & 0x07))
#define NRF24_CMD_W_TX_PAYLOAD_NOACK             (0xB0)
#define NRF24_CMD_NOP                            (0xFF)

/*Registers*/
#define NRF24_REG_CONFIG                         (0x00)
#define NRF24_REG_CONFIG_RESET_VALUE             (0b00001000)

#define NRF2RL01_REG_CONFIG_MASK_PRIM_RX         (1 << 0)
#define NRF2RL01_REG_CONFIG_MASK_PWR_UP          (1 << 1)
#define NRF2RL01_REG_CONFIG_MASK_CRCO            (1 << 2)
#define NRF2RL01_REG_CONFIG_MASK_EN_CRC          (1 << 3)
#define NRF2RL01_REG_CONFIG_MASK_MASK_MAX_RT     (1 << 4)
#define NRF2RL01_REG_CONFIG_MASK_MASK_TX_DS      (1 << 5)
#define NRF2RL01_REG_CONFIG_MASK_MASK_RX_DR      (1 << 6)

#define NRF24_REG_EN_AA                          (0x01)
#define NRF24_REG_EN_AA_RESET_VALUE              (0b00111111)

#define NRF24_REG_EN_AA_MASK_ENAA_P0             (1 << 0)
#define NRF24_REG_EN_AA_MASK_ENAA_P1             (1 << 1)
#define NRF24_REG_EN_AA_MASK_ENAA_P2             (1 << 2)
#define NRF24_REG_EN_AA_MASK_ENAA_P3             (1 << 3)
#define NRF24_REG_EN_AA_MASK_ENAA_P4             (1 << 4)
#define NRF24_REG_EN_AA_MASK_ENAA_P5             (1 << 5)

#define NRF24_REG_EN_RXADDR                      (0x02)
#define NRF24_REG_EN_RXADDR_RESET_VALUE          (0b00000011)

#define NRF24_REG_EN_RXADDR_MASK_ERX_P0          (1 << 0)
#define NRF24_REG_EN_RXADDR_MASK_ERX_P1          (1 << 1)
#define NRF24_REG_EN_RXADDR_MASK_ERX_P2          (1 << 2)
#define NRF24_REG_EN_RXADDR_MASK_ERX_P3          (1 << 3)
#define NRF24_REG_EN_RXADDR_MASK_ERX_P4          (1 << 4)
#define NRF24_REG_EN_RXADDR_MASK_ERX_P5          (1 << 5)

#define NRF24_REG_SETUP_AW                       (0x03)
#define NRF24_REG_SETUP_AW_RESET_VALUE           (0b00000011)

#define NRF24_REG_SETUP_AW_MASK_3_BYTES          (0x01)
#define NRF24_REG_SETUP_AW_MASK_4_BYTES          (0x02)
#define NRF24_REG_SETUP_AW_MASK_5_BYTES          (0x03)

#define NRF24_REG_SETUP_RETR                     (0x04)
#define NRF24_REG_SETUP_RETR_RESET_VALUE         (0b00000011)

#define NRF24_REG_RF_CH                          (0x05)
#define NRF24_REG_RF_CH_RESET_VALUE              (0b00000010)

#define NRF24_REG_RF_SETUP                       (0x06)
#define NRF24_REG_RF_SETUP_RESET_VALUE           (0b00001111)

#define NRF24_REG_STATUS                         (0x07)
#define NRF24_REG_STATUS_RESET_VALUE             (0b01111110)
#define NRF24_REG_STATUS_MASK_RX_DR              (1 << 6)
#define NRF24_REG_STATUS_MASK_TX_DS              (1 << 5)
#define NRF24_REG_STATUS_MASK_MAX_RT             (1 << 4)
#define NRF24_REG_STATUS_MASK_RX_P_NO            (0b00001110)
#define NRF24_REG_STATUS_MASK_TX_FULL            (1 << 0)

#define NRF24_REG_OBSERVE_TX                     (0x08)
#define NRF24_REG_OBSERVE_TX_RESET_VALUE         (0b00000000)

#define NRF24_REG_CD                             (0x09)
#define NRF24_REG_RX_ADDR_P0                     (0x0A)
#define NRF24_REG_RX_ADDR_P1                     (0x0B)
#define NRF24_REG_RX_ADDR_P2                     (0x0C)
#define NRF24_REG_RX_ADDR_P3                     (0x0D)
#define NRF24_REG_RX_ADDR_P4                     (0x0E)
#define NRF24_REG_RX_ADDR_P5                     (0x0F)
#define NRF24_REG_TX_ADDR                        (0x10)
#define NRF24_REG_RX_PW_P0                       (0x11)
#define NRF24_REG_RX_PW_P1                       (0x12)
#define NRF24_REG_RX_PW_P2                       (0x13)
#define NRF24_REG_RX_PW_P3                       (0x14)
#define NRF24_REG_RX_PW_P4                       (0x15)
#define NRF24_REG_RX_PW_P5                       (0x16)
#define NRF24_REG_RX_PW_RESET_VALUE              (0b00000000)

#define NRF24_REG_FIFO_STATUS                    (0x17)
#define NRF24_REG_FIFO_STATUS_RESET_VALUE        (0b00010001)
#define NRF24_REG_FIFO_STATUS_MASK_RX_EMPTY      (1 << 0)
#define NRF24_REG_FIFO_STATUS_MASK_RX_FULL       (1 << 1)
#define NRF24_REG_FIFO_STATUS_MASK_TX_EMPTY      (1 << 4)
#define NRF24_REG_FIFO_STATUS_MASK_TX_FULL       (1 << 5)

#define NRF24_REG_DYNPD                          (0x1C)
#define NRF24_REG_DYNPD_RESET_VALUE              (0b00000000)

#define NRF24_REG_FEATURE                        (0x1D)
#define NRF24_REG_FEATURE_RESET_VALUE            (0b00000000)

#define NRF24_BASE_FREQUENCY                     (2400)
#define NRF24_MAX_CHANNEL                        (127)
#define NRF24_MIN_CHANNEL                        (1)

#define NRF24_MAX_PAYLOAD_SIZE                   (32)

#define NRF24_PIPES_COUNT                        (6)

struct nrf24_t {
    nrf24_spi_t *spi;
    nrf24_gpio_pin_t csn;
    nrf24_gpio_pin_t ce;
    nrf24_gpio_pin_t irq;
};

enum nrf24_address_width_t {
    NRF24_AW_ILLEGAL = 0,
    NRF24_AW_3_BYTES = 3,
    NRF24_AW_4_BYTES,
    NRF24_AW_5_BYTES
};

enum nrf24_auto_retransmit_delay_t {
    NRF24_ARD_250_US = 0,
    NRF24_ARD_500_US,
    NRF24_ARD_750_US,
    NRF24_ARD_1000_US,
    NRF24_ARD_1250_US,
    NRF24_ARD_1500_US,
    NRF24_ARD_1750_US,
    NRF24_ARD_2000_US,
    NRF24_ARD_2250_US,
    NRF24_ARD_2500_US,
    NRF24_ARD_2750_US,
    NRF24_ARD_3000_US,
    NRF24_ARD_3250_US,
    NRF24_ARD_3500_US,
    NRF24_ARD_3750_US,
    NRF24_ARD_4000_US
};

enum nrf24_auto_retransmit_count_t {
    NRF24_ARC_DISABLED = 0,
    NRF24_ARC_1,
    NRF24_ARC_2,
    NRF24_ARC_3,
    NRF24_ARC_4,
    NRF24_ARC_5,
    NRF24_ARC_6,
    NRF24_ARC_7,
    NRF24_ARC_8,
    NRF24_ARC_9,
    NRF24_ARC_10,
    NRF24_ARC_11,
    NRF24_ARC_12,
    NRF24_ARC_13,
    NRF24_ARC_14,
    NRF24_ARC_15
};

enum nrf24_air_data_rate_t {
    NRF24_ADR_1_MBPS = 0,
    NRF24_ADR_2_MBPS
};

enum nrf24_crc_mode_t {
    NRF24_CRC_DISABLED,
    NRF24_CRC_1_BYTE = 2,
    NRF24_CRC_2_BYTES
};

enum nrf24_mode_t {
    NRF24_PM_TX,
    NRF24_PM_RX
};

enum nrf24_power_t {
    NRF24_PWR_DOWN = 0,
    NRF24_PWR_UP,
};

enum nrf24_tx_power_t {
    NRF24_TXP_ATTENUATION_18_DBM = 0,
    NRF24_TXP_ATTENUATION_12_DBM,
    NRF24_TXP_ATTENUATION_6_DBM,
    NRF24_TXP_0_DBM
};

void nrf24_ce_on(struct nrf24_t *nrf24);
void nrf24_ce_off(struct nrf24_t *nrf24);
nrf24_hal_status_t nrf24_read_register(struct nrf24_t *nrf24, uint8_t reg, uint8_t *data, size_t len);
nrf24_hal_status_t nrf24_write_register(struct nrf24_t *nrf24, uint8_t reg, uint8_t *data, size_t len);
nrf24_hal_status_t nrf24_power_up(struct nrf24_t *nrf24);
nrf24_hal_status_t nrf24_power_down(struct nrf24_t *nrf24);
nrf24_hal_status_t nrf24_set_prx_mode(struct nrf24_t *nrf24);
nrf24_hal_status_t nrf24_set_ptx_mode(struct nrf24_t *nrf24);
nrf24_hal_status_t nrf24_set_crc_mode(struct nrf24_t *nrf24, enum nrf24_crc_mode_t crc_mode);
nrf24_hal_status_t nrf24_get_crc_mode(struct nrf24_t *nrf24, enum nrf24_crc_mode_t *crc_mode);
nrf24_hal_status_t nrf24_get_config(struct nrf24_t *nrf24, uint8_t *config);
nrf24_hal_status_t nrf24_set_auto_ack(struct nrf24_t *nrf24, uint8_t pipe, bool enable);
nrf24_hal_status_t nrf24_get_auto_ack(struct nrf24_t *nrf24, uint8_t *en_aa);
nrf24_hal_status_t nrf24_set_en_rx_pipe(struct nrf24_t *nrf24, uint8_t pipe, bool enable);
nrf24_hal_status_t nrf24_get_en_rx_addr(struct nrf24_t *nrf24, uint8_t *en_rxaddr);
nrf24_hal_status_t nrf24_set_address_width(struct nrf24_t *nrf24, enum nrf24_address_width_t addr_width);
nrf24_hal_status_t nrf24_get_address_width(struct nrf24_t *nrf24, enum nrf24_address_width_t *addr_width);
nrf24_hal_status_t nrf24_setup_retransmission(struct nrf24_t *nrf24, enum nrf24_auto_retransmit_delay_t delay, enum nrf24_auto_retransmit_count_t count);
nrf24_hal_status_t nrf24_get_setup_retransmission(struct nrf24_t *nrf24, enum nrf24_auto_retransmit_delay_t *delay, enum nrf24_auto_retransmit_count_t *count);
nrf24_hal_status_t nrf24_set_radio_output_power(struct nrf24_t *nrf24, enum nrf24_tx_power_t power);
nrf24_hal_status_t nrf24_get_radio_output_power(struct nrf24_t *nrf24, enum nrf24_tx_power_t *power);
nrf24_hal_status_t nrf24_set_radio_channel(struct nrf24_t *nrf24, uint8_t channel);
nrf24_hal_status_t nrf24_get_radio_channel(struct nrf24_t *nrf24, uint8_t *rf_ch);
nrf24_hal_status_t nrf24_set_radio_data_rate(struct nrf24_t *nrf24, enum nrf24_air_data_rate_t data_rate);
nrf24_hal_status_t nrf24_get_radio_data_rate(struct nrf24_t *nrf24, enum nrf24_air_data_rate_t *data_rate);
nrf24_hal_status_t nrf24_get_rf_setup(struct nrf24_t *nrf24, uint8_t *rf_setup);
nrf24_hal_status_t nrf24_get_status(struct nrf24_t *nrf24, uint8_t *status);
nrf24_hal_status_t nrf24_set_status(struct nrf24_t *nrf24, uint8_t *status);
nrf24_hal_status_t nrf24_clear_status_bit(struct nrf24_t *nrf24, uint8_t mask);
nrf24_hal_status_t nrf24_get_observe_tx(struct nrf24_t *nrf24, uint8_t *observe_tx);
nrf24_hal_status_t nrf24_get_carrier_detect(struct nrf24_t *nrf24, uint8_t *cd);
nrf24_hal_status_t nrf24_set_major_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address);
nrf24_hal_status_t nrf24_set_minor_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address);
nrf24_hal_status_t nrf24_set_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address);
nrf24_hal_status_t nrf24_get_major_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address);
nrf24_hal_status_t nrf24_get_minor_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address);
nrf24_hal_status_t nrf24_get_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address);
nrf24_hal_status_t nrf24_set_tx_address(struct nrf24_t *nrf24, uint8_t *address);
nrf24_hal_status_t nrf24_get_tx_address(struct nrf24_t *nrf24, uint8_t *address);
nrf24_hal_status_t nrf24_set_rx_payload_width(struct nrf24_t *nrf24, uint8_t pipe, uint8_t width);
nrf24_hal_status_t nrf24_get_rx_payload_width(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *width);
nrf24_hal_status_t nrf24_get_fifo_status(struct nrf24_t *nrf24, uint8_t *fifo_status);
nrf24_hal_status_t nrf24_get_dynpd(struct nrf24_t *nrf24, uint8_t pipe_id, uint8_t *dynpd);
nrf24_hal_status_t nrf24_set_dynpd(struct nrf24_t *nrf24, uint8_t pipe, bool enable);
nrf24_hal_status_t nrf24_get_feature(struct nrf24_t *nrf24, uint8_t *feature);
nrf24_hal_status_t nrf24_set_feature(struct nrf24_t *nrf24, uint8_t feature);
nrf24_hal_status_t nrf24_soft_reset(struct nrf24_t *nrf24);
nrf24_hal_status_t nrf24_write_tx_fifo(struct nrf24_t *nrf24, uint8_t *data, size_t len);
nrf24_hal_status_t nrf24_read_rx_fifo(struct nrf24_t *nrf24, uint8_t *data, size_t len);
nrf24_hal_status_t nrf24_flush_tx_fifo(struct nrf24_t *nrf24);
nrf24_hal_status_t nrf24_flush_rx_fifo(struct nrf24_t *nrf24);

#endif // __NRF24_HAL_H__