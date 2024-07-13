#include "nrf24_hal.h"

#ifndef __KERNEL__
static void nrf24_csn_on(struct nrf24_t *nrf24) {
    NRF24_HAL_SET_PIN_LOW(nrf24->csn);
}

static void nrf24_csn_off(struct nrf24_t *nrf24) {
    NRF24_HAL_SET_PIN_HIGH(nrf24->csn);
}
#endif

void nrf24_ce_on(struct nrf24_t *nrf24) {
    NRF24_HAL_SET_PIN_HIGH(nrf24->ce);
}

void nrf24_ce_off(struct nrf24_t *nrf24) {
    NRF24_HAL_SET_PIN_LOW(nrf24->ce);
}

nrf24_hal_status_t nrf24_read_register(struct nrf24_t *nrf24, uint8_t reg, uint8_t *data, size_t len) {
    uint8_t cmd = NRF24_CMD_R_REGISTER(reg);
#ifdef __KERNEL__
    ssize_t ret;
#else
    nrf24_hal_status_t status;
#endif

#ifndef __KERNEL__
    nrf24_csn_on(nrf24);
    status = HAL_SPI_Transmit(nrf24->spi, &cmd, 1, 100);
    if (status != HAL_OK) {
        goto read_reg_exit;
    }
    status = HAL_SPI_Receive(nrf24->spi, data, len, 100);

read_reg_exit:
    nrf24_csn_off(nrf24);

    return status;
#else
    ret = spi_write_then_read(nrf24->spi, &cmd, 1, data, len);
    if(ret < 0){
        dev_err(&(nrf24->spi->dev), "%s: failed to read register\n", __func__);
        return HAL_ERROR;
    }

    return HAL_OK;
#endif
}

nrf24_hal_status_t nrf24_write_register(struct nrf24_t *nrf24, uint8_t reg, uint8_t *data, size_t len) {
    uint8_t cmd = NRF24_CMD_W_REGISTER(reg);
#ifdef __KERNEL__
    ssize_t ret;
    uint8_t buffer[NRF24_MAX_PAYLOAD_SIZE + 1];
#else
    nrf24_hal_status_t status;
#endif

#ifndef __KERNEL__
    nrf24_csn_on(nrf24);
    status = HAL_SPI_Transmit(nrf24->spi, &cmd, 1, 100);
    if (status != HAL_OK) {
        goto write_reg_exit;
    }
    status = HAL_SPI_Transmit(nrf24->spi, data, len, 100);

write_reg_exit:
    nrf24_csn_off(nrf24);

    return status;
#else
    buffer[0] = cmd;
    memcpy(buffer + 1, data, len);

    ret = spi_write(nrf24->spi, buffer, len + 1);
    if(ret < 0){
        dev_err(&(nrf24->spi->dev), "%s: failed to write register\n", __func__);
        return HAL_ERROR;
    }

    return HAL_OK;
#endif
}

nrf24_hal_status_t nrf24_power_up(struct nrf24_t *nrf24) {
    nrf24_hal_status_t status;
    uint8_t config;

    status = nrf24_read_register(nrf24, NRF24_REG_CONFIG, &config, 1);
    if (status != HAL_OK) {
        return status;
    }

    config |= NRF2RL01_REG_CONFIG_MASK_PWR_UP;

    return nrf24_write_register(nrf24, NRF24_REG_CONFIG, &config, 1);
}

nrf24_hal_status_t nrf24_power_down(struct nrf24_t *nrf24) {
    nrf24_hal_status_t status;
    uint8_t config;

    status = nrf24_read_register(nrf24, NRF24_REG_CONFIG, &config, 1);
    if (status != HAL_OK) {
        return status;
    }

    config &= ~NRF2RL01_REG_CONFIG_MASK_PWR_UP;

    return nrf24_write_register(nrf24, NRF24_REG_CONFIG, &config, 1);
}

nrf24_hal_status_t nrf24_set_prx_mode(struct nrf24_t *nrf24) {
    nrf24_hal_status_t status;
    uint8_t config;

    status = nrf24_read_register(nrf24, NRF24_REG_CONFIG, &config, 1);
    if (status != HAL_OK) {
        return status;
    }

    config |= NRF2RL01_REG_CONFIG_MASK_PRIM_RX;

    return nrf24_write_register(nrf24, NRF24_REG_CONFIG, &config, 1);
}

nrf24_hal_status_t nrf24_set_ptx_mode(struct nrf24_t *nrf24) {
    nrf24_hal_status_t status;
    uint8_t config;

    status = nrf24_read_register(nrf24, NRF24_REG_CONFIG, &config, 1);
    if (status != HAL_OK) {
        return status;
    }

    config &= ~NRF2RL01_REG_CONFIG_MASK_PRIM_RX;

    return nrf24_write_register(nrf24, NRF24_REG_CONFIG, &config, 1);
}

nrf24_hal_status_t nrf24_set_crc_mode(struct nrf24_t *nrf24, enum nrf24_crc_mode_t crc_mode) {
    nrf24_hal_status_t status;
    uint8_t config;

    status = nrf24_read_register(nrf24, NRF24_REG_CONFIG, &config, 1);
    if (status != HAL_OK) {
        return status;
    }

    config &= ~(NRF2RL01_REG_CONFIG_MASK_CRCO | NRF2RL01_REG_CONFIG_MASK_EN_CRC);
    config |= (crc_mode << 2);

    return nrf24_write_register(nrf24, NRF24_REG_CONFIG, &config, 1);
}

nrf24_hal_status_t nrf24_get_crc_mode(struct nrf24_t *nrf24, enum nrf24_crc_mode_t *crc_mode) {
    nrf24_hal_status_t status;
    uint8_t config;

    status = nrf24_get_config(nrf24, &config);
    if(status != HAL_OK){
        return status;
    }

    config = (config >> 2) & 0x03;

    *crc_mode = (enum nrf24_crc_mode_t)config;

    return status;
}

nrf24_hal_status_t nrf24_get_config(struct nrf24_t *nrf24, uint8_t *config) {
    return nrf24_read_register(nrf24, NRF24_REG_CONFIG, config, 1);
}

nrf24_hal_status_t nrf24_set_auto_ack(struct nrf24_t *nrf24, uint8_t pipe, bool enable) {
    nrf24_hal_status_t status;
    uint8_t en_aa;

    status = nrf24_read_register(nrf24, NRF24_REG_EN_AA, &en_aa, 1);
    if (status != HAL_OK) {
        return status;
    }

    if (enable) {
        en_aa |= (1 << pipe);
    } else {
        en_aa &= ~(1 << pipe);
    }

    return nrf24_write_register(nrf24, NRF24_REG_EN_AA, &en_aa, 1);
}

nrf24_hal_status_t nrf24_get_auto_ack(struct nrf24_t *nrf24, uint8_t *en_aa) {
    return nrf24_read_register(nrf24, NRF24_REG_EN_AA, en_aa, 1);
}

nrf24_hal_status_t nrf24_set_en_rx_pipe(struct nrf24_t *nrf24, uint8_t pipe, bool enable) {
    nrf24_hal_status_t status;
    uint8_t en_rxaddr;

    status = nrf24_read_register(nrf24, NRF24_REG_EN_RXADDR, &en_rxaddr, 1);
    if (status != HAL_OK) {
        return status;
    }

    if (enable) {
        en_rxaddr |= (1 << pipe);
    } else {
        en_rxaddr &= ~(1 << pipe);
    }

    return nrf24_write_register(nrf24, NRF24_REG_EN_RXADDR, &en_rxaddr, 1);
}

nrf24_hal_status_t nrf24_get_en_rx_addr(struct nrf24_t *nrf24, uint8_t *en_rxaddr) {
    return nrf24_read_register(nrf24, NRF24_REG_EN_RXADDR, en_rxaddr, 1);
}

nrf24_hal_status_t nrf24_set_address_width(struct nrf24_t *nrf24, enum nrf24_address_width_t addr_width) {
    uint8_t setup_aw = addr_width - 2;

    return nrf24_write_register(nrf24, NRF24_REG_SETUP_AW, &setup_aw, 1);
}

nrf24_hal_status_t nrf24_get_address_width(struct nrf24_t *nrf24, enum nrf24_address_width_t *addr_width) {
    uint8_t aw;
    nrf24_hal_status_t status;

    status = nrf24_read_register(nrf24, NRF24_REG_SETUP_AW, &aw, 1);
    
    aw &= 0x03;

    if(aw == 0){
        return HAL_ERROR;
    }

    *addr_width = (enum nrf24_address_width_t)(aw + 2);

    return status;
}

nrf24_hal_status_t nrf24_setup_retransmission(struct nrf24_t *nrf24, enum nrf24_auto_retransmit_delay_t delay, enum nrf24_auto_retransmit_count_t count) {
    uint8_t setup_retr = (delay << 4) | count;

    return nrf24_write_register(nrf24, NRF24_REG_SETUP_RETR, &setup_retr, 1);
}

nrf24_hal_status_t nrf24_get_setup_retransmission(struct nrf24_t *nrf24, enum nrf24_auto_retransmit_delay_t *delay, enum nrf24_auto_retransmit_count_t *count) {
    uint8_t setup_retr;
    nrf24_hal_status_t status;

    status = nrf24_read_register(nrf24, NRF24_REG_SETUP_RETR, &setup_retr, 1);

    *delay = (enum nrf24_auto_retransmit_delay_t)((setup_retr >> 4) & 0x0F);
    *count = (enum nrf24_auto_retransmit_count_t)(setup_retr & 0x0F);

    return status;
}

nrf24_hal_status_t nrf24_set_radio_channel(struct nrf24_t *nrf24, uint8_t channel) {
    uint8_t rf_ch = channel;

    if(rf_ch < NRF24_MIN_CHANNEL || rf_ch > NRF24_MAX_CHANNEL){
        return HAL_ERROR;
    }

    return nrf24_write_register(nrf24, NRF24_REG_RF_CH, &rf_ch, 1);
}

nrf24_hal_status_t nrf24_get_radio_channel(struct nrf24_t *nrf24, uint8_t *rf_ch) {
    return nrf24_read_register(nrf24, NRF24_REG_RF_CH, rf_ch, 1);
}

nrf24_hal_status_t nrf24_set_radio_output_power(struct nrf24_t *nrf24, enum nrf24_tx_power_t power) {
    nrf24_hal_status_t status;
    uint8_t rf_setup;

    status = nrf24_read_register(nrf24, NRF24_REG_RF_SETUP, &rf_setup, 1);
    if (status != HAL_OK) {
        return status;
    }

    rf_setup &= ~(NRF24_TXP_0_DBM << 1);
    rf_setup |= (power << 1);

    return nrf24_write_register(nrf24, NRF24_REG_RF_SETUP, &rf_setup, 1);
}

nrf24_hal_status_t nrf24_get_radio_output_power(struct nrf24_t *nrf24, enum nrf24_tx_power_t *power) {
    nrf24_hal_status_t status;
    uint8_t rf_setup;

    status = nrf24_read_register(nrf24, NRF24_REG_RF_SETUP, &rf_setup, 1);
    if (status != HAL_OK) {
        return status;
    }

    *power = (enum nrf24_tx_power_t)((rf_setup >> 1) & 0x03);

    return HAL_OK;
}

nrf24_hal_status_t nrf24_set_radio_data_rate(struct nrf24_t *nrf24, enum nrf24_air_data_rate_t data_rate) {
    nrf24_hal_status_t status;
    uint8_t rf_setup;

    status = nrf24_read_register(nrf24, NRF24_REG_RF_SETUP, &rf_setup, 1);
    if (status != HAL_OK) {
        return status;
    }

    rf_setup &= ~(NRF24_ADR_2_MBPS << 3);
    rf_setup |= (data_rate << 3);

    return nrf24_write_register(nrf24, NRF24_REG_RF_SETUP, &rf_setup, 1);
}

nrf24_hal_status_t nrf24_get_radio_data_rate(struct nrf24_t *nrf24, enum nrf24_air_data_rate_t *data_rate) {
    nrf24_hal_status_t status;
    uint8_t rf_setup;

    status = nrf24_read_register(nrf24, NRF24_REG_RF_SETUP, &rf_setup, 1);
    if (status != HAL_OK) {
        return status;
    }

    *data_rate = (enum nrf24_air_data_rate_t)((rf_setup >> 3) & 0x01);

    return HAL_OK;
}

nrf24_hal_status_t nrf24_get_rf_setup(struct nrf24_t *nrf24, uint8_t *rf_setup) {
    return nrf24_read_register(nrf24, NRF24_REG_RF_SETUP, rf_setup, 1);
}

nrf24_hal_status_t nrf24_get_status(struct nrf24_t *nrf24, uint8_t *status) {
    return nrf24_read_register(nrf24, NRF24_REG_STATUS, status, 1);
}

nrf24_hal_status_t nrf24_set_status(struct nrf24_t *nrf24, uint8_t *status) {
    return nrf24_write_register(nrf24, NRF24_REG_STATUS, status, 1);
}

nrf24_hal_status_t nrf24_clear_status_bit(struct nrf24_t *nrf24, uint8_t mask) {
    uint8_t status;
    nrf24_hal_status_t hal_status;

    hal_status = nrf24_read_register(nrf24, NRF24_REG_STATUS, &status, 1);
    if (hal_status != HAL_OK) {
        return hal_status;
    }

    status |= mask;

    hal_status = nrf24_write_register(nrf24, NRF24_REG_STATUS, &status, 1);
    return hal_status;
}

nrf24_hal_status_t nrf24_get_observe_tx(struct nrf24_t *nrf24, uint8_t *observe_tx) {
    return nrf24_read_register(nrf24, NRF24_REG_OBSERVE_TX, observe_tx, 1);
}

nrf24_hal_status_t nrf24_get_carrier_detect(struct nrf24_t *nrf24, uint8_t *cd) {
    return nrf24_read_register(nrf24, NRF24_REG_CD, cd, 1);
}

nrf24_hal_status_t nrf24_set_major_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address) {
    if(pipe > 1){
        return HAL_ERROR;
    }

    return nrf24_write_register(nrf24, NRF24_REG_RX_ADDR_P0 + pipe, address, 5);
}

nrf24_hal_status_t nrf24_set_minor_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address) {
    if(pipe > 5 && pipe < 2){
        return HAL_ERROR;
    }

    return nrf24_write_register(nrf24, NRF24_REG_RX_ADDR_P0 + pipe, address, 1);
}

nrf24_hal_status_t nrf24_set_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address) {
    if(pipe > 1 && pipe < 6){
        return nrf24_set_minor_pipe_address(nrf24, pipe, address);
    }
    else if(pipe < 2){
        return nrf24_set_major_pipe_address(nrf24, pipe, address);
    }

    return HAL_ERROR;
}

nrf24_hal_status_t nrf24_get_major_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address) {
    if(pipe > 1){
        return HAL_ERROR;
    }

    return nrf24_read_register(nrf24, NRF24_REG_RX_ADDR_P0 + pipe, address, 5);
}

nrf24_hal_status_t nrf24_get_minor_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address) {
    if(pipe > 5 && pipe < 2){
        return HAL_ERROR;
    }

    return nrf24_read_register(nrf24, NRF24_REG_RX_ADDR_P0 + pipe, address, 1);
}

nrf24_hal_status_t nrf24_get_pipe_address(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *address){
    if(pipe < 2){
        return nrf24_get_major_pipe_address(nrf24, pipe, address);
    }
    else if(pipe > 1 && pipe < 6){
        return nrf24_get_minor_pipe_address(nrf24, pipe, address);
    }

    return HAL_ERROR;
}

nrf24_hal_status_t nrf24_set_tx_address(struct nrf24_t *nrf24, uint8_t *address) {
    return nrf24_write_register(nrf24, NRF24_REG_TX_ADDR, address, 5);
}

nrf24_hal_status_t nrf24_get_tx_address(struct nrf24_t *nrf24, uint8_t *address) {
    return nrf24_read_register(nrf24, NRF24_REG_TX_ADDR, address, 5);
}

nrf24_hal_status_t nrf24_set_rx_payload_width(struct nrf24_t *nrf24, uint8_t pipe, uint8_t width) {
    if(pipe > 5){
        return HAL_ERROR;
    }
    return nrf24_write_register(nrf24, NRF24_REG_RX_PW_P0 + pipe, &width, 1);
}

nrf24_hal_status_t nrf24_get_rx_payload_width(struct nrf24_t *nrf24, uint8_t pipe, uint8_t *width) {
    if(pipe > 5){
        return HAL_ERROR;
    }
    return nrf24_read_register(nrf24, NRF24_REG_RX_PW_P0 + pipe, width, 1);
}

nrf24_hal_status_t nrf24_get_fifo_status(struct nrf24_t *nrf24, uint8_t *fifo_status) {
    return nrf24_read_register(nrf24, NRF24_REG_FIFO_STATUS, fifo_status, 1);
}

nrf24_hal_status_t nrf24_get_dynpd(struct nrf24_t *nrf24, uint8_t pipe_id, uint8_t *dynpd) {
    uint8_t config;
    nrf24_hal_status_t status;

    status = nrf24_read_register(nrf24, NRF24_REG_DYNPD, &config, 1);
    if(status != HAL_OK){
        return status;
    }

    *dynpd = (config >> pipe_id) & 0x01;

    return status;
}

nrf24_hal_status_t nrf24_set_dynpd(struct nrf24_t *nrf24, uint8_t pipe, bool enable) {
    nrf24_hal_status_t status;
    uint8_t dynpd;

    status = nrf24_read_register(nrf24, NRF24_REG_DYNPD, &dynpd, 1);
    if (status != HAL_OK) {
        return status;
    }

    if (enable) {
        dynpd |= (1 << pipe);
    } else {
        dynpd &= ~(1 << pipe);
    }

    return nrf24_write_register(nrf24, NRF24_REG_DYNPD, &dynpd, 1);
}

nrf24_hal_status_t nrf24_get_feature(struct nrf24_t *nrf24, uint8_t *feature) {
    return nrf24_read_register(nrf24, NRF24_REG_FEATURE, feature, 1);
}

nrf24_hal_status_t nrf24_set_feature(struct nrf24_t *nrf24, uint8_t feature) {
    return nrf24_write_register(nrf24, NRF24_REG_FEATURE, &feature, 1);
}

nrf24_hal_status_t nrf24_soft_reset(struct nrf24_t *nrf24) {
    nrf24_hal_status_t status;
    uint8_t rx_pipe_0_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t rx_pipe_1_addr[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t rx_pipe_2_addr = 0xC3;
    uint8_t rx_pipe_3_addr = 0xC4;
    uint8_t rx_pipe_4_addr = 0xC5;
    uint8_t rx_pipe_5_addr = 0xC6;
    
    uint8_t reg_val = NRF24_REG_CONFIG_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_CONFIG, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    reg_val = NRF24_REG_EN_AA_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_EN_AA, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    reg_val = NRF24_REG_EN_RXADDR_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_EN_RXADDR, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    reg_val = NRF24_REG_SETUP_AW_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_SETUP_AW, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    reg_val = NRF24_REG_SETUP_RETR_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_SETUP_RETR, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    reg_val = NRF24_REG_RF_CH_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_RF_CH, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    reg_val = NRF24_REG_RF_SETUP_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_RF_SETUP, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    reg_val = NRF24_REG_STATUS_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_STATUS, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    status = nrf24_write_register(nrf24, NRF24_REG_RX_ADDR_P0, rx_pipe_0_addr, (sizeof(rx_pipe_0_addr) / sizeof(rx_pipe_0_addr[0])));
    if (status != HAL_OK) {
        return status;
    }

    status = nrf24_write_register(nrf24, NRF24_REG_RX_ADDR_P1, rx_pipe_1_addr, (sizeof(rx_pipe_1_addr) / sizeof(rx_pipe_1_addr[0])));
    if (status != HAL_OK) {
        return status;
    }

    status = nrf24_write_register(nrf24, NRF24_REG_RX_ADDR_P2, &rx_pipe_2_addr, 1);
    if (status != HAL_OK) {
        return status;
    }

    status = nrf24_write_register(nrf24, NRF24_REG_RX_ADDR_P3, &rx_pipe_3_addr, 1);
    if (status != HAL_OK) {
        return status;
    }

    status = nrf24_write_register(nrf24, NRF24_REG_RX_ADDR_P4, &rx_pipe_4_addr, 1);
    if (status != HAL_OK) {
        return status;
    }

    status = nrf24_write_register(nrf24, NRF24_REG_RX_ADDR_P5, &rx_pipe_5_addr, 1);
    if (status != HAL_OK) {
        return status;
    }

    status = nrf24_write_register(nrf24, NRF24_REG_TX_ADDR, rx_pipe_0_addr, (sizeof(rx_pipe_0_addr) / sizeof(rx_pipe_0_addr[0])));
    if (status != HAL_OK) {
        return status;
    }

    reg_val = NRF24_REG_RX_PW_RESET_VALUE;
    for(uint8_t i = 0; i < 6; i++){
        status = nrf24_write_register(nrf24, NRF24_REG_RX_PW_P0 + i, &reg_val, 1);
        if (status != HAL_OK) {
            return status;
        }
    }

    reg_val = NRF24_REG_DYNPD_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_DYNPD, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    reg_val = NRF24_REG_FEATURE_RESET_VALUE;
    status = nrf24_write_register(nrf24, NRF24_REG_FEATURE, &reg_val, 1);
    if (status != HAL_OK) {
        return status;
    }

    status = nrf24_flush_tx_fifo(nrf24);
    if (status != HAL_OK) {
        return status;
    }

    status = nrf24_flush_rx_fifo(nrf24);
    if (status != HAL_OK) {
        return status;
    }

    nrf24_ce_off(nrf24);

    return HAL_OK;
}

nrf24_hal_status_t nrf24_write_tx_fifo(struct nrf24_t *nrf24, uint8_t *data, size_t len) {
    uint8_t cmd = NRF24_CMD_W_TX_PAYLOAD;
#ifdef __KERNEL__
    ssize_t ret;
    uint8_t buffer[NRF24_MAX_PAYLOAD_SIZE + 1];
#else
    nrf24_hal_status_t status;
#endif

#ifndef __KERNEL__
    nrf24_csn_on(nrf24);
    status = HAL_SPI_Transmit(nrf24->spi, &cmd, 1, 100);
    if (status != HAL_OK) {
        goto write_tx_fifo_exit;
    }
    status = HAL_SPI_Transmit(nrf24->spi, data, len, 100);

write_tx_fifo_exit:
    nrf24_csn_off(nrf24);

    return status;
#else
    buffer[0] = cmd;
    memcpy(buffer + 1, data, len);

    ret = spi_write(nrf24->spi, buffer, len + 1);
    if(ret < 0){
        dev_err(&(nrf24->spi->dev), "%s: failed to write tx fifo\n", __func__);
        return HAL_ERROR;
    }

    return HAL_OK;
#endif
}

nrf24_hal_status_t nrf24_read_rx_fifo(struct nrf24_t *nrf24, uint8_t *data, size_t len) {
    uint8_t cmd = NRF24_CMD_R_RX_PAYLOAD;
#ifdef __KERNEL__
    ssize_t ret;
#else
    nrf24_hal_status_t status;
#endif

#ifndef __KERNEL__
    nrf24_csn_on(nrf24);
    status = HAL_SPI_Transmit(nrf24->spi, &cmd, 1, 100);
    if (status != HAL_OK) {
        goto read_rx_fifo_exit;
    }
    status = HAL_SPI_Receive(nrf24->spi, data, len, 100);

read_rx_fifo_exit:
    nrf24_csn_off(nrf24);

    return status;
#else
    ret = spi_write_then_read(nrf24->spi, &cmd, 1, data, len);
    if(ret < 0){
        dev_err(&(nrf24->spi->dev), "%s: failed to read rx fifo\n", __func__);
        return HAL_ERROR;
    }

    return HAL_OK;
#endif
}

nrf24_hal_status_t nrf24_flush_tx_fifo(struct nrf24_t *nrf24) {
    uint8_t cmd = NRF24_CMD_FLUSH_TX;
#ifdef __KERNEL__
    ssize_t ret;
#else
    nrf24_hal_status_t status;
#endif

#ifndef __KERNEL__
    nrf24_csn_on(nrf24);
    status = HAL_SPI_Transmit(nrf24->spi, &cmd, 1, 100);
    nrf24_csn_off(nrf24);

    return status;
#else
    ret = spi_write(nrf24->spi, &cmd, 1);
    if(ret < 0){
        dev_err(&(nrf24->spi->dev), "%s: failed to flush tx fifo\n", __func__);
        return HAL_ERROR;
    }

    return HAL_OK;
#endif
}

nrf24_hal_status_t nrf24_flush_rx_fifo(struct nrf24_t *nrf24) {
    uint8_t cmd = NRF24_CMD_FLUSH_RX;
#ifdef __KERNEL__
    ssize_t ret;
#else
    nrf24_hal_status_t status;
#endif

#ifndef __KERNEL__
    nrf24_csn_on(nrf24);
    status = HAL_SPI_Transmit(nrf24->spi, &cmd, 1, 100);
    nrf24_csn_off(nrf24);

    return status;
#else
    ret = spi_write(nrf24->spi, &cmd, 1);
    if(ret < 0){
        dev_err(&(nrf24->spi->dev), "%s: failed to flush rx fifo\n", __func__);
        return HAL_ERROR;
    }

    return HAL_OK;
#endif
}