#include "nrf24_sysfs.h"
#include "nrf24_hal.h"
#include "nrf24_mod.h"

static struct nrf24_pipe_t *nrf24_find_pipe_ptr(struct device *dev){
	struct nrf24_pipe_t *pipe;
    int i;

	for(i = 0; i < NRF24_PIPES_COUNT; i++){
		if (pipe->dev == dev){
			return pipe;
        }
    }

	return ERR_PTR(-ENODEV);
}

static ssize_t crc_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_crc_mode_t crc_mode;

    status = nrf24_get_crc_mode(&(nrf24_dev->nrf24_hal_dev), &crc_mode);
    if(status != HAL_OK){
        return -EIO;
    }

    if(crc_mode == NRF24_CRC_DISABLED){
        return scnprintf(buf, PAGE_SIZE, "0 (disabled)\n");
    }
    else{
        return scnprintf(buf, PAGE_SIZE, "%d\n", (crc_mode - 1));
    }
}

static ssize_t crc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_crc_mode_t crc_mode;

    if(kstrtou8(buf, 0, (u8 *)(&crc_mode))){
        return -EINVAL;
    }

    if(crc_mode == 1 || crc_mode > NRF24_CRC_2_BYTES){
        return -EINVAL;
    }

    status = nrf24_set_crc_mode(&(nrf24_dev->nrf24_hal_dev), crc_mode);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t ack_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
    nrf24_hal_status_t status;
    uint8_t auto_ack;
    struct nrf24_pipe_t *pipe = nrf24_find_pipe_ptr(dev);
    if(IS_ERR(pipe)){
        return PTR_ERR(pipe);
    }

    status = nrf24_get_auto_ack(&(nrf24_dev->nrf24_hal_dev), &auto_ack);
    if(status != HAL_OK){
        return -EIO;
    }

    auto_ack = (auto_ack >> pipe->id) & 1;

    return scnprintf(buf, PAGE_SIZE, "%u\n", auto_ack);
}

static ssize_t ack_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
    nrf24_hal_status_t status;
    uint8_t auto_ack;
    struct nrf24_pipe_t *pipe = nrf24_find_pipe_ptr(dev);
    if(IS_ERR(pipe)){
        return PTR_ERR(pipe);
    }

    if(kstrtou8(buf, 0, &auto_ack)){
        return -EINVAL;
    }

    if(auto_ack > 1){
        return -EINVAL;
    }

    status = nrf24_set_auto_ack(&(nrf24_dev->nrf24_hal_dev), pipe->id, auto_ack);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t aw_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_address_width_t addr_width;

    status = nrf24_get_address_width(&(nrf24_dev->nrf24_hal_dev), &addr_width);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", addr_width);
}

static ssize_t aw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_address_width_t addr_width;

    if(kstrtou8(buf, 0, (u8 *)(&addr_width))){
        return -EINVAL;
    }

    if(addr_width < NRF24_AW_3_BYTES || addr_width > NRF24_AW_5_BYTES){
        return -EINVAL;
    }

    status = nrf24_set_address_width(&(nrf24_dev->nrf24_hal_dev), addr_width);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t retr_delay_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_auto_retransmit_delay_t delay_config;
    enum nrf24_auto_retransmit_count_t count_config;

    status = nrf24_get_setup_retransmission(&(nrf24_dev->nrf24_hal_dev), &delay_config, &count_config);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", (250 * (delay_config + 1)));
}

static ssize_t retr_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_auto_retransmit_delay_t delay_config;
    enum nrf24_auto_retransmit_count_t count_config;
    u8 configured_delay;

    if(kstrtou8(buf, 0, &configured_delay)){
        return -EINVAL;
    }

    status = nrf24_get_setup_retransmission(&(nrf24_dev->nrf24_hal_dev), &delay_config, &count_config);
    if(status != HAL_OK){
        return -EIO;
    }

    delay_config = (enum nrf24_auto_retransmit_delay_t)((configured_delay / 250) - 1);

    if(delay_config < NRF24_ARD_250_US || delay_config > NRF24_ARD_4000_US){
        return -EINVAL;
    }

    status = nrf24_setup_retransmission(&(nrf24_dev->nrf24_hal_dev), delay_config, count_config);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t retr_count_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_auto_retransmit_delay_t delay_config;
    enum nrf24_auto_retransmit_count_t count_config;

    status = nrf24_get_setup_retransmission(&(nrf24_dev->nrf24_hal_dev), &delay_config, &count_config);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", count_config);
}

static ssize_t retr_count_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_auto_retransmit_delay_t delay_config;
    enum nrf24_auto_retransmit_count_t count_config;

    status = nrf24_get_setup_retransmission(&(nrf24_dev->nrf24_hal_dev), &delay_config, &count_config);
    if(status != HAL_OK){
        return -EIO;
    }

    if(kstrtou8(buf, 0, (u8 *)(&count_config))){
        return -EINVAL;
    }

    status = nrf24_setup_retransmission(&(nrf24_dev->nrf24_hal_dev), delay_config, count_config);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t power_up_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    uint8_t config;

    status = nrf24_get_config(&(nrf24_dev->nrf24_hal_dev), &config);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ((config >> 1) & NRF24_PWR_UP));
}

static ssize_t rx_mode_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    uint8_t config;

    status = nrf24_get_config(&(nrf24_dev->nrf24_hal_dev), &config);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", (config & NRF24_PM_RX));
}

static ssize_t rx_enabled_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
    struct nrf24_pipe_t *pipe = nrf24_find_pipe_ptr(dev);
    nrf24_hal_status_t status;
    uint8_t config;

    status = nrf24_get_en_rx_addr(&(nrf24_dev->nrf24_hal_dev), &config);
    if(status != HAL_OK){
        return -EIO;
    }

    config = (config >> pipe->id) & 1;

    return scnprintf(buf, PAGE_SIZE, "%u\n", config);
}

static ssize_t rf_ch_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 channel;

    status = nrf24_get_radio_channel(&(nrf24_dev->nrf24_hal_dev), &channel);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", channel);
}

static ssize_t rf_ch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 channel;

    if(kstrtou8(buf, 0, &channel)){
        return -EINVAL;
    }

    status = nrf24_set_radio_channel(&(nrf24_dev->nrf24_hal_dev), channel);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t rf_dr_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_air_data_rate_t data_rate;

    status = nrf24_get_radio_data_rate(&(nrf24_dev->nrf24_hal_dev), &data_rate);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", (data_rate + 1));
}

static ssize_t rf_dr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_air_data_rate_t data_rate;

    if(kstrtou8(buf, 0, (u8 *)(&data_rate))){
        return -EINVAL;
    }

    data_rate--;

    if(data_rate < NRF24_ADR_1_MBPS || data_rate > NRF24_ADR_2_MBPS){
        return -EINVAL;
    }

    status = nrf24_set_radio_data_rate(&(nrf24_dev->nrf24_hal_dev), data_rate);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t rf_pwr_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_tx_power_t tx_power;

    status = nrf24_get_radio_output_power(&(nrf24_dev->nrf24_hal_dev), &tx_power);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%d (%d dBm)\n", tx_power, ((3 - tx_power) * (-6)));
}

static ssize_t rf_pwr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    enum nrf24_tx_power_t tx_power;

    if(kstrtou8(buf, 0, (u8 *)(&tx_power))){
        return -EINVAL;
    }

    if(tx_power < NRF24_TXP_ATTENUATION_18_DBM || tx_power > NRF24_TXP_0_DBM){
        return -EINVAL;
    }

    status = nrf24_set_radio_output_power(&(nrf24_dev->nrf24_hal_dev), tx_power);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 status_reg;

    status = nrf24_get_status(&(nrf24_dev->nrf24_hal_dev), &status_reg);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "0x%02X\n", status_reg);
}

static ssize_t plos_cnt_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 status_reg;

    status = nrf24_get_observe_tx(&(nrf24_dev->nrf24_hal_dev), &status_reg);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", ((status_reg >> 4) & 0x0F));
}

static ssize_t arc_cnt_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 status_reg;

    status = nrf24_get_observe_tx(&(nrf24_dev->nrf24_hal_dev), &status_reg);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", (status_reg & 0x0F));
}

static ssize_t cd_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 status_reg;

    status = nrf24_get_carrier_detect(&(nrf24_dev->nrf24_hal_dev), &status_reg);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", (status_reg & 1));
}

static ssize_t rx_addr_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
    struct nrf24_pipe_t *pipe = nrf24_find_pipe_ptr(dev);
    nrf24_hal_status_t status;
    if(IS_ERR(pipe)){
        return PTR_ERR(pipe);
    }

    status = nrf24_get_pipe_address(&(nrf24_dev->nrf24_hal_dev), pipe->id, (u8 *)(&(pipe->config.addr)));
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%llX\n", pipe->config.addr);
}

static ssize_t rx_addr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
    struct nrf24_pipe_t *pipe = nrf24_find_pipe_ptr(dev);
    nrf24_hal_status_t status;
    if(IS_ERR(pipe)){
        return PTR_ERR(pipe);
    }

    if(kstrtoull(buf, 0, &(pipe->config.addr))){
        return -EINVAL;
    }

    status = nrf24_set_pipe_address(&(nrf24_dev->nrf24_hal_dev), pipe->id, (u8 *)(&(pipe->config.addr)));
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t tx_addr_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u64 addr;

    status = nrf24_get_tx_address(&(nrf24_dev->nrf24_hal_dev), (u8 *)(&addr));
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%llX\n", addr);
}

static ssize_t tx_addr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u64 addr;

    if(kstrtoull(buf, 0, &addr)){
        return -EINVAL;
    }

    status = nrf24_set_tx_address(&(nrf24_dev->nrf24_hal_dev), (u8 *)(&addr));
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t rx_pw_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
    struct nrf24_pipe_t *pipe = nrf24_find_pipe_ptr(dev);
    nrf24_hal_status_t status;
    if(IS_ERR(pipe)){
        return PTR_ERR(pipe);
    }

    status = nrf24_get_rx_payload_width(&(nrf24_dev->nrf24_hal_dev), pipe->id, &(pipe->config.plw));
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", pipe->config.plw);
}

static ssize_t rx_pw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
    struct nrf24_pipe_t *pipe = nrf24_find_pipe_ptr(dev);
    nrf24_hal_status_t status;
    if(IS_ERR(pipe)){
        return PTR_ERR(pipe);
    }

    if(kstrtou8(buf, 0, &(pipe->config.plw))){
        return -EINVAL;
    }

    status = nrf24_set_rx_payload_width(&(nrf24_dev->nrf24_hal_dev), pipe->id, pipe->config.plw);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t fifo_status_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 fifo_status;

    status = nrf24_get_fifo_status(&(nrf24_dev->nrf24_hal_dev), &fifo_status);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", fifo_status);
}

static ssize_t dynpd_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 dynpd;

    status = nrf24_get_dynpd(&(nrf24_dev->nrf24_hal_dev), &dynpd);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", dynpd);
}

static ssize_t en_dpl_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 feature;

    status = nrf24_get_feature(&(nrf24_dev->nrf24_hal_dev), &feature);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", (feature >> 2) & 1);
}

static ssize_t en_dpl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 feature;
    u8 en_dpl_feature;

    status = nrf24_get_feature(&(nrf24_dev->nrf24_hal_dev), &feature);
    if(status != HAL_OK){
        return -EIO;
    }

    if(kstrtou8(buf, 0, &en_dpl_feature)){
        return -EINVAL;
    }

    if(en_dpl_feature > 1){
        return -EINVAL;
    }

    feature &= ~(1 << 2);
    feature |= (en_dpl_feature << 2);

    status = nrf24_set_feature(&(nrf24_dev->nrf24_hal_dev), feature);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t en_ack_pay_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 feature;

    status = nrf24_get_feature(&(nrf24_dev->nrf24_hal_dev), &feature);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", (feature >> 1) & 1);
}

static ssize_t en_ack_pay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 feature;
    u8 en_ack_pay_feature;

    status = nrf24_get_feature(&(nrf24_dev->nrf24_hal_dev), &feature);
    if(status != HAL_OK){
        return -EIO;
    }

    if(kstrtou8(buf, 0, &en_ack_pay_feature)){
        return -EINVAL;
    }

    if(en_ack_pay_feature > 1){
        return -EINVAL;
    }

    feature &= ~(1 << 1);
    feature |= (en_ack_pay_feature << 1);

    status = nrf24_set_feature(&(nrf24_dev->nrf24_hal_dev), feature);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static ssize_t en_dyn_ack_show(struct device *dev, struct device_attribute *attr, char *buf){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 feature;

    status = nrf24_get_feature(&(nrf24_dev->nrf24_hal_dev), &feature);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", feature & 1);
}

static ssize_t en_dyn_ack_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);
    nrf24_hal_status_t status;
    u8 feature;
    u8 en_dyn_ack_feature;

    status = nrf24_get_feature(&(nrf24_dev->nrf24_hal_dev), &feature);
    if(status != HAL_OK){
        return -EIO;
    }

    if(kstrtou8(buf, 0, &en_dyn_ack_feature)){
        return -EINVAL;
    }

    if(en_dyn_ack_feature > 1){
        return -EINVAL;
    }

    feature &= ~1;
    feature |= en_dyn_ack_feature;

    status = nrf24_set_feature(&(nrf24_dev->nrf24_hal_dev), feature);
    if(status != HAL_OK){
        return -EIO;
    }

    return count;
}

static DEVICE_ATTR_RW(crc);
static DEVICE_ATTR_RW(ack);
static DEVICE_ATTR_RW(aw);
static DEVICE_ATTR_RW(retr_delay);
static DEVICE_ATTR_RW(retr_count);
static DEVICE_ATTR_RO(power_up);
static DEVICE_ATTR_RO(rx_mode);
static DEVICE_ATTR_RO(rx_enabled);
static DEVICE_ATTR_RW(rf_ch);
static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RO(plos_cnt);
static DEVICE_ATTR_RO(arc_cnt);
static DEVICE_ATTR_RO(cd);
static DEVICE_ATTR_RW(rx_addr);
static DEVICE_ATTR_RW(tx_addr);
static DEVICE_ATTR_RW(rx_pw);
static DEVICE_ATTR_RO(fifo_status);
static DEVICE_ATTR_RO(dynpd);
static DEVICE_ATTR_RW(en_dpl);
static DEVICE_ATTR_RW(en_ack_pay);
static DEVICE_ATTR_RW(en_dyn_ack);
static DEVICE_ATTR_RW(rf_dr);
static DEVICE_ATTR_RW(rf_pwr);

struct attribute *nrf24_pipe_attrs[] = {
    &(dev_attr_ack.attr),
    &(dev_attr_rx_enabled.attr),
    &(dev_attr_rx_addr.attr),
    &(dev_attr_rx_pw.attr),
    &(dev_attr_dynpd.attr),
    NULL
};

struct attribute *nrf24_attrs[] = {
    &(dev_attr_crc.attr),
    &(dev_attr_aw.attr),
    &(dev_attr_retr_delay.attr),
    &(dev_attr_retr_count.attr),
    &(dev_attr_power_up.attr),
    &(dev_attr_rx_mode.attr),
    &(dev_attr_rf_ch.attr),
    &(dev_attr_status.attr),
    &(dev_attr_plos_cnt.attr),
    &(dev_attr_arc_cnt.attr),
    &(dev_attr_cd.attr),
    &(dev_attr_tx_addr.attr),
    &(dev_attr_fifo_status.attr),
    &(dev_attr_en_dpl.attr),
    &(dev_attr_en_ack_pay.attr),
    &(dev_attr_en_dyn_ack.attr),
    &(dev_attr_rf_dr.attr),
    &(dev_attr_rf_pwr.attr),
    NULL
};