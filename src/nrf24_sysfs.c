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
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
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
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
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
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
    nrf24_hal_status_t status;
    enum nrf24_address_width_t addr_width;

    status = nrf24_get_address_width(&(nrf24_dev->nrf24_hal_dev), &addr_width);
    if(status != HAL_OK){
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", addr_width);
}

static ssize_t aw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev->parent);
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

static DEVICE_ATTR_RW(crc);
static DEVICE_ATTR_RW(ack);
static DEVICE_ATTR_RW(aw);

struct attribute *nrf24_pipe_attrs[] = {
    &(dev_attr_ack.attr),
    NULL
};

struct attribute *nrf24_attrs[] = {
    &(dev_attr_crc.attr),
    &(dev_attr_aw.attr),
    NULL
};