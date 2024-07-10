#include "nrf24_ioctl.h"

long nrf24_handle_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    long ret = 0;
    nrf24_hal_status_t status;
    struct nrf24_pipe_t *pipe = filp->private_data;
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(pipe->dev->parent);

    if(_IOC_TYPE(cmd) != NRF24_IOCTL_MAGIC) {
        return -ENOTTY;
    }

    switch (cmd) {
        case NRF24_IOCTL_SET_CHANNEL:
            if(copy_from_user(&(nrf24_dev->config.channel), (u8 *)arg, sizeof(u8))) {
                return -EFAULT;
            }

            status = nrf24_set_radio_channel(&(nrf24_dev->nrf24_hal_dev), nrf24_dev->config.channel);
            if(status != HAL_OK) {
                return -EIO;
            }

            break;

        case NRF24_IOCTL_GET_CHANNEL:
            status = nrf24_get_radio_channel(&(nrf24_dev->nrf24_hal_dev), &(nrf24_dev->config.channel));

            if(status != HAL_OK) {
                return -EIO;
            }

            if(copy_to_user((u8 *)arg, &(nrf24_dev->config.channel), sizeof(u8))) {
                return -EFAULT;
            }

            break;

        case NRF24_IOCTL_SET_RX_PAYLOAD_SIZE:
            if(copy_from_user(&(pipe->config.plw), (u8 *)arg, sizeof(u8))) {
                return -EFAULT;
            }

            status = nrf24_set_rx_payload_width(&(nrf24_dev->nrf24_hal_dev), pipe->id, pipe->config.plw);
            if(status != HAL_OK) {
                return -EIO;
            }

            break;

        case NRF24_IOCTL_GET_RX_PAYLOAD_SIZE:
            status = nrf24_get_rx_payload_width(&(nrf24_dev->nrf24_hal_dev), pipe->id, &(pipe->config.plw));

            if(status != HAL_OK) {
                return -EIO;
            }

            if(copy_to_user((u8 *)arg, &(pipe->config.plw), sizeof(u8))) {
                return -EFAULT;
            }

            break;

        case NRF24_IOCTL_SET_DATA_RATE:
            if(copy_from_user(&(nrf24_dev->config.data_rate), (u8 *)arg, sizeof(u8))) {
                return -EFAULT;
            }

            status = nrf24_set_radio_data_rate(&(nrf24_dev->nrf24_hal_dev), nrf24_dev->config.data_rate);
            if(status != HAL_OK) {
                return -EIO;
            }

            break;

        case NRF24_IOCTL_GET_DATA_RATE:
            status = nrf24_get_radio_data_rate(&(nrf24_dev->nrf24_hal_dev), &(nrf24_dev->config.data_rate));

            if(status != HAL_OK) {
                return -EIO;
            }

            if(copy_to_user((u8 *)arg, &(nrf24_dev->config.data_rate), sizeof(u8))) {
                return -EFAULT;
            }

            break;
    }

    return ret;
}