#include "nrf24_mod.h"
#include "nrf24_sysfs.h"
#include "nrf24_ioctl.h"

const enum nrf24_crc_mode_t nrf24_default_crc_mode = NRF24_CRC_1_BYTE;
const enum nrf24_air_data_rate_t nrf24_default_air_data_rate = NRF24_ADR_1_MBPS;
const enum nrf24_address_width_t nrf24_default_address_width = NRF24_AW_5_BYTES;
const enum nrf24_auto_retransmit_count_t nrf24_default_auto_retransmit_count = NRF24_ARC_3;
const enum nrf24_auto_retransmit_delay_t nrf24_default_auto_retransmit_delay = NRF24_ARD_250_US;
const enum nrf24_mode_t nrf24_default_mode = NRF24_PM_RX;
const enum nrf24_tx_power_t nrf24_default_tx_power = NRF24_TXP_0_DBM;
const u8 nrf24_default_channel = 36;

static struct ida *dev_ida;
static struct ida *pipe_ida;

ATTRIBUTE_GROUPS(nrf24);
ATTRIBUTE_GROUPS(nrf24_pipe);

static void nrf24_dev_release(struct device *dev){
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(dev);

    ida_simple_remove(dev_ida, nrf24_dev->id);

    kfree(nrf24_dev);
}

static struct device_type nrf24_dev_type = {
	.name = "nrf24_device",
	.release = nrf24_dev_release,
};

static void nrf24_isr_work_handler(struct work_struct *work){
    struct nrf24_device_t *device;
    nrf24_hal_status_t status;
    uint8_t nrf24_status;
    uint8_t nrf24_clear_status;
    u32 usecs;

    device = container_of(work, struct nrf24_device_t, isr_work);

    status = nrf24_get_status(&(device->nrf24_hal_dev), &nrf24_status);
    if(status != HAL_OK){
        dev_err(&(device->dev), "%s: failed to get status\n", __func__);
        return;
    }

    if(nrf24_status & NRF24_REG_STATUS_MASK_RX_DR){
        dev_dbg(&(device->dev), "%s: rx dr\n", __func__);
        device->rx_active = true;
        usecs = (8 * (1 + device->config.addr_width + NRF24_MAX_PAYLOAD_SIZE + (device->config.crc_mode - 1))) + 9;
        usecs /= (device->config.data_rate);
        usecs += (usecs / 2);
        mod_timer(&(device->rx_active_timer), jiffies + usecs_to_jiffies(usecs));
        nrf24_clear_status = NRF24_REG_STATUS_MASK_RX_DR;
        status = nrf24_set_status(&(device->nrf24_hal_dev), &nrf24_clear_status);
        if(status != HAL_OK){
            dev_err(&(device->dev), "%s: failed to set status\n", __func__);
            return;
        }

        schedule_work(&(device->rx_work));
    }

    if(nrf24_status & NRF24_REG_STATUS_MASK_TX_DS){
        dev_dbg(&(device->dev), "%s: tx ds\n", __func__);
        device->tx_done = true;
        device->tx_failed = false;
        nrf24_clear_status = NRF24_REG_STATUS_MASK_TX_DS;
        status = nrf24_set_status(&(device->nrf24_hal_dev), &nrf24_clear_status);
        if(status != HAL_OK){
            dev_err(&(device->dev), "%s: failed to set status\n", __func__);
            return;
        }
        wake_up_interruptible(&(device->tx_done_wait_queue));
    }

    if(nrf24_status & NRF24_REG_STATUS_MASK_MAX_RT){
        dev_err(&(device->dev), "%s: max rt\n", __func__);
        device->tx_failed = true;
        device->tx_done = true;
        status = nrf24_flush_tx_fifo(&(device->nrf24_hal_dev));
        if(status != HAL_OK){
            dev_err(&(device->dev), "%s: failed to flush tx fifo\n", __func__);
            return;
        }
        nrf24_clear_status = NRF24_REG_STATUS_MASK_MAX_RT;
        status = nrf24_set_status(&(device->nrf24_hal_dev), &nrf24_clear_status);
        if(status != HAL_OK){
            dev_err(&(device->dev), "%s: failed to set status\n", __func__);
            return;
        }
        wake_up_interruptible(&(device->tx_done_wait_queue));
    }
}

static void nrf24_rx_work_handler(struct work_struct *work){
    struct nrf24_device_t *device;
    struct nrf24_pipe_t *pipe;
    u8 data_buffer[NRF24_MAX_PAYLOAD_SIZE];
    uint8_t fifo_status;
    uint8_t status;
    uint8_t pipe_id;
    nrf24_hal_status_t hal_status;

    device = container_of(work, struct nrf24_device_t, rx_work);

    while(true){
        hal_status = nrf24_get_fifo_status(&(device->nrf24_hal_dev), &fifo_status);
        if(hal_status != HAL_OK){
            dev_err(&(device->dev), "%s: failed to get fifo status\n", __func__);
            return;
        }

        dev_dbg(&(device->dev), "%s: fifo status: 0x%02x\n", __func__, fifo_status);

        if(fifo_status & NRF24_REG_FIFO_STATUS_MASK_RX_EMPTY){
            break;
        }

        hal_status = nrf24_get_status(&(device->nrf24_hal_dev), &status);
        if(hal_status != HAL_OK){
            dev_err(&(device->dev), "%s: failed to get status\n", __func__);
            return;
        }

        pipe_id = (status & NRF24_REG_STATUS_MASK_RX_P_NO) >> 1;
        if(pipe_id > (NRF24_PIPES_COUNT - 1)){
            dev_err(&(device->dev), "%s: invalid pipe id (%u) or rx fifo empty\n", __func__, pipe_id);
            return;
        }

        pipe = device->pipes[pipe_id];

        memset(data_buffer, 0, (sizeof(data_buffer) / sizeof(*data_buffer)));

        hal_status = nrf24_get_rx_payload_width(&(device->nrf24_hal_dev), pipe_id, &(pipe->config.plw));
        if(hal_status != HAL_OK){
            dev_err(&(device->dev), "%s: failed to get rx payload width\n", __func__);
            return;
        }

        hal_status = nrf24_read_rx_fifo(&(device->nrf24_hal_dev), data_buffer, pipe->config.plw);
        if(hal_status != HAL_OK){
            dev_err(&(device->dev), "%s: failed to read rx fifo\n", __func__);
            return;
        }

        if(mutex_lock_interruptible(&(pipe->rx_fifo_lock))){
            dev_err(&(device->dev), "%s: failed to lock rx fifo mutex\n", __func__);
            return;
        }

        dev_dbg(&(device->dev), "%s: received %u bytes\n", __func__, pipe->config.plw);

        kfifo_in(&(pipe->rx_fifo), data_buffer, pipe->config.plw);
        mutex_unlock(&(pipe->rx_fifo_lock));

        wake_up_interruptible(&(pipe->read_wait_queue));
    }
}

static void nrf24_rx_active_timer_handler(struct timer_list *timer){
    struct nrf24_device_t *device = from_timer(device, timer, rx_active_timer);

    dev_dbg(&(device->dev), "%s: rx routine finished!\n", __func__);

    device->rx_active = false;

    if(!kfifo_is_empty(&(device->tx_fifo))){
        dev_dbg(&(device->dev), "%s: waking up TX\n", __func__);
        wake_up_interruptible(&(device->tx_wait_queue));
    }
}

static irqreturn_t nrf24_irq_handler(int irq, void *dev_id){
    struct nrf24_device_t *device = dev_id;
    unsigned long flags;

    spin_lock_irqsave(&(device->lock), flags);

    schedule_work(&(device->isr_work));

    spin_unlock_irqrestore(&(device->lock), flags);

    return IRQ_HANDLED;
}

static int nrf24_tx_task(void *data){
    struct nrf24_device_t *nrf24_dev = data;
    struct nrf24_pipe_t *pipe;
    struct nrf24_tx_data_t tx_data;
    nrf24_hal_status_t hal_status;
    int ret;

    while(true){
        dev_dbg(&(nrf24_dev->dev), "%s: waiting for new messages in TX FIFO\n", __func__);

        wait_event_interruptible(nrf24_dev->tx_wait_queue,
                                    kthread_should_stop() || (!nrf24_dev->rx_active && !kfifo_is_empty(&(nrf24_dev->tx_fifo))));

        if(kthread_should_stop()){
            break;
        }

        if(mutex_lock_interruptible(&(nrf24_dev->tx_fifo_lock))){
            dev_err(&(nrf24_dev->dev), "%s: failed to lock tx fifo mutex\n", __func__);
            continue;
        }

        dev_dbg(&(nrf24_dev->dev), "%s: reading from TX FIFO\n", __func__);

        ret = kfifo_out(&(nrf24_dev->tx_fifo), &tx_data, sizeof(tx_data));
        if(ret != sizeof(tx_data)){
            dev_err(&(nrf24_dev->dev), "%s: failed to read tx fifo\n", __func__);
            mutex_unlock(&(nrf24_dev->tx_fifo_lock));
            continue;
        }

        mutex_unlock(&(nrf24_dev->tx_fifo_lock));
        pipe = tx_data.pipe;

        nrf24_ce_off(&(nrf24_dev->nrf24_hal_dev));

        dev_dbg(&(nrf24_dev->dev), "%s: CE set to off\n", __func__);

        dev_dbg(&(nrf24_dev->dev), "%s: setting PTX mode\n", __func__);

        hal_status = nrf24_set_ptx_mode(&(nrf24_dev->nrf24_hal_dev));
        if(hal_status != HAL_OK){
            dev_err(&(nrf24_dev->dev), "%s: failed to set ptx mode\n", __func__);
            goto restore_rx_mode;
        }

        dev_dbg(&(nrf24_dev->dev), "%s: setting major pipe address (%llu)\n", __func__, pipe->config.addr);

        hal_status = nrf24_set_major_pipe_address(&(nrf24_dev->nrf24_hal_dev), 0, (u8 *)&(pipe->config.addr));
        if(hal_status != HAL_OK){
            dev_err(&(nrf24_dev->dev), "%s: failed to set major pipe address\n", __func__);
            goto restore_rx_mode;
        }

        dev_dbg(&(nrf24_dev->dev), "%s: setting tx address\n", __func__);

        hal_status = nrf24_set_tx_address(&(nrf24_dev->nrf24_hal_dev), (u8 *)&(pipe->config.addr));
        if(hal_status != HAL_OK){
            dev_err(&(nrf24_dev->dev), "%s: failed to set tx address\n", __func__);
            goto restore_rx_mode;
        }

        dev_dbg(&(nrf24_dev->dev), "%s: writing to tx FIFO\n", __func__);

        hal_status = nrf24_write_tx_fifo(&(nrf24_dev->nrf24_hal_dev), tx_data.payload, pipe->config.plw);
        if(hal_status != HAL_OK){
            dev_err(&(nrf24_dev->dev), "%s: failed to write to tx FIFO\n", __func__);
            goto restore_rx_mode;
        }

        nrf24_dev->tx_done = false;

        nrf24_ce_on(&(nrf24_dev->nrf24_hal_dev));

        dev_dbg(&(nrf24_dev->dev), "%s: CE set to on, sending data (%d) bytes...\n", __func__, pipe->config.plw);

        wait_event_interruptible(nrf24_dev->tx_done_wait_queue, (nrf24_dev->tx_done || kthread_should_stop()));

        if(kthread_should_stop()){
            break;
        }

        if(nrf24_dev->tx_failed){
            pipe->sent = 0;
        }
        else{
            pipe->sent = tx_data.size;
        }

        pipe->write_done = true;
        wake_up_interruptible(&(pipe->write_wait_queue));

restore_rx_mode:
        if(kfifo_is_empty(&(nrf24_dev->tx_fifo)) || nrf24_dev->rx_active){
            dev_dbg(&(nrf24_dev->dev), "%s: entering RX mode\n", __func__);

            nrf24_ce_off(&(nrf24_dev->nrf24_hal_dev));

            hal_status = nrf24_flush_tx_fifo(&(nrf24_dev->nrf24_hal_dev));
            if(hal_status != HAL_OK){
                dev_err(&(nrf24_dev->dev), "%s: failed to flush tx fifo\n", __func__);
                continue;
            }

            hal_status = nrf24_flush_rx_fifo(&(nrf24_dev->nrf24_hal_dev));
            if(hal_status != HAL_OK){
                dev_err(&(nrf24_dev->dev), "%s: failed to flush rx fifo\n", __func__);
                continue;
            }

            pipe = nrf24_dev->pipes[0];
            dev_dbg(&(nrf24_dev->dev), "%s: setting pipe (%d) address: (%llu)\n", __func__, pipe->id, pipe->config.addr);
            hal_status = nrf24_set_major_pipe_address(&(nrf24_dev->nrf24_hal_dev), pipe->id, (u8 *)&(pipe->config.addr));
            if(hal_status != HAL_OK){
                dev_err(&(nrf24_dev->dev), "%s: failed to set major pipe address\n", __func__);
                continue;
            }
            
            hal_status = nrf24_set_prx_mode(&(nrf24_dev->nrf24_hal_dev));
            if(hal_status != HAL_OK){
                dev_err(&(nrf24_dev->dev), "%s: failed to set prx mode\n", __func__);
                continue;
            }

            nrf24_ce_on(&(nrf24_dev->nrf24_hal_dev));

            dev_dbg(&(nrf24_dev->dev), "%s: CE set to on, reverted to PRX\n", __func__);
        }
    }
    
    return 0;
}

static void nrf24_init_device_configuration(struct nrf24_device_t *nrf24_dev){
    nrf24_dev->config.crc_mode = nrf24_default_crc_mode;
    nrf24_dev->config.data_rate = nrf24_default_air_data_rate;
    nrf24_dev->config.addr_width = nrf24_default_address_width;
    nrf24_dev->config.auto_retransmit_count = nrf24_default_auto_retransmit_count;
    nrf24_dev->config.auto_retransmit_delay = nrf24_default_auto_retransmit_delay;
    nrf24_dev->config.mode = nrf24_default_mode;
    nrf24_dev->config.tx_power = nrf24_default_tx_power;
    nrf24_dev->config.channel = nrf24_default_channel;
}

static void nrf24_destroy_devices(struct nrf24_device_t *nrf24_dev){
    struct nrf24_pipe_t *pipe;
    int i;

    for(i = 0; i < NRF24_PIPES_COUNT; i++){
        pipe = nrf24_dev->pipes[i];
        cdev_del(&(pipe->cdev));
        device_destroy(nrf24_dev->dev.class, MINOR(pipe->devt));
        ida_simple_remove(pipe_ida, pipe->id);
        kfree(pipe);
    }
}

static void nrf24_gpio_free(struct nrf24_device_t *nrf24_dev){
    free_irq(nrf24_dev->nrf24_hal_dev.spi->irq, nrf24_dev);

    if(!IS_ERR(nrf24_dev->nrf24_hal_dev.ce)){
        gpiod_put(nrf24_dev->nrf24_hal_dev.ce);
    }
}

static struct nrf24_device_t *nrf24_device_init(struct spi_device *spi, struct class *nrf24_class){
    int ret;
    struct nrf24_device_t *nrf24_dev;
    int id;

    id = ida_simple_get(dev_ida, 0, 0, GFP_KERNEL);
    if (id < 0) {
        dev_err(&(spi->dev), "%s: failed to get id\n", __func__);
        return ERR_PTR(id);
    }

    nrf24_dev = kzalloc(sizeof(struct nrf24_device_t), GFP_KERNEL);
    if (!nrf24_dev) {
        dev_err(&(spi->dev), "%s: failed to allocate memory\n", __func__);
        nrf24_dev = ERR_PTR(-ENOMEM);
        goto err_free_id;
    }
    nrf24_dev->nrf24_hal_dev.spi = spi;

    dev_set_name(&(nrf24_dev->dev), "nrf24-%d", id);

    nrf24_dev->id = id;

    nrf24_dev->rx_active = false;

    nrf24_dev->dev.parent = &(spi->dev);
    nrf24_dev->dev.class = nrf24_class;
    nrf24_dev->dev.type = &(nrf24_dev_type);

    nrf24_dev->dev.groups = nrf24_groups;

    ret = device_register(&(nrf24_dev->dev));
    if (ret) {
        dev_err(&(nrf24_dev->dev), "%s: failed to register device\n", __func__);
        nrf24_dev = ERR_PTR(ret);
        goto err_free_id;
    }

    init_waitqueue_head(&(nrf24_dev->tx_wait_queue));
    init_waitqueue_head(&(nrf24_dev->tx_done_wait_queue));

    INIT_WORK(&(nrf24_dev->isr_work), nrf24_isr_work_handler);
    INIT_WORK(&(nrf24_dev->rx_work), nrf24_rx_work_handler);
    INIT_KFIFO(nrf24_dev->tx_fifo);
    spin_lock_init(&(nrf24_dev->lock));
    mutex_init(&(nrf24_dev->tx_fifo_lock));

    timer_setup(&(nrf24_dev->rx_active_timer), nrf24_rx_active_timer_handler, 0);

    goto exit;

err_free_id:
    ida_simple_remove(dev_ida, id);
exit:
    return nrf24_dev;
}

static int nrf24_hal_rx_mode_init(struct nrf24_device_t *nrf24_dev){
    nrf24_hal_status_t status;
    struct nrf24_pipe_t *pipe;
    int i;

    nrf24_init_device_configuration(nrf24_dev);

    status = nrf24_soft_reset(&(nrf24_dev->nrf24_hal_dev));
    if(status != HAL_OK){
        dev_err(&(nrf24_dev->dev), "%s: failed to reset nrf24\n", __func__);
        return -EIO;
    }

    for(i = 0; i < NRF24_PIPES_COUNT; i++){
        pipe = nrf24_dev->pipes[i];

        status = nrf24_get_pipe_address(&(nrf24_dev->nrf24_hal_dev), pipe->id, (u8 *)&(pipe->config.addr));
        if(status != HAL_OK){
            dev_err(&(nrf24_dev->dev), "%s: failed to get pipe address\n", __func__);
            return -EIO;
        }

        status = nrf24_set_rx_payload_width(&(nrf24_dev->nrf24_hal_dev), pipe->id, pipe->config.plw);
        if(status != HAL_OK){
            dev_err(&(nrf24_dev->dev), "%s: failed to set rx payload width\n", __func__);
            return -EIO;
        }

        status = nrf24_set_en_rx_pipe(&(nrf24_dev->nrf24_hal_dev), pipe->id, true);
        if(status != HAL_OK){
            dev_err(&(nrf24_dev->dev), "%s: failed to set en rx pipe\n", __func__);
            return -EIO;
        }
    }

    status = nrf24_set_crc_mode(&(nrf24_dev->nrf24_hal_dev), nrf24_dev->config.crc_mode);
    if(status != HAL_OK){
        dev_err(&(nrf24_dev->dev), "%s: failed to set crc mode\n", __func__);
        return -EIO;
    }

    status = nrf24_set_radio_data_rate(&(nrf24_dev->nrf24_hal_dev), nrf24_dev->config.data_rate);
    if(status != HAL_OK){
        dev_err(&(nrf24_dev->dev), "%s: failed to set radio data rate\n", __func__);
        return -EIO;
    }

    status = nrf24_set_address_width(&(nrf24_dev->nrf24_hal_dev), nrf24_dev->config.addr_width);
    if(status != HAL_OK){
        dev_err(&(nrf24_dev->dev), "%s: failed to set address width\n", __func__);
        return -EIO;
    }

    status = nrf24_setup_retransmission(&(nrf24_dev->nrf24_hal_dev), nrf24_dev->config.auto_retransmit_delay, nrf24_dev->config.auto_retransmit_count);
    if(status != HAL_OK){
        dev_err(&(nrf24_dev->dev), "%s: failed to setup retransmission\n", __func__);
        return -EIO;
    }
    //FIXME
    // if(nrf24_dev->config.mode == NRF24_PM_TX){
    //     status = nrf24_set_ptx_mode(&(nrf24_dev->nrf24_hal_dev));
    //     if(status != HAL_OK){
    //         dev_err(&(nrf24_dev->dev), "%s: failed to set ptx mode\n", __func__);
    //         return -EIO;
    //     }
    // }
    // else{
    //     status = nrf24_set_prx_mode(&(nrf24_dev->nrf24_hal_dev));
    //     if(status != HAL_OK){
    //         dev_err(&(nrf24_dev->dev), "%s: failed to set prx mode\n", __func__);
    //         return -EIO;
    //     }
    // }

    status = nrf24_set_radio_output_power(&(nrf24_dev->nrf24_hal_dev), nrf24_dev->config.tx_power);
    if(status != HAL_OK){
        dev_err(&(nrf24_dev->dev), "%s: failed to set radio output power\n", __func__);
        return -EIO;
    }

    status = nrf24_set_radio_channel(&(nrf24_dev->nrf24_hal_dev), nrf24_dev->config.channel);
    if(status != HAL_OK){
        dev_err(&(nrf24_dev->dev), "%s: failed to set radio channel\n", __func__);
        return -EIO;
    }
    
    status = nrf24_power_up(&(nrf24_dev->nrf24_hal_dev));
    if(status != HAL_OK){
        dev_err(&(nrf24_dev->dev), "%s: failed to power up\n", __func__);
        return -EIO;
    }

    // nrf24_ce_on(&(nrf24_dev->nrf24_hal_dev));

    return status;
}

static int nrf24_gpio_init(struct nrf24_device_t *nrf24_dev){
    int ret;

    nrf24_dev->nrf24_hal_dev.ce = gpiod_get(&(nrf24_dev->nrf24_hal_dev.spi->dev), "ce", GPIOD_ASIS);

    if(IS_ERR(nrf24_dev->nrf24_hal_dev.ce)){
        dev_err(&(nrf24_dev->dev), "%s: failed to get ce gpio\n", __func__);
        return PTR_ERR(nrf24_dev->nrf24_hal_dev.ce);
    }

    nrf24_ce_off(&(nrf24_dev->nrf24_hal_dev));

    // IRQF_TRIGGER_NONE was set because the trigger is set in the device tree
    ret = request_irq(nrf24_dev->nrf24_hal_dev.spi->irq, nrf24_irq_handler, IRQF_TRIGGER_NONE, dev_name(&(nrf24_dev->dev)), nrf24_dev);

    if(ret < 0){
        dev_err(&(nrf24_dev->dev), "%s: failed to request irq\n", __func__);
        goto err_gpio_put;
    }

    return 0;

err_gpio_put:
    gpiod_put(nrf24_dev->nrf24_hal_dev.ce);
    return ret;
}

static int nrf24_open(struct inode *inode, struct file *filp)
{
	struct nrf24_pipe_t *pipe;

	pipe = container_of(inode->i_cdev, struct nrf24_pipe_t, cdev);

	if (!pipe) {
		pr_err("device: minor %d unknown.\n", iminor(inode));
		return -ENODEV;
	}

	filp->private_data = pipe;
	nonseekable_open(inode, filp);

	return 0;
}

static int nrf24_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static ssize_t nrf24_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct nrf24_pipe_t *pipe = filp->private_data;
    int ret;
    ssize_t n;
    unsigned int copied;

    dev_dbg(pipe->dev, "%s: reading %zu bytes\n", __func__, count);

    if(kfifo_is_empty(&(pipe->rx_fifo))){
        if(filp->f_flags & O_NONBLOCK){
            return -EAGAIN;
        }
        else{
            wait_event_interruptible(pipe->read_wait_queue, !kfifo_is_empty(&(pipe->rx_fifo)));
        }
    }

    ret = mutex_lock_interruptible(&(pipe->rx_fifo_lock));
    if(ret) {
        return ret;
    }

    n = kfifo_to_user(&(pipe->rx_fifo), buf, count, &copied);

    mutex_unlock(&(pipe->rx_fifo_lock));

    return n ? n : copied;
}

static ssize_t nrf24_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    struct nrf24_pipe_t *pipe = filp->private_data;
    struct nrf24_device_t *nrf24_dev = to_nrf24_device(pipe->dev->parent);
    struct nrf24_tx_data_t tx_data;
    ssize_t copied = 0;

    tx_data.pipe = pipe;

    dev_dbg(&(nrf24_dev->dev), "%s: writing %zu bytes\n", __func__, count);

    while(count > 0){
        tx_data.size = pipe->config.plw != 0 ? pipe->config.plw : min_t(size_t, count, NRF24_MAX_PAYLOAD_SIZE);

        memset(tx_data.payload, 0, (sizeof(tx_data.payload) / sizeof(*tx_data.payload)));
        if(copy_from_user(tx_data.payload, buf + copied, tx_data.size)){
            dev_err(&(nrf24_dev->dev), "%s: failed to copy data from user\n", __func__);
            goto exit;
        }

        if(mutex_lock_interruptible(&(nrf24_dev->tx_fifo_lock))){
            dev_err(&(nrf24_dev->dev), "%s: failed to lock tx fifo mutex\n", __func__);
            goto exit;
        }

        if(kfifo_in(&(nrf24_dev->tx_fifo), &tx_data, sizeof(tx_data)) != sizeof(tx_data)){
            dev_err(&(nrf24_dev->dev), "%s: failed to write to tx fifo\n", __func__);
            goto exit_unlock_mutex;
        }

        mutex_unlock(&(nrf24_dev->tx_fifo_lock));

        if(filp->f_flags & O_NONBLOCK){
            copied += tx_data.size;
        }
        else{
            wake_up_interruptible(&(nrf24_dev->tx_wait_queue));

            pipe->write_done = false;
            if(wait_event_interruptible(pipe->write_wait_queue, pipe->write_done) < 0){
                dev_err(&(nrf24_dev->dev), "%s: wait event interrupted\n", __func__);
                goto exit;
            }
            copied += pipe->sent;
        }

        count -= tx_data.size;
    }

exit_unlock_mutex:
    mutex_unlock(&(nrf24_dev->tx_fifo_lock));
exit:
    if(filp->f_flags & O_NONBLOCK){
        wake_up_interruptible(&(nrf24_dev->tx_wait_queue));
    }
    return copied;
}

static long nrf24_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return nrf24_handle_ioctl(filp, cmd, arg);
}

static const struct file_operations nrf24_fops = {
    .owner = THIS_MODULE,
    .read = nrf24_read,
    .write = nrf24_write,
    .open = nrf24_open,
    .release = nrf24_release,
    .llseek = no_llseek,
    .unlocked_ioctl = nrf24_ioctl
};

static struct nrf24_pipe_t *nrf24_create_pipe(struct nrf24_device_t *nrf24_dev, dev_t *devt, int pipe_id){
    int ret;
    struct nrf24_pipe_t *p;

    p = kzalloc(sizeof(*p), GFP_KERNEL);
    if(!p){
        ret = -ENOMEM;
        goto err_exit;
    }

    ret = ida_simple_get(pipe_ida, 0, 0, GFP_KERNEL);
    if(ret < 0){
        dev_err(&(nrf24_dev->dev), "%s: failed to get id\n", __func__);
        goto err_free_pipe;
    }

    p->devt = MKDEV(MAJOR(*devt), ret);
    p->id = pipe_id;

    INIT_KFIFO(p->rx_fifo);
    mutex_init(&(p->rx_fifo_lock));
    init_waitqueue_head(&(p->read_wait_queue));
    init_waitqueue_head(&(p->write_wait_queue));

    p->dev = device_create_with_groups(nrf24_dev->dev.class,
                            &(nrf24_dev->dev),
                            p->devt, p,
                            nrf24_pipe_groups,
                            "%s.%d",
                            dev_name(&(nrf24_dev->dev)),
                            p->id);
    if(IS_ERR(p->dev)){
        dev_err(&(nrf24_dev->dev), "%s: failed to create device (pipe: %d)\n", __func__, p->id);
        ret = PTR_ERR(p->dev);
        goto err_free_id;
    }

    cdev_init(&(p->cdev), &nrf24_fops);
    p->cdev.owner = THIS_MODULE;
    ret = cdev_add(&(p->cdev), p->devt, 1);
    if(ret < 0){
        dev_err(&(nrf24_dev->dev), "%s: failed to add cdev (pipe: %d)\n", __func__, p->id);
        goto err_device_destroy;
    }

    p->config.plw = NRF24_MAX_PAYLOAD_SIZE;

    return p;

err_device_destroy:
    device_destroy(nrf24_dev->dev.class, p->devt);
err_free_id:
    ida_simple_remove(pipe_ida, MINOR(p->devt));
err_free_pipe:
    kfree(p);
err_exit:
    return ERR_PTR(ret);
}

int nrf24_mod_probe(struct spi_device *spi, dev_t *devt, struct class *nrf24_class, struct ida *pipe_ida_ptr, struct ida *dev_ida_ptr){
    int ret;
    struct nrf24_device_t *nrf24_dev;
    struct nrf24_pipe_t *pipe;
    int i;

    dev_ida = dev_ida_ptr;
    pipe_ida = pipe_ida_ptr;

    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;

    ret = spi_setup(spi);
    if (ret < 0) {
        dev_err(&(spi->dev), "%s: failed to setup spi\n", __func__);
        return ret;
    }

    nrf24_dev = nrf24_device_init(spi, nrf24_class);
    if(IS_ERR(nrf24_dev)){
        dev_err(&(spi->dev), "%s: failed to init nrf24 device\n", __func__);
        return PTR_ERR(nrf24_dev);
    }

    ret = nrf24_gpio_init(nrf24_dev);
    if(ret < 0){
        dev_err(&(spi->dev), "%s: failed to init nrf24 gpio\n", __func__);
        goto err_device_unregister;
    }

    for(i = 0; i < NRF24_PIPES_COUNT; i++){
        pipe = nrf24_create_pipe(nrf24_dev, devt, i);
        if(IS_ERR(pipe)){
            dev_err(&(spi->dev), "%s: failed to create pipe\n", __func__);
            ret = PTR_ERR(pipe);
            goto err_devices_destroy;
        }
        nrf24_dev->pipes[i] = pipe;
    }

    ret = nrf24_hal_rx_mode_init(nrf24_dev);
    if(ret < 0){
        dev_err(&(spi->dev), "%s: failed to init nrf24 for rx mode\n", __func__);
        goto err_devices_destroy;
    }

    nrf24_dev->tx_task_struct = kthread_run(nrf24_tx_task, nrf24_dev, "nrf24-%d_tx_task", nrf24_dev->id);

    if(IS_ERR(nrf24_dev->tx_task_struct)){
        dev_err(&(spi->dev), "%s: failed to create tx task\n", __func__);
        goto err_devices_destroy;
    }
    
    spi_set_drvdata(spi, nrf24_dev);

    return 0;

err_devices_destroy:
    nrf24_destroy_devices(nrf24_dev);
    nrf24_gpio_free(nrf24_dev);
err_device_unregister:
    device_unregister(&(nrf24_dev->dev));
    return ret;
}

void nrf24_mod_remove(struct spi_device *spi, struct class *nrf24_class){
    struct nrf24_device_t *nrf24_dev = spi_get_drvdata(spi);

    nrf24_gpio_free(nrf24_dev);

    kthread_stop(nrf24_dev->tx_task_struct);

    nrf24_destroy_devices(nrf24_dev);

    device_unregister(&(nrf24_dev->dev));
}