#include "nrf24_mod.h"

static struct ida *dev_ida;
static struct ida *pipe_ida;

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
    // TODO: implement isr work handler
}

static void nrf24_rx_work_handler(struct work_struct *work){
    // TODO: implement rx work handler
}

static void nrf24_rx_active_timer_handler(struct timer_list *timer){
    // TODO: implement rx active timer handler
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
    nrf24_dev->dev.parent = &(spi->dev);
    nrf24_dev->dev.class = nrf24_class;
    nrf24_dev->dev.type = &(nrf24_dev_type);

    // FIXME: add sysfs attributes https://github.com/CeSiumUA/nrf24l01_linux_driver/issues/5
    // nrf24_dev->dev.groups = nrf24_groups;

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

    INIT_LIST_HEAD(&(nrf24_dev->pipes));

    timer_setup(&(nrf24_dev->rx_active_timer), nrf24_rx_active_timer_handler, 0);

    goto exit;

err_free_id:
    ida_simple_remove(dev_ida, id);
exit:
    return nrf24_dev;
}

int nrf24_mod_probe(struct spi_device *spi, dev_t *devt, struct class *nrf24_class, struct ida *pipe_ida_ptr, struct ida *dev_ida_ptr){
    int ret;
    struct nrf24_device_t *nrf24_dev;
    struct nrf24_pipe *pipe;
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

    
    
    return 0;
}

void nrf24_mod_remove(struct spi_device *spi, struct class *nrf24_class){

}