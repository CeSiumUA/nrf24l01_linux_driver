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

static irqreturn_t nrf24_irq_handler(int irq, void *dev_id){
    // TODO: implement irq handler
    return IRQ_HANDLED;
}

static int nrf24_tx_task(void *data){
    // TODO: implement tx task
    
    return 0;
}

static void nrf24_destroy_devices(struct nrf24_device_t *nrf24_dev){
    struct nrf24_pipe_t *pipe, *tmp;

    list_for_each_entry_safe(pipe, tmp, &(nrf24_dev->pipes), list){
        list_del(&(pipe->list));
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

    // TODO: Create pipes and initialize NRF24 for RX mode

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

}