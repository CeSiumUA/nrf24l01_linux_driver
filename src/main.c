#include "main.h"
#include "nrf24_mod.h"

#define NRF24_MINORS            BIT(MINORBITS)

MODULE_AUTHOR("CeSiumUA");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("SPI driver for NRF24");
MODULE_ALIAS("spi:nrf24");

static const struct of_device_id nrf24_dt_ids[] = {
    { .compatible = "nordic,nrf24", },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, nrf24_dt_ids);

static int nrf24_probe(struct spi_device *spi);
static void nrf24_remove(struct spi_device *spi);

static struct spi_driver nrf24_spi_driver = {
    .driver = {
        .name = "nrf24",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(nrf24_dt_ids),
    },
    .probe = nrf24_probe,
    .remove = nrf24_remove,
};

static dev_t nrf24_dev;
static DEFINE_IDA(nrf24_ida_pipe);
static DEFINE_IDA(nrf24_ida_dev);
static struct class *nrf24_class;

static int nrf24_probe(struct spi_device *spi){
    return nrf24_mod_probe(spi, &nrf24_dev, nrf24_class, &nrf24_ida_pipe, &nrf24_ida_dev);
}

static void nrf24_remove(struct spi_device *spi){
    nrf24_mod_remove(spi, nrf24_class);
}

static int __init nrf24_init(void)
{
    int ret;

    ret = alloc_chrdev_region(&nrf24_dev, 0, NRF24_MINORS, nrf24_spi_driver.driver.name);
    if (ret < 0) {
        pr_err("nrf24: failed to allocate chrdev region\n");
        goto err_destroy_ida;
    }

    nrf24_class = class_create(nrf24_spi_driver.driver.name);
    if (IS_ERR(nrf24_class)) {
        pr_err("nrf24: failed to create class\n");
        ret = PTR_ERR(nrf24_class);
        goto err_unregister_chrdev;
    }

    ret = spi_register_driver(&nrf24_spi_driver);
    if (ret < 0) {
        pr_err("nrf24: failed to register spi driver\n");
        goto err_destroy_class;
    }

    return 0;

err_destroy_class:
    class_destroy(nrf24_class);
err_unregister_chrdev:
    unregister_chrdev(MAJOR(nrf24_dev), nrf24_spi_driver.driver.name);
err_destroy_ida:
    ida_destroy(&nrf24_ida_pipe);
    ida_destroy(&nrf24_ida_dev);

    return ret;
}

static void __exit nrf24_exit(void)
{
    spi_unregister_driver(&nrf24_spi_driver);
    class_destroy(nrf24_class);
    unregister_chrdev(MAJOR(nrf24_dev), nrf24_spi_driver.driver.name);
    ida_destroy(&nrf24_ida_pipe);
    ida_destroy(&nrf24_ida_dev);
}

module_init(nrf24_init);
module_exit(nrf24_exit);