#include <linux/module.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

int aesd_major =   0;
int aesd_minor =   0;

MODULE_AUTHOR("CeSiumUA");
MODULE_LICENSE("Dual BSD/GPL");

int nrf24l01_init_module(void)
{
    printk(KERN_INFO "nrf24l01: Hello, world\n");
    return 0;
}

void nrf24l01_cleanup_module(void)
{
    printk(KERN_INFO "nrf24l01: Goodbye, world\n");
}

module_init(nrf24l01_init_module);
module_exit(nrf24l01_cleanup_module);