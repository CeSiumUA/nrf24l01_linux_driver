#DEBUG = y

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g -DSCULL_DEBUG # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

EXTRA_CFLAGS += $(DEBFLAGS)
EXTRA_CFLAGS += -I$(PWD)/inc
EXTRA_CFLAGS += -DDEBUG

ifneq ($(KERNELRELEASE),)
# call from kernel build system
obj-m	:= nrf24.o
nrf24-y := src/main.o src/nrf24_mod.o src/nrf24_hal.o
else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

endif

dtbo: dts/nrf24_overlay.dts
	dtc -@ -I dts -O dtb -o nrf24_overlay.dtbo dts/nrf24_overlay.dts

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c *.mod .tmp_versions *.dtb *.dtbo modules.order Module.symvers
