dts: dts/nrf24.dts
	dtc -@ -I dts -O dtb -o nrf24_overlay.dtbo dts/nrf24.dts

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.dtb *.dtbo