dts: dts/nrf24_overlay.dts
	dtc -@ -I dts -O dtb -o nrf24_overlay.dtbo dts/nrf24_overlay.dts

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.dtb *.dtbo