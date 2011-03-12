# rule to flash the firmware

flash:
	@cp $(BUILDDIR)/$(PROJECT).bin jtag/fw.bin
	@cd jtag; openocd -f /usr/share/openocd/scripts/interface/olimex-jtag-tiny.cfg -f /usr/share/openocd/scripts/target/stm32.cfg -f flash.cfg
	@rm jtag/fw.bin

# *** EOF ***
