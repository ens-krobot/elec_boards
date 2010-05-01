# rule to flash the firmware

flash: size
	@cp $(BUILDDIR)/$(PROJECT).bin jtag/fw.bin
	@cd jtag; openocd -f openocd.cfg -f flash.cfg
	@rm jtag/fw.bin

# *** EOF ***
