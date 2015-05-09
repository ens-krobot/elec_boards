#!/bin/sh

file=images/*.bin
cp $file jtag/fw.bin
cd jtag
openocd -f olimex-arm-usb-ocd.cfg -f krobot.cfg -f flash.cfg
rm -f fw.bin

