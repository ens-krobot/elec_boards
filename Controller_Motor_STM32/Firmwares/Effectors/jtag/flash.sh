#!/bin/sh

file=images/*.bin
cp $file jtag/fw.bin
cd jtag
openocd -f openocd.cfg -f flash.cfg
rm -f fw.bin

