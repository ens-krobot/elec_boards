#
# Wizard autogenerated makefile.
# DO NOT EDIT, use the usb_can_user.mk file instead.
#

# Constants automatically defined by the selected modules
Firmware_DEBUG = 1

# Our target application
TRG += usb_can

usb_can_PREFIX = "arm-none-eabi-"

usb_can_SUFFIX = ""

usb_can_SRC_PATH = usb_can

usb_can_HW_PATH = usb_can

# Files automatically generated by the wizard. DO NOT EDIT, USE usb_can_USER_CSRC INSTEAD!
usb_can_WIZARD_CSRC = \
	bertos/cpu/cortex-m3/drv/ser_stm32.c \
	bertos/cpu/cortex-m3/hw/switch_ctx_cm3.c \
	bertos/mware/event.c \
	bertos/kern/sem.c \
	bertos/kern/monitor.c \
	bertos/cpu/cortex-m3/drv/timer_cm3.c \
	bertos/struct/heap.c \
	bertos/drv/can.c \
	bertos/cpu/cortex-m3/drv/can_stm32.c \
	bertos/io/kfile.c \
	bertos/mware/formatwr.c \
	bertos/mware/sprintf.c \
	bertos/drv/timer.c \
	bertos/kern/signal.c \
	bertos/kern/proc.c \
	bertos/drv/ser.c \
	bertos/mware/hex.c \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE usb_can_USER_PCSRC INSTEAD!
usb_can_WIZARD_PCSRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE usb_can_USER_CPPASRC INSTEAD!
usb_can_WIZARD_CPPASRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE usb_can_USER_CXXSRC INSTEAD!
usb_can_WIZARD_CXXSRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE usb_can_USER_ASRC INSTEAD!
usb_can_WIZARD_ASRC = \
	 \
	#

usb_can_CPPFLAGS = -D'CPU_FREQ=(72000000UL)' -D'ARCH=(ARCH_DEFAULT)' -D'WIZ_AUTOGEN' -I$(usb_can_HW_PATH) -I$(usb_can_SRC_PATH) $(usb_can_CPU_CPPFLAGS) $(usb_can_USER_CPPFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
usb_can_LDFLAGS = $(usb_can_CPU_LDFLAGS) $(usb_can_WIZARD_LDFLAGS) $(usb_can_USER_LDFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
usb_can_CPPAFLAGS = $(usb_can_CPU_CPPAFLAGS) $(usb_can_WIZARD_CPPAFLAGS) $(usb_can_USER_CPPAFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
usb_can_CSRC = $(usb_can_CPU_CSRC) $(usb_can_WIZARD_CSRC) $(usb_can_USER_CSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
usb_can_PCSRC = $(usb_can_CPU_PCSRC) $(usb_can_WIZARD_PCSRC) $(usb_can_USER_PCSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
usb_can_CPPASRC = $(usb_can_CPU_CPPASRC) $(usb_can_WIZARD_CPPASRC) $(usb_can_USER_CPPASRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
usb_can_CXXSRC = $(usb_can_CPU_CXXSRC) $(usb_can_WIZARD_CXXSRC) $(usb_can_USER_CXXSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
usb_can_ASRC = $(usb_can_CPU_ASRC) $(usb_can_WIZARD_ASRC) $(usb_can_USER_ASRC)

# CPU specific flags and options, defined in the CPU definition files.
# Automatically generated by the wizard. PLEASE DO NOT EDIT!
usb_can_CPU_CPPASRC = bertos/cpu/cortex-m3/hw/crt_cm3.S bertos/cpu/cortex-m3/hw/vectors_cm3.S 
usb_can_CPU_CPPAFLAGS = -g -gdwarf-2 -mthumb -mno-thumb-interwork
usb_can_CPU_CPPFLAGS = -O0 -g3 -gdwarf-2 -mthumb -mno-thumb-interwork -fno-strict-aliasing -fwrapv -fverbose-asm -Ibertos/cpu/cortex-m3/ -D__ARM_STM32F103RE__
usb_can_CPU_CSRC = bertos/cpu/cortex-m3/hw/init_cm3.c bertos/cpu/cortex-m3/drv/irq_cm3.c bertos/cpu/cortex-m3/drv/gpio_stm32.c bertos/cpu/cortex-m3/drv/clock_stm32.c 
usb_can_PROGRAMMER_CPU = stm32
usb_can_CPU_LDFLAGS = -mthumb -mno-thumb-interwork -nostartfiles -Wl,--no-warn-mismatch -Wl,-dT bertos/cpu/cortex-m3/scripts/stm32f103re_rom.ld
usb_can_STOPFLASH_SCRIPT = bertos/prg_scripts/arm/stopopenocd.sh
usb_can_CPU = cortex-m3
usb_can_STOPDEBUG_SCRIPT = bertos/prg_scripts/arm/stopopenocd.sh
usb_can_DEBUG_SCRIPT = bertos/prg_scripts/arm/debug.sh
usb_can_FLASH_SCRIPT = bertos/prg_scripts/arm/flash-stm32.sh

include $(usb_can_SRC_PATH)/usb_can_user.mk
