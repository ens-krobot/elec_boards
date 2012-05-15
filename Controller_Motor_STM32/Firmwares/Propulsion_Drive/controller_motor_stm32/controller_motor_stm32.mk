#
# Wizard autogenerated makefile.
# DO NOT EDIT, use the controller_motor_stm32_user.mk file instead.
#

# Constants automatically defined by the selected modules
Firmware_DEBUG = 1

# Our target application
TRG += controller_motor_stm32

controller_motor_stm32_PREFIX = "/media/data/krobot/arm-i386/bin/arm-none-eabi-"

controller_motor_stm32_SUFFIX = ""

controller_motor_stm32_SRC_PATH = controller_motor_stm32

controller_motor_stm32_HW_PATH = controller_motor_stm32

# Files automatically generated by the wizard. DO NOT EDIT, USE controller_motor_stm32_USER_CSRC INSTEAD!
controller_motor_stm32_WIZARD_CSRC = \
	bertos/cpu/cortex-m3/drv/can_stm32.c \
	bertos/cpu/cortex-m3/hw/switch_ctx_cm3.c \
	bertos/mware/event.c \
	bertos/kern/monitor.c \
	bertos/cpu/cortex-m3/drv/timer_cm3.c \
	bertos/struct/heap.c \
	bertos/drv/can.c \
	bertos/mware/formatwr.c \
	bertos/drv/timer.c \
	bertos/kern/signal.c \
	bertos/kern/proc.c \
	bertos/mware/hex.c \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE controller_motor_stm32_USER_PCSRC INSTEAD!
controller_motor_stm32_WIZARD_PCSRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE controller_motor_stm32_USER_CPPASRC INSTEAD!
controller_motor_stm32_WIZARD_CPPASRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE controller_motor_stm32_USER_CXXSRC INSTEAD!
controller_motor_stm32_WIZARD_CXXSRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE controller_motor_stm32_USER_ASRC INSTEAD!
controller_motor_stm32_WIZARD_ASRC = \
	 \
	#

controller_motor_stm32_CPPFLAGS = -D'CPU_FREQ=(72000000UL)' -D'ARCH=(ARCH_DEFAULT)' -D'WIZ_AUTOGEN' -I$(controller_motor_stm32_HW_PATH) -I$(controller_motor_stm32_SRC_PATH) $(controller_motor_stm32_CPU_CPPFLAGS) $(controller_motor_stm32_USER_CPPFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
controller_motor_stm32_LDFLAGS = $(controller_motor_stm32_CPU_LDFLAGS) $(controller_motor_stm32_WIZARD_LDFLAGS) $(controller_motor_stm32_USER_LDFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
controller_motor_stm32_CPPAFLAGS = $(controller_motor_stm32_CPU_CPPAFLAGS) $(controller_motor_stm32_WIZARD_CPPAFLAGS) $(controller_motor_stm32_USER_CPPAFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
controller_motor_stm32_CSRC = $(controller_motor_stm32_CPU_CSRC) $(controller_motor_stm32_WIZARD_CSRC) $(controller_motor_stm32_USER_CSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
controller_motor_stm32_PCSRC = $(controller_motor_stm32_CPU_PCSRC) $(controller_motor_stm32_WIZARD_PCSRC) $(controller_motor_stm32_USER_PCSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
controller_motor_stm32_CPPASRC = $(controller_motor_stm32_CPU_CPPASRC) $(controller_motor_stm32_WIZARD_CPPASRC) $(controller_motor_stm32_USER_CPPASRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
controller_motor_stm32_CXXSRC = $(controller_motor_stm32_CPU_CXXSRC) $(controller_motor_stm32_WIZARD_CXXSRC) $(controller_motor_stm32_USER_CXXSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
controller_motor_stm32_ASRC = $(controller_motor_stm32_CPU_ASRC) $(controller_motor_stm32_WIZARD_ASRC) $(controller_motor_stm32_USER_ASRC)

# CPU specific flags and options, defined in the CPU definition files.
# Automatically generated by the wizard. PLEASE DO NOT EDIT!
controller_motor_stm32_CPU_CPPASRC = bertos/cpu/cortex-m3/hw/crt_cm3.S bertos/cpu/cortex-m3/hw/vectors_cm3.S 
controller_motor_stm32_CPU_CPPAFLAGS = -g -gdwarf-2 -mthumb -mno-thumb-interwork
controller_motor_stm32_CPU_CPPFLAGS = -O0 -g3 -gdwarf-2 -mthumb -mno-thumb-interwork -fno-strict-aliasing -fwrapv -fverbose-asm -Ibertos/cpu/cortex-m3/ -D__ARM_STM32F103RE__
controller_motor_stm32_CPU_CSRC = bertos/cpu/cortex-m3/hw/init_cm3.c bertos/cpu/cortex-m3/drv/irq_cm3.c bertos/cpu/cortex-m3/drv/gpio_stm32.c bertos/cpu/cortex-m3/drv/clock_stm32.c 
controller_motor_stm32_PROGRAMMER_CPU = stm32
controller_motor_stm32_CPU_LDFLAGS = -mthumb -mno-thumb-interwork -nostartfiles -Wl,--no-warn-mismatch -Wl,-dT bertos/cpu/cortex-m3/scripts/stm32f103re_rom.ld
controller_motor_stm32_STOPFLASH_SCRIPT = bertos/prg_scripts/arm/stopopenocd.sh
controller_motor_stm32_CPU = cortex-m3
controller_motor_stm32_STOPDEBUG_SCRIPT = bertos/prg_scripts/arm/stopopenocd.sh
controller_motor_stm32_DEBUG_SCRIPT = bertos/prg_scripts/arm/debug.sh
controller_motor_stm32_FLASH_SCRIPT = bertos/prg_scripts/arm/flash-stm32.sh

include $(controller_motor_stm32_SRC_PATH)/controller_motor_stm32_user.mk
