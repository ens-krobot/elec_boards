#
# Wizard autogenerated makefile.
# DO NOT EDIT, use the rotary_beacon_user.mk file instead.
#

# Constants automatically defined by the selected modules
rotary_beacon_DEBUG = 1

# Our target application
TRG += rotary_beacon

rotary_beacon_PREFIX = "/home/nicolasd/opt/arm/bin/arm-none-eabi-"

rotary_beacon_SUFFIX = ""

rotary_beacon_SRC_PATH = rotary_beacon

rotary_beacon_HW_PATH = rotary_beacon

# Files automatically generated by the wizard. DO NOT EDIT, USE rotary_beacon_USER_CSRC INSTEAD!
rotary_beacon_WIZARD_CSRC = \
	bertos/cpu/cortex-m3/drv/can_stm32.c \
	bertos/cpu/cortex-m3/hw/switch_ctx_cm3.c \
	bertos/mware/event.c \
	bertos/kern/sem.c \
	bertos/kern/monitor.c \
	bertos/cpu/cortex-m3/drv/timer_cm3.c \
	bertos/struct/heap.c \
	bertos/drv/can.c \
	bertos/io/kfile.c \
	bertos/mware/formatwr.c \
	bertos/drv/timer.c \
	bertos/kern/signal.c \
	bertos/kern/proc.c \
	bertos/mware/hex.c \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE rotary_beacon_USER_PCSRC INSTEAD!
rotary_beacon_WIZARD_PCSRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE rotary_beacon_USER_CPPASRC INSTEAD!
rotary_beacon_WIZARD_CPPASRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE rotary_beacon_USER_CXXSRC INSTEAD!
rotary_beacon_WIZARD_CXXSRC = \
	 \
	#

# Files automatically generated by the wizard. DO NOT EDIT, USE rotary_beacon_USER_ASRC INSTEAD!
rotary_beacon_WIZARD_ASRC = \
	 \
	#

rotary_beacon_CPPFLAGS = -D'CPU_FREQ=(72000000UL)' -D'ARCH=(ARCH_DEFAULT)' -D'WIZ_AUTOGEN' -I$(rotary_beacon_HW_PATH) -I$(rotary_beacon_SRC_PATH) $(rotary_beacon_CPU_CPPFLAGS) $(rotary_beacon_USER_CPPFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
rotary_beacon_LDFLAGS = $(rotary_beacon_CPU_LDFLAGS) $(rotary_beacon_WIZARD_LDFLAGS) $(rotary_beacon_USER_LDFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
rotary_beacon_CPPAFLAGS = $(rotary_beacon_CPU_CPPAFLAGS) $(rotary_beacon_WIZARD_CPPAFLAGS) $(rotary_beacon_USER_CPPAFLAGS)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
rotary_beacon_CSRC = $(rotary_beacon_CPU_CSRC) $(rotary_beacon_WIZARD_CSRC) $(rotary_beacon_USER_CSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
rotary_beacon_PCSRC = $(rotary_beacon_CPU_PCSRC) $(rotary_beacon_WIZARD_PCSRC) $(rotary_beacon_USER_PCSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
rotary_beacon_CPPASRC = $(rotary_beacon_CPU_CPPASRC) $(rotary_beacon_WIZARD_CPPASRC) $(rotary_beacon_USER_CPPASRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
rotary_beacon_CXXSRC = $(rotary_beacon_CPU_CXXSRC) $(rotary_beacon_WIZARD_CXXSRC) $(rotary_beacon_USER_CXXSRC)

# Automatically generated by the wizard. PLEASE DO NOT EDIT!
rotary_beacon_ASRC = $(rotary_beacon_CPU_ASRC) $(rotary_beacon_WIZARD_ASRC) $(rotary_beacon_USER_ASRC)

# CPU specific flags and options, defined in the CPU definition files.
# Automatically generated by the wizard. PLEASE DO NOT EDIT!
rotary_beacon_CPU_CPPASRC = bertos/cpu/cortex-m3/hw/crt_cm3.S bertos/cpu/cortex-m3/hw/vectors_cm3.S 
rotary_beacon_CPU_CPPAFLAGS = -g -gdwarf-2 -mthumb -mno-thumb-interwork
rotary_beacon_CPU_CPPFLAGS = -O0 -g3 -gdwarf-2 -mthumb -mno-thumb-interwork -fno-strict-aliasing -fwrapv -fverbose-asm -Ibertos/cpu/cortex-m3/ -D__ARM_STM32F103RB__
rotary_beacon_CPU_CSRC = bertos/cpu/cortex-m3/hw/init_cm3.c bertos/cpu/cortex-m3/drv/irq_cm3.c bertos/cpu/cortex-m3/drv/gpio_stm32.c bertos/cpu/cortex-m3/drv/clock_stm32.c 
rotary_beacon_PROGRAMMER_CPU = stm32
rotary_beacon_CPU_LDFLAGS = -mthumb -mno-thumb-interwork -nostartfiles -Wl,--no-warn-mismatch -Wl,-dT bertos/cpu/cortex-m3/scripts/stm32f103rb_rom.ld
rotary_beacon_STOPFLASH_SCRIPT = bertos/prg_scripts/arm/stopopenocd.sh
rotary_beacon_CPU = cortex-m3
rotary_beacon_STOPDEBUG_SCRIPT = bertos/prg_scripts/arm/stopopenocd.sh
rotary_beacon_DEBUG_SCRIPT = bertos/prg_scripts/arm/debug.sh
rotary_beacon_FLASH_SCRIPT = bertos/prg_scripts/arm/flash-stm32.sh

include $(rotary_beacon_SRC_PATH)/rotary_beacon_user.mk
