#
# User makefile.
# Edit this file to change compiler options and related stuff.
#

# Programmer interface configuration, see http://dev.bertos.org/wiki/ProgrammerInterface for help
controller_motor_stm32_PROGRAMMER_TYPE = none
controller_motor_stm32_PROGRAMMER_PORT = none

# Files included by the user.
controller_motor_stm32_USER_CSRC = \
	$(controller_motor_stm32_SRC_PATH)/stm32lib/stm32f10x_rcc.c \
	$(controller_motor_stm32_SRC_PATH)/stm32lib/stm32f10x_tim.c \
	$(controller_motor_stm32_SRC_PATH)/motor.c \
	$(controller_motor_stm32_SRC_PATH)/encoder.c \
	$(controller_motor_stm32_SRC_PATH)/asservissement.c \
	$(controller_motor_stm32_SRC_PATH)/intelligence.c \
	$(controller_motor_stm32_SRC_PATH)/odometry.c \
	$(controller_motor_stm32_SRC_PATH)/can_monitor.c \
	$(controller_motor_stm32_SRC_PATH)/AX12.c \
	$(controller_motor_stm32_SRC_PATH)/avoid.c \
	$(controller_motor_stm32_SRC_PATH)/main.c \
	#

# Files included by the user.
controller_motor_stm32_USER_PCSRC = \
	#

# Files included by the user.
controller_motor_stm32_USER_CPPASRC = \
	#

# Files included by the user.
controller_motor_stm32_USER_CXXSRC = \
	#

# Files included by the user.
controller_motor_stm32_USER_ASRC = \
	#

# Flags included by the user.
controller_motor_stm32_USER_LDFLAGS = \
	#

# Flags included by the user.
controller_motor_stm32_USER_CPPAFLAGS = \
	#

# Flags included by the user.
controller_motor_stm32_USER_CPPFLAGS = \
	-fno-strict-aliasing \
	-fwrapv \
	#
