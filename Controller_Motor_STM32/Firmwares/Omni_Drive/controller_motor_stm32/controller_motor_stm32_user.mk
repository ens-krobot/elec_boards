#
# User makefile.
# Edit this file to change compiler options and related stuff.
#

# Programmer interface configuration, see http://dev.bertos.org/wiki/ProgrammerInterface for help
controller_motor_stm32_PROGRAMMER_TYPE = none
controller_motor_stm32_PROGRAMMER_PORT = none

# Support board library
controller_motor_stm32_LIB_PATH = $(controller_motor_stm32_SRC_PATH)/../../lib
CFLAGS += -I$(controller_motor_stm32_LIB_PATH)

# Files included by the user.
controller_motor_stm32_USER_CSRC = \
	$(controller_motor_stm32_LIB_PATH)/stm32lib/stm32f10x_rcc.c \
	$(controller_motor_stm32_LIB_PATH)/stm32lib/stm32f10x_tim.c \
	$(controller_motor_stm32_LIB_PATH)/motor.c \
	$(controller_motor_stm32_LIB_PATH)/encoder.c \
	$(controller_motor_stm32_LIB_PATH)/odometry.c \
	$(controller_motor_stm32_LIB_PATH)/odometry_holonomic.c \
	$(controller_motor_stm32_LIB_PATH)/motor_controller.c \
	$(controller_motor_stm32_LIB_PATH)/command_generator.c \
	$(controller_motor_stm32_LIB_PATH)/trajectory_controller.c \
	$(controller_motor_stm32_LIB_PATH)/bezier_utils.c \
	$(controller_motor_stm32_LIB_PATH)/differential_drive.c \
	$(controller_motor_stm32_LIB_PATH)/holonomic_drive.c \
	$(controller_motor_stm32_LIB_PATH)/lift_controller.c \
	$(controller_motor_stm32_LIB_PATH)/arm2R_controller.c \
	$(controller_motor_stm32_SRC_PATH)/can_monitor.c \
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
