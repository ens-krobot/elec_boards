#
# User makefile.
# Edit this file to change compiler options and related stuff.
#

# Programmer interface configuration, see http://dev.bertos.org/wiki/ProgrammerInterface for help
sensor_actuator_PROGRAMMER_TYPE = none
sensor_actuator_PROGRAMMER_PORT = none

# Files included by the user.
sensor_actuator_USER_CSRC = \
	$(sensor_actuator_SRC_PATH)/main.c \
	$(sensor_actuator_SRC_PATH)/beacon/stm32lib/stm32f10x_tim.c \
	$(sensor_actuator_SRC_PATH)/beacon/beacon.c \
	$(sensor_actuator_SRC_PATH)/can/can_monitor.c \
	$(sensor_actuator_SRC_PATH)/switch/switch.c \
#	$(sensor_actuator_SRC_PATH)/ax12/serial.c \
#	$(sensor_actuator_SRC_PATH)/ax12/ax12.c \
	#

# Files included by the user.
sensor_actuator_USER_PCSRC = \
	#

# Files included by the user.
sensor_actuator_USER_CPPASRC = \
	#

# Files included by the user.
sensor_actuator_USER_CXXSRC = \
	#

# Files included by the user.
sensor_actuator_USER_ASRC = \
	#

# Flags included by the user.
sensor_actuator_USER_LDFLAGS = \
	#

# Flags included by the user.
sensor_actuator_USER_CPPAFLAGS = \
	#

# Flags included by the user.
sensor_actuator_USER_CPPFLAGS = \
	-fno-strict-aliasing \
	-fwrapv \
	#
