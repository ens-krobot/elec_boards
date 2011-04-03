#
# User makefile.
# Edit this file to change compiler options and related stuff.
#

# Programmer interface configuration, see http://dev.bertos.org/wiki/ProgrammerInterface for help
rotary_beacon_PROGRAMMER_TYPE = none
rotary_beacon_PROGRAMMER_PORT = none

# Files included by the user.
rotary_beacon_USER_CSRC = \
	$(rotary_beacon_SRC_PATH)/stm32lib/stm32f10x_tim.c \
	$(rotary_beacon_SRC_PATH)/rotary_beacon.c \
	$(rotary_beacon_SRC_PATH)/main.c \
	#

# Files included by the user.
rotary_beacon_USER_PCSRC = \
	#

# Files included by the user.
rotary_beacon_USER_CPPASRC = \
	#

# Files included by the user.
rotary_beacon_USER_CXXSRC = \
	#

# Files included by the user.
rotary_beacon_USER_ASRC = \
	#

# Flags included by the user.
rotary_beacon_USER_LDFLAGS = \
	#

# Flags included by the user.
rotary_beacon_USER_CPPAFLAGS = \
	#

# Flags included by the user.
rotary_beacon_USER_CPPFLAGS = \
	-fno-strict-aliasing \
	-fwrapv \
	#
