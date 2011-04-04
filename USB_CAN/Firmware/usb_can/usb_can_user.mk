#
# User makefile.
# Edit this file to change compiler options and related stuff.
#

# Programmer interface configuration, see http://dev.bertos.org/wiki/ProgrammerInterface for help
usb_can_PROGRAMMER_TYPE = none
usb_can_PROGRAMMER_PORT = none

# Files included by the user.
usb_can_USER_CSRC = \
	$(usb_can_SRC_PATH)/battery_monitoring/ads7828.c \
	$(usb_can_SRC_PATH)/battery_monitoring/battery_monitoring.c \
	$(usb_can_SRC_PATH)/usb_can/usb_can.c \
	$(usb_can_SRC_PATH)/main.c \
	#

# Files included by the user.
usb_can_USER_PCSRC = \
	#

# Files included by the user.
usb_can_USER_CPPASRC = \
	#

# Files included by the user.
usb_can_USER_CXXSRC = \
	#

# Files included by the user.
usb_can_USER_ASRC = \
	#

# Flags included by the user.
usb_can_USER_LDFLAGS = \
	#

# Flags included by the user.
usb_can_USER_CPPAFLAGS = \
	#

# Flags included by the user.
usb_can_USER_CPPFLAGS = \
	-fno-strict-aliasing \
	-fwrapv \
	#
