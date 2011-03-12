#
# User makefile.
# Edit this file to change compiler options and related stuff.
#

# Programmer interface configuration, see http://dev.bertos.org/wiki/ProgrammerInterface for help
Firmware_Test_PROGRAMMER_TYPE = none
Firmware_Test_PROGRAMMER_PORT = none

# Files included by the user.
Firmware_Test_USER_CSRC = \
	$(Firmware_Test_SRC_PATH)/main.c \
	$(Firmware_Test_SRC_PATH)/ads7828.c \
	#

# Files included by the user.
Firmware_Test_USER_PCSRC = \
	#

# Files included by the user.
Firmware_Test_USER_CPPASRC = \
	#

# Files included by the user.
Firmware_Test_USER_CXXSRC = \
	#

# Files included by the user.
Firmware_Test_USER_ASRC = \
	#

# Flags included by the user.
Firmware_Test_USER_LDFLAGS = \
	#

# Flags included by the user.
Firmware_Test_USER_CPPAFLAGS = \
	#

# Flags included by the user.
Firmware_Test_USER_CPPFLAGS = \
	-fno-strict-aliasing \
	-fwrapv \
	#

BUILDDIR = images
PROJECT = Firmware_Test
include jtag/flash.mk

