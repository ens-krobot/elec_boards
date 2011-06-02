#
# User makefile.
# Edit this file to change compiler options and related stuff.
#

# Programmer interface configuration, see http://dev.bertos.org/wiki/ProgrammerInterface for help
test_ax12_PROGRAMMER_TYPE = none
test_ax12_PROGRAMMER_PORT = none

# Files included by the user.
test_ax12_USER_CSRC = \
	$(test_ax12_SRC_PATH)/ax12/serial.c \
	$(test_ax12_SRC_PATH)/ax12/ax12.c \
	$(test_ax12_SRC_PATH)/main.c \
	#

# Files included by the user.
test_ax12_USER_PCSRC = \
	#

# Files included by the user.
test_ax12_USER_CPPASRC = \
	#

# Files included by the user.
test_ax12_USER_CXXSRC = \
	#

# Files included by the user.
test_ax12_USER_ASRC = \
	#

# Flags included by the user.
test_ax12_USER_LDFLAGS = \
	#

# Flags included by the user.
test_ax12_USER_CPPAFLAGS = \
	#

# Flags included by the user.
test_ax12_USER_CPPFLAGS = \
	-fno-strict-aliasing \
	-fwrapv \
	#
