#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!
#
##############################################################################################
#
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#

##############################################################################################
# Start of default section
#

TRGT = mingw32-
CC   = $(TRGT)gcc
AS   = $(TRGT)gcc -x assembler-with-cpp
COV  = gcov

# List all default C defines here, like -D_DEBUG=1
DDEFS = -DSIMULATOR

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = -lws2_32

#
# End of default section
##############################################################################################

##############################################################################################
# Start of user section
#

# Define project name here
PROJECT = ch

# Define linker script file here
LDSCRIPT=

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# Imported source files
CHIBIOS = ../..
include ${CHIBIOS}/os/hal/hal.mk
include ${CHIBIOS}/os/hal/platforms/Win32/platform.mk
include ${CHIBIOS}/os/ports/GCC/SIMIA32/port.mk
include ${CHIBIOS}/os/kernel/kernel.mk
include ${CHIBIOS}/test/test.mk

# List C source files here
SRC  = ${PORTSRC} \
       ${KERNSRC} \
       ${TESTSRC} \
       ${HALSRC} \
       ${PLATFORMSRC} \
       ${CHIBIOS}/os/hal/platforms/Win32/console.c \
       main.c

# List ASM source files here
ASRC =

# List all user directories here
UINCDIR = $(PORTINC) $(KERNINC) $(TESTINC) $(HALINC) $(PLATFORMINC) \
          ${CHIBIOS}/os/various

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

# Define optimisation level here
OPT = -ggdb -O0 -fomit-frame-pointer -fprofile-arcs -ftest-coverage

#
# End of user defines
##############################################################################################


INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))
DEFS    = $(DDEFS) $(UDEFS)
ADEFS   = $(DADEFS) $(UADEFS)
OBJS    = $(ASRC:.s=.o) $(SRC:.c=.o)
LIBS    = $(DLIBS) $(ULIBS)

LDFLAGS = -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch -lgcov $(LIBDIR)
ASFLAGS = -Wa,-amhls=$(<:.s=.lst) $(ADEFS)
CPFLAGS = $(OPT) -Wall -Wextra -Wstrict-prototypes -fverbose-asm -Wa,-alms=$(<:.c=.lst) $(DEFS)

# Generate dependency information
CPFLAGS += -MD -MP -MF .dep/$(@F).d

#
# makefile rules
#

all: $(OBJS) $(PROJECT).exe

%o : %c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

%o : %s
	$(AS) -c $(ASFLAGS) $< -o $@

%exe: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

.PHONY: gcov
gcov:
	-mkdir gcov
	$(COV) -u $(subst /,\,$(KERNSRC))
	-mv -f *.gcov ./gcov

.PHONY: clean
clean:
	-rm -f $(OBJS)
	-rm -f $(PROJECT).exe
	-rm -f $(PROJECT).map
	-rm -f $(SRC:.c=.c.bak)
	-rm -f $(SRC:.c=.lst)
	-rm -f $(SRC:.c=.gcno)
	-rm -f $(SRC:.c=.gcda)
	-rm -f $(ASRC:.s=.s.bak)
	-rm -f $(ASRC:.s=.lst)
	-rm -fR .dep

#
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
