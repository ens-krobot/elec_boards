# MCC18 has to be installed on Wine for this to work

PIC=18F4550		# PIC model
ARGS=-w2		# Compiler flags

WINE=/usr/bin/wine
MCCDIR=$(HOME)/.wine/drive_c/MCC18/
MCC18=LC_ALL=C $(WINE) $(MCCDIR)bin/mcc18
# LC_ALL=C to avoid a possible issue with __DATE__ constant
LINKER=$(WINE) $(MCCDIR)bin/mplink
LIB_PATH=$(MCCDIR)lib
LIB_PATHW=C:/MCC18/lib
INCLUDE_PATH=$(MCCDIR)h
INCLUDE_PATHW=C:/MCC18/h

include .deps/*
include .conf/*

# Default: make the hex file
all: clean USB_Dev.hex

# Remove deps folder
clean-deps:
	@echo "Cleaning dependency folder..."
	@rm -rf .deps
	@mkdir .deps

# Removes all compiled files
clean: clean-deps
	@echo "Cleaning compiler intermediate files..."
	@rm -f *.err
	@cd output && rm -f *.o	
	@echo "Cleaning linker output and hex files..."
	@cd output && rm -f *.hex *.cod *.cof *.map

%.o: %.c
	@echo "Compiling $@..."
	@echo "1. Checking Dependencies for $@"
	@cpp -I"." -I"./Include" -nostdinc -isystem $(INCLUDE_PATH) -D "__18CXX" -D "__$(PIC)" -M -MP $< -MF .deps/$*.deps
	@echo "2. Launching Compiler..."
	@$(MCC18) -p=$(PIC) -I"./Include" -I"$(INCLUDE_PATHW)" $(ARGS) -fo output/$@ $<
	#sdcc -c -V -mpic16 -p$(PIC) -D"__18CXX" -D"__$(PIC)" -D"SDCC_pic16" -D"auto=" -D"rom=" -I"." -I"./Include" -I"$(INCLUDE_PATH)" -o "output/" $<
	@echo "Done with $@."

USB_Dev.hex: update_targets $(ALL_TARGETS) #$(INCLUDE_FILES)
	@echo "Linking..."
	@cd output && $(LINKER) /l"$(LIB_PATHW)" "../rm18f4550.lkr" `find . -name "*.o" -printf " %f"` /m"USB_Dev.map" /o"USB_Dev.cof"

#USB_Dev.hex: $(ALL_TARGETS)
#	@echo "Linking..."
#	cd output && sdcc -mpic16 -p$(PIC) -L"$(LIB_PATH)" -Wl-s"rm18f4550.lkr" $+ -o $@

.deps/*: 
	@mkdir -p .deps

.conf/*: 
	@mkdir -p .conf

update_targets: clean-deps
	@echo "Generating Target Names..."
	@echo -n 'ALL_TARGETS=' > .conf/00all_targets
	@find . -name Bootloader -prune -or -name "*.c" -printf "%f " | awk '{gsub(/\.c/,".o");print}' >> .conf/00all_targets
	@echo >> .conf/00all_targets
	@echo "Generating Include File Names..."
	@echo -n 'INCLUDE_FILES=' > .conf/01includes
	@find . -name Bootloader -prune -or -name "*.h" -printf " %p" >> .conf/01includes
	@echo >> .conf/01includes

.PHONY: all clean-deps clean

