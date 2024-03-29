;;*****************************************************************************
;;  FILENAME: compteur.asm
;;
;;  Version: 1.1, Updated on 2009/10/15 at 17:11:37
;;  Generated by PSoC Designer 5.0.1127.0
;;
;;  DESCRIPTION: TachTimer16 User Module software implementation file
;;
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress Semiconductor 2009. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"
include "memory.inc"
include "compteur.inc"

;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------
export  compteur_EnableInt
export _compteur_EnableInt
export  compteur_EnableTerminalInt
export _compteur_EnableTerminalInt
export  compteur_EnableCaptureInt
export _compteur_EnableCaptureInt


export  compteur_DisableInt
export _compteur_DisableInt
export  compteur_DisableTerminalInt
export _compteur_DisableTerminalInt
export  compteur_DisableCaptureInt
export _compteur_DisableCaptureInt

export  compteur_Start
export _compteur_Start
export  compteur_Stop
export _compteur_Stop
export  compteur_WritePeriod
export _compteur_WritePeriod
export  compteur_WriteCompareValue
export _compteur_WriteCompareValue
export  compteur_wReadCompareValue
export _compteur_wReadCompareValue
export  compteur_wReadTimer
export _compteur_wReadTimer
export  compteur_wReadTimerSaveCV
export _compteur_wReadTimerSaveCV


AREA bss (RAM,REL)

;-----------------------------------------------
;  Constant Definitions
;-----------------------------------------------


;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------


AREA UserModules (ROM, REL)

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_EnableInt
;
;  DESCRIPTION:
;     Enables this timer's interrupt by setting the interrupt enable mask bit
;     associated with this User Module. This function has no effect until and
;     unless the global interrupts are enabled (for example by using the
;     macro M8C_EnableGInt).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None.
;  RETURNS:      Nothing.
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_EnableInt:
_compteur_EnableInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   compteur_EnableInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_EnableTerminalInt
;
;  DESCRIPTION:
;     Enables only the terminal count interrupt or MSB interrupt.  This interrupt
;     fires once per cycle on terminal count.
;
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None.
;  RETURNS:      Nothing.
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_EnableTerminalInt:
_compteur_EnableTerminalInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   compteur_EnableTerminalInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_EnableCaptureInt
;
;  DESCRIPTION:
;     Enables only the capture interrupt or LSB interrupt.                           
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None.
;  RETURNS:      Nothing.
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_EnableCaptureInt:
_compteur_EnableCaptureInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   compteur_EnableCaptureInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_DisableInt
;
;  DESCRIPTION:
;     Disables this timer's interrupt by clearing the interrupt enable
;     mask bit associated with this User Module.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_DisableInt:
_compteur_DisableInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   compteur_DisableInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_DisableTerminalInt
;
;  DESCRIPTION:
;     Disables just the terminal (MSB) interrupt.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_DisableTerminalInt:
_compteur_DisableTerminalInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   compteur_DisableTerminalInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_DisableCaptureInt
;
;  DESCRIPTION:
;     Disables just the terminal (MSB) interrupt.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_DisableCaptureInt:
_compteur_DisableCaptureInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   compteur_DisableCaptureInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_Start
;
;  DESCRIPTION:
;     Sets the start bit in the Control register of this user module.  The
;     timer will begin counting on the next input clock.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_Start:
_compteur_Start:
   RAM_PROLOGUE RAM_USE_CLASS_1
   compteur_Start_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_Stop
;
;  DESCRIPTION:
;     Disables timer operation by clearing the start bit in the Control
;     register of the LSB block.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_Stop:
_compteur_Stop:
   RAM_PROLOGUE RAM_USE_CLASS_1
   compteur_Stop_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_WritePeriod
;
;  DESCRIPTION:
;     Write the 16-bit period value into the Period register (DR1). If the
;     Timer user module is stopped, then this value will also be latched
;     into the Count register (DR0).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: fastcall WORD wPeriodValue (LSB in A, MSB in X)
;  RETURNS:   Nothing
;  SIDE EFFECTS:
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_WritePeriod:
_compteur_WritePeriod:
   RAM_PROLOGUE RAM_USE_CLASS_1
   mov   reg[compteur_PERIOD_LSB_REG], A
   mov   A, X
   mov   reg[compteur_PERIOD_MSB_REG], A
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_WriteCompareValue
;
;  DESCRIPTION:
;     Writes compare value into the Compare register (DR2).
;
;     NOTE! The Timer user module must be STOPPED in order to write the
;           Compare register. (Call compteur_Stop to disable).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    fastcall WORD wCompareValue (LSB in A, MSB in X)
;  RETURNS:      Nothing
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_WriteCompareValue:
_compteur_WriteCompareValue:
   RAM_PROLOGUE RAM_USE_CLASS_1
   mov   reg[compteur_COMPARE_LSB_REG], A
   mov   A, X
   mov   reg[compteur_COMPARE_MSB_REG], A
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_wReadCompareValue
;
;  DESCRIPTION:
;     Reads the Compare registers.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      fastcall WORD wCompareValue (value of DR2 in the X & A registers)
;  SIDE EFFECTS: REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
 compteur_wReadCompareValue:
_compteur_wReadCompareValue:

   RAM_PROLOGUE RAM_USE_CLASS_1
   mov   A, reg[compteur_COMPARE_MSB_REG]
   mov   X, A
   mov   A, reg[compteur_COMPARE_LSB_REG]
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_wReadTimerSaveCV
;
;  DESCRIPTION:
;     Returns the value in the Count register (DR0), preserving the
;     value in the compare register (DR2).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: None
;  RETURNS:   fastcall WORD wCount (value of DR0 in the X & A registers)
;  SIDE EFFECTS:
;     1) May cause an interrupt, if interrupt on Compare is enabled.
;     2) If enabled, Global interrupts are momentarily disabled.
;     3) The user module is stopped momentarily while the compare value is
;        restored.  This may cause the Count register to miss one or more
;        counts depending on the input clock speed.
;     4) REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION:
;     1) Read and save the Compare register.
;     2) Read the Count register, causing its data to be latched into
;        the Compare register.
;     3) Read and save the Counter value, now in the Compare register,
;        to the buffer.
;     4) Disable global interrupts
;     5) Halt the timer
;     6) Restore the Compare register values
;     7) Start the Timer again
;     8) Restore global interrupt state
;
 compteur_wReadTimerSaveCV:
_compteur_wReadTimerSaveCV:


CpuFlags:      equ   0
wCount_MSB:    equ   1
wCount_LSB:    equ   2

   RAM_PROLOGUE RAM_USE_CLASS_2
   mov   X, SP                                   ; X <- stack frame pointer
   add   SP, 3                                   ; Reserve space for flags, count
   mov   A, reg[compteur_CONTROL_LSB_REG]        ; save the Control register
   push  A
   mov   A, reg[compteur_COMPARE_LSB_REG]        ; save the Compare register
   push  A
   mov   A, reg[compteur_COMPARE_MSB_REG]
   push  A
   mov   A, reg[compteur_COUNTER_LSB_REG]        ; synchronous copy DR2 <- DR0
                                                 ; This may cause an interrupt!
   mov   A, reg[compteur_COMPARE_MSB_REG]        ; Now grab DR2 (DR0) and save
   mov   [X+wCount_MSB], A
   mov   A, reg[compteur_COMPARE_LSB_REG]
   mov   [X+wCount_LSB], A
   mov   A, 0                                    ; Guess the global interrupt state
   tst   reg[CPU_SCR0], CPU_SCR0_GIE_MASK        ; Currently Disabled?
   jz    .SetupStatusFlag                        ;   Yes, guess was correct
   mov   A, FLAG_GLOBAL_IE                       ;    No, modify our guess
.SetupStatusFlag:                                ; and ...
   mov   [X+CpuFlags], A                         ;   StackFrame[0] <- Flag Reg image
   M8C_DisableGInt                               ; Disable interrupts globally
   compteur_Stop_M                               ; Disable (stop) the timer
   pop   A                                       ; Restore the Compare register
   mov   reg[compteur_COMPARE_LSB_REG], A
   pop   A
   mov   reg[compteur_COMPARE_MSB_REG], A
   pop   A                                       ; restore start state of the timer
   mov   reg[compteur_CONTROL_LSB_REG], A
   pop   A                                       ; Return result stored in stack frame
   pop   X
   RAM_EPILOGUE RAM_USE_CLASS_2
   reti                                          ; Flag Reg <- StackFrame[0]

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: compteur_wReadTimer
;
;  DESCRIPTION:
;     Performs a software capture of the Count register.  A synchronous
;     read of the Count register is performed.  The timer is NOT stopped.
;
;     WARNING - this will cause loss of data in the Compare register.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      fastcall WORD wCount, (value of DR0 in the X & A registers)
;  SIDE EFFECTS:
;     May cause an interrupt.
;     REGISTERS ARE VOLATILE: THE A AND X REGISTERS MAY BE MODIFIED!
;
;  THEORY of OPERATION:
;     1) Read the Count register - this causes the count value to be
;        latched into the Compare registers.
;     2) Read and return the Count register values from the Compare
;        registers into the return buffer.
;
 compteur_wReadTimer:
_compteur_wReadTimer:


   RAM_PROLOGUE RAM_USE_CLASS_1
   mov   A, reg[compteur_COUNTER_LSB_REG]        ; synchronous copy DR2 <- DR0
                                                 ; This may cause an interrupt!

   mov   A, reg[compteur_COMPARE_MSB_REG]        ; Return DR2 (actually DR0)
   mov   X, A
   mov   A, reg[compteur_COMPARE_LSB_REG]
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION

; End of File compteur.asm
