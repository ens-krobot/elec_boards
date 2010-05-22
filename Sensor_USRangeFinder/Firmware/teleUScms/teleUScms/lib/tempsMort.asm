;;*****************************************************************************
;;*****************************************************************************
;;  FILENAME: tempsMort.asm
;;   Version: 2.6, Updated on 2009/10/15 at 17:11:37
;;  Generated by PSoC Designer 5.0.1127.0
;;
;;  DESCRIPTION: Timer8 User Module software implementation file
;;
;;  NOTE: User Module APIs conform to the fastcall16 convention for marshalling
;;        arguments and observe the associated "Registers are volatile" policy.
;;        This means it is the caller's responsibility to preserve any values
;;        in the X and A registers that are still needed after the API functions
;;        returns. For Large Memory Model devices it is also the caller's 
;;        responsibility to perserve any value in the CUR_PP, IDX_PP, MVR_PP and 
;;        MVW_PP registers. Even though some of these registers may not be modified
;;        now, there is no guarantee that will remain the case in future releases.
;;-----------------------------------------------------------------------------
;;  Copyright (c) Cypress Semiconductor 2009. All Rights Reserved.
;;*****************************************************************************
;;*****************************************************************************

include "m8c.inc"
include "memory.inc"
include "tempsMort.inc"

;-----------------------------------------------
;  Global Symbols
;-----------------------------------------------
export  tempsMort_EnableInt
export _tempsMort_EnableInt
export  tempsMort_DisableInt
export _tempsMort_DisableInt
export  tempsMort_Start
export _tempsMort_Start
export  tempsMort_Stop
export _tempsMort_Stop
export  tempsMort_WritePeriod
export _tempsMort_WritePeriod
export  tempsMort_WriteCompareValue
export _tempsMort_WriteCompareValue
export  tempsMort_bReadCompareValue
export _tempsMort_bReadCompareValue
export  tempsMort_bReadTimer
export _tempsMort_bReadTimer
export  tempsMort_bReadTimerSaveCV
export _tempsMort_bReadTimerSaveCV

; The following functions are deprecated and subject to omission in future releases
;
export  btempsMort_ReadCompareValue  ; deprecated
export _btempsMort_ReadCompareValue  ; deprecated
export  btempsMort_ReadTimer         ; deprecated
export _btempsMort_ReadTimer         ; deprecated
export  btempsMort_ReadTimerSaveCV   ; deprecated
export _btempsMort_ReadTimerSaveCV   ; deprecated

export  btempsMort_ReadCounter       ; obsolete
export _btempsMort_ReadCounter       ; obsolete
export  btempsMort_CaptureCounter    ; obsolete
export _btempsMort_CaptureCounter    ; obsolete


AREA teleuscms_RAM (RAM,REL)

;-----------------------------------------------
;  Constant Definitions
;-----------------------------------------------


;-----------------------------------------------
; Variable Allocation
;-----------------------------------------------


AREA UserModules (ROM, REL)

.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: tempsMort_EnableInt
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
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 tempsMort_EnableInt:
_tempsMort_EnableInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   tempsMort_EnableInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: tempsMort_DisableInt
;
;  DESCRIPTION:
;     Disables this timer's interrupt by clearing the interrupt enable
;     mask bit associated with this User Module.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 tempsMort_DisableInt:
_tempsMort_DisableInt:
   RAM_PROLOGUE RAM_USE_CLASS_1
   tempsMort_DisableInt_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: tempsMort_Start
;
;  DESCRIPTION:
;     Sets the start bit in the Control register of this user module.  The
;     timer will begin counting on the next input clock.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 tempsMort_Start:
_tempsMort_Start:
   RAM_PROLOGUE RAM_USE_CLASS_1
   tempsMort_Start_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: tempsMort_Stop
;
;  DESCRIPTION:
;     Disables timer operation by clearing the start bit in the Control
;     register.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      Nothing
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 tempsMort_Stop:
_tempsMort_Stop:
   RAM_PROLOGUE RAM_USE_CLASS_1
   tempsMort_Stop_M
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: tempsMort_WritePeriod
;
;  DESCRIPTION:
;     Write the 8-bit period value into the Period register (DR1). If the
;     Timer user module is stopped, then this value will also be latched
;     into the Count register (DR0).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: fastcall16 BYTE bPeriodValue (passed in A)
;  RETURNS:   Nothing
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 tempsMort_WritePeriod:
_tempsMort_WritePeriod:
   RAM_PROLOGUE RAM_USE_CLASS_1
   mov   reg[tempsMort_PERIOD_REG], A
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: tempsMort_WriteCompareValue
;
;  DESCRIPTION:
;     Writes compare value into the Compare register (DR2).
;
;     NOTE! The Timer user module must be STOPPED in order to write the
;           Compare register. (Call tempsMort_Stop to disable).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    fastcall16 BYTE bCompareValue (passed in A)
;  RETURNS:      Nothing
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 tempsMort_WriteCompareValue:
_tempsMort_WriteCompareValue:
   RAM_PROLOGUE RAM_USE_CLASS_1
   mov   reg[tempsMort_COMPARE_REG], A
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: tempsMort_bReadCompareValue
;
;  DESCRIPTION:
;     Reads the Compare register.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      fastcall16 BYTE bCompareValue (value of DR2 in the A register)
;  SIDE EFFECTS: 
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
 tempsMort_bReadCompareValue:
_tempsMort_bReadCompareValue:
 btempsMort_ReadCompareValue:                    ; this name deprecated
_btempsMort_ReadCompareValue:                    ; this name deprecated
   RAM_PROLOGUE RAM_USE_CLASS_1
   mov   A, reg[tempsMort_COMPARE_REG]
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: tempsMort_bReadTimerSaveCV
;
;  DESCRIPTION:
;     Returns the value in the Count register (DR0), preserving the
;     value in the compare register (DR2).
;-----------------------------------------------------------------------------
;
;  ARGUMENTS: None
;  RETURNS:   fastcall16 BYTE bCount (value of DR0 in the A register)
;  SIDE EFFECTS:
;     1) May cause an interrupt, if interrupt on Compare is enabled.
;     2) If enabled, Global interrupts are momentarily disabled.
;     3) The user module is stopped momentarily while the compare value is
;        restored.  This may cause the Count register to miss one or more
;        counts depending on the input clock speed.
;     4) The A and X registers may be modified by this or future implementations
;        of this function.  The same is true for all RAM page pointer registers in
;        the Large Memory Model.  When necessary, it is the calling function's
;        responsibility to perserve their values across calls to fastcall16 
;        functions.
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
 tempsMort_bReadTimerSaveCV:
_tempsMort_bReadTimerSaveCV:
 btempsMort_ReadTimerSaveCV:                     ; this name deprecated
_btempsMort_ReadTimerSaveCV:                     ; this name deprecated
 btempsMort_ReadCounter:                         ; this name deprecated
_btempsMort_ReadCounter:                         ; this name deprecated

CpuFlags:      equ   0
bCount:        equ   1

   RAM_PROLOGUE RAM_USE_CLASS_2
   mov   X, SP                                   ; X <- stack frame pointer
   add   SP, 2                                   ; Reserve space for flags, count
   mov   A, reg[tempsMort_CONTROL_REG]           ; save the Control register
   push  A
   mov   A, reg[tempsMort_COMPARE_REG]           ; save the Compare register
   push  A
   mov   A, reg[tempsMort_COUNTER_REG]           ; synchronous copy DR2 <- DR0
                                                 ; This may cause an interrupt!
   mov   A, reg[tempsMort_COMPARE_REG]           ; Now grab DR2 (DR0) and save
   mov   [X+bCount], A
   mov   A, 0                                    ; Guess the global interrupt state
   tst   reg[CPU_F], FLAG_GLOBAL_IE              ; Currently Disabled?
   jz    .SetupStatusFlag                        ;   Yes, guess was correct
   mov   A, FLAG_GLOBAL_IE                       ;    No, modify our guess
.SetupStatusFlag:                                ; and ...
   mov   [X+CpuFlags], A                         ;   StackFrame[0] <- Flag Reg image
   M8C_DisableGInt                               ; Disable interrupts globally
   tempsMort_Stop_M                              ; Stop the timer
   pop   A                                       ; Restore the Compare register
   mov   reg[tempsMort_COMPARE_REG], A
   pop   A                                       ; restore start state of the timer
   mov   reg[tempsMort_CONTROL_REG], A
   pop   A                                       ; Return result stored in stack frame
   RAM_EPILOGUE RAM_USE_CLASS_2
   reti                                          ; Flag Reg <- StackFrame[0]

.ENDSECTION


.SECTION
;-----------------------------------------------------------------------------
;  FUNCTION NAME: tempsMort_bReadTimer
;
;  DESCRIPTION:
;     Performs a software capture of the Count register.  A synchronous
;     read of the Count register is performed.  The timer is NOT stopped.
;
;     WARNING - this will cause loss of data in the Compare register.
;-----------------------------------------------------------------------------
;
;  ARGUMENTS:    None
;  RETURNS:      fastcall16 BYTE bCount, (value of DR0 in the A register)
;  SIDE EFFECTS:
;    May cause an interrupt.
;
;    The A and X registers may be modified by this or future implementations
;    of this function.  The same is true for all RAM page pointer registers in
;    the Large Memory Model.  When necessary, it is the calling function's
;    responsibility to perserve their values across calls to fastcall16 
;    functions.
;
;  THEORY of OPERATION:
;     1) Read the Count register - this causes the count value to be
;        latched into the Compare register.
;     2) Read and return the Count register values from the Compare
;        registers into the return buffer.
;
 tempsMort_bReadTimer:
_tempsMort_bReadTimer:
 btempsMort_ReadTimer:                           ; this name deprecated
_btempsMort_ReadTimer:                           ; this name deprecated
 btempsMort_CaptureCounter:                      ; this name deprecated
_btempsMort_CaptureCounter:                      ; this name deprecated

   RAM_PROLOGUE RAM_USE_CLASS_1
   mov   A, reg[tempsMort_COUNTER_REG]           ; synchronous copy DR2 <- DR0
                                                 ; This may cause an interrupt!
   mov   A, reg[tempsMort_COMPARE_REG]           ; Return DR2 (actually DR0)
   RAM_EPILOGUE RAM_USE_CLASS_1
   ret

.ENDSECTION

; End of File tempsMort.asm
