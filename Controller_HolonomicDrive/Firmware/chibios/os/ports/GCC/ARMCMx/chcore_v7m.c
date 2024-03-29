/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    ARMCMx/chcore_v7m.c
 * @brief   ARMv7-M architecture port code.
 *
 * @addtogroup ARMCMx_V7M_CORE
 * @{
 */

#include "ch.h"

/**
 * @brief   Internal context stacking.
 */
#define PUSH_CONTEXT(sp) {                                                  \
  asm volatile ("push    {r4, r5, r6, r7, r8, r9, r10, r11, lr}");          \
}


/**
 * @brief   Internal context unstacking.
 */
#define POP_CONTEXT(sp) {                                                   \
  asm volatile ("pop     {r4, r5, r6, r7, r8, r9, r10, r11, pc}"            \
                :  : "r" (sp));                                             \
}

#if !CH_OPTIMIZE_SPEED
void _port_lock(void) {
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_KERNEL;
  asm volatile ("msr     BASEPRI, %0" : : "r" (tmp));
}

void _port_unlock(void) {
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_DISABLED;
  asm volatile ("msr     BASEPRI, %0" : : "r" (tmp));
}
#endif

/**
 * @brief   System Timer vector.
 * @details This interrupt is used as system tick.
 * @note    The timer must be initialized in the startup code.
 */
CH_IRQ_HANDLER(SysTickVector) {

  CH_IRQ_PROLOGUE();

  chSysLockFromIsr();
  chSysTimerHandlerI();
  chSysUnlockFromIsr();

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   SVC vector.
 * @details The SVC vector is used for exception mode re-entering after a
 *          context switch.
 */
void SVCallVector(void) {
  register struct extctx *ctxp;

  /* Discarding the current exception context and positioning the stack to
     point to the real one.*/
  asm volatile ("mrs     %0, PSP" : "=r" (ctxp) : );
  ctxp++;
  asm volatile ("msr     PSP, %0" :  : "r" (ctxp));
  port_unlock_from_isr();
}

/**
 * @brief   Reschedule verification and setup after an IRQ.
 */
void _port_irq_epilogue(void) {

  port_lock_from_isr();
  if ((SCB_ICSR & ICSR_RETTOBASE) && chSchIsRescRequiredExI()) {
    register struct extctx *ctxp;

    /* Adding an artificial exception return context, there is no need to
       populate it fully.*/
    asm volatile ("mrs     %0, PSP" : "=r" (ctxp) : );
    ctxp--;
    asm volatile ("msr     PSP, %0" :  : "r" (ctxp));
    ctxp->pc = _port_switch_from_isr;
    ctxp->xpsr = (regarm_t)0x01000000;
    /* Note, returning without unlocking is intentional, this is done in
       order to keep the rest of the context switching atomic.*/
    return;
  }
  /* ISR exit without context switching.*/
  port_unlock_from_isr();
}

/**
 * @brief   Post-IRQ switch code.
 * @details Exception handlers return here for context switching.
 */
#if !defined(__DOXYGEN__)
__attribute__((naked))
#endif
void _port_switch_from_isr(void) {

  chSchDoRescheduleI();
  asm volatile ("svc     #0");
}

/**
 * @brief   Performs a context switch between two threads.
 * @details This is the most critical code in any port, this function
 *          is responsible for the context switch between 2 threads.
 * @note    The implementation of this code affects <b>directly</b> the context
 *          switch performance so optimize here as much as you can.
 *
 * @param[in] ntp       the thread to be switched in
 * @param[in] otp       the thread to be switched out
 */
#if !defined(__DOXYGEN__)
__attribute__((naked))
#endif
void port_switch(Thread *ntp, Thread *otp) {
  register struct intctx *r13 asm ("r13");

  /* Stack overflow check, if enabled.*/
#if CH_DBG_ENABLE_STACK_CHECK
  if ((void *)(r13 - 1) < (void *)(otp + 1))
    asm volatile ("movs    r0, #0                               \n\t"
                  "b       chDbgPanic");
#endif /* CH_DBG_ENABLE_STACK_CHECK */

  PUSH_CONTEXT(r13);

  otp->p_ctx.r13 = r13;
  r13 = ntp->p_ctx.r13;

  POP_CONTEXT(r13);
}

/**
 * @brief   Start a thread by invoking its work function.
 * @details If the work function returns @p chThdExit() is automatically
 *          invoked.
 */
void _port_thread_start(void) {

  port_unlock();
  asm volatile ("mov     r0, r5                                 \n\t"
                "blx     r4                                     \n\t"
                "bl      chThdExit");
}

/** @} */
