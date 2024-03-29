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
 * @addtogroup SIMIA32_CORE
 * @{
 */

#ifndef _CHCORE_H_
#define _CHCORE_H_

/**
 * Macro defining the a simulated architecture into x86.
 */
#define CH_ARCHITECTURE_SIMIA32

/**
 * Name of the implemented architecture.
 */
#define CH_ARCHITECTURE_NAME "Simulator"

/**
 * @brief   Name of the architecture variant (optional).
 */
#define CH_CORE_VARIANT_NAME "x86 (integer only)"

/**
 * 16 bytes stack alignment.
 */
typedef struct {
  uint8_t a[16];
} stkalign_t __attribute__((aligned(16)));

/**
 * Generic x86 register.
 */
typedef void *regx86;

/**
 * Interrupt saved context.
 * This structure represents the stack frame saved during a preemption-capable
 * interrupt handler.
 */
struct extctx {
};

/**
 * System saved context.
 * @note In this demo the floating point registers are not saved.
 */
struct intctx {
  regx86  ebx;
  regx86  edi;
  regx86  esi;
  regx86  ebp;
  regx86  eip;
};

/**
 * Platform dependent part of the @p Thread structure.
 * This structure usually contains just the saved stack pointer defined as a
 * pointer to a @p intctx structure.
 */
struct context {
  struct intctx volatile *esp;
};

#define APUSH(p, a) (p) -= sizeof(void *), *(void **)(p) = (void*)(a)

/**
 * Platform dependent part of the @p chThdInit() API.
 * This code usually setup the context switching frame represented by a
 * @p intctx structure.
 */
#define SETUP_CONTEXT(workspace, wsize, pf, arg) {                      \
  uint8_t *esp = (uint8_t *)workspace + wsize;                          \
  APUSH(esp, 0);                                                        \
  APUSH(esp, 0);                                                        \
  APUSH(esp, 0);                                                        \
  APUSH(esp, arg);                                                      \
  APUSH(esp, threadexit);                                               \
  esp -= sizeof(struct intctx);                                         \
  ((struct intctx *)esp)->eip = pf;                                     \
  ((struct intctx *)esp)->ebx = 0;                                      \
  ((struct intctx *)esp)->edi = 0;                                      \
  ((struct intctx *)esp)->esi = 0;                                      \
  ((struct intctx *)esp)->ebp = 0;                                      \
  tp->p_ctx.esp = (struct intctx *)esp;                                 \
}

/**
 * Stack size for the system idle thread.
 */
#ifndef IDLE_THREAD_STACK_SIZE
#define IDLE_THREAD_STACK_SIZE 256
#endif

/**
 * Per-thread stack overhead for interrupts servicing, it is used in the
 * calculation of the correct working area size.
 * It requires stack space because the simulated "interrupt handlers" can
 * invoke host library functions inside so it better have a lot of space.
 */
#ifndef INT_REQUIRED_STACK
#define INT_REQUIRED_STACK 16384
#endif

/**
 * Enforces a correct alignment for a stack area size value.
 */
#define STACK_ALIGN(n) ((((n) - 1) | (sizeof(stkalign_t) - 1)) + 1)

 /**
  * Computes the thread working area global size.
  */
#define THD_WA_SIZE(n) STACK_ALIGN(sizeof(Thread) +                     \
                                   sizeof(void *) * 4 +                 \
                                   sizeof(struct intctx) +              \
                                   sizeof(struct extctx) +              \
                                  (n) + (INT_REQUIRED_STACK))

/**
 * Macro used to allocate a thread working area aligned as both position and
 * size.
 */
#define WORKING_AREA(s, n) stkalign_t s[THD_WA_SIZE(n) / sizeof(stkalign_t)];

/**
 * IRQ prologue code, inserted at the start of all IRQ handlers enabled to
 * invoke system APIs.
 */
#define PORT_IRQ_PROLOGUE()

/**
 * IRQ epilogue code, inserted at the end of all IRQ handlers enabled to
 * invoke system APIs.
 */
#define PORT_IRQ_EPILOGUE()

/**
 * IRQ handler function declaration.
 */
#define PORT_IRQ_HANDLER(id) void id(void)

/**
 * Simulator initialization.
 */
#define port_init()

/**
 * Does nothing in this simulator.
 */
#define port_lock() asm volatile("nop")

/**
 * Does nothing in this simulator.
 */
#define port_unlock() asm volatile("nop")

/**
 * Does nothing in this simulator.
 */
#define port_lock_from_isr()

/**
 * Does nothing in this simulator.
 */
#define port_unlock_from_isr()

/**
 * Does nothing in this simulator.
 */
#define port_disable()

/**
 * Does nothing in this simulator.
 */
#define port_suspend()

/**
 * Does nothing in this simulator.
 */
#define port_enable()

/**
 * In the simulator this does a polling pass on the simulated interrupt
 * sources.
 */
#define port_wait_for_interrupt() ChkIntSources()

#ifdef __cplusplus
extern "C" {
#endif
  __attribute__((fastcall)) void port_switch(Thread *ntp, Thread *otp);
  __attribute__((fastcall)) void port_halt(void);
  void threadexit(void);
  void ChkIntSources(void);
#ifdef __cplusplus
}
#endif

#endif /* _CHCORE_H_ */

/** @} */
