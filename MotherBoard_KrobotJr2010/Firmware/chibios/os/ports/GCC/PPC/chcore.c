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
 * @file    PPC/chcore.c
 * @brief   PowerPC architecture port code.
 *
 * @addtogroup PPC_CORE
 * @{
 */

#include "ch.h"

/**
 * @brief   Halts the system.
 * @details This function is invoked by the operating system when an
 *          unrecoverable error is detected (as example because a programming
 *          error in the application code that triggers an assertion while
 *          in debug mode).
 */
void port_halt(void) {

  port_disable();
  while (TRUE) {
  }
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
void port_switch(Thread *ntp, Thread *otp) {

  (void)otp;
  (void)ntp;

  asm ("subi        %sp, %sp, 80");     /* Size of the intctx structure.    */
  asm ("mflr        %r0");
  asm ("stw         %r0, 84(%sp)");     /* LR into the caller frame.        */
  asm ("mfcr        %r0");
  asm ("stw         %r0, 0(%sp)");      /* CR.                              */
  asm ("stmw        %r14, 4(%sp)");     /* GPR14...GPR31.                   */
  
  asm ("stw         %sp, 12(%r4)");     /* Store swapped-out stack.         */
  asm ("lwz         %sp, 12(%r3)");     /* Load swapped-in stack.           */
  
  asm ("lmw         %r14, 4(%sp)");     /* GPR14...GPR31.                   */
  asm ("lwz         %r0, 0(%sp)");      /* CR.                              */
  asm ("mtcr        %r0");
  asm ("lwz         %r0, 84(%sp)");     /* LR from the caller frame.        */
  asm ("mtlr        %r0");
  asm ("addi        %sp, %sp, 80");     /* Size of the intctx structure.    */
}

/**
 * @brief   Start a thread by invoking its work function.
 * @details If the work function returns @p chThdExit() is automatically
 *          invoked.
 */
void _port_thread_start(void) {
  asm ("wrteei      1");
  asm ("mr          %r3, %r31");        /* Thread parameter.                */
  asm ("mtctr       %r30");
  asm ("bctrl");                        /* Invoke thread function.          */
  asm ("bl          chThdExit");        /* Thread termination on exit.      */
}

/** @} */
