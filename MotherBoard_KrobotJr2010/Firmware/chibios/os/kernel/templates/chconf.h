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
 * @file    templates/chconf.h
 * @brief   Configuration file template.
 * @details A copy of this file must be placed in each project directory, it
 *          contains the application specific kernel settings.
 *
 * @addtogroup config
 * @details Kernel related settings and hooks.
 * @{
 */

#ifndef _CHCONF_H_
#define _CHCONF_H_

/*===========================================================================*/
/* Kernel parameters.                                                        */
/*===========================================================================*/

/**
 * @brief   System tick frequency.
 * @details Frequency of the system timer that drives the system ticks. This
 *          setting also defines the system tick time unit.
 */
#if !defined(CH_FREQUENCY) || defined(__DOXYGEN__)
#define CH_FREQUENCY                    1000
#endif

/**
 * @brief   Round robin interval.
 * @details This constant is the number of system ticks allowed for the
 *          threads before preemption occurs. Setting this value to zero
 *          disables the preemption for threads with equal priority and the
 *          round robin becomes cooperative. Note that higher priority
 *          threads can still preempt, the kernel is always preemptive.
 *
 * @note    Disabling the round robin preemption makes the kernel more compact
 *          and generally faster.
 */
#if !defined(CH_TIME_QUANTUM) || defined(__DOXYGEN__)
#define CH_TIME_QUANTUM                 20
#endif

/**
 * @brief   Nested locks.
 * @details If enabled then the use of nested @p chSysLock() / @p chSysUnlock()
 *          operations is allowed.<br>
 *          For performance and code size reasons the recommended setting
 *          is to leave this option disabled.<br>
 *          You may use this option if you need to merge ChibiOS/RT with
 *          external libraries that require nested lock/unlock operations.
 *
 * @note T  he default is @p FALSE.
 */
#if !defined(CH_USE_NESTED_LOCKS) || defined(__DOXYGEN__)
#define CH_USE_NESTED_LOCKS             FALSE
#endif

/**
 * @brief   Managed RAM size.
 * @details Size of the RAM area to be managed by the OS. If set to zero
 *          then the whole available RAM is used. The core memory is made
 *          available to the heap allocator and/or can be used directly through
 *          the simplified core memory allocator.
 *
 * @note    In order to let the OS manage the whole RAM the linker script must
 *          provide the @p __heap_base__ and @p __heap_end__ symbols.
 * @note    Requires @p CH_USE_COREMEM.
 */
#if !defined(CH_MEMCORE_SIZE) || defined(__DOXYGEN__)
#define CH_MEMCORE_SIZE                 0
#endif

/*===========================================================================*/
/* Performance options.                                                      */
/*===========================================================================*/

/**
 * @brief   OS optimization.
 * @details If enabled then time efficient rather than space efficient code
 *          is used when two possible implementations exist.
 *
 * @note    This is not related to the compiler optimization options.
 * @note    The default is @p TRUE.
 */
#if !defined(CH_OPTIMIZE_SPEED) || defined(__DOXYGEN__)
#define CH_OPTIMIZE_SPEED               TRUE
#endif

/**
 * @brief   Exotic optimization.
 * @details If defined then a CPU register is used as storage for the global
 *          @p currp variable. Caching this variable in a register greatly
 *          improves both space and time OS efficiency. A side effect is that
 *          one less register has to be saved during the context switch
 *          resulting in lower RAM usage and faster context switch.
 *
 * @note    This option is only usable with the GCC compiler and is only useful
 *          on processors with many registers like ARM cores.
 * @note    If this option is enabled then ALL the libraries linked to the
 *          ChibiOS/RT code <b>must</b> be recompiled with the GCC option @p
 *          -ffixed-@<reg@>.
 * @note    This option must be enabled in the Makefile, it is listed here for
 *          documentation only.
 */
#if defined(__DOXYGEN__)
#define CH_CURRP_REGISTER_CACHE         "reg"
#endif

/*===========================================================================*/
/* Subsystem options.                                                        */
/*===========================================================================*/

/**
 * @brief   Threads registry APIs.
 * @details If enabled then the registry APIs are included in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_USE_REGISTRY) || defined(__DOXYGEN__)
#define CH_USE_REGISTRY                 TRUE
#endif

/**
 * @brief   Threads synchronization APIs.
 * @details If enabled then the @p chThdWait() function is included in
 *          the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_USE_WAITEXIT) || defined(__DOXYGEN__)
#define CH_USE_WAITEXIT                 TRUE
#endif

/**
 * @brief   Semaphores APIs.
 * @details If enabled then the Semaphores APIs are included in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_USE_SEMAPHORES) || defined(__DOXYGEN__)
#define CH_USE_SEMAPHORES               TRUE
#endif

/**
 * @brief   Semaphores queuing mode.
 * @details If enabled then the threads are enqueued on semaphores by
 *          priority rather than in FIFO order.
 *
 * @note    The default is @p FALSE. Enable this if you have special requirements.
 * @note    Requires @p CH_USE_SEMAPHORES.
 */
#if !defined(CH_USE_SEMAPHORES_PRIORITY) || defined(__DOXYGEN__)
#define CH_USE_SEMAPHORES_PRIORITY      FALSE
#endif

/**
 * @brief   Atomic semaphore API.
 * @details If enabled then the semaphores the @p chSemSignalWait() API
 *          is included in the kernel.
 *
 * @note    The default is @p TRUE.
 * @note    Requires @p CH_USE_SEMAPHORES.
 */
#if !defined(CH_USE_SEMSW) || defined(__DOXYGEN__)
#define CH_USE_SEMSW                    TRUE
#endif

/**
 * @brief   Mutexes APIs.
 * @details If enabled then the mutexes APIs are included in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_USE_MUTEXES) || defined(__DOXYGEN__)
#define CH_USE_MUTEXES                  TRUE
#endif

/**
 * @brief   Conditional Variables APIs.
 * @details If enabled then the conditional variables APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 * @note    Requires @p CH_USE_MUTEXES.
 */
#if !defined(CH_USE_CONDVARS) || defined(__DOXYGEN__)
#define CH_USE_CONDVARS                 TRUE
#endif

/**
 * @brief   Conditional Variables APIs with timeout.
 * @details If enabled then the conditional variables APIs with timeout
 *          specification are included in the kernel.
 *
 * @note    The default is @p TRUE.
 * @note    Requires @p CH_USE_CONDVARS.
 */
#if !defined(CH_USE_CONDVARS_TIMEOUT) || defined(__DOXYGEN__)
#define CH_USE_CONDVARS_TIMEOUT         TRUE
#endif

/**
 * @brief   Events Flags APIs.
 * @details If enabled then the event flags APIs are included in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_USE_EVENTS) || defined(__DOXYGEN__)
#define CH_USE_EVENTS                   TRUE
#endif

/**
 * @brief   Events Flags APIs with timeout.
 * @details If enabled then the events APIs with timeout specification
 *          are included in the kernel.
 *
 * @note    The default is @p TRUE.
 * @note    Requires @p CH_USE_EVENTS.
 */
#if !defined(CH_USE_EVENTS_TIMEOUT) || defined(__DOXYGEN__)
#define CH_USE_EVENTS_TIMEOUT           TRUE
#endif

/**
 * @brief   Synchronous Messages APIs.
 * @details If enabled then the synchronous messages APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_USE_MESSAGES) || defined(__DOXYGEN__)
#define CH_USE_MESSAGES                 TRUE
#endif

/**
 * @brief   Synchronous Messages queuing mode.
 * @details If enabled then messages are served by priority rather than in
 *          FIFO order.
 *
 * @note    The default is @p FALSE. Enable this if you have special requirements.
 * @note    Requires @p CH_USE_MESSAGES.
 */
#if !defined(CH_USE_MESSAGES_PRIORITY) || defined(__DOXYGEN__)
#define CH_USE_MESSAGES_PRIORITY        FALSE
#endif

/**
 * @brief   Mailboxes APIs.
 * @details If enabled then the asynchronous messages (mailboxes) APIs are
 *          included in the kernel.
 *
 * @note    The default is @p TRUE.
 * @note    Requires @p CH_USE_SEMAPHORES.
 */
#if !defined(CH_USE_MAILBOXES) || defined(__DOXYGEN__)
#define CH_USE_MAILBOXES                TRUE
#endif

/**
 * @brief   I/O Queues APIs.
 * @details If enabled then the I/O queues APIs are included in the kernel.
 *
 * @note    The default is @p TRUE.
 * @note    Requires @p CH_USE_SEMAPHORES.
 */
#if !defined(CH_USE_QUEUES) || defined(__DOXYGEN__)
#define CH_USE_QUEUES                   TRUE
#endif

/**
 * @brief   Core Memory Manager APIs.
 * @details If enabled then the core memory manager APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_USE_MEMCORE) || defined(__DOXYGEN__)
#define CH_USE_MEMCORE                  TRUE
#endif

/**
 * @brief   Heap Allocator APIs.
 * @details If enabled then the memory heap allocator APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 * @note    Requires @p CH_USE_COREMEM and either @p CH_USE_MUTEXES or
 *          @p CH_USE_SEMAPHORES.
 * @note    Mutexes are recommended.
 */
#if !defined(CH_USE_HEAP) || defined(__DOXYGEN__)
#define CH_USE_HEAP                     TRUE
#endif

/**
 * @brief   C-runtime allocator.
 * @details If enabled the the heap allocator APIs just wrap the C-runtime
 *          @p malloc() and @p free() functions.
 *
 * @note    The default is @p FALSE.
 * @note    Requires @p CH_USE_HEAP.
 * @note    The C-runtime may or may not require @p CH_USE_COREMEM, see the
 *          appropriate documentation.
 */
#if !defined(CH_USE_MALLOC_HEAP) || defined(__DOXYGEN__)
#define CH_USE_MALLOC_HEAP              FALSE
#endif

/**
 * @brief   Memory Pools Allocator APIs.
 * @details If enabled then the memory pools allocator APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 */
#if !defined(CH_USE_MEMPOOLS) || defined(__DOXYGEN__)
#define CH_USE_MEMPOOLS                 TRUE
#endif

/**
 * @brief   Dynamic Threads APIs.
 * @details If enabled then the dynamic threads creation APIs are included
 *          in the kernel.
 *
 * @note    The default is @p TRUE.
 * @note    Requires @p CH_USE_WAITEXIT.
 * @note    Requires @p CH_USE_HEAP and/or @p CH_USE_MEMPOOLS.
 */
#if !defined(CH_USE_DYNAMIC) || defined(__DOXYGEN__)
#define CH_USE_DYNAMIC                  TRUE
#endif

/*===========================================================================*/
/* Debug options.                                                            */
/*===========================================================================*/

/**
 * @brief   Debug option, parameters checks.
 * @details If enabled then the checks on the API functions input
 *          parameters are activated.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(CH_DBG_ENABLE_CHECKS) || defined(__DOXYGEN__)
#define CH_DBG_ENABLE_CHECKS            FALSE
#endif

/**
 * @brief   Debug option, consistency checks.
 * @details If enabled then all the assertions in the kernel code are
 *          activated. This includes consistency checks inside the kernel,
 *          runtime anomalies and port-defined checks.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(CH_DBG_ENABLE_ASSERTS) || defined(__DOXYGEN__)
#define CH_DBG_ENABLE_ASSERTS           FALSE
#endif

/**
 * @brief   Debug option, trace buffer.
 * @details If enabled then the context switch circular trace buffer is
 *          activated.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(CH_DBG_ENABLE_TRACE) || defined(__DOXYGEN__)
#define CH_DBG_ENABLE_TRACE             FALSE
#endif

/**
 * @brief   Debug option, stack checks.
 * @details If enabled then a runtime stack check is performed.
 *
 * @note    The default is @p FALSE.
 * @note    The stack check is performed in a architecture/port dependent way.
 *          It may not be implemented or some ports.
 * @note    The default failure mode is to halt the system with the global
 *          @p panic_msg variable set to @p NULL.
 */
#if !defined(CH_DBG_ENABLE_STACK_CHECK) || defined(__DOXYGEN__)
#define CH_DBG_ENABLE_STACK_CHECK       FALSE
#endif

/**
 * @brief   Debug option, stacks initialization.
 * @details If enabled then the threads working area is filled with a byte
 *          value when a thread is created. This can be useful for the
 *          runtime measurement of the used stack.
 *
 * @note    The default is @p FALSE.
 */
#if !defined(CH_DBG_FILL_THREADS) || defined(__DOXYGEN__)
#define CH_DBG_FILL_THREADS             FALSE
#endif

/**
 * @brief   Debug option, threads profiling.
 * @details If enabled then a field is added to the @p Thread structure that
 *          counts the system ticks occurred while executing the thread.
 *
 * @note    The default is @p TRUE.
 * @note    This debug option is defaulted to TRUE because it is required by
 *          some test cases into the test suite.
 */
#if !defined(CH_DBG_THREADS_PROFILING) || defined(__DOXYGEN__)
#define CH_DBG_THREADS_PROFILING        TRUE
#endif

/*===========================================================================*/
/* Kernel hooks.                                                             */
/*===========================================================================*/

/**
 * @brief   Threads descriptor structure hook.
 * @details User fields added to the end of the @p Thread structure.
 */
#if !defined(THREAD_EXT_FIELDS) || defined(__DOXYGEN__)
#define THREAD_EXT_FIELDS                                               \
struct {                                                                \
  /* Add threads custom fields here.*/                                  \
};
#endif

/**
 * @brief   Threads initialization hook.
 * @details User initialization code added to the @p chThdInit() API.
 *
 * @note    It is invoked from within @p chThdInit() and implicitily from all
 *          the threads creation APIs.
 */
#if !defined(THREAD_EXT_INIT) || defined(__DOXYGEN__)
#define THREAD_EXT_INIT(tp) {                                           \
  /* Add threads initialization code here.*/                            \
}
#endif

/**
 * @brief   Threads finalization hook.
 * @details User finalization code added to the @p chThdExit() API.
 *
 * @note    It is inserted into lock zone.
 * @note    It is also invoked when the threads simply return in order to
 *          terminate.
 */
#if !defined(THREAD_EXT_EXIT) || defined(__DOXYGEN__)
#define THREAD_EXT_EXIT(tp) {                                           \
  /* Add threads finalization code here.*/                              \
}
#endif

/**
 * @brief   Idle Loop hook.
 * @details This hook is continuously invoked by the idle thread loop.
 */
#if !defined(IDLE_LOOP_HOOK) || defined(__DOXYGEN__)
#define IDLE_LOOP_HOOK() {                                              \
  /* Idle loop code here.*/                                             \
}
#endif

#endif  /* _CHCONF_H_ */

/** @} */
