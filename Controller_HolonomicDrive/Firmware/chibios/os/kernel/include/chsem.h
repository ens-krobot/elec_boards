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
 * @file    chsem.h
 * @brief   Semaphores macros and structures.
 *
 * @addtogroup semaphores
 * @{
 */

#ifndef _CHSEM_H_
#define _CHSEM_H_

#if CH_USE_SEMAPHORES

/**
 * @brief   Semaphore structure.
 */
typedef struct Semaphore {
  ThreadsQueue          s_queue;    /**< @brief Queue of the threads sleeping
                                                on this semaphore.          */
  cnt_t                 s_cnt;      /**< @brief The semaphore counter.      */
} Semaphore;

#ifdef __cplusplus
extern "C" {
#endif
  void chSemInit(Semaphore *sp, cnt_t n);
  void chSemReset(Semaphore *sp, cnt_t n);
  void chSemResetI(Semaphore *sp, cnt_t n);
  msg_t chSemWait(Semaphore *sp);
  msg_t chSemWaitS(Semaphore *sp);
  msg_t chSemWaitTimeout(Semaphore *sp, systime_t time);
  msg_t chSemWaitTimeoutS(Semaphore *sp, systime_t time);
  void chSemSignal(Semaphore *sp);
  void chSemSignalI(Semaphore *sp);
#if CH_USE_SEMSW
  msg_t chSemSignalWait(Semaphore *sps, Semaphore *spw);
#endif
#ifdef __cplusplus
}
#endif

/**
 * @brief   Data part of a static semaphore initializer.
 * @details This macro should be used when statically initializing a semaphore
 *          that is part of a bigger structure.
 *
 * @param[in] name      the name of the semaphore variable
 * @param[in] n         the counter initial value, this value must be
 *                      non-negative
 */
#define _SEMAPHORE_DATA(name, n) {_THREADSQUEUE_DATA(name.s_queue), n}

/**
 * @brief   Static semaphore initializer.
 * @details Statically initialized semaphores require no explicit
 *          initialization using @p chSemInit().
 *
 * @param[in] name      the name of the semaphore variable
 * @param[in] n         the counter initial value, this value must be
 *                      non-negative
 */
#define SEMAPHORE_DECL(name, n) Semaphore name = _SEMAPHORE_DATA(name, n)

/**
 * @brief   Decreases the semaphore counter.
 * @details This macro can be used when the counter is known to be positive.
 */
#define chSemFastWaitI(sp)      ((sp)->s_cnt--)

/**
 * @brief   Increases the semaphore counter.
 * @details This macro can be used when the counter is known to be not negative.
 */
#define chSemFastSignalI(sp)    ((sp)->s_cnt++)

/**
 * @brief   Returns the semaphore counter current value.
 */
#define chSemGetCounterI(sp)    ((sp)->s_cnt)

#endif /* CH_USE_SEMAPHORES */

#endif /* _CHSEM_H_ */

/** @} */
