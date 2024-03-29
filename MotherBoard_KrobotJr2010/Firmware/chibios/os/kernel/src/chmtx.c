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
 * @file    chmtx.c
 * @brief   Mutexes code.
 *
 * @addtogroup mutexes
 * @details Mutexes related APIs and services.
 *
 *          <h2>Operation mode</h2>
 *          A mutex is a threads synchronization object that can be in two
 *          distinct states:
 *          - Not owned.
 *          - Owned by a thread.
 *          .
 *          Operations defined for mutexes:
 *          - <b>Lock</b>: The mutex is checked, if the mutex is not owned by
 *            some other thread then it is associated to the locking thread
 *            else the thread is queued on the mutex in a list ordered by
 *            priority.
 *          - <b>Unlock</b>: The mutex is released by the owner and the highest
 *            priority thread waiting in the queue, if any, is resumed and made
 *            owner of the mutex.
 *          .
 *          In order to use the Mutexes APIs the @p CH_USE_MUTEXES option must
 *          be enabled in @p chconf.h.
 *          <h2>Constraints</h2>
 *          In ChibiOS/RT the Unlock operations are always performed in
 *          lock-reverse order. The unlock API does not even have a parameter,
 *          the mutex to unlock is selected from an internal, per-thread, stack
 *          of owned mutexes. This both improves the performance and is
 *          required for an efficient implementation of the priority
 *          inheritance mechanism.
 *
 *          <h2>The priority inversion problem</h2>
 *          The mutexes in ChibiOS/RT implements the <b>full</b> priority
 *          inheritance mechanism in order handle the priority inversion
 *          problem.<br>
 *          When a thread is queued on a mutex, any thread, directly or
 *          indirectly, holding the mutex gains the same priority of the
 *          waiting thread (if their priority was not already equal or higher).
 *          The mechanism works with any number of nested mutexes and any
 *          number of involved threads. The algorithm complexity (worst case)
 *          is N with N equal to the number of nested mutexes.
 * @{
 */

#include "ch.h"

#if CH_USE_MUTEXES

/**
 * @brief   Initializes s @p Mutex structure.
 *
 * @param[out] mp       pointer to a @p Mutex structure
 */
void chMtxInit(Mutex *mp) {

  chDbgCheck(mp != NULL, "chMtxInit");

  queue_init(&mp->m_queue);
  mp->m_owner = NULL;
}

/**
 * @brief   Locks the specified mutex.
 *
 * @param[in] mp        pointer to the @p Mutex structure
 */
void chMtxLock(Mutex *mp) {

  chSysLock();

  chMtxLockS(mp);

  chSysUnlock();
}

/**
 * @brief   Locks the specified mutex.
 *
 * @param[in] mp        pointer to the @p Mutex structure
 */
void chMtxLockS(Mutex *mp) {
  Thread *ctp = currp;

  chDbgCheck(mp != NULL, "chMtxLockS");

  /* Ia the mutex already locked? */
  if (mp->m_owner != NULL) {
    /* Priority inheritance protocol; explores the thread-mutex dependencies
       boosting the priority of all the affected threads to equal the priority
       of the running thread requesting the mutex.*/
    Thread *tp = mp->m_owner;
    /* Does the running thread have higher priority than the mutex
       ownning thread? */
    while (tp->p_prio < ctp->p_prio) {
      /* Make priority of thread tp match the running thread's priority.*/
      tp->p_prio = ctp->p_prio;
      /* The following states need priority queues reordering.*/
      switch (tp->p_state) {
      case THD_STATE_WTMTX:
        /* Re-enqueues the mutex owner with its new priority.*/
        prio_insert(dequeue(tp), (ThreadsQueue *)tp->p_u.wtobjp);
        tp = ((Mutex *)tp->p_u.wtobjp)->m_owner;
        continue;
#if CH_USE_CONDVARS | CH_USE_SEMAPHORES_PRIORITY | CH_USE_MESSAGES_PRIORITY
#if CH_USE_CONDVARS
      case THD_STATE_WTCOND:
#endif
#if CH_USE_SEMAPHORES_PRIORITY
      case THD_STATE_WTSEM:
#endif
#if CH_USE_MESSAGES_PRIORITY
      case THD_STATE_SNDMSG:
#endif
        /* Re-enqueues tp with its new priority on the queue.*/
        prio_insert(dequeue(tp), (ThreadsQueue *)tp->p_u.wtobjp);
        break;
#endif
      case THD_STATE_READY:
        /* Re-enqueues  tp with its new priority on the ready list.*/
        chSchReadyI(dequeue(tp));
      }
      break;
    }
    /* Sleep on the mutex.*/
    prio_insert(ctp, &mp->m_queue);
    ctp->p_u.wtobjp = mp;
    chSchGoSleepS(THD_STATE_WTMTX);
    chDbgAssert(mp->m_owner == NULL, "chMtxLockS(), #1", "still owned");
  }
  /* The mutex is now inserted in the owned mutexes list.*/
  mp->m_owner = ctp;
  mp->m_next = ctp->p_mtxlist;
  ctp->p_mtxlist = mp;
}

/**
 * @brief   Tries to lock a mutex.
 * @details This function does not have any overhead related to
 *          the priority inheritance mechanism because it does not try to
 *          enter a sleep state on the mutex.
 *
 * @param[in] mp        pointer to the @p Mutex structure
 * @retval TRUE         if the mutex was successfully acquired
 * @retval FALSE        if the lock attempt failed.
 */
bool_t chMtxTryLock(Mutex *mp) {
  bool_t b;

  chSysLock();

  b = chMtxTryLockS(mp);

  chSysUnlock();
  return b;
}

/**
 * @brief   Tries to lock a mutex.
 * @details This function does not have any overhead related to
 *          the priority inheritance mechanism because it does not try to
 *          enter a sleep state on the mutex.
 *
 * @param[in] mp        pointer to the @p Mutex structure
 * @retval TRUE         if the mutex was successfully acquired
 * @retval FALSE        if the lock attempt failed.
 */
bool_t chMtxTryLockS(Mutex *mp) {

  chDbgCheck(mp != NULL, "chMtxTryLockS");

  if (mp->m_owner != NULL)
    return FALSE;
  mp->m_owner = currp;
  mp->m_next = currp->p_mtxlist;
  currp->p_mtxlist = mp;
  return TRUE;
}

/**
 * @brief   Unlocks the next owned mutex in reverse lock order.
 *
 * @return              The pointer to the unlocked mutex.
 */
Mutex *chMtxUnlock(void) {
  Thread *ctp = currp;
  Mutex *ump, *mp;

  chSysLock();
  chDbgAssert(ctp->p_mtxlist != NULL,
              "chMtxUnlock(), #1",
              "owned mutexes list empty");
  chDbgAssert(ctp->p_mtxlist->m_owner == ctp,
              "chMtxUnlock(), #2",
              "ownership failure");
  /* Removes the top Mutex from the Threads's owned mutexes list and matk it
     as not owned.*/
  ump = ctp->p_mtxlist;
  ctp->p_mtxlist = ump->m_next;
  ump->m_owner = NULL;
  /* If a thread is waiting on the mutex then the fun part begins.*/
  if (chMtxQueueNotEmptyS(ump)) {
    /* Recalculates the optimal thread priority by scanning the owned
       mutexes list.*/
    tprio_t newprio = ctp->p_realprio;
    mp = ctp->p_mtxlist;
    while (mp != NULL) {
      /* If the highest priority thread waiting in the mutexes list has a
         greater priority than the current thread base priority then the final
         priority will have at least that priority.*/
      if (chMtxQueueNotEmptyS(mp) && (mp->m_queue.p_next->p_prio > newprio))
        newprio = mp->m_queue.p_next->p_prio;
      mp = mp->m_next;
    }
    /* Assigns to the current thread the highest priority among all the
       waiting threads.*/
    ctp->p_prio = newprio;
    /* Awakens the highest priority thread waiting for the unlocked mutex.*/
    chSchWakeupS(fifo_remove(&ump->m_queue), RDY_OK);
  }
  chSysUnlock();
  return ump;
}

/**
 * @brief   Unlocks the next owned mutex in reverse lock order.
 * @note    This function does not reschedule internally.
 *
 * @return              The pointer to the unlocked mutex.
 */
Mutex *chMtxUnlockS(void) {
  Thread *ctp = currp;
  Mutex *ump, *mp;

  chDbgAssert(ctp->p_mtxlist != NULL,
              "chMtxUnlockS(), #1",
              "owned mutexes list empty");
  chDbgAssert(ctp->p_mtxlist->m_owner == ctp,
              "chMtxUnlockS(), #2",
              "ownership failure");

  /* Removes the top Mutex from the owned mutexes list and marks it as not
     owned.*/
  ump = ctp->p_mtxlist;
  ctp->p_mtxlist = ump->m_next;
  ump->m_owner = NULL;
  /* If a thread is waiting on the mutex then the fun part begins.*/
  if (chMtxQueueNotEmptyS(ump)) {
    /* Recalculates the optimal thread priority by scanning the owned
       mutexes list.*/
    tprio_t newprio = ctp->p_realprio;
    mp = ctp->p_mtxlist;
    while (mp != NULL) {
      /* If the highest priority thread waiting in the mutexes list has a
         greater priority than the current thread base priority then the final
         priority will have at least that priority.*/
      if (chMtxQueueNotEmptyS(mp) && (mp->m_queue.p_next->p_prio > newprio))
        newprio = mp->m_queue.p_next->p_prio;
      mp = mp->m_next;
    }
    ctp->p_prio = newprio;
    chSchReadyI(fifo_remove(&ump->m_queue));
  }
  return ump;
}

/**
 * @brief   Unlocks all the mutexes owned by the invoking thread.
 * @details This function is <b>MUCH MORE</b> efficient than releasing the
 *          mutexes one by one and not just because the call overhead,
 *          this function does not have any overhead related to the priority
 *          inheritance mechanism.
 */
void chMtxUnlockAll(void) {
  Thread *ctp = currp;

  chSysLock();
  if (ctp->p_mtxlist != NULL) {
    do {
      Mutex *mp = ctp->p_mtxlist;
      ctp->p_mtxlist = mp->m_next;
      mp->m_owner = NULL;
      if (chMtxQueueNotEmptyS(mp))
        chSchReadyI(fifo_remove(&mp->m_queue));
    } while (ctp->p_mtxlist != NULL);
    ctp->p_prio = ctp->p_realprio;
    chSchRescheduleS();
  }
  chSysUnlock();
}

#endif /* CH_USE_MUTEXES */

/** @} */
