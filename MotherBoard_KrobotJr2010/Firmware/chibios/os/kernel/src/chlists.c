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
 * @file    chlists.c
 * @brief   Thread queues/lists code.
 *
 * @addtogroup internals
 * @details All the functions present in this module, while public, are not
 *          an OS API and should not be directly used in the user applications
 *          code.
 * @{
 */
#include "ch.h"

#if !CH_OPTIMIZE_SPEED || defined(__DOXYGEN__)
/**
 * @brief   Inserts a thread into a priority ordered queue.
 * @note    The insertion is done by scanning the list from the highest priority
 *          toward the lowest.
 * @note    This function is @b not an API.
 *
 * @param[in] tp        the pointer to the thread to be inserted in the list
 * @param[in] tqp       the pointer to the threads list header
 */
void prio_insert(Thread *tp, ThreadsQueue *tqp) {

  /* cp iterates over the queue.*/
  Thread *cp = (Thread *)tqp;
  do {
    /* Iterate to next thread in queue.*/
    cp = cp->p_next;
    /* Not end of queue? and cp has equal or higher priority than tp?.*/
  } while ((cp != (Thread *)tqp) && (cp->p_prio >= tp->p_prio));
  /* Insert before cp, point tp to next and prev in queue.*/
  tp->p_prev = (tp->p_next = cp)->p_prev;
  /* Make prev point to tp, and cp point back to tp.*/
  tp->p_prev->p_next = cp->p_prev = tp;
}

/**
 * @brief   Inserts a Thread into a queue.
 * @note    This function is @b not an API.
 *
 * @param[in] tp        the pointer to the thread to be inserted in the list
 * @param[in] tqp       the pointer to the threads list header
 */
void queue_insert(Thread *tp, ThreadsQueue *tqp) {

  tp->p_prev = (tp->p_next = (Thread *)tqp)->p_prev;
  tp->p_prev->p_next = tqp->p_prev = tp;
}

/**
 * @brief   Removes the first-out Thread from a queue and returns it.
 * @note    If the queue is priority ordered then this function returns the
 *          thread with the highest priority.
 * @note    This function is @b not an API.
 *
 * @param[in] tqp       the pointer to the threads list header
 * @return              The removed thread pointer.
 */
Thread *fifo_remove(ThreadsQueue *tqp) {
  Thread *tp = tqp->p_next;

  (tqp->p_next = tp->p_next)->p_prev = (Thread *)tqp;
  return tp;
}

/**
 * @brief   Removes the last-out Thread from a queue and returns it.
 * @note    If the queue is priority ordered then this function returns the
 *          thread with the lowest priority.
 * @note    This function is @b not an API.
 *
 * @param[in] tqp   the pointer to the threads list header
 * @return          The removed thread pointer.
 */
Thread *lifo_remove(ThreadsQueue *tqp) {
  Thread *tp = tqp->p_prev;

  (tqp->p_prev = tp->p_prev)->p_next = (Thread *)tqp;
  return tp;
}

/**
 * @brief   Removes a Thread from a queue and returns it.
 * @details The thread is removed from the queue regardless of its relative
 *          position and regardless the used insertion method.
 * @note    This function is @b not an API.
 *
 * @param[in] tp        the pointer to the thread to be removed from the queue
 * @return              The removed thread pointer.
 */
Thread *dequeue(Thread *tp) {

  tp->p_prev->p_next = tp->p_next;
  tp->p_next->p_prev = tp->p_prev;
  return tp;
}

/**
 * @brief   Pushes a Thread on top of a stack list.
 * @note    This function is @b not an API.
 *
 * @param[in] tp    the pointer to the thread to be inserted in the list
 * @param[in] tlp   the pointer to the threads list header
 */
void list_insert(Thread *tp, ThreadsList *tlp) {

  tp->p_next = tlp->p_next;
  tlp->p_next = tp;
}

/**
 * @brief   Pops a Thread from the top of a stack list and returns it.
 * @note    The list must be non-empty before calling this function.
 * @note    This function is @b not an API.
 *
 * @param[in] tlp       the pointer to the threads list header
 * @return              The removed thread pointer.
 */
Thread *list_remove(ThreadsList *tlp) {

  Thread *tp = tlp->p_next;
  tlp->p_next = tp->p_next;
  return tp;
}
#endif /* CH_OPTIMIZE_SPEED */

/** @} */
