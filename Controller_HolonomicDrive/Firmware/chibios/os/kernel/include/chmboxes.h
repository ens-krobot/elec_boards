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
 * @file    chmboxes.h
 * @brief   Mailboxes macros and structures.
 *
 * @addtogroup mailboxes
 * @{
 */

#ifndef _CHMBOXES_H_
#define _CHMBOXES_H_

#if CH_USE_MAILBOXES

/*
 * Module dependencies check.
 */
#if !CH_USE_SEMAPHORES
#error "CH_USE_MAILBOXES requires CH_USE_SEMAPHORES"
#endif

typedef struct {
  msg_t                 *mb_buffer;     /**< @brief Pointer to the mailbox
                                                    buffer.                 */
  msg_t                 *mb_top;        /**< @brief Pointer to the location
                                                    after the buffer.       */
  msg_t                 *mb_wrptr;      /**< @brief Write pointer.          */
  msg_t                 *mb_rdptr;      /**< @brief Read pointer.           */
  Semaphore             mb_fullsem;     /**< @brief Full counter
                                                    @p Semaphore.           */
  Semaphore             mb_emptysem;    /**< @brief Empty counter
                                                    @p Semaphore.           */
} Mailbox;

#ifdef __cplusplus
extern "C" {
#endif
  void chMBInit(Mailbox *mbp, msg_t *buf, cnt_t n);
  void chMBReset(Mailbox *mbp);
  msg_t chMBPost(Mailbox *mbp, msg_t msg, systime_t timeout);
  msg_t chMBPostS(Mailbox *mbp, msg_t msg, systime_t timeout);
  msg_t chMBPostAhead(Mailbox *mbp, msg_t msg, systime_t timeout);
  msg_t chMBPostAheadS(Mailbox *mbp, msg_t msg, systime_t timeout);
  msg_t chMBFetch(Mailbox *mbp, msg_t *msgp, systime_t timeout);
  msg_t chMBFetchS(Mailbox *mbp, msg_t *msgp, systime_t timeout);
#ifdef __cplusplus
}
#endif

/**
 * @brief   Returns the mailbox buffer size.
 *
 * @param[in] mbp       the pointer to an initialized Mailbox object
 */
#define chMBSize(mbp)                                                   \
        ((mbp)->mb_top - (mbp)->mb_buffer)

/**
 * @brief   Returns the free space into the mailbox.
 * @note    Can be invoked in any system state but if invoked out of a locked
 *          state then the returned value may change after reading.
 * @note    The returned value can be less than zero when there are waiting
 *          threads on the internal semaphore.
 *
 * @param[in] mbp       the pointer to an initialized Mailbox object
 * @return              The number of empty message slots.
 */
#define chMBGetEmpty(mbp) chSemGetCounterI(&(mbp)->mb_emptysem)

/**
 * @brief   Returns the number of messages into the mailbox.
 * @note    Can be invoked in any system state but if invoked out of a locked
 *          state then the returned value may change after reading.
 * @note    The returned value can be less than zero when there are waiting
 *          threads on the internal semaphore.
 *
 * @param[in] mbp       the pointer to an initialized Mailbox object
 * @return              The number of queued messages.
 */
#define chMBGetFull(mbp) chSemGetCounterI(&(mbp)->mb_fullsem)

/**
 * @brief   Returns the next message in the queue without removing it.
 * @note    A message must be waiting in the queue for this function to work or
 *          it would return garbage. The correct way to use this macro is to
 *          use @p chMBGetFull() and then use this macro, all within a lock
 *          state.
 */
#define chMBPeek(mbp) (*(mbp)->mb_rdptr)

/**
 * @brief   Data part of a static mailbox initializer.
 * @details This macro should be used when statically initializing a
 *          mailbox that is part of a bigger structure.
 *
 * @param[in] name      the name of the mailbox variable
 * @param[in] buffer    pointer to the mailbox buffer area
 * @param[in] size      size of the mailbox buffer area
 */
#define _MAILBOX_DATA(name, buffer, size) {                             \
  (msg_t *)(buffer),                                                    \
  (msg_t *)(buffer) + size,                                             \
  (msg_t *)(buffer),                                                    \
  (msg_t *)(buffer),                                                    \
  _SEMAPHORE_DATA(name.mb_fullsem, 0),                                  \
  _SEMAPHORE_DATA(name.mb_emptysem, size),                              \
}

/**
 * @brief   Static mailbox initializer.
 * @details Statically initialized mailboxes require no explicit
 *          initialization using @p chMBInit().
 *
 * @param[in] name      the name of the mailbox variable
 * @param[in] buffer    pointer to the mailbox buffer area
 * @param[in] size      size of the mailbox buffer area
 */
#define MAILBOX_DECL(name, buffer, size)                                \
  Mailbox name = _MAILBOX_DATA(name, buffer, size)

#endif /* CH_USE_MAILBOXES */

#endif /* _CHMBOXES_H_ */

/** @} */
