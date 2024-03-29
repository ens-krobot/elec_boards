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
 * @file    chevents.h
 * @brief   Events macros and structures.
 *
 * @addtogroup events
 * @{
 */

#ifndef _CHEVENTS_H_
#define _CHEVENTS_H_

#if CH_USE_EVENTS

typedef struct EventListener EventListener;

/**
 * @brief   Event Listener structure.
 */
struct EventListener {
  EventListener         *el_next;       /**< @brief Next Event Listener
                                                    registered on the Event
                                                    Source.                 */
  Thread                *el_listener;   /**< @brief Thread interested in the
                                                    Event Source.           */
  eventmask_t           el_mask;        /**< @brief Event flags mask associated
                                                    by the thread to the Event
                                                    Source.                 */
};

/**
 * @brief   Event Source structure.
 */
typedef struct EventSource {
  EventListener         *es_next;       /**< @brief First Event Listener
                                                    registered on the Event
                                                    Source.                 */
} EventSource;

/**
 * @brief   Data part of a static event source initializer.
 * @details This macro should be used when statically initializing an event
 *          source that is part of a bigger structure.
 * @param name the name of the event source variable
 */
#define _EVENTSOURCE_DATA(name) {(void *)(&name)}

/**
 * @brief   Static event source initializer.
 * @details Statically initialized event sources require no explicit
 *          initialization using @p chEvtInit().
 *
 * @param name          the name of the event source variable
 */
#define EVENTSOURCE_DECL(name) EventSource name = _EVENTSOURCE_DATA(name)

/** All events allowed mask.*/
#define ALL_EVENTS -1

/** Returns the event mask from the event identifier.*/
#define EVENT_MASK(eid) (1 << (eid))

/**
 * @brief   Registers an Event Listener on an Event Source.
 * @note    Multiple Event Listeners can use the same event identifier, the
 *          listener will share the callback function.
 *
 * @param[in] esp       pointer to the  @p EventSource structure
 * @param[out] elp      pointer to the @p EventListener structure
 * @param[in] eid       numeric identifier assigned to the Event Listener. The
 *                      identifier is used as index for the event callback
 *                      function.
 *                      The value must range between zero and the size, in bit,
 *                      of the @p eventid_t type minus one.
 */
#define chEvtRegister(esp, elp, eid) chEvtRegisterMask(esp, elp, EVENT_MASK(eid))

/**
 * @brief   Initializes an Event Source.
 * @note    Can be used with interrupts disabled or enabled.
 *
 * @param[in] esp       pointer to the @p EventSource structure
 */
#define chEvtInit(esp) \
        ((esp)->es_next = (EventListener *)(void *)(esp))

/**
 * @brief   Verifies if there is at least one @p EventListener registered.
 * @note    Can be called with interrupts disabled or enabled.
 *
 * @param[in] esp       pointer to the @p EventSource structure
 */
#define chEvtIsListening(esp) \
                ((void *)(esp) != (void *)(esp)->es_next)

/**
 * @brief   Event Handler callback function.
 */
typedef void (*evhandler_t)(eventid_t);

#ifdef __cplusplus
extern "C" {
#endif
  void chEvtRegisterMask(EventSource *esp,
                         EventListener *elp,
                         eventmask_t mask);
  void chEvtUnregister(EventSource *esp, EventListener *elp);
  eventmask_t chEvtClear(eventmask_t mask);
  eventmask_t chEvtPend(eventmask_t mask);
  void chEvtSignal(Thread *tp, eventmask_t mask);
  void chEvtSignalI(Thread *tp, eventmask_t mask);
  void chEvtBroadcast(EventSource *esp);
  void chEvtBroadcastI(EventSource *esp);
  void chEvtDispatch(const evhandler_t *handlers, eventmask_t mask);
#if CH_OPTIMIZE_SPEED || !CH_USE_EVENTS_TIMEOUT
  eventmask_t chEvtWaitOne(eventmask_t mask);
  eventmask_t chEvtWaitAny(eventmask_t mask);
  eventmask_t chEvtWaitAll(eventmask_t mask);
#endif
#if CH_USE_EVENTS_TIMEOUT
  eventmask_t chEvtWaitOneTimeout(eventmask_t mask, systime_t time);
  eventmask_t chEvtWaitAnyTimeout(eventmask_t mask, systime_t time);
  eventmask_t chEvtWaitAllTimeout(eventmask_t mask, systime_t time);
#endif
#ifdef __cplusplus
}
#endif

#if !CH_OPTIMIZE_SPEED && CH_USE_EVENTS_TIMEOUT
#define chEvtWaitOne(mask) chEvtWaitOneTimeout(mask, TIME_INFINITE)
#define chEvtWaitAny(mask) chEvtWaitAnyTimeout(mask, TIME_INFINITE)
#define chEvtWaitAll(mask) chEvtWaitAllTimeout(mask, TIME_INFINITE)
#endif

#endif /* CH_USE_EVENTS */

#endif /* _CHEVENTS_H_ */

/** @} */
