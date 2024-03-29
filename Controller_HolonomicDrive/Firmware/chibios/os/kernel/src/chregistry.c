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
 * @file    chregistry.c
 * @brief   Threads registry code.
 *
 * @addtogroup registry
 * @details Threads Registry related APIs and services.
 *
 *          <h2>Operation mode</h2>
 *          The Threads Registry is a double linked list that holds all the
 *          active threads in the system.<br>
 *          Operations defined for the registry:
 *          - <b>First</b>, returns the first, in creation order, active thread
 *            in the system.
 *          - <b>Next</b>, returns the next, in creation order, active thread
 *            in the system.
 *          .
 *          The registry is meant to be mainly a debug feature, as example,
 *          using the registry a debugger can enumerate the active threads
 *          in any given moment or the shell can print the active threads
 *          and their state.<br>
 *          Another possible use is for centralized threads memory management,
 *          terminating threads can pulse an event source and an event handler
 *          can perform a scansion of the registry in order to recover the
 *          memory.<br>
 *          In order to use the threads registry the @p CH_USE_REGISTRY option
 *          must be enabled in @p chconf.h.
 * @{
 */
#include "ch.h"

#if CH_USE_REGISTRY

/**
 * @brief   Returns the first thread in the system.
 * @details Returns the most ancient thread in the system, usually this is
 *          the main thread unless it terminated.
 * @note    A reference is added to the returned thread in order to make sure
 *          it status is not lost.
 * @note    This function cannot return @p NULL because there is always at
 *          least one thread in the system.
 *
 * @return              A reference to the first thread.
 */
Thread *chRegFirstThread(void) {
  Thread *tp;

  chSysLock();
  tp = rlist.r_newer;
#if CH_USE_DYNAMIC
  tp->p_refs++;
#endif
  chSysUnlock();
  return tp;
}

/**
 * @brief   Returns the thread next to the specified one.
 * @details The reference counter of the specified thread is decremented and
 *          the reference counter of the returned thread is incremented.
 *
 * @param[in] tp        pointer to the thread
 * @return              A reference to the next thread.
 * @retval NULL         if there is no next thread.
 */
Thread *chRegNextThread(Thread *tp) {
  Thread *ntp;

  chSysLock();
  ntp = tp->p_newer;
  if (ntp == (Thread *)&rlist)
    ntp = NULL;
#if CH_USE_DYNAMIC
  else {
    chDbgAssert(ntp->p_refs < 255, "chRegNextThread(), #1",
                "too many references");
    ntp->p_refs++;
  }
#endif
  chSysUnlock();
#if CH_USE_DYNAMIC
  chThdRelease(tp);
#endif
  return ntp;
}

#endif /* CH_USE_REGISTRY */

/** @} */
