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

#ifndef _WFI_H_
#define _WFI_H_

#include "lpc214x.h"

#ifndef port_wait_for_interrupt
#if ENABLE_WFI_IDLE != 0
#define port_wait_for_interrupt() {                                     \
  PCON = 1;                                                             \
}
#else
#define port_wait_for_interrupt()
#endif
#endif

#endif /* _WFI_H_ */
