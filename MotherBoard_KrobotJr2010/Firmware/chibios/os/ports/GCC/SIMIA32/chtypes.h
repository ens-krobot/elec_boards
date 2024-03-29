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

#ifndef _CHTYPES_H_
#define _CHTYPES_H_

#define __need_NULL
#define __need_size_t
#define __need_ptrdiff_t
#include <stddef.h>

#if !defined(_STDINT_H) && !defined(__STDINT_H_)
#include <stdint.h>
#endif

typedef int32_t         bool_t;         /**< Fast boolean type.             */
typedef uint8_t         tmode_t;        /**< Thread flags.                  */
typedef uint8_t         tstate_t;       /**< Thread state.                  */
typedef uint8_t         trefs_t;        /**< Thread references counter.     */
typedef uint32_t        tprio_t;        /**< Thread priority.               */
typedef int32_t         msg_t;          /**< Inter-thread message.          */
typedef int32_t         eventid_t;      /**< Event Id.                      */
typedef uint32_t        eventmask_t;    /**< Events mask.                   */
typedef uint32_t        systime_t;      /**< System time.                   */
typedef int32_t         cnt_t;          /**< Resources counter.             */

#define INLINE      inline
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

#endif /* _CHTYPES_H_ */
