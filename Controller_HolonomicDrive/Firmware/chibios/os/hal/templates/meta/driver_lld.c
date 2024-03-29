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
 * @file    templates/xxx_lld.c
 * @brief   XXX Driver subsystem low level driver source template.
 *
 * @addtogroup XXX_LLD
 * @{
 */

#include "ch.h"
#include "hal.h"

#if CH_HAL_USE_XXX || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level XXX driver initialization.
 */
void xxx_lld_init(void) {

}

/**
 * @brief   Configures and activates the XXX peripheral.
 *
 * @param[in] xxxp      pointer to the @p XXXDriver object
 */
void xxx_lld_start(XXXDriver *xxxp) {

  if (xxxp->xxx_state == XXX_STOP) {
    /* Clock activation.*/
  }
  /* Configuration.*/
}

/**
 * @brief   Deactivates the XXX peripheral.
 *
 * @param[in] xxxp      pointer to the @p XXXDriver object
 */
void xxx_lld_stop(XXXDriver *xxxp) {

}

#endif /* CH_HAL_USE_XXX */

/** @} */
