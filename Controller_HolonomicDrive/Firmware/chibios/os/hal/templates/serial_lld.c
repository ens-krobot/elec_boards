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
 * @file    templates/serial_lld.c
 * @brief   Serial Driver subsystem low level driver source template.
 *
 * @addtogroup SERIAL_LLD
 * @{
 */

#include "ch.h"
#include "hal.h"

#if CH_HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
};

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
 * @brief   Low level serial driver initialization.
 */
void sd_lld_init(void) {

}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
void sd_lld_start(SerialDriver *sdp) {

  if (sdp->config == NULL)
    sdp->config = &default_config;

}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
void sd_lld_stop(SerialDriver *sdp) {

}

#endif /* CH_HAL_USE_SERIAL */

/** @} */
