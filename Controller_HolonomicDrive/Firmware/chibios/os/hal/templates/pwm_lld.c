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
 * @file    templates/pwm_lld.c
 * @brief   PWM Driver subsystem low level driver source template.
 *
 * @addtogroup PWM_LLD
 * @{
 */

#include "ch.h"
#include "hal.h"

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
 * @brief   Low level PWM driver initialization.
 */
void pwm_lld_init(void) {

}

/**
 * @brief   Configures and activates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 */
void pwm_lld_start(PWMDriver *pwmp) {

  if (pwmp->pd_state == PWM_STOP) {
    /* Clock activation.*/
  }
  /* Configuration.*/
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 */
void pwm_lld_stop(PWMDriver *pwmp) {

}

/**
 * @brief   Determines whatever the PWM channel is already enabled.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 * @param[in] channel   PWM channel identifier
 * @return              The PWM channel status.
 * @retval FALSE        the channel is not enabled.
 * @retval TRUE         the channel is enabled.
 */
bool_t pwm_lld_is_enabled(PWMDriver *pwmp, pwmchannel_t channel) {

  return FALSE;
}

/**
 * @brief   Enables a callback mode for the specified PWM channel.
 * @details The callback mode must be set before starting a PWM channel.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 * @param[in] channel   PWM channel identifier
 * @param[in] edge      output edge mode
 * @param[in] callback  callback function
 */
void pwm_lld_set_callback(PWMDriver *pwmp, pwmchannel_t channel,
                          pwmedge_t edge, pwmcallback_t callback) {

}

/**
 * @brief   Enables a PWM channel.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 * @param[in] channel   PWM channel identifier
 * @param[in] width     PWM pulse width as clock pulses number
 */
void pwm_lld_enable_channel(PWMDriver *pwmp,
                            pwmchannel_t channel,
                            pwmcnt_t width) {

}

/**
 * @brief   Disables a PWM channel.
 * @details The channel is disabled and its output line returned to the
 *          idle state.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 * @param[in] channel   PWM channel identifier
 */
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel) {

}

/** @} */
