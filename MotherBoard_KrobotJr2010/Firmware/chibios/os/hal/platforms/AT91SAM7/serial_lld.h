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
 * @file AT91SAM7/serial_lld.h
 * @brief AT91SAM7 low level serial driver header.
 * @addtogroup AT91SAM7_SERIAL
 * @{
 */

#ifndef _SERIAL_LLD_H_
#define _SERIAL_LLD_H_

#if CH_HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief UART0 driver enable switch.
 * @details If set to @p TRUE the support for USART1 is included.
 * @note The default is @p TRUE.
 */
#if !defined(USE_SAM7_USART0) || defined(__DOXYGEN__)
#define USE_SAM7_USART0             TRUE
#endif

/**
 * @brief UART1 driver enable switch.
 * @details If set to @p TRUE the support for USART2 is included.
 * @note The default is @p TRUE.
 */
#if !defined(USE_SAM7_USART1) || defined(__DOXYGEN__)
#define USE_SAM7_USART1             TRUE
#endif

/**
 * @brief UART1 interrupt priority level setting.
 */
#if !defined(SAM7_USART0_PRIORITY) || defined(__DOXYGEN__)
#define SAM7_USART0_PRIORITY        (AT91C_AIC_PRIOR_HIGHEST - 2)
#endif

/**
 * @brief UART2 interrupt priority level setting.
 */
#if !defined(SAM7_USART1_PRIORITY) || defined(__DOXYGEN__)
#define SAM7_USART1_PRIORITY        (AT91C_AIC_PRIOR_HIGHEST - 2)
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Serial Driver condition flags type.
 */
typedef uint32_t sdflags_t;

/**
 * @brief AT91SAM7 Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct {
  /**
   * @brief Bit rate.
   */
  uint32_t                  sc_speed;
  /**
   * @brief Initialization value for the MR register.
   */
  uint32_t                  sc_mr;
} SerialConfig;

/**
 * @brief @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Current configuration data.*/                                          \
  const SerialConfig        *config;                                        \
  /* Input queue.*/                                                         \
  InputQueue                iqueue;                                         \
  /* Output queue.*/                                                        \
  OutputQueue               oqueue;                                         \
  /* Status Change @p EventSource.*/                                        \
  EventSource               sevent;                                         \
  /* I/O driver status flags.*/                                             \
  sdflags_t                 flags;                                          \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                         \
  /* Pointer to the USART registers block.*/                                \
  AT91PS_USART              usart;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if USE_SAM7_USART0 && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif
#if USE_SAM7_USART1 && !defined(__DOXYGEN__)
extern SerialDriver SD2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp);
  void sd_lld_stop(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* CH_HAL_USE_SERIAL */

#endif /* _SERIAL_LLD_H_ */

/** @} */
