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
 * @file    serial.h
 * @brief   Serial Driver macros and structures.
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef _SERIAL_H_
#define _SERIAL_H_

#if CH_HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/** @brief No pending conditions.*/
#define SD_NO_ERROR             0
/** @brief Connection happened.*/
#define SD_CONNECTED            1
/** @brief Disconnection happened.*/
#define SD_DISCONNECTED         2
/** @brief Parity error happened.*/
#define SD_PARITY_ERROR         4
/** @brief Framing error happened.*/
#define SD_FRAMING_ERROR        8
/** @brief Overflow happened.*/
#define SD_OVERRUN_ERROR        16
/** @brief Noise on the line.*/
#define SD_NOISE_ERROR          32
/** @brief Break detected.*/
#define SD_BREAK_DETECTED       64

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Default bit rate.
 * @details Configuration parameter, this is the baud rate selected for the
 *          default configuration.
 */
#if !defined(SERIAL_DEFAULT_BITRATE) || defined(__DOXYGEN__)
#define SERIAL_DEFAULT_BITRATE      38400
#endif

/**
 * @brief   Serial buffers size.
 * @details Configuration parameter, you can change the depth of the queue
 *          buffers depending on the requirements of your application.
 * @note    The default is 64 bytes for both the transmission and receive
 *          buffers.
 */
#if !defined(SERIAL_BUFFERS_SIZE) || defined(__DOXYGEN__)
#define SERIAL_BUFFERS_SIZE         64
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !CH_USE_QUEUES && !CH_USE_EVENTS
#error "Serial Driver requires CH_USE_QUEUES and CH_USE_EVENTS"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Driver state machine possible states.
 */
typedef enum {
  SD_UNINIT = 0,                    /**< @brief Not initialized.            */
  SD_STOP = 1,                      /**< @brief Stopped.                    */
  SD_READY = 2                      /**< @brief Ready.                      */
} sdstate_t;

/**
 * @brief   Structure representing a serial driver.
 */
typedef struct _SerialDriver SerialDriver;

#include "serial_lld.h"

/**
 * @brief   @p SerialDriver specific methods.
 */
#define _serial_driver_methods                                              \
  _base_asynchronous_channel_methods

/**
 * @brief   @p SerialDriver virtual methods table.
 */
struct SerialDriverVMT {
  _serial_driver_methods
};

/**
 * @extends BaseAsynchronousChannel
 *
 * @brief   Full duplex serial driver class.
 * @details This class extends @p BaseAsynchronousChannel by adding physical
 *          I/O queues.
 */
struct _SerialDriver {
  /** @brief Virtual Methods Table.*/
  const struct SerialDriverVMT *vmt;
  _serial_driver_data
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Direct output check on a @p SerialDriver.
 * @details This function bypasses the indirect access to the channel and
 *          checks directly the output queue. This is faster but cannot
 *          be used to check different channels implementations.
 *
 * @see     chIOPutWouldBlock()
 */
#define sdPutWouldBlock(sdp) chOQIsFull(&(sdp)->oqueue)

/**
 * @brief   Direct input check on a @p SerialDriver.
 * @details This function bypasses the indirect access to the channel and
 *          checks directly the input queue. This is faster but cannot
 *          be used to check different channels implementations.
 *
 * @see     chIOGetWouldBlock()
 */
#define sdGetWouldBlock(sdp) chIQIsEmpty(&(sdp)->iqueue)

/**
 * @brief   Direct write to a @p SerialDriver.
 * @details This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @see     chIOPut()
 */
#define sdPut(sdp, b) chOQPut(&(sdp)->oqueue, b)

/**
 * @brief   Direct write to a @p SerialDriver with timeout specification.
 * @details This function bypasses the indirect access to the channel and
 *          writes directly on the output queue. This is faster but cannot
 *          be used to write to different channels implementations.
 *
 * @see     chIOPutTimeout()
 */
#define sdPutTimeout(sdp, b, t) chOQPutTimeout(&(sdp)->iqueue, b, t)

/**
 * @brief   Direct read from a @p SerialDriver.
 * @details This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chIOGet()
 */
#define sdGet(sdp) chIQGet(&(sdp)->iqueue)

/**
 * @brief   Direct read from a @p SerialDriver with timeout specification.
 * @details This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chIOGetTimeout()
 */
#define sdGetTimeout(sdp, t) chIQGetTimeout(&(sdp)->iqueue, t)

/**
 * @brief   Direct blocking write to a @p SerialDriver.
 * @details This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write from different channels implementations.
 *
 * @see     chIOWriteTimeout()
 */
#define sdWrite(sdp, b, n)                                                  \
  chOQWriteTimeout(&(sdp)->oqueue, b, n, TIME_INFINITE)

/**
 * @brief   Direct blocking write to a @p SerialDriver with timeout
 *          specification.
 * @details This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write from different channels implementations.
 *
 * @see     chIOWriteTimeout()
 */
#define sdWriteTimeout(sdp, b, n, t)                                        \
  chOQWriteTimeout(&(sdp)->oqueue, b, n, t)

/**
 * @brief   Direct non-blocking write to a @p SerialDriver.
 * @details This function bypasses the indirect access to the channel and
 *          writes directly to the output queue. This is faster but cannot
 *          be used to write from different channels implementations.
 *
 * @see     chIOWriteTimeout()
 */
#define sdAsynchronousWrite(sdp, b, n)                                      \
  chOQWriteTimeout(&(sdp)->oqueue, b, n, TIME_IMMEDIATE)

/**
 * @brief   Direct blocking read from a @p SerialDriver.
 * @details This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chIOReadTimeout()
 */
#define sdRead(sdp, b, n)                                                   \
  chIQReadTimeout(&(sdp)->iqueue, b, n, TIME_INFINITE)

/**
 * @brief   Direct blocking read from a @p SerialDriver with timeout
 *          specification.
 * @details This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chIOReadTimeout()
 */
#define sdReadTimeout(sdp, b, n, t)                                         \
  chIQReadTimeout(&(sdp)->iqueue, b, n, t)

/**
 * @brief   Direct non-blocking read from a @p SerialDriver.
 * @details This function bypasses the indirect access to the channel and
 *          reads directly from the input queue. This is faster but cannot
 *          be used to read from different channels implementations.
 *
 * @see     chIOReadTimeout()
 */
#define sdAsynchronousRead(sdp, b, n)                                       \
  chIQReadTimeout(&(sdp)->iqueue, b, n, TIME_IMMEDIATE)

/**
 * @brief   Returns the status change event source.
 * @details The status change event source is broadcasted when the channel
 *          status is updated, the status flags can then be fetched and
 *          cheared by using @p sdGetAndClearFlags().
 *
 * @param[in] ip        pointer to a @p SerialDriver object
 * @return              A pointer to an @p EventSource object.
 */
#define sdGetStatusChangeEventSource(ip) (&((ip)->vmt->sevent))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void sdInit(void);
  void sdObjectInit(SerialDriver *sdp, qnotify_t inotify, qnotify_t onotify);
  void sdStart(SerialDriver *sdp, const SerialConfig *config);
  void sdStop(SerialDriver *sdp);
  void sdIncomingDataI(SerialDriver *sdp, uint8_t b);
  msg_t sdRequestDataI(SerialDriver *sdp);
  void sdAddFlagsI(SerialDriver *sdp, sdflags_t mask);
  sdflags_t sdGetAndClearFlags(SerialDriver *sdp);
#ifdef __cplusplus
}
#endif

#endif /* CH_HAL_USE_SERIAL */

#endif /* _SERIAL_H_ */

/** @} */
