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
 * @file    chioch.h
 * @brief   I/O channels.
 * @details This header defines abstract interfaces useful to access generic
 *          I/O resources in a standardized way.
 *
 * @addtogroup io_channels
 * @details This module defines an abstract interface for I/O channels by
 *          extending the @p BaseSequentialStream interface. Note that no code
 *          is present, I/O channels are just abstract interface like
 *          structures, you should look at the systems as to a set of abstract
 *          C++ classes (even if written in C). Specific device drivers can
 *          use/extend the interface and implement them.<br>
 *          This system has the advantage to make the access to channels
 *          independent from the implementation logic.
 * @{
 */

#ifndef _CHIOCH_H_
#define _CHIOCH_H_

/**
 * @brief   @p BaseChannel specific methods.
 */
#define _base_channel_methods                                               \
  _base_sequental_stream_methods                                            \
  /* Channel output check.*/                                                \
  bool_t (*putwouldblock)(void *instance);                                  \
  /* Channel input check.*/                                                 \
  bool_t (*getwouldblock)(void *instance);                                  \
  /* Channel put method with timeout specification.*/                       \
  msg_t (*put)(void *instance, uint8_t b, systime_t time);                  \
  /* Channel get method with timeout specification.*/                       \
  msg_t (*get)(void *instance, systime_t time);                             \
  /* Channel write method with timeout specification.*/                     \
  size_t (*writet)(void *instance, const uint8_t *bp,                       \
                  size_t n, systime_t time);                                \
  /* Channel read method with timeout specification.*/                      \
  size_t (*readt)(void *instance, uint8_t *bp, size_t n, systime_t time);

/**
 * @brief   @p BaseChannel specific data.
 * @note    It is empty because @p BaseChannel is only an interface without
 *          implementation.
 */
#define _base_channel_data                                                  \
  _base_sequental_stream_data

/**
 * @brief   @p BaseChannel virtual methods table.
 */
struct BaseChannelVMT {                                                     \
  _base_channel_methods                                                     \
};

/**
 * @extends BaseSequentialStream
 *
 * @brief   Base channel class.
 * @details This class represents a generic, byte-wide, I/O channel. This class
 *          introduces generic I/O primitives with timeout specification.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseChannelVMT *vmt;
  _base_channel_data
} BaseChannel;

/**
 * @brief   Channel output check.
 * @details This function verifies if a subsequent put/write operation would
 *          block.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 * @return              The output queue status:
 * @retval FALSE        if the output queue has space and would not block a
 *                      write operation.
 * @retval TRUE         if the output queue is full and would block a write
 *                      operation.
 */
#define chIOPutWouldBlock(ip) ((ip)->vmt->putwouldblock(ip))

/**
 * @brief   Channel input check.
 * @details This function verifies if a subsequent get/read operation would
 *          block.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 * @return              The input queue status:
 * @retval FALSE        if the input queue contains data and would not block a
 *                      read operation.
 * @retval TRUE         if the input queue is empty and would block a read
 *                      operation.
 */
#define chIOGetWouldBlock(ip) ((ip)->vmt->getwouldblock(ip))

/**
 * @brief   Channel blocking byte write.
 * @details This function writes a byte value to a channel. If the channel
 *          is not ready to accept data then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 * @param[in] b         the byte value to be written to the channel
 * @return              The operation status:
 * @retval Q_OK         if the operation succeeded.
 * @retval Q_RESET      if the channel associated queue (if any) was reset.
 */
#define chIOPut(ip, b) ((ip)->vmt->put(ip, b, TIME_INFINITE))

/**
 * @brief   Channel blocking byte write with timeout.
 * @details This function writes a byte value to a channel. If the channel
 *          is not ready to accept data then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 * @param[in] b         the byte value to be written to the channel
 * @param[in] time      the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status:
 * @retval Q_OK         if the operation succeeded.
 * @retval Q_TIMEOUT    if the specified time expired.
 * @retval Q_RESET      if the channel associated queue (if any) was reset.
 */
#define chIOPutTimeout(ip, b, time) ((ip)->vmt->put(ip, b, time))

/**
 * @brief   Channel blocking byte read.
 * @details This function reads a byte value from a channel. If the data
 *          is not available then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 * @return              A byte value from the queue or:
 * @retval Q_RESET      if the channel associated queue (if any) was reset.
 */
#define chIOGet(ip) ((ip)->vmt->get(ip, TIME_INFINITE))

/**
 * @brief   Channel blocking byte read with timeout.
 * @details This function reads a byte value from a channel. If the data
 *          is not available then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 * @param[in] time      the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              A byte value from the queue or:
 * @retval Q_TIMEOUT    if the specified time expired.
 * @retval Q_RESET      if the channel associated queue (if any) was reset.
 */
#define chIOGetTimeout(ip, time) ((ip)->vmt->get(ip, time))

/**
 * @brief   Channel blocking write with timeout.
 * @details The function writes data from a buffer to a channel. If the channel
 *          is not ready to accept data then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 * @param[out] bp       pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred
 * @param[in] time      the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The number of bytes transferred.
 */
#define chIOWriteTimeout(ip, bp, n, time)                                   \
  ((ip)->vmt->write(ip, bp, n, time))

/**
 * @brief   Channel blocking read with timeout.
 * @details The function reads data from a channel into a buffer. If the data
 *          is not available then the calling thread is suspended.
 *
 * @param[in] ip        pointer to a @p BaseChannel or derived class
 * @param[in] bp        pointer to the data buffer
 * @param[in] n         the maximum amount of data to be transferred
 * @param[in] time      the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The number of bytes transferred.
 */
#define chIOReadTimeout(ip, bp, n, time)                                    \
  ((ip)->vmt->read(ip, bp, n, time))

#if CH_USE_EVENTS
/**
 * @brief   @p BaseAsynchronousChannel specific methods.
 */
#define _base_asynchronous_channel_methods                                  \
  _base_channel_methods

/**
 * @brief   @p BaseAsynchronousChannel specific data.
 */
#define _base_asynchronous_channel_data                                     \
  _base_channel_data                                                        \
  /* Data Available EventSource.*/                                          \
  EventSource           ievent;                                             \
  /* Data Transmitted EventSource.*/                                        \
  EventSource           oevent;

/**
 * @brief   @p BaseAsynchronousChannel virtual methods table.
 */
struct BaseAsynchronousChannelVMT {
  _base_asynchronous_channel_methods
};

/**
 * @extends BaseChannel
 *
 * @brief   Base asynchronous channel class.
 * @details This class extends @p BaseChannel by adding event sources fields
 *          for asynchronous I/O for use in an event-driven environment.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseAsynchronousChannelVMT *vmt;
  _base_asynchronous_channel_data
} BaseAsynchronousChannel;

/**
 * @brief   Returns the write event source.
 * @details The write event source is broadcasted when the channel is ready
 *          for write operations. This usually happens when the internal
 *          output queue becomes empty.
 *
 * @param[in] ip        pointer to a @p BaseAsynchronousChannel or derived
 *                      class
 * @return              A pointer to an @p EventSource object.
 */
#define chIOGetWriteEventSource(ip) (&((ip)->vmt->oevent))

/**
 * @brief   Returns the read event source.
 * @details The read event source is broadcasted when the channel is ready
 *          for read operations. This usually happens when the internal
 *          input queue becomes non-empty.
 *
 * @param[in] ip        pointer to a @p BaseAsynchronousChannel or derived
 *                      class
 * @return              A pointer to an @p EventSource object.
 */
#define chIOGetReadEventSource(ip) (&((ip)->vmt->ievent))

#endif /* CH_USE_EVENTS */

#endif /* _CHIOCH_H_ */

/** @} */
