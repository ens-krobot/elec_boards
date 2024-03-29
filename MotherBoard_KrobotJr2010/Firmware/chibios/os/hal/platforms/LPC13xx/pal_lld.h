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
 * @file    LPC13xx/pal_lld.h
 * @brief   LPC13xx GPIO low level driver header.
 *
 * @addtogroup LPC13xx_PAL
 * @{
 */

#ifndef _PAL_LLD_H_
#define _PAL_LLD_H_

#if CH_HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/

#undef PAL_MODE_INPUT_PULLUP
#undef PAL_MODE_INPUT_PULLDOWN
#undef PAL_MODE_INPUT_ANALOG
#undef PAL_MODE_OUTPUT_OPENDRAIN

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

/**
 * @brief GPIO port setup info.
 */
typedef struct {
  /** Initial value for FIO_PIN register.*/
  uint32_t      data;
  /** Initial value for FIO_DIR register.*/
  uint32_t      dir;
} LPC13xx_gpio_setup_t;

/**
 * @brief   GPIO static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialized the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 * @note    The @p IOCON block is not configured, initially all pins have
 *          enabled pullups and are programmed as GPIO. It is responsibility
 *          of the various drivers to reprogram the pins in the proper mode.
 *          Pins that are not handled by any driver may be programmed in
 *          @p board.c.
 */
typedef struct {
  /** @brief GPIO 0 setup data.*/
  LPC13xx_gpio_setup_t   P0;
  /** @brief GPIO 1 setup data.*/
  LPC13xx_gpio_setup_t   P1;
  /** @brief GPIO 2 setup data.*/
  LPC13xx_gpio_setup_t   P2;
  /** @brief GPIO 3 setup data.*/
  LPC13xx_gpio_setup_t   P3;
} PALConfig;

/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH 32

/**
 * @brief   Whole port mask.
 * @brief   This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT ((ioportmask_t)0xFFFFFFFF)

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Port Identifier.
 */
typedef LPC_GPIO_TypeDef *ioportid_t;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/*===========================================================================*/

/**
 * @brief   GPIO0 port identifier.
 */
#define IOPORT1         LPC_GPIO0
#define GPIO0           LPC_GPIO0

/**
 * @brief   GPIO1 port identifier.
 */
#define IOPORT2         LPC_GPIO1
#define GPIO1           LPC_GPIO1

/**
 * @brief   GPIO2 port identifier.
 */
#define IOPORT3         LPC_GPIO2
#define GPIO2           LPC_GPIO2

/**
 * @brief   GPIO3 port identifier.
 */
#define IOPORT4         LPC_GPIO3
#define GPIO3           LPC_GPIO3

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @param[in] config    architecture-dependent ports configuration
 */
#define pal_lld_init(config) _pal_lld_init(config)

/**
 * @brief   Reads the physical I/O port states.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 */
#define pal_lld_readport(port) ((port)->DATA)

/**
 * @brief   Reads the output latch.
 * @details The purpose of this function is to read back the latched output
 *          value.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 *
 * @param[in] port      port identifier
 * @return              The latched logical states.
 */
#define pal_lld_readlatch(port) ((port)->DATA)

/**
 * @brief   Writes a bits mask on a I/O port.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 */
#define pal_lld_writeport(port, bits) ((port)->DATA = (bits))

/**
 * @brief   Sets a bits mask on a I/O port.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 */
#define pal_lld_setport(port, bits) ((port)->MASKED_ACCESS[bits] = 0xFFFFFFFF)

/**
 * @brief   Clears a bits mask on a I/O port.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 */
#define pal_lld_clearport(port, bits) ((port)->MASKED_ACCESS[bits] = 0)

/**
 * @brief   Reads a group of bits.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @return              The group logical states.
 */
#define pal_lld_readgroup(port, mask, offset)                               \
  ((port)->MASKED_ACCESS[(mask) << (offset)])

/**
 * @brief   Writes a group of bits.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] bits      bits to be written. Values exceeding the group width
 *                      are masked.
 */
#define pal_lld_writegroup(port, mask, offset, bits)                        \
  ((port)->MASKED_ACCESS[(mask) << (offset)] = (bits))

/**
 * @brief   Pads group mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] mode      group mode
 */
#define pal_lld_setgroupmode(port, mask, mode)                              \
  _pal_lld_setgroupmode(port, mask, mode)

/**
 * @brief   Writes a logical state on an output pad.
 * @note    This function is not meant to be invoked directly by the
 *          application  code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[out] bit      logical value, the value must be @p PAL_LOW or
 *                      @p PAL_HIGH
 */
#define pal_lld_writepad(port, pad, bit)                                    \
  ((port)->MASKED_ACCESS[(mask) << (pad)] = (bit) << (pad))

/**
 * @brief   Sets a pad logical state to @p PAL_HIGH.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 */
#define pal_lld_setpad(port, pad)                                           \
  ((port)->MASKED_ACCESS[1 << (pad)] = 1 << (pad))

/**
 * @brief   Clears a pad logical state to @p PAL_LOW.
 * @note    This function is not meant to be invoked directly by the
 *          application code.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 */
#define pal_lld_clearpad(port, pad)                                         \
  ((port)->MASKED_ACCESS[1 << (pad)] = 0)

#if !defined(__DOXYGEN__)
extern const PALConfig pal_default_config;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_init(const PALConfig *config);
  void _pal_lld_setgroupmode(ioportid_t port,
                             ioportmask_t mask,
                             uint_fast8_t mode);
#ifdef __cplusplus
}
#endif

#endif /* CH_HAL_USE_PAL */

#endif /* _PAL_LLD_H_ */

/** @} */
