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
 * @file    templates/can_lld.h
 * @brief   CAN Driver subsystem low level driver header template.
 *
 * @addtogroup CAN_LLD
 * @{
 */

#ifndef _CAN_LLD_H_
#define _CAN_LLD_H_

#if CH_HAL_USE_CAN || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   This switch defines whether the driver implementation supports
 *          a low power switch mode with automatic an wakeup feature.
 */
#define CAN_SUPPORTS_SLEEP  TRUE

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Sleep mode related APIs inclusion switch.
 * @note    This switch is enforced to @p FALSE if the driver implementation
 *          does not support the sleep mode.
 */
#if CAN_SUPPORTS_SLEEP || defined(__DOXYGEN__)
#if !defined(CAN_USE_SLEEP_MODE) || defined(__DOXYGEN__)
#define CAN_USE_SLEEP_MODE  TRUE
#endif
#else /* !CAN_SUPPORTS_SLEEP */
#define CAN_USE_SLEEP_MODE  FALSE
#endif /* !CAN_SUPPORTS_SLEEP */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if CAN_USE_SLEEP_MODE && !CAN_SUPPORTS_SLEEP
#error "CAN sleep mode not supported in this architecture"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   CAN status flags.
 */
typedef uint32_t canstatus_t;

/**
 * @brief   CAN transmission frame.
 * @note    Accessing the frame data as word16 or word32 is not portable
 *          because machine data endianness, it can be still useful for a
 *          quick filling.
 */
typedef struct {
  struct {
    uint8_t                 cf_DLC:4;       /**< @brief Data length.        */
    uint8_t                 cf_RTR:1;       /**< @brief Frame type.         */
    uint8_t                 cf_IDE:1;       /**< @brief Identifier type.    */
  };
  union {
    struct {
      uint32_t              cf_SID:11;      /**< @brief Standard identifier.*/
    };
    struct {
      uint32_t              cf_EID:29;      /**< @brief Extended identifier.*/
    };
  };
  union {
    uint8_t                 cf_data8[8];    /**< @brief Frame data.         */
    uint16_t                cf_data16[4];   /**< @brief Frame data.         */
    uint32_t                cf_data32[2];   /**< @brief Frame data.         */
  };
} CANTxFrame;

/**
 * @brief   CAN received frame.
 * @note    Accessing the frame data as word16 or word32 is not portable
 *          because machine data endianness, it can be still useful for a
 *          quick filling.
 */
typedef struct {
  struct {
    uint8_t                 cf_DLC:4;       /**< @brief Data length.        */
    uint8_t                 cf_RTR:1;       /**< @brief Frame type.         */
    uint8_t                 cf_IDE:1;       /**< @brief Identifier type.    */
  };
  union {
    struct {
      uint32_t              cf_SID:11;      /**< @brief Standard identifier.*/
    };
    struct {
      uint32_t              cf_EID:29;      /**< @brief Extended identifier.*/
    };
  };
  union {
    uint8_t                 cf_data8[8];    /**< @brief Frame data.         */
    uint16_t                cf_data16[4];   /**< @brief Frame data.         */
    uint32_t                cf_data32[2];   /**< @brief Frame data.         */
  };
} CANRxFrame;

/**
 * @brief   CAN filter.
 * @note    It could not be present on some architectures.
 */
typedef struct {
} CANFilter;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
} CANConfig;

/**
 * @brief   Structure representing an CAN driver.
 */
typedef struct {
  /**
   * @brief Driver state.
   */
  canstate_t                cd_state;
  /**
   * @brief Current configuration data.
   */
  const CANConfig           *cd_config;
  /**
   * @brief Transmission queue semaphore.
   */
  Semaphore                 cd_txsem;
  /**
   * @brief Receive queue semaphore.
   */
  Semaphore                 cd_rxsem;
  /**
   * @brief One or more frames become available.
   * @note  After broadcasting this event it will not be broadcasted again
   *        until the received frames queue has been completely emptied. It
   *        is <b>not</b> broadcasted for each received frame. It is
   *        responsibility of the application to empty the queue by repeatedly
   *        invoking @p chReceive() when listening to this event. This behavior
   *        minimizes the interrupt served by the system because CAN traffic.
   */
  EventSource               cd_rxfull_event;
  /**
   * @brief One or more transmission slots become available.
   */
  EventSource               cd_txempty_event;
  /**
   * @brief A CAN bus error happened.
   */
  EventSource               cd_error_event;
  /**
   * @brief Error flags set when an error event is broadcasted.
   */
  canstatus_t               cd_status;
#if CAN_USE_SLEEP_MODE || defined (__DOXYGEN__)
  /**
   * @brief Entering sleep state event.
   */
  EventSource               cd_sleep_event;
  /**
   * @brief Exiting sleep state event.
   */
  EventSource               cd_wakeup_event;
#endif /* CAN_USE_SLEEP_MODE */
  /* End of the mandatory fields.*/
} CANDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void can_lld_init(void);
  void can_lld_start(CANDriver *canp);
  void can_lld_stop(CANDriver *canp);
  bool_t can_lld_can_transmit(CANDriver *canp);
  void can_lld_transmit(CANDriver *canp, const CANTxFrame *crfp);
  bool_t can_lld_can_receive(CANDriver *canp);
  void can_lld_receive(CANDriver *canp, CANRxFrame *ctfp);
#if CAN_USE_SLEEP_MODE
  void can_lld_sleep(CANDriver *canp);
  void can_lld_wakeup(CANDriver *canp);
#endif /* CAN_USE_SLEEP_MODE */
#ifdef __cplusplus
}
#endif

#endif /* CH_HAL_USE_CAN */

#endif /* _CAN_LLD_H_ */

/** @} */
