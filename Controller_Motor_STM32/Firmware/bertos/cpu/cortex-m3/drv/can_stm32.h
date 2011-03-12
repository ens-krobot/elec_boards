/**
 * \file
 * <!--
 * This file is part of BeRTOS.
 *
 * Bertos is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * As a special exception, you may use this file as part of a free software
 * library without restriction.  Specifically, if other files instantiate
 * templates or use macros or inline functions from this file, or you compile
 * this file and link it with other files to produce an executable, this
 * file does not by itself cause the resulting executable to be covered by
 * the GNU General Public License.  This exception does not however
 * invalidate any other reasons why the executable file might be covered by
 * the GNU General Public License.
 *
 * Copyright © 2011 Nicolas Dandrimont <Nicolas.Dandrimont@crans.org>
 *
 * -->
 *
 * \brief CAN hardware-specific definition
 *
 * \author Nicolas Dandrimont <Nicolas.Dandrimont@crans.org
 */

#ifndef DRV_CAN_STM32_H
#define DRV_CAN_STM32_H

#include <hw/hw_cpufreq.h>

#include "cfg/cfg_can.h"

#include <cfg/compiler.h>

#include <mware/event.h>

#include <io/stm32_can.h>

#undef CAN_BTR_BRP
#undef CAN_BTR_TS1
#undef CAN_BTR_TS2
#undef CAN_BTR_SJW
#define CAN_BTR_BRP(n)              (n)         //< BRP field macro.
#define CAN_BTR_TS1(n)              ((n) << 16) //< TS1 field macro.
#define CAN_BTR_TS2(n)              ((n) << 20) //< TS2 field macro.
#define CAN_BTR_SJW(n)              ((n) << 24) //< SJW field macro.


typedef enum {
    CAN_NOTSETUP = 0,
    CAN_STOP,
    CAN_STARTING,
    CAN_READY,
    CAN_SLEEP,
} can_state;

typedef uint8_t can_errorflags;

/**
 * @brief   Errors rate warning.
 */
#define CAN_LIMIT_WARNING           1
/**
 * @brief   Errors rate error.
 */
#define CAN_LIMIT_ERROR             2
/**
 * @brief   Bus off condition reached.
 */
#define CAN_BUS_OFF_ERROR           4
/**
 * @brief   Framing error of some kind on the CAN bus.
 */
#define CAN_FRAMING_ERROR           8
/**
 * @brief   Overflow in receive queue.
 */
#define CAN_OVERFLOW_ERROR          16

#define CAN_MAX_FILTERS 14

typedef struct _can_tx_frame {
    uint8_t dlc:4;          //< Data length
    uint8_t rtr:1;          //< Remote transmission request
    uint8_t ide:1;          //< Identifier type
    union {
        uint32_t sid:11;    //< Standard identifier
        uint32_t eid:29;    //< Extended identifier
    };
    union {
        uint8_t data8[8];   //< Frame data
        uint16_t data16[4]; //< Frame data
        uint32_t data32[2]; //< Frame data
    };
} can_tx_frame;

typedef struct _can_rx_frame {
    uint8_t fmi;            //< Filter Match ID
    uint16_t time;          //< Timestamp
    uint8_t dlc:4;          //< Data length
    uint8_t rtr:1;          //< Remote transmission reauest
    uint8_t ide:1;          //< Identifier type
    union {
        uint32_t sid:11;    //< Standard identifier
        uint32_t eid:29;    //< Extended identifier
    };
    union {
        uint8_t data8[8];   //< Frame data
        uint16_t data16[4]; //< Frame data
        uint32_t data32[2]; //< Frame data
    };
} can_rx_frame;

typedef struct _can_filter {
    uint8_t mode:1;       //< Filter mode (0: mask, 1: list)
    uint8_t scale:1;      //< Filter scale (0: 16 bit, 1: 32 bit)
    uint8_t assignment:1; //< Filter assignment (0: fifo0, 1: fifo1). Forced to 0
    uint32_t register1;   //< Filter register 1
    uint32_t register2;   //< Filter register 2
} can_filter;

typedef struct _can_config {
    uint32_t mcr;              //< Contents of the MCR register
    uint32_t btr;              //< Contents of the BTR register
    uint32_t n_filters;        //< Number of filters
    const can_filter *filters; //< Contents of the filters
} can_config;

typedef struct _can_driver {
    can_state state;            //< Driver state
    const can_config *config;   //< Driver configuration data
    can_errorflags error_flags; //< Driver error flags
    Event rx_available_event;   //< One or more frames become available
    Event tx_empty_event;       //< One or more TX slots become available
    Event sleep_event;          //< Entering sleep state
    Event wakeup_event;         //< Exiting sleep state
    Event error_event;          //< A CAN error happened
    CAN *can;                   //< The CAN register structure
} can_driver;

extern can_driver *CAND1;
extern CAN *CAN1;

void can_hw_init(void);
void can_hw_start(can_driver *drv);
void can_hw_stop(can_driver *drv);
void can_hw_sleep(can_driver *drv);
void can_hw_wakeup(can_driver *drv);
bool can_hw_can_transmit(can_driver *drv);
bool can_hw_can_receive(can_driver *drv);
void can_hw_transmit(can_driver *drv, const can_tx_frame *frame);
void can_hw_receive(can_driver *drv, can_rx_frame *frame);


#endif /* DRV_CAN_STM32_H */
