/**
 * USB-CAN converter
 *
 * This file contains the logic for the USB <-> CAN transceiver.
 *
 * Copyright © 2011 Nicolas Dandrimont <olasd@crans.org>
 * Authors: Nicolas Dandrimont <olasd@crans.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef USB_CAN_H
#define USB_CAN_H

#include <stdio.h>
#include <string.h>

#include <kern/sem.h>
#include <drv/can.h>
#include <drv/timer.h>
#include <hw/hw_led.h>
#include <io/kfile.h>

#include "serial.h"

typedef uint16_t (usb_can_timestamp(void));

typedef struct _usb_can {
    can_driver *can;      // CAN Device
    bool is_open;         // Channel open?
    bool timestamped;     // Emit timestamps?
    usb_can_timestamp *get_timestamp; // Get Timestamp function
    struct Semaphore sem_receive;    // Avoid receiving when tx_ing
} usb_can;

uint16_t get_timestamp(void);

usb_can *usb_can_init(can_driver *can);

void usb_can_open(usb_can *usbcan);
void usb_can_close(usb_can *usbcan);

void usb_can_set_baudrate(usb_can *usbcan, char *baudrate);

int usb_can_execute_command(usb_can *usbcan, char *command, size_t len);

int usb_can_emit(usb_can *usbcan, can_rx_frame *frame);

#endif /* !USB_CAN_H */
