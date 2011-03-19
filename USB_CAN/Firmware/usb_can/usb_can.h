/*
 * USB CAN specific logic
 * Copyright © 2011 Nicolas Dandrimont <olasd@crans.org>
 * License : GPLv3+
 */

#ifndef USB_CAN_H
#define USB_CAN_H

#include <stdio.h>

#include <drv/can.h>
#include <drv/ser.h>
#include <drv/timer.h>
#include <io/kfile.h>

int usb_can_execute_command(can_driver *candrv, struct Serial *serial, char *command);

int usb_can_emit(can_driver *candrv, struct Serial *serial, can_rx_frame *frame);

#endif /* !USB_CAN_H */
