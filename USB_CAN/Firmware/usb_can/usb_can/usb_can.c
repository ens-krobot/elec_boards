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

#include "usb_can.h"

PROC_DEFINE_STACK(stack_ser_recv, KERN_MINSTACKSIZE * 4);
PROC_DEFINE_STACK(stack_can_recv, KERN_MINSTACKSIZE * 4);

static usb_can _usbcan;
static usb_can *usbcan = &_usbcan;

static void NORETURN can_receive_process(void);
static void NORETURN serial_receive_process(void);

#define MAX_CMD_SIZE 64

INLINE uint32_t nibble_to_uint32(char *nibbles, size_t length) {

    char *p;
    uint32_t ret = 0;
    uint8_t tmp = 0;

    for (p = nibbles; p < nibbles + length; p++) {
        ret = (ret << 4);
        if ('0' <= *p && *p <= '9')
            tmp = *p - '0';
        else if ('A' <= *p && *p <= 'F')
            tmp = (*p - 'A') + 10;
        else if ('a' <= *p && *p <= 'f')
            tmp = (*p - 'a') + 10;
        else
            tmp = 0;
        ret = ret + tmp;
    }

    return ret;
}

uint16_t get_timestamp(void) {
    /* The timestamp should wrap around every minute (it works until
       the 32 bits of timer_clock are exhausted) */
    return ticks_to_ms(timer_clock()) % 60000;
}

usb_can *usb_can_init(can_driver *can) {

    usbcan->can = can;
    usbcan->is_open = false;
    usbcan->timestamped = false;
    usbcan->get_timestamp = get_timestamp;
    sem_init(&usbcan->sem_receive);

    proc_new(serial_receive_process, NULL, sizeof(stack_ser_recv), stack_ser_recv);
    proc_new(can_receive_process, NULL, sizeof(stack_can_recv), stack_can_recv);

    return usbcan;
}

void usb_can_open(usb_can *usbcan) {
    if (usbcan->is_open)
        serial_transmit("\a", 1);
    else {
        usbcan->is_open = true;
        serial_transmit("\r", 1);
    }
}

void usb_can_close(usb_can *usbcan) {
    if (usbcan->is_open) {
        usbcan->is_open = false;
        serial_transmit("\r", 1);
    }
    else {
        serial_transmit("\a", 1);
    }
}

void usb_can_set_baudrate(usb_can *usbcan, char *baudrate) {
    /* FIXME: Implement it for real! */
    (void)baudrate;
    (void)usbcan;
    serial_transmit("\r", 1);
}

int usb_can_execute_command(usb_can *usbcan, char *command, size_t len) {

    can_tx_frame frame;
    bool send = false, ret = false, broken = false;
    int i;

    can_rx_frame error_frame;

    frame.rtr = 0;
    frame.ide = 0;
    frame.eid = 0;
    frame.sid = 0;
    frame.dlc = 0;
    frame.data32[0] = 0;
    frame.data32[1] = 0;

    error_frame.rtr = 0;
    error_frame.ide = 1;
    error_frame.eid = 0x4242;
    error_frame.dlc = 0;

    while(!sem_attempt(&usbcan->sem_receive))
        cpu_relax();

    switch (command[0]) {
      case 'R':
        /* Send an extended RTR frame */
        frame.rtr = 1;
        /* Fallthrough */
      case 'T':
        frame.ide = 1;
        frame.eid = nibble_to_uint32(&command[1], 8);
        frame.dlc = nibble_to_uint32(&command[9], 1);
        for (i = 0; i < frame.dlc; i++)
            frame.data8[i] = nibble_to_uint32(&command[10 + 2*i], 2);
        if (len == (size_t)(9 + 1 + 2*frame.dlc))
            send = true;
        else
            broken = true;
        break;
      case 'r':
        /* Send a classic RTR frame */
        frame.rtr = 1;
        /* Fallthrough */
      case 't':
        frame.sid = nibble_to_uint32(&command[1], 3);
        frame.dlc = nibble_to_uint32(&command[4], 1);
        for (i = 0; i < frame.dlc; i++)
            frame.data8[i] = nibble_to_uint32(&command[5 + 2*i], 2);
        if (len == (size_t)(4 + 1 + 2*frame.dlc))
            send = true;
        else
            broken = true;
        break;
      case 'V':
        /* Version number */
        serial_transmit("V0402\r", strlen("V0402\r"));
        break;
      case 'N':
        /* Serial number */
        serial_transmit("NKROB\r", strlen("NKROB\r"));
        break;
      case 'O':
        /* Open CAN Channel */
        usb_can_open(usbcan);
        break;
      case 'S':
        /* Set CAN Channel Baudrate numerically */
      case 's':
        /* Set CAN Channel Baudrate with BTR0/1 */
        usb_can_set_baudrate(usbcan, command);
        break;
      case 'Z':
        /* Set Timestamped packet mode */
        if (command[1] == '0')
            usbcan->timestamped = false;
        else if (command[0] == '1')
            usbcan->timestamped = true;
        serial_transmit("\r", 1);
        break;
      default:
        break;
    }

    if (send) {
        if (!broken) {
            ret = can_transmit(usbcan->can, &frame, ms_to_ticks(10));
            if (ret)
                serial_transmit("\r", 1);
            else
                serial_transmit("\a", 1);
        } else {
            serial_transmit("\a", 1);
        }
    }

    // You are in a maze of twisty little deadlocks, all alike
    sem_release(&usbcan->sem_receive);

    if (send && (broken || !ret))
        usb_can_emit(usbcan, &error_frame);

    return 0;
}

int usb_can_emit(usb_can *usbcan, can_rx_frame *frame) {

    char buffer[32] = "";
    int i = 0, j = 0;
    uint16_t timestamp;

    // Do not interfere with the send ACK
    while(!sem_attempt(&usbcan->sem_receive))
        cpu_relax();

    buffer[0] = frame->rtr ? 'r' : 't';

    // Extended identifier : uppercase identifier
    if (frame->ide) {
        buffer[0] += 'A' - 'a';
        sprintf(&buffer[1], "%08x", frame->eid);
        i = 9;
    }
    else {
        sprintf(&buffer[1], "%03x", frame->sid);
        i = 4;
    }

    sprintf(&buffer[i], "%01d", frame->dlc);
    i++;

    for(j = 0; j < frame->dlc; i+=2, j++)
        sprintf(&buffer[i], "%02x", frame->data8[j]);

    if (usbcan->timestamped) {
        timestamp = usbcan->get_timestamp();

        sprintf(&buffer[i], "%04x", timestamp);
        i+=4;
    }

    buffer[i] = '\r';
    i++;

    serial_transmit(buffer, i);
    //serial_transmit("T000044420\r", strlen("T000044420\r"));

    sem_release(&usbcan->sem_receive);
    return 0;
}

static void NORETURN serial_receive_process(void)
{
    int nbytes = 0, retval, i = 0;
    char command[MAX_CMD_SIZE];

    for (;;) {
        i = !i;
        nbytes = serial_readline(command, MAX_CMD_SIZE);
        retval = usb_can_execute_command(usbcan, command, nbytes);
        if (i)
            LED1_ON();
        else
            LED1_OFF();
        cpu_relax();
    }
}

static void NORETURN can_receive_process(void) {

    can_rx_frame frame;
    int retval;
    bool received = false;
    bool i = false;

    for (;;) {
        received = can_receive(usbcan->can, &frame, ms_to_ticks(100));
        if (received) {
            i = !i;
            retval = usb_can_emit(usbcan, &frame);
            if (i)
                LED2_ON();
            else
                LED2_OFF();
        }
    }
}

