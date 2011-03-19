#include "usb_can.h"

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


int usb_can_execute_command(can_driver *candrv, struct Serial *serial, char *command) {

    can_tx_frame frame;
    bool send, ret;
    int i;

    frame.rtr = 0;
    frame.ide = 0;
    frame.eid = 0;
    frame.sid = 0;
    frame.dlc = 0;
    frame.data32[0] = 0;
    frame.data32[1] = 0;

    switch (command[0]) {
      case 'V':
        /* Version number */
        kfile_write(&serial->fd, "V0402\r", 6);
        break;
      case 'N':
        /* Serial number */
        kfile_write(&serial->fd, "NKROB\r", 6);
        break;
      case 'O':
        /* Open CAN Channel */
        /* FIXME: Implement! */
        break;
      case 'S':
        /* Set CAN Channel Baudrate numerically */
        /* FIXME: Implement! */
        break;
      case 's':
        /* Set CAN Channel Baudrate with BTR0/1 */
        /* FIXME: Implement! */
        break;
      case 'R':
        /* Send an extended RTR frame */
        frame.rtr = 1;
        frame.ide = 1;
        frame.eid = nibble_to_uint32(&command[1], 8);
        frame.dlc = nibble_to_uint32(&command[9], 1);
        for (i = 0; i < frame.dlc; i++)
            frame.data8[i] = nibble_to_uint32(&command[10 + 2*i], 2);
        send = true;
        break;
      case 'r':
        /* Send a classic RTR frame */
        frame.rtr = 1;
        frame.sid = nibble_to_uint32(&command[1], 3);
        frame.dlc = nibble_to_uint32(&command[4], 1);
        for (i = 0; i < frame.dlc; i++)
            frame.data8[i] = nibble_to_uint32(&command[5 + 2*i], 2);
        send = true;
        break;
      case 'T':
        frame.ide = 1;
        frame.eid = nibble_to_uint32(&command[1], 8);
        frame.dlc = nibble_to_uint32(&command[9], 1);
        for (i = 0; i < frame.dlc; i++)
            frame.data8[i] = nibble_to_uint32(&command[10 + 2*i], 2);
        send = true;
        break;
      case 't':
        frame.sid = nibble_to_uint32(&command[1], 3);
        frame.dlc = nibble_to_uint32(&command[4], 1);
        for (i = 0; i < frame.dlc; i++)
            frame.data8[i] = nibble_to_uint32(&command[5 + 2*i], 2);
        send = true;
        break;
      default:
        break;
    }

    if (send) {
        ret = can_transmit(candrv, &frame, ms_to_ticks(10));
        if (ret)
            kfile_write(&serial->fd, frame.ide ? "Z\r" : "z\r", 2);
        else
            kfile_write(&serial->fd, "\a", 1);
    }

    return 0;
}

int usb_can_emit(UNUSED_ARG(can_driver *, candrv), struct Serial *serial, can_rx_frame *frame) {

    char buffer[32] = "";
    int i = 0, j = 0;
    uint16_t timestamp;

    /* The timestamp should wrap around every minute (it works until
       the 32 bits of timer_clock are exhausted) */
    timestamp = ticks_to_ms(timer_clock()) % 60000;

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

    sprintf(&buffer[i], "%04x", timestamp);
    i+=4;

    buffer[i] = '\r';
    i++;

    kfile_write(&serial->fd, buffer, i);

    return 0;
}
