#include "usb_can.h"

int usb_can_execute_command(UNUSED_ARG(can_driver *, candrv), UNUSED_ARG(struct Serial *, serial), UNUSED_ARG(char *, command)) {

    return 0;
}

int usb_can_emit(UNUSED_ARG(can_driver *, candrv), struct Serial *serial, can_rx_frame *frame, uint16_t timestamp) {

    char buffer[32] = "";
    int i = 0, j = 0;

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
    i+=2;

    buffer[i] = '\r';
    i++;

    kfile_write(&serial->fd, buffer, i);

    return 0;
}
