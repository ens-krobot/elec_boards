/**
 * AX12 Library
 *
 * This file contains the interface for AX12 communication
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

#include "ax12.h"

/*
 * Compute the checksum of an AX12 packet
 */
static uint8_t compute_checksum(ax12_cmd_packet *pkt) {
    uint8_t cksum = 0;
    uint8_t i;

    cksum = pkt->address + pkt->command + pkt->length + 2;

    for (i = 0; i < pkt->length; i++)
        cksum += pkt->args[i];

    cksum = ~cksum;

    return cksum;
}

/*
 * Verify the checksum of an AX12 status packet
 */
static int verify_checksum(ax12_st_packet *pkt, uint8_t cksum) {
    uint8_t pkt_cksum = 0;
    uint8_t i;

    pkt_cksum = pkt->address + pkt->error.i + pkt->length + 2;

    for (i = 0; i < pkt->length; i++)
        pkt_cksum += pkt->args[i];

    pkt_cksum = ~pkt_cksum;

    return (cksum == pkt_cksum);
}

/*
 * Write the packet to the serial line
 */
void ax12_write(ax12_cmd_packet *pkt) {
    uint8_t cksum = compute_checksum(pkt);

    int i;

    serial_putchar(0xff);
    serial_putchar(0xff);
    serial_putchar(pkt->address);
    serial_putchar(pkt->length + 2);
    serial_putchar(pkt->command);
    for (i = 0; i < pkt->length; i++)
        serial_putchar(pkt->args[i]);
    serial_putchar(cksum);

    // Discard echo data...
    ax12_read(NULL);
}

typedef enum {
    ST_START1,
    ST_START2,
    ST_ID,
    ST_LEN,
    ST_ERR,
    ST_ARGS,
    ST_CKSUM,
    ST_END
} read_state;

/*
 * Read a packet from the serial line
 * Returns :
 * - 0 on success;
 * - -1 on failure to verify the checksum
 * - -2 on failure to read the packet
 *
 * if pkt is NULL, discards the data and returns 0.
 */
int ax12_read(ax12_st_packet *pkt) {

    read_state state = ST_START1;
    uint8_t args_ctr = 0;

    int chr = -1;

    uint8_t length;

    int ret = 0;

    for (;;) {
        if (state == ST_END)
            break;

        chr = serial_getchar();

        if (state == ST_START1 && chr < 0) {
            ret = -2;
            break;
        }

        switch (state) {
        case ST_START1:
            if(chr == 0xFF)
                state = ST_START2;
            break;
        case ST_START2:
            if(chr == 0xFF)
                state = ST_ID;
            break;
        case ST_ID:
            if (pkt)
                pkt->address = chr;
            state = ST_LEN;
            break;
        case ST_LEN:
            if (pkt)
                pkt->length = chr - 2;
            length = chr - 2;
            state = ST_ERR;
            break;
        case ST_ERR:
            if (pkt)
                pkt->error.i = chr;
            if (length)
                state = ST_ARGS;
            else
                state = ST_CKSUM;
            break;
        case ST_ARGS:
            if (pkt)
                pkt->args[args_ctr] = chr;
            args_ctr++;
            if (length == args_ctr)
                state = ST_CKSUM;
            break;
        case ST_CKSUM:
            if (!pkt) {
                ret = 0;
            } else {
                if (verify_checksum(pkt, chr))
                    ret = 0;
                else
                    ret = -1;
            }
            state = ST_END;
            break;
        case ST_END:
            break;
        }
    }

    return ret;
}
