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
 * Copyright © 2010 Nicolas Dandrimont <Nicolas.Dandrimont@crans.org>
 *
 * -->
 * \author Nicolas Dandrimont <Nicolas.Dandrimont@crans.org>
 *
 * \brief STM32F103xx CAN register definitions.
 */

#ifndef IO_STM32_CAN_H
#define IO_STM32_CAN_H

#include <cpu/types.h>


/**
 * \{
 * \name CAN registers data structures
 */
/**
  * CAN Transmit MailBox
  */
typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox;

/**
  * CAN Receive FIFO MailBox
  */
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox;

/**
  * CAN Filter Register
  */
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister;

/**
  * CAN Register structure
  */
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox sTxMailBox[3];
  CAN_FIFOMailBox sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];
  CAN_FilterRegister sFilterRegister[14];
} CAN;
/*\}*/

/**
 * \{
 * \name CAN registers bit definitions
 */
/**
 * \{
 * \name CAN control and status registers
 */
/**
 * \{
 * \name CAN_MCR register
 */
#define  CAN_MCR_INRQ         ((uint16_t)0x0001)     ///< Initialization Request
#define  CAN_MCR_SLEEP        ((uint16_t)0x0002)     ///< Sleep Mode Request
#define  CAN_MCR_TXFP         ((uint16_t)0x0004)     ///< Transmit FIFO Priority
#define  CAN_MCR_RFLM         ((uint16_t)0x0008)     ///< Receive FIFO Locked Mode
#define  CAN_MCR_NART         ((uint16_t)0x0010)     ///< No Automatic Retransmission
#define  CAN_MCR_AWUM         ((uint16_t)0x0020)     ///< Automatic Wakeup Mode
#define  CAN_MCR_ABOM         ((uint16_t)0x0040)     ///< Automatic Bus-Off Management
#define  CAN_MCR_TTCM         ((uint16_t)0x0080)     ///< Time Triggered Communication Mode
#define  CAN_MCR_RESET        ((uint16_t)0x8000)     ///< bxCAN software master reset
/*\}*/

/**
 * \{
 * \name CAN_MSR register
 */
#define  CAN_MSR_INAK         ((uint16_t)0x0001)     ///< Initialization Acknowledge
#define  CAN_MSR_SLAK         ((uint16_t)0x0002)     ///< Sleep Acknowledge
#define  CAN_MSR_ERRI         ((uint16_t)0x0004)     ///< Error Interrupt
#define  CAN_MSR_WKUI         ((uint16_t)0x0008)     ///< Wakeup Interrupt
#define  CAN_MSR_SLAKI        ((uint16_t)0x0010)     ///< Sleep Acknowledge Interrupt
#define  CAN_MSR_TXM          ((uint16_t)0x0100)     ///< Transmit Mode
#define  CAN_MSR_RXM          ((uint16_t)0x0200)     ///< Receive Mode
#define  CAN_MSR_SAMP         ((uint16_t)0x0400)     ///< Last Sample Point
#define  CAN_MSR_RX           ((uint16_t)0x0800)     ///< CAN Rx Signal
/*\}*/

/**
 * \{
 * \name CAN_TSR register
 */
#define  CAN_TSR_RQCP0        ((uint32_t)0x00000001) ///< Request Completed Mailbox0
#define  CAN_TSR_TXOK0        ((uint32_t)0x00000002) ///< Transmission OK of Mailbox0
#define  CAN_TSR_ALST0        ((uint32_t)0x00000004) ///< Arbitration Lost for Mailbox0
#define  CAN_TSR_TERR0        ((uint32_t)0x00000008) ///< Transmission Error of Mailbox0
#define  CAN_TSR_ABRQ0        ((uint32_t)0x00000080) ///< Abort Request for Mailbox0
#define  CAN_TSR_RQCP1        ((uint32_t)0x00000100) ///< Request Completed Mailbox1
#define  CAN_TSR_TXOK1        ((uint32_t)0x00000200) ///< Transmission OK of Mailbox1
#define  CAN_TSR_ALST1        ((uint32_t)0x00000400) ///< Arbitration Lost for Mailbox1
#define  CAN_TSR_TERR1        ((uint32_t)0x00000800) ///< Transmission Error of Mailbox1
#define  CAN_TSR_ABRQ1        ((uint32_t)0x00008000) ///< Abort Request for Mailbox 1
#define  CAN_TSR_RQCP2        ((uint32_t)0x00010000) ///< Request Completed Mailbox2
#define  CAN_TSR_TXOK2        ((uint32_t)0x00020000) ///< Transmission OK of Mailbox 2
#define  CAN_TSR_ALST2        ((uint32_t)0x00040000) ///< Arbitration Lost for mailbox 2
#define  CAN_TSR_TERR2        ((uint32_t)0x00080000) ///< Transmission Error of Mailbox 2
#define  CAN_TSR_ABRQ2        ((uint32_t)0x00800000) ///< Abort Request for Mailbox 2
#define  CAN_TSR_CODE         ((uint32_t)0x03000000) ///< Mailbox Code

#define  CAN_TSR_TME          ((uint32_t)0x1C000000) ///< TME[2:0] bits
#define  CAN_TSR_TME0         ((uint32_t)0x04000000) ///< Transmit Mailbox 0 Empty
#define  CAN_TSR_TME1         ((uint32_t)0x08000000) ///< Transmit Mailbox 1 Empty
#define  CAN_TSR_TME2         ((uint32_t)0x10000000) ///< Transmit Mailbox 2 Empty

#define  CAN_TSR_LOW          ((uint32_t)0xE0000000) ///< LOW[2:0] bits
#define  CAN_TSR_LOW0         ((uint32_t)0x20000000) ///< Lowest Priority Flag for Mailbox 0
#define  CAN_TSR_LOW1         ((uint32_t)0x40000000) ///< Lowest Priority Flag for Mailbox 1
#define  CAN_TSR_LOW2         ((uint32_t)0x80000000) ///< Lowest Priority Flag for Mailbox 2
/*\}*/

/**
 * \{
 * \name CAN_RF0R register
 */
#define  CAN_RF0R_FMP0        ((uint8_t)0x03)        ///< FIFO 0 Message Pending
#define  CAN_RF0R_FULL0       ((uint8_t)0x08)        ///< FIFO 0 Full
#define  CAN_RF0R_FOVR0       ((uint8_t)0x10)        ///< FIFO 0 Overrun
#define  CAN_RF0R_RFOM0       ((uint8_t)0x20)        ///< Release FIFO 0 Output Mailbox
/*\}*/

/**
 * \{
 * \name CAN_RF1R register
 */
#define  CAN_RF1R_FMP1        ((uint8_t)0x03)        ///< FIFO 1 Message Pending
#define  CAN_RF1R_FULL1       ((uint8_t)0x08)        ///< FIFO 1 Full
#define  CAN_RF1R_FOVR1       ((uint8_t)0x10)        ///< FIFO 1 Overrun
#define  CAN_RF1R_RFOM1       ((uint8_t)0x20)        ///< Release FIFO 1 Output Mailbox
/*\}*/

/**
 * \{
 * \name CAN_IER register
 **/
#define  CAN_IER_TMEIE        ((uint32_t)0x00000001) ///< Transmit Mailbox Empty Interrupt Enable
#define  CAN_IER_FMPIE0       ((uint32_t)0x00000002) ///< FIFO Message Pending Interrupt Enable
#define  CAN_IER_FFIE0        ((uint32_t)0x00000004) ///< FIFO Full Interrupt Enable
#define  CAN_IER_FOVIE0       ((uint32_t)0x00000008) ///< FIFO Overrun Interrupt Enable
#define  CAN_IER_FMPIE1       ((uint32_t)0x00000010) ///< FIFO Message Pending Interrupt Enable
#define  CAN_IER_FFIE1        ((uint32_t)0x00000020) ///< FIFO Full Interrupt Enable
#define  CAN_IER_FOVIE1       ((uint32_t)0x00000040) ///< FIFO Overrun Interrupt Enable
#define  CAN_IER_EWGIE        ((uint32_t)0x00000100) ///< Error Warning Interrupt Enable
#define  CAN_IER_EPVIE        ((uint32_t)0x00000200) ///< Error Passive Interrupt Enable
#define  CAN_IER_BOFIE        ((uint32_t)0x00000400) ///< Bus-Off Interrupt Enable
#define  CAN_IER_LECIE        ((uint32_t)0x00000800) ///< Last Error Code Interrupt Enable
#define  CAN_IER_ERRIE        ((uint32_t)0x00008000) ///< Error Interrupt Enable
#define  CAN_IER_WKUIE        ((uint32_t)0x00010000) ///< Wakeup Interrupt Enable
#define  CAN_IER_SLKIE        ((uint32_t)0x00020000) ///< Sleep Interrupt Enable
/*\}*/

/**
 * \{
 * \name CAN_ESR register
 */
#define  CAN_ESR_EWGF         ((uint32_t)0x00000001) ///< Error Warning Flag
#define  CAN_ESR_EPVF         ((uint32_t)0x00000002) ///< Error Passive Flag
#define  CAN_ESR_BOFF         ((uint32_t)0x00000004) ///< Bus-Off Flag

#define  CAN_ESR_LEC          ((uint32_t)0x00000070) ///< LEC[2:0] bits (Last Error Code)
#define  CAN_ESR_LEC_0        ((uint32_t)0x00000010) ///< Bit 0
#define  CAN_ESR_LEC_1        ((uint32_t)0x00000020) ///< Bit 1
#define  CAN_ESR_LEC_2        ((uint32_t)0x00000040) ///< Bit 2

#define  CAN_ESR_TEC          ((uint32_t)0x00FF0000) ///< Least significant byte of the 9-bit Transmit Error Counter
#define  CAN_ESR_REC          ((uint32_t)0xFF000000) ///< Receive Error Counter
/*\}*/

/**
 * \{
 * \name CAN_BTR register
 */
#define  CAN_BTR_BRP          ((uint32_t)0x000003FF) ///< Baud Rate Prescaler
#define  CAN_BTR_TS1          ((uint32_t)0x000F0000) ///< Time Segment 1
#define  CAN_BTR_TS2          ((uint32_t)0x00700000) ///< Time Segment 2
#define  CAN_BTR_SJW          ((uint32_t)0x03000000) ///< Resynchronization Jump Width
#define  CAN_BTR_LBKM         ((uint32_t)0x40000000) ///< Loop Back Mode (Debug)
#define  CAN_BTR_SILM         ((uint32_t)0x80000000) ///< Silent Mode
/*\}*/
/*\}*/

/**
 * \{
 * \name Mailbox registers
 */
/**
 * \{
 * \name CAN_TI0R register
 */
#define  CAN_TI0R_TXRQ        ((uint32_t)0x00000001) ///< Transmit Mailbox Request
#define  CAN_TI0R_RTR         ((uint32_t)0x00000002) ///< Remote Transmission Request
#define  CAN_TI0R_IDE         ((uint32_t)0x00000004) ///< Identifier Extension
#define  CAN_TI0R_EXID        ((uint32_t)0x001FFFF8) ///< Extended Identifier
#define  CAN_TI0R_STID        ((uint32_t)0xFFE00000) ///< Standard Identifier or Extended Identifier
/*\}*/

/**
 * \{
 * \name CAN_TDT0R register
 */
#define  CAN_TDT0R_DLC        ((uint32_t)0x0000000F) ///< Data Length Code
#define  CAN_TDT0R_TGT        ((uint32_t)0x00000100) ///< Transmit Global Time
#define  CAN_TDT0R_TIME       ((uint32_t)0xFFFF0000) ///< Message Time Stamp
/*\}*/

/**
 * \{
 * \name CAN_TDL0R register
 */
#define  CAN_TDL0R_DATA0      ((uint32_t)0x000000FF) ///< Data byte 0
#define  CAN_TDL0R_DATA1      ((uint32_t)0x0000FF00) ///< Data byte 1
#define  CAN_TDL0R_DATA2      ((uint32_t)0x00FF0000) ///< Data byte 2
#define  CAN_TDL0R_DATA3      ((uint32_t)0xFF000000) ///< Data byte 3
/*\}*/

/**
 * \{
 * \name CAN_TDH0R register
 */
#define  CAN_TDH0R_DATA4      ((uint32_t)0x000000FF) ///< Data byte 4
#define  CAN_TDH0R_DATA5      ((uint32_t)0x0000FF00) ///< Data byte 5
#define  CAN_TDH0R_DATA6      ((uint32_t)0x00FF0000) ///< Data byte 6
#define  CAN_TDH0R_DATA7      ((uint32_t)0xFF000000) ///< Data byte 7
/*\}*/

/**
 * \{
 * \name CAN_TI1R register
 */
#define  CAN_TI1R_TXRQ        ((uint32_t)0x00000001) ///< Transmit Mailbox Request
#define  CAN_TI1R_RTR         ((uint32_t)0x00000002) ///< Remote Transmission Request
#define  CAN_TI1R_IDE         ((uint32_t)0x00000004) ///< Identifier Extension
#define  CAN_TI1R_EXID        ((uint32_t)0x001FFFF8) ///< Extended Identifier
#define  CAN_TI1R_STID        ((uint32_t)0xFFE00000) ///< Standard Identifier or Extended Identifier
/*\}*/

/**
 * \{
 * \name CAN_TDT1R register
 */
#define  CAN_TDT1R_DLC        ((uint32_t)0x0000000F) ///< Data Length Code
#define  CAN_TDT1R_TGT        ((uint32_t)0x00000100) ///< Transmit Global Time
#define  CAN_TDT1R_TIME       ((uint32_t)0xFFFF0000) ///< Message Time Stamp
/*\}*/

/**
 * \{
 * \name CAN_TDL1R register
 */
#define  CAN_TDL1R_DATA0      ((uint32_t)0x000000FF) ///< Data byte 0
#define  CAN_TDL1R_DATA1      ((uint32_t)0x0000FF00) ///< Data byte 1
#define  CAN_TDL1R_DATA2      ((uint32_t)0x00FF0000) ///< Data byte 2
#define  CAN_TDL1R_DATA3      ((uint32_t)0xFF000000) ///< Data byte 3
/*\}*/

/**
 * \{
 * \name CAN_TDH1R register
 */
#define  CAN_TDH1R_DATA4      ((uint32_t)0x000000FF) ///< Data byte 4
#define  CAN_TDH1R_DATA5      ((uint32_t)0x0000FF00) ///< Data byte 5
#define  CAN_TDH1R_DATA6      ((uint32_t)0x00FF0000) ///< Data byte 6
#define  CAN_TDH1R_DATA7      ((uint32_t)0xFF000000) ///< Data byte 7
/*\}*/

/**
 * \{
 * \name CAN_TI2R register
 */
#define  CAN_TI2R_TXRQ        ((uint32_t)0x00000001) ///< Transmit Mailbox Request
#define  CAN_TI2R_RTR         ((uint32_t)0x00000002) ///< Remote Transmission Request
#define  CAN_TI2R_IDE         ((uint32_t)0x00000004) ///< Identifier Extension
#define  CAN_TI2R_EXID        ((uint32_t)0x001FFFF8) ///< Extended identifier
#define  CAN_TI2R_STID        ((uint32_t)0xFFE00000) ///< Standard Identifier or Extended Identifier
/*\}*/

/**
 * \{
 * \name CAN_TDT2R register
 */
#define  CAN_TDT2R_DLC        ((uint32_t)0x0000000F) ///< Data Length Code
#define  CAN_TDT2R_TGT        ((uint32_t)0x00000100) ///< Transmit Global Time
#define  CAN_TDT2R_TIME       ((uint32_t)0xFFFF0000) ///< Message Time Stamp
/*\}*/

/**
 * \{
 * \name CAN_TDL2R register
 */
#define  CAN_TDL2R_DATA0      ((uint32_t)0x000000FF) ///< Data byte 0
#define  CAN_TDL2R_DATA1      ((uint32_t)0x0000FF00) ///< Data byte 1
#define  CAN_TDL2R_DATA2      ((uint32_t)0x00FF0000) ///< Data byte 2
#define  CAN_TDL2R_DATA3      ((uint32_t)0xFF000000) ///< Data byte 3
/*\}*/

/**
 * \{
 * \name CAN_TDH2R register
 */
#define  CAN_TDH2R_DATA4      ((uint32_t)0x000000FF) ///< Data byte 4
#define  CAN_TDH2R_DATA5      ((uint32_t)0x0000FF00) ///< Data byte 5
#define  CAN_TDH2R_DATA6      ((uint32_t)0x00FF0000) ///< Data byte 6
#define  CAN_TDH2R_DATA7      ((uint32_t)0xFF000000) ///< Data byte 7
/*\}*/

/**
 * \{
 * \name CAN_RI0R register
 */
#define  CAN_RI0R_RTR         ((uint32_t)0x00000002) ///< Remote Transmission Request
#define  CAN_RI0R_IDE         ((uint32_t)0x00000004) ///< Identifier Extension
#define  CAN_RI0R_EXID        ((uint32_t)0x001FFFF8) ///< Extended Identifier
#define  CAN_RI0R_STID        ((uint32_t)0xFFE00000) ///< Standard Identifier or Extended Identifier
/*\}*/

/**
 * \{
 * \name CAN_RDT0R register
 */
#define  CAN_RDT0R_DLC        ((uint32_t)0x0000000F) ///< Data Length Code
#define  CAN_RDT0R_FMI        ((uint32_t)0x0000FF00) ///< Filter Match Index
#define  CAN_RDT0R_TIME       ((uint32_t)0xFFFF0000) ///< Message Time Stamp
/*\}*/

/**
 * \{
 * \name CAN_RDL0R register
 */
#define  CAN_RDL0R_DATA0      ((uint32_t)0x000000FF) ///< Data byte 0
#define  CAN_RDL0R_DATA1      ((uint32_t)0x0000FF00) ///< Data byte 1
#define  CAN_RDL0R_DATA2      ((uint32_t)0x00FF0000) ///< Data byte 2
#define  CAN_RDL0R_DATA3      ((uint32_t)0xFF000000) ///< Data byte 3
/*\}*/

/**
 * \{
 * \name CAN_RDH0R register
 */
#define  CAN_RDH0R_DATA4      ((uint32_t)0x000000FF) ///< Data byte 4
#define  CAN_RDH0R_DATA5      ((uint32_t)0x0000FF00) ///< Data byte 5
#define  CAN_RDH0R_DATA6      ((uint32_t)0x00FF0000) ///< Data byte 6
#define  CAN_RDH0R_DATA7      ((uint32_t)0xFF000000) ///< Data byte 7
/*\}*/

/**
 * \{
 * \name CAN_RI1R register
 */
#define  CAN_RI1R_RTR         ((uint32_t)0x00000002) ///< Remote Transmission Request
#define  CAN_RI1R_IDE         ((uint32_t)0x00000004) ///< Identifier Extension
#define  CAN_RI1R_EXID        ((uint32_t)0x001FFFF8) ///< Extended identifier
#define  CAN_RI1R_STID        ((uint32_t)0xFFE00000) ///< Standard Identifier or Extended Identifier
/*\}*/

/**
 * \{
 * \name CAN_RDT1R register
 */
#define  CAN_RDT1R_DLC        ((uint32_t)0x0000000F) ///< Data Length Code
#define  CAN_RDT1R_FMI        ((uint32_t)0x0000FF00) ///< Filter Match Index
#define  CAN_RDT1R_TIME       ((uint32_t)0xFFFF0000) ///< Message Time Stamp
/*\}*/

/**
 * \{
 * \name CAN_RDL1R register
 */
#define  CAN_RDL1R_DATA0      ((uint32_t)0x000000FF) ///< Data byte 0
#define  CAN_RDL1R_DATA1      ((uint32_t)0x0000FF00) ///< Data byte 1
#define  CAN_RDL1R_DATA2      ((uint32_t)0x00FF0000) ///< Data byte 2
#define  CAN_RDL1R_DATA3      ((uint32_t)0xFF000000) ///< Data byte 3
/*\}*/

/**
 * \{
 * \name CAN_RDH1R register
 */
#define  CAN_RDH1R_DATA4      ((uint32_t)0x000000FF) ///< Data byte 4
#define  CAN_RDH1R_DATA5      ((uint32_t)0x0000FF00) ///< Data byte 5
#define  CAN_RDH1R_DATA6      ((uint32_t)0x00FF0000) ///< Data byte 6
#define  CAN_RDH1R_DATA7      ((uint32_t)0xFF000000) ///< Data byte 7
/*\}*/
/*\}*/

/**
 * \{
 * \name CAN filter registers
 */
/**
 * \{
 * \name CAN_FMR register
 */
#define  CAN_FMR_FINIT        ((uint8_t)0x01)        ///< Filter Init Mode
/*\}*/

/**
 * \{
 * \name CAN_FM1R register
 */
#define  CAN_FM1R_FBM         ((uint16_t)0x3FFF)     ///< Filter Mode
#define  CAN_FM1R_FBM0        ((uint16_t)0x0001)     ///< Filter Init Mode bit 0
#define  CAN_FM1R_FBM1        ((uint16_t)0x0002)     ///< Filter Init Mode bit 1
#define  CAN_FM1R_FBM2        ((uint16_t)0x0004)     ///< Filter Init Mode bit 2
#define  CAN_FM1R_FBM3        ((uint16_t)0x0008)     ///< Filter Init Mode bit 3
#define  CAN_FM1R_FBM4        ((uint16_t)0x0010)     ///< Filter Init Mode bit 4
#define  CAN_FM1R_FBM5        ((uint16_t)0x0020)     ///< Filter Init Mode bit 5
#define  CAN_FM1R_FBM6        ((uint16_t)0x0040)     ///< Filter Init Mode bit 6
#define  CAN_FM1R_FBM7        ((uint16_t)0x0080)     ///< Filter Init Mode bit 7
#define  CAN_FM1R_FBM8        ((uint16_t)0x0100)     ///< Filter Init Mode bit 8
#define  CAN_FM1R_FBM9        ((uint16_t)0x0200)     ///< Filter Init Mode bit 9
#define  CAN_FM1R_FBM10       ((uint16_t)0x0400)     ///< Filter Init Mode bit 10
#define  CAN_FM1R_FBM11       ((uint16_t)0x0800)     ///< Filter Init Mode bit 11
#define  CAN_FM1R_FBM12       ((uint16_t)0x1000)     ///< Filter Init Mode bit 12
#define  CAN_FM1R_FBM13       ((uint16_t)0x2000)     ///< Filter Init Mode bit 13
/*\}*/

/**
 * \{
 * \name CAN_FS1R register
 */
#define  CAN_FS1R_FSC         ((uint16_t)0x3FFF)     ///< Filter Scale Configuration
#define  CAN_FS1R_FSC0        ((uint16_t)0x0001)     ///< Filter Scale Configuration bit 0
#define  CAN_FS1R_FSC1        ((uint16_t)0x0002)     ///< Filter Scale Configuration bit 1
#define  CAN_FS1R_FSC2        ((uint16_t)0x0004)     ///< Filter Scale Configuration bit 2
#define  CAN_FS1R_FSC3        ((uint16_t)0x0008)     ///< Filter Scale Configuration bit 3
#define  CAN_FS1R_FSC4        ((uint16_t)0x0010)     ///< Filter Scale Configuration bit 4
#define  CAN_FS1R_FSC5        ((uint16_t)0x0020)     ///< Filter Scale Configuration bit 5
#define  CAN_FS1R_FSC6        ((uint16_t)0x0040)     ///< Filter Scale Configuration bit 6
#define  CAN_FS1R_FSC7        ((uint16_t)0x0080)     ///< Filter Scale Configuration bit 7
#define  CAN_FS1R_FSC8        ((uint16_t)0x0100)     ///< Filter Scale Configuration bit 8
#define  CAN_FS1R_FSC9        ((uint16_t)0x0200)     ///< Filter Scale Configuration bit 9
#define  CAN_FS1R_FSC10       ((uint16_t)0x0400)     ///< Filter Scale Configuration bit 10
#define  CAN_FS1R_FSC11       ((uint16_t)0x0800)     ///< Filter Scale Configuration bit 11
#define  CAN_FS1R_FSC12       ((uint16_t)0x1000)     ///< Filter Scale Configuration bit 12
#define  CAN_FS1R_FSC13       ((uint16_t)0x2000)     ///< Filter Scale Configuration bit 13
/*\}*/

/**
 * \{
 * \name CAN_FFA1R register
 */
#define  CAN_FFA1R_FFA        ((uint16_t)0x3FFF)     ///< Filter FIFO Assignment
#define  CAN_FFA1R_FFA0       ((uint16_t)0x0001)     ///< Filter FIFO Assignment for Filter 0
#define  CAN_FFA1R_FFA1       ((uint16_t)0x0002)     ///< Filter FIFO Assignment for Filter 1
#define  CAN_FFA1R_FFA2       ((uint16_t)0x0004)     ///< Filter FIFO Assignment for Filter 2
#define  CAN_FFA1R_FFA3       ((uint16_t)0x0008)     ///< Filter FIFO Assignment for Filter 3
#define  CAN_FFA1R_FFA4       ((uint16_t)0x0010)     ///< Filter FIFO Assignment for Filter 4
#define  CAN_FFA1R_FFA5       ((uint16_t)0x0020)     ///< Filter FIFO Assignment for Filter 5
#define  CAN_FFA1R_FFA6       ((uint16_t)0x0040)     ///< Filter FIFO Assignment for Filter 6
#define  CAN_FFA1R_FFA7       ((uint16_t)0x0080)     ///< Filter FIFO Assignment for Filter 7
#define  CAN_FFA1R_FFA8       ((uint16_t)0x0100)     ///< Filter FIFO Assignment for Filter 8
#define  CAN_FFA1R_FFA9       ((uint16_t)0x0200)     ///< Filter FIFO Assignment for Filter 9
#define  CAN_FFA1R_FFA10      ((uint16_t)0x0400)     ///< Filter FIFO Assignment for Filter 10
#define  CAN_FFA1R_FFA11      ((uint16_t)0x0800)     ///< Filter FIFO Assignment for Filter 11
#define  CAN_FFA1R_FFA12      ((uint16_t)0x1000)     ///< Filter FIFO Assignment for Filter 12
#define  CAN_FFA1R_FFA13      ((uint16_t)0x2000)     ///< Filter FIFO Assignment for Filter 13
/*\}*/

/**
 * \{
 * \name CAN_FA1R register
 */
#define  CAN_FA1R_FACT        ((uint16_t)0x3FFF)     ///< Filter Active
#define  CAN_FA1R_FACT0       ((uint16_t)0x0001)     ///< Filter 0 Active
#define  CAN_FA1R_FACT1       ((uint16_t)0x0002)     ///< Filter 1 Active
#define  CAN_FA1R_FACT2       ((uint16_t)0x0004)     ///< Filter 2 Active
#define  CAN_FA1R_FACT3       ((uint16_t)0x0008)     ///< Filter 3 Active
#define  CAN_FA1R_FACT4       ((uint16_t)0x0010)     ///< Filter 4 Active
#define  CAN_FA1R_FACT5       ((uint16_t)0x0020)     ///< Filter 5 Active
#define  CAN_FA1R_FACT6       ((uint16_t)0x0040)     ///< Filter 6 Active
#define  CAN_FA1R_FACT7       ((uint16_t)0x0080)     ///< Filter 7 Active
#define  CAN_FA1R_FACT8       ((uint16_t)0x0100)     ///< Filter 8 Active
#define  CAN_FA1R_FACT9       ((uint16_t)0x0200)     ///< Filter 9 Active
#define  CAN_FA1R_FACT10      ((uint16_t)0x0400)     ///< Filter 10 Active
#define  CAN_FA1R_FACT11      ((uint16_t)0x0800)     ///< Filter 11 Active
#define  CAN_FA1R_FACT12      ((uint16_t)0x1000)     ///< Filter 12 Active
#define  CAN_FA1R_FACT13      ((uint16_t)0x2000)     ///< Filter 13 Active
/*\}*/

/**
 * \{
 * \name CAN_F0R1 register
 */
#define  CAN_F0R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F0R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F0R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F0R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F0R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F0R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F0R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F0R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F0R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F0R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F0R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F0R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F0R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F0R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F0R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F0R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F0R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F0R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F0R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F0R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F0R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F0R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F0R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F0R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F0R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F0R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F0R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F0R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F0R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F0R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F0R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F0R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F1R1 register
 */
#define  CAN_F1R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F1R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F1R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F1R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F1R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F1R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F1R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F1R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F1R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F1R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F1R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F1R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F1R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F1R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F1R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F1R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F1R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F1R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F1R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F1R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F1R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F1R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F1R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F1R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F1R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F1R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F1R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F1R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F1R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F1R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F1R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F1R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F2R1 register
 */
#define  CAN_F2R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F2R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F2R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F2R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F2R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F2R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F2R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F2R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F2R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F2R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F2R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F2R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F2R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F2R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F2R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F2R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F2R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F2R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F2R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F2R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F2R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F2R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F2R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F2R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F2R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F2R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F2R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F2R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F2R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F2R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F2R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F2R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F3R1 register
 */
#define  CAN_F3R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F3R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F3R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F3R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F3R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F3R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F3R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F3R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F3R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F3R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F3R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F3R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F3R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F3R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F3R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F3R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F3R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F3R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F3R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F3R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F3R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F3R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F3R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F3R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F3R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F3R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F3R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F3R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F3R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F3R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F3R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F3R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F4R1 register
 */
#define  CAN_F4R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F4R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F4R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F4R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F4R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F4R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F4R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F4R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F4R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F4R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F4R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F4R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F4R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F4R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F4R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F4R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F4R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F4R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F4R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F4R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F4R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F4R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F4R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F4R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F4R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F4R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F4R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F4R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F4R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F4R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F4R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F4R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F5R1 register
 */
#define  CAN_F5R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F5R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F5R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F5R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F5R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F5R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F5R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F5R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F5R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F5R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F5R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F5R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F5R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F5R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F5R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F5R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F5R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F5R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F5R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F5R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F5R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F5R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F5R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F5R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F5R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F5R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F5R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F5R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F5R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F5R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F5R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F5R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F6R1 register
 */
#define  CAN_F6R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F6R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F6R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F6R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F6R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F6R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F6R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F6R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F6R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F6R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F6R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F6R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F6R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F6R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F6R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F6R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F6R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F6R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F6R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F6R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F6R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F6R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F6R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F6R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F6R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F6R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F6R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F6R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F6R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F6R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F6R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F6R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F7R1 register
 */
#define  CAN_F7R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F7R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F7R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F7R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F7R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F7R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F7R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F7R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F7R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F7R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F7R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F7R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F7R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F7R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F7R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F7R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F7R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F7R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F7R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F7R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F7R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F7R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F7R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F7R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F7R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F7R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F7R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F7R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F7R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F7R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F7R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F7R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F8R1 register
 */
#define  CAN_F8R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F8R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F8R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F8R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F8R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F8R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F8R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F8R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F8R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F8R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F8R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F8R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F8R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F8R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F8R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F8R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F8R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F8R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F8R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F8R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F8R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F8R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F8R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F8R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F8R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F8R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F8R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F8R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F8R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F8R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F8R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F8R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F9R1 register
 */
#define  CAN_F9R1_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F9R1_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F9R1_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F9R1_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F9R1_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F9R1_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F9R1_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F9R1_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F9R1_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F9R1_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F9R1_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F9R1_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F9R1_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F9R1_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F9R1_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F9R1_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F9R1_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F9R1_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F9R1_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F9R1_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F9R1_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F9R1_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F9R1_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F9R1_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F9R1_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F9R1_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F9R1_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F9R1_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F9R1_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F9R1_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F9R1_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F9R1_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F10R1 register
 */
#define  CAN_F10R1_FB0        ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F10R1_FB1        ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F10R1_FB2        ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F10R1_FB3        ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F10R1_FB4        ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F10R1_FB5        ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F10R1_FB6        ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F10R1_FB7        ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F10R1_FB8        ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F10R1_FB9        ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F10R1_FB10       ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F10R1_FB11       ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F10R1_FB12       ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F10R1_FB13       ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F10R1_FB14       ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F10R1_FB15       ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F10R1_FB16       ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F10R1_FB17       ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F10R1_FB18       ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F10R1_FB19       ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F10R1_FB20       ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F10R1_FB21       ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F10R1_FB22       ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F10R1_FB23       ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F10R1_FB24       ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F10R1_FB25       ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F10R1_FB26       ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F10R1_FB27       ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F10R1_FB28       ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F10R1_FB29       ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F10R1_FB30       ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F10R1_FB31       ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F11R1 register
 */
#define  CAN_F11R1_FB0        ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F11R1_FB1        ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F11R1_FB2        ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F11R1_FB3        ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F11R1_FB4        ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F11R1_FB5        ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F11R1_FB6        ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F11R1_FB7        ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F11R1_FB8        ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F11R1_FB9        ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F11R1_FB10       ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F11R1_FB11       ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F11R1_FB12       ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F11R1_FB13       ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F11R1_FB14       ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F11R1_FB15       ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F11R1_FB16       ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F11R1_FB17       ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F11R1_FB18       ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F11R1_FB19       ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F11R1_FB20       ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F11R1_FB21       ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F11R1_FB22       ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F11R1_FB23       ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F11R1_FB24       ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F11R1_FB25       ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F11R1_FB26       ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F11R1_FB27       ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F11R1_FB28       ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F11R1_FB29       ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F11R1_FB30       ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F11R1_FB31       ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F12R1 register
 */
#define  CAN_F12R1_FB0        ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F12R1_FB1        ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F12R1_FB2        ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F12R1_FB3        ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F12R1_FB4        ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F12R1_FB5        ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F12R1_FB6        ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F12R1_FB7        ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F12R1_FB8        ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F12R1_FB9        ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F12R1_FB10       ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F12R1_FB11       ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F12R1_FB12       ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F12R1_FB13       ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F12R1_FB14       ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F12R1_FB15       ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F12R1_FB16       ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F12R1_FB17       ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F12R1_FB18       ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F12R1_FB19       ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F12R1_FB20       ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F12R1_FB21       ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F12R1_FB22       ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F12R1_FB23       ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F12R1_FB24       ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F12R1_FB25       ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F12R1_FB26       ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F12R1_FB27       ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F12R1_FB28       ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F12R1_FB29       ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F12R1_FB30       ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F12R1_FB31       ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F13R1 register
 */
#define  CAN_F13R1_FB0        ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F13R1_FB1        ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F13R1_FB2        ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F13R1_FB3        ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F13R1_FB4        ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F13R1_FB5        ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F13R1_FB6        ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F13R1_FB7        ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F13R1_FB8        ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F13R1_FB9        ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F13R1_FB10       ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F13R1_FB11       ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F13R1_FB12       ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F13R1_FB13       ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F13R1_FB14       ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F13R1_FB15       ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F13R1_FB16       ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F13R1_FB17       ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F13R1_FB18       ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F13R1_FB19       ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F13R1_FB20       ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F13R1_FB21       ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F13R1_FB22       ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F13R1_FB23       ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F13R1_FB24       ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F13R1_FB25       ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F13R1_FB26       ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F13R1_FB27       ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F13R1_FB28       ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F13R1_FB29       ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F13R1_FB30       ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F13R1_FB31       ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F0R2 register
 */
#define  CAN_F0R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F0R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F0R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F0R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F0R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F0R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F0R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F0R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F0R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F0R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F0R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F0R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F0R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F0R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F0R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F0R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F0R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F0R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F0R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F0R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F0R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F0R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F0R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F0R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F0R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F0R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F0R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F0R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F0R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F0R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F0R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F0R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F1R2 register
 */
#define  CAN_F1R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F1R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F1R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F1R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F1R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F1R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F1R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F1R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F1R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F1R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F1R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F1R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F1R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F1R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F1R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F1R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F1R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F1R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F1R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F1R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F1R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F1R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F1R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F1R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F1R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F1R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F1R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F1R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F1R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F1R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F1R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F1R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F2R2 register
 */
#define  CAN_F2R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F2R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F2R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F2R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F2R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F2R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F2R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F2R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F2R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F2R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F2R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F2R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F2R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F2R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F2R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F2R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F2R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F2R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F2R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F2R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F2R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F2R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F2R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F2R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F2R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F2R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F2R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F2R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F2R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F2R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F2R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F2R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F3R2 register
 */
#define  CAN_F3R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F3R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F3R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F3R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F3R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F3R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F3R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F3R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F3R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F3R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F3R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F3R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F3R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F3R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F3R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F3R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F3R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F3R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F3R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F3R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F3R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F3R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F3R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F3R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F3R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F3R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F3R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F3R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F3R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F3R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F3R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F3R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F4R2 register
 */
#define  CAN_F4R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F4R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F4R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F4R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F4R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F4R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F4R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F4R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F4R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F4R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F4R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F4R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F4R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F4R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F4R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F4R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F4R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F4R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F4R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F4R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F4R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F4R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F4R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F4R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F4R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F4R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F4R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F4R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F4R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F4R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F4R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F4R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F5R2 register
 */
#define  CAN_F5R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F5R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F5R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F5R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F5R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F5R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F5R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F5R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F5R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F5R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F5R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F5R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F5R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F5R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F5R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F5R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F5R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F5R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F5R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F5R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F5R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F5R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F5R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F5R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F5R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F5R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F5R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F5R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F5R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F5R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F5R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F5R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F6R2 register
 */
#define  CAN_F6R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F6R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F6R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F6R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F6R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F6R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F6R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F6R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F6R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F6R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F6R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F6R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F6R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F6R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F6R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F6R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F6R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F6R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F6R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F6R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F6R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F6R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F6R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F6R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F6R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F6R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F6R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F6R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F6R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F6R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F6R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F6R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F7R2 register
 */
#define  CAN_F7R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F7R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F7R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F7R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F7R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F7R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F7R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F7R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F7R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F7R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F7R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F7R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F7R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F7R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F7R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F7R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F7R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F7R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F7R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F7R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F7R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F7R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F7R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F7R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F7R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F7R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F7R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F7R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F7R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F7R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F7R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F7R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F8R2 register
 */
#define  CAN_F8R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F8R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F8R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F8R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F8R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F8R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F8R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F8R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F8R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F8R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F8R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F8R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F8R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F8R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F8R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F8R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F8R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F8R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F8R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F8R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F8R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F8R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F8R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F8R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F8R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F8R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F8R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F8R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F8R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F8R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F8R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F8R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F9R2 register
 */
#define  CAN_F9R2_FB0         ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F9R2_FB1         ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F9R2_FB2         ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F9R2_FB3         ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F9R2_FB4         ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F9R2_FB5         ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F9R2_FB6         ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F9R2_FB7         ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F9R2_FB8         ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F9R2_FB9         ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F9R2_FB10        ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F9R2_FB11        ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F9R2_FB12        ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F9R2_FB13        ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F9R2_FB14        ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F9R2_FB15        ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F9R2_FB16        ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F9R2_FB17        ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F9R2_FB18        ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F9R2_FB19        ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F9R2_FB20        ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F9R2_FB21        ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F9R2_FB22        ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F9R2_FB23        ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F9R2_FB24        ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F9R2_FB25        ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F9R2_FB26        ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F9R2_FB27        ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F9R2_FB28        ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F9R2_FB29        ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F9R2_FB30        ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F9R2_FB31        ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F10R2 register
 */
#define  CAN_F10R2_FB0        ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F10R2_FB1        ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F10R2_FB2        ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F10R2_FB3        ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F10R2_FB4        ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F10R2_FB5        ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F10R2_FB6        ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F10R2_FB7        ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F10R2_FB8        ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F10R2_FB9        ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F10R2_FB10       ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F10R2_FB11       ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F10R2_FB12       ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F10R2_FB13       ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F10R2_FB14       ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F10R2_FB15       ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F10R2_FB16       ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F10R2_FB17       ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F10R2_FB18       ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F10R2_FB19       ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F10R2_FB20       ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F10R2_FB21       ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F10R2_FB22       ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F10R2_FB23       ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F10R2_FB24       ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F10R2_FB25       ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F10R2_FB26       ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F10R2_FB27       ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F10R2_FB28       ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F10R2_FB29       ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F10R2_FB30       ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F10R2_FB31       ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F11R2 register
 */
#define  CAN_F11R2_FB0        ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F11R2_FB1        ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F11R2_FB2        ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F11R2_FB3        ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F11R2_FB4        ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F11R2_FB5        ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F11R2_FB6        ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F11R2_FB7        ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F11R2_FB8        ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F11R2_FB9        ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F11R2_FB10       ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F11R2_FB11       ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F11R2_FB12       ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F11R2_FB13       ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F11R2_FB14       ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F11R2_FB15       ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F11R2_FB16       ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F11R2_FB17       ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F11R2_FB18       ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F11R2_FB19       ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F11R2_FB20       ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F11R2_FB21       ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F11R2_FB22       ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F11R2_FB23       ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F11R2_FB24       ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F11R2_FB25       ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F11R2_FB26       ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F11R2_FB27       ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F11R2_FB28       ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F11R2_FB29       ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F11R2_FB30       ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F11R2_FB31       ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/**
 * \{
 * \name CAN_F12R2 register
 */
#define  CAN_F12R2_FB0        ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F12R2_FB1        ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F12R2_FB2        ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F12R2_FB3        ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F12R2_FB4        ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F12R2_FB5        ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F12R2_FB6        ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F12R2_FB7        ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F12R2_FB8        ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F12R2_FB9        ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F12R2_FB10       ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F12R2_FB11       ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F12R2_FB12       ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F12R2_FB13       ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F12R2_FB14       ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F12R2_FB15       ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F12R2_FB16       ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F12R2_FB17       ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F12R2_FB18       ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F12R2_FB19       ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F12R2_FB20       ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F12R2_FB21       ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F12R2_FB22       ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F12R2_FB23       ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F12R2_FB24       ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F12R2_FB25       ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F12R2_FB26       ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F12R2_FB27       ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F12R2_FB28       ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F12R2_FB29       ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F12R2_FB30       ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F12R2_FB31       ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/

/*\{*/
/**
 * \name CAN_F13R2 register
 */
#define  CAN_F13R2_FB0        ((uint32_t)0x00000001) ///< Filter bit 0
#define  CAN_F13R2_FB1        ((uint32_t)0x00000002) ///< Filter bit 1
#define  CAN_F13R2_FB2        ((uint32_t)0x00000004) ///< Filter bit 2
#define  CAN_F13R2_FB3        ((uint32_t)0x00000008) ///< Filter bit 3
#define  CAN_F13R2_FB4        ((uint32_t)0x00000010) ///< Filter bit 4
#define  CAN_F13R2_FB5        ((uint32_t)0x00000020) ///< Filter bit 5
#define  CAN_F13R2_FB6        ((uint32_t)0x00000040) ///< Filter bit 6
#define  CAN_F13R2_FB7        ((uint32_t)0x00000080) ///< Filter bit 7
#define  CAN_F13R2_FB8        ((uint32_t)0x00000100) ///< Filter bit 8
#define  CAN_F13R2_FB9        ((uint32_t)0x00000200) ///< Filter bit 9
#define  CAN_F13R2_FB10       ((uint32_t)0x00000400) ///< Filter bit 10
#define  CAN_F13R2_FB11       ((uint32_t)0x00000800) ///< Filter bit 11
#define  CAN_F13R2_FB12       ((uint32_t)0x00001000) ///< Filter bit 12
#define  CAN_F13R2_FB13       ((uint32_t)0x00002000) ///< Filter bit 13
#define  CAN_F13R2_FB14       ((uint32_t)0x00004000) ///< Filter bit 14
#define  CAN_F13R2_FB15       ((uint32_t)0x00008000) ///< Filter bit 15
#define  CAN_F13R2_FB16       ((uint32_t)0x00010000) ///< Filter bit 16
#define  CAN_F13R2_FB17       ((uint32_t)0x00020000) ///< Filter bit 17
#define  CAN_F13R2_FB18       ((uint32_t)0x00040000) ///< Filter bit 18
#define  CAN_F13R2_FB19       ((uint32_t)0x00080000) ///< Filter bit 19
#define  CAN_F13R2_FB20       ((uint32_t)0x00100000) ///< Filter bit 20
#define  CAN_F13R2_FB21       ((uint32_t)0x00200000) ///< Filter bit 21
#define  CAN_F13R2_FB22       ((uint32_t)0x00400000) ///< Filter bit 22
#define  CAN_F13R2_FB23       ((uint32_t)0x00800000) ///< Filter bit 23
#define  CAN_F13R2_FB24       ((uint32_t)0x01000000) ///< Filter bit 24
#define  CAN_F13R2_FB25       ((uint32_t)0x02000000) ///< Filter bit 25
#define  CAN_F13R2_FB26       ((uint32_t)0x04000000) ///< Filter bit 26
#define  CAN_F13R2_FB27       ((uint32_t)0x08000000) ///< Filter bit 27
#define  CAN_F13R2_FB28       ((uint32_t)0x10000000) ///< Filter bit 28
#define  CAN_F13R2_FB29       ((uint32_t)0x20000000) ///< Filter bit 29
#define  CAN_F13R2_FB30       ((uint32_t)0x40000000) ///< Filter bit 30
#define  CAN_F13R2_FB31       ((uint32_t)0x80000000) ///< Filter bit 31
/*\}*/
/*\}*/
/*\}*/
#endif /* IO_STM32_CAN_H */
