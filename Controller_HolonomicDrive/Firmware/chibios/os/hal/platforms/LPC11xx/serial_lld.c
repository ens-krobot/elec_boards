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
 * @file    LPC11xx/serial_lld.c
 * @brief   LPC11xx low level serial driver code.
 *
 * @addtogroup LPC11xx_SERIAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if CH_HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if USE_LPC11xx_UART0 || defined(__DOXYGEN__)
/** @brief UART0 serial driver identifier.*/
SerialDriver SD1;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   UART initialization.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void uart_init(SerialDriver *sdp) {
  LPC_UART_TypeDef *u = sdp->uart;

  uint32_t div = LPC11xx_UART_PCLK / (sdp->config->sc_speed << 4);
  u->LCR = sdp->config->sc_lcr | LCR_DLAB;
  u->DLL = div;
  u->DLM = div >> 8;
  u->LCR = sdp->config->sc_lcr;
  u->FCR = FCR_ENABLE | FCR_RXRESET | FCR_TXRESET | sdp->config->sc_fcr;
  u->ACR = 0;
  u->FDR = 0x10;
  u->TER = TER_ENABLE;
  u->IER = IER_RBR | IER_STATUS;
}

/**
 * @brief   UART de-initialization.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(LPC_UART_TypeDef *u) {

  u->LCR = LCR_DLAB;
  u->DLL = 1;
  u->DLM = 0;
  u->LCR = 0;
  u->FDR = 0x10;
  u->IER = 0;
  u->FCR = FCR_RXRESET | FCR_TXRESET;
  u->ACR = 0;
  u->TER = TER_ENABLE;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       communication channel associated to the UART
 * @param[in] err       UART LSR register value
 */
static void set_error(SerialDriver *sdp, IOREG32 err) {
  sdflags_t sts = 0;

  if (err & LSR_OVERRUN)
    sts |= SD_OVERRUN_ERROR;
  if (err & LSR_PARITY)
    sts |= SD_PARITY_ERROR;
  if (err & LSR_FRAMING)
    sts |= SD_FRAMING_ERROR;
  if (err & LSR_BREAK)
    sts |= SD_BREAK_DETECTED;
  chSysLockFromIsr();
  sdAddFlagsI(sdp, sts);
  chSysUnlockFromIsr();
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *          after.
 *
 * @param[in] u         pointer to an UART I/O block
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  LPC_UART_TypeDef *u = sdp->uart;

  while (TRUE) {
    switch (u->IIR & IIR_SRC_MASK) {
    case IIR_SRC_NONE:
      return;
    case IIR_SRC_ERROR:
      set_error(sdp, u->LSR);
      break;
    case IIR_SRC_TIMEOUT:
    case IIR_SRC_RX:
      chSysLockFromIsr();
      if (chIQIsEmpty(&sdp->iqueue))
        chEvtBroadcastI(&sdp->ievent);
      chSysUnlockFromIsr();
      while (u->LSR & LSR_RBR_FULL) {
        chSysLockFromIsr();
        if (chIQPutI(&sdp->iqueue, u->RBR) < Q_OK)
           sdAddFlagsI(sdp, SD_OVERRUN_ERROR);
        chSysUnlockFromIsr();
      }
      break;
    case IIR_SRC_TX:
      {
        int i = LPC11xx_UART_FIFO_PRELOAD;
        do {
          msg_t b;

          chSysLockFromIsr();
          b = chOQGetI(&sdp->oqueue);
          chSysUnlockFromIsr();
          if (b < Q_OK) {
            u->IER &= ~IER_THRE;
            chSysLockFromIsr();
            chEvtBroadcastI(&sdp->oevent);
            chSysUnlockFromIsr();
            break;
          }
          u->THR = b;
        } while (--i);
      }
      break;
    default:
      (void) u->THR;
      (void) u->RBR;
    }
  }
}

/**
 * @brief   Attempts a TX FIFO preload.
 */
static void preload(SerialDriver *sdp) {
  LPC_UART_TypeDef *u = sdp->uart;

  if (u->LSR & LSR_THRE) {
    int i = LPC11xx_UART_FIFO_PRELOAD;
    do {
      msg_t b = chOQGetI(&sdp->oqueue);
      if (b < Q_OK) {
        chEvtBroadcastI(&sdp->oevent);
        return;
      }
      u->THR = b;
    } while (--i);
  }
  u->IER |= IER_THRE;
}

/**
 * @brief   Driver SD1 output notification.
 */
#if USE_LPC11xx_UART0 || defined(__DOXYGEN__)
static void notify1(void) {

  preload(&SD1);
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   UART0 IRQ handler.
 */
#if USE_LPC11xx_UART0 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(Vector94) {

  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 */
void sd_lld_init(void) {

#if USE_LPC11xx_UART0
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = LPC_UART;
  LPC_IOCON->PIO1_6 = 0xC1;                 /* RDX without resistors.       */
  LPC_IOCON->PIO1_7 = 0xC1;                 /* TDX without resistors.       */
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
void sd_lld_start(SerialDriver *sdp) {

  if (sdp->config == NULL)
    sdp->config = &default_config;

  if (sdp->state == SD_STOP) {
#if USE_LPC11xx_UART0
    if (&SD1 == sdp) {
      LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 12);
      NVICEnableVector(UART_IRQn,
                       CORTEX_PRIORITY_MASK(LPC11xx_UART0_PRIORITY));
    }
#endif
  }
  uart_init(sdp);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);
#if USE_LPC11xx_UART0
    if (&SD1 == sdp) {
      LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 12);
      NVICDisableVector(UART_IRQn);
      return;
    }
#endif
  }
}

#endif /* CH_HAL_USE_SERIAL */

/** @} */
