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
 * @file AT91SAM7/serial_lld.c
 * @brief AT91SAM7 low level serial driver code.
 * @addtogroup AT91SAM7_SERIAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if CH_HAL_USE_SERIAL || defined(__DOXYGEN__)

#if SAM7_PLATFORM == SAM7S256

#define SAM7_USART0_RX    AT91C_PA5_RXD0
#define SAM7_USART0_TX    AT91C_PA6_TXD0
#define SAM7_USART1_RX    AT91C_PA21_RXD1
#define SAM7_USART1_TX    AT91C_PA22_TXD1

#elif SAM7_PLATFORM == SAM7X256

#define SAM7_USART0_RX    AT91C_PA0_RXD0
#define SAM7_USART0_TX    AT91C_PA1_TXD0
#define SAM7_USART1_RX    AT91C_PA5_RXD1
#define SAM7_USART1_TX    AT91C_PA6_TXD1

#else
#error "serial lines not defined for this SAM7 version"
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if USE_SAM7_USART0 || defined(__DOXYGEN__)
/** @brief USART0 serial driver identifier.*/
SerialDriver SD1;
#endif

#if USE_SAM7_USART1 || defined(__DOXYGEN__)
/** @brief USART1 serial driver identifier.*/
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  AT91C_US_USMODE_NORMAL | AT91C_US_CLKS_CLOCK |
  AT91C_US_CHRL_8_BITS | AT91C_US_PAR_NONE | AT91C_US_NBSTOP_1_BIT
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief USART initialization.
 *
 * @param[in] sdp communication channel associated to the USART
 */
static void usart_init(SerialDriver *sdp) {
  AT91PS_USART u = sdp->usart;

  /* Disables IRQ sources and stop operations.*/
  u->US_IDR = 0xFFFFFFFF;
  u->US_CR = AT91C_US_RSTRX | AT91C_US_RSTTX | AT91C_US_RSTSTA;

  /* New parameters setup.*/
  if (sdp->config->sc_mr & AT91C_US_OVER)
    u->US_BRGR = MCK / (sdp->config->sc_speed * 8);
  else
    u->US_BRGR = MCK / (sdp->config->sc_speed * 16);
  u->US_MR = sdp->config->sc_mr;
  u->US_RTOR = 0;
  u->US_TTGR = 0;

  /* Enables operations and IRQ sources.*/
  u->US_CR = AT91C_US_RXEN | AT91C_US_TXEN | AT91C_US_DTREN | AT91C_US_RTSEN;
  u->US_IER = AT91C_US_RXRDY | AT91C_US_OVRE | AT91C_US_FRAME | AT91C_US_PARE |
              AT91C_US_RXBRK;
}

/**
 * @brief USART de-initialization.
 * @param[in] u pointer to an USART I/O block
 */
static void usart_deinit(AT91PS_USART u) {

  /* Disables IRQ sources and stop operations.*/
  u->US_IDR = 0xFFFFFFFF;
  u->US_CR = AT91C_US_RSTRX | AT91C_US_RSTTX | AT91C_US_RSTSTA;
  u->US_MR = 0;
  u->US_RTOR = 0;
  u->US_TTGR = 0;
}

/**
 * @brief Error handling routine.
 * @param[in] err USART CSR register value
 * @param[in] sdp communication channel associated to the USART
 */
static void set_error(SerialDriver *sdp, AT91_REG csr) {
  sdflags_t sts = 0;

  if (csr & AT91C_US_OVRE)
    sts |= SD_OVERRUN_ERROR;
  if (csr & AT91C_US_PARE)
    sts |= SD_PARITY_ERROR;
  if (csr & AT91C_US_FRAME)
    sts |= SD_FRAMING_ERROR;
  if (csr & AT91C_US_RXBRK)
    sts |= SD_BREAK_DETECTED;
  chSysLockFromIsr();
  sdAddFlagsI(sdp, sts);
  chSysUnlockFromIsr();
}

#if defined(__GNU__)
__attribute__((noinline))
#endif
/**
 * @brief Common IRQ handler.
 *
 * @param[in] sdp communication channel associated to the USART
 */
static void serve_interrupt(SerialDriver *sdp) {
  uint32_t csr;
  AT91PS_USART u = sdp->usart;

  csr = u->US_CSR;
  if (csr & AT91C_US_RXRDY) {
    chSysLockFromIsr();
    sdIncomingDataI(sdp, u->US_RHR);
    chSysUnlockFromIsr();
  }
  if ((u->US_IMR & AT91C_US_TXRDY) && (csr & AT91C_US_TXRDY)) {
    msg_t b;

    chSysLockFromIsr();
    b = chOQGetI(&sdp->oqueue);
    if (b < Q_OK) {
      chEvtBroadcastI(&sdp->oevent);
      u->US_IDR = AT91C_US_TXRDY;
    }
    else
      u->US_THR = b;
    chSysUnlockFromIsr();
  }
  csr &= (AT91C_US_OVRE | AT91C_US_FRAME | AT91C_US_PARE | AT91C_US_RXBRK);
  if (csr != 0) {
    set_error(sdp, csr);
    u->US_CR = AT91C_US_RSTSTA;
  }
  AT91C_BASE_AIC->AIC_EOICR = 0;
}

#if USE_SAM7_USART0 || defined(__DOXYGEN__)
static void notify1(void) {

  AT91C_BASE_US0->US_IER = AT91C_US_TXRDY;
}
#endif

#if USE_SAM7_USART1 || defined(__DOXYGEN__)
static void notify2(void) {

  AT91C_BASE_US1->US_IER = AT91C_US_TXRDY;
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if USE_SAM7_USART0 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(USART0IrqHandler) {

  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  CH_IRQ_EPILOGUE();
}
#endif

#if USE_SAM7_USART1 || defined(__DOXYGEN__)
CH_IRQ_HANDLER(USART1IrqHandler) {

  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  CH_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * Low level serial driver initialization.
 */
void sd_lld_init(void) {

#if USE_SAM7_USART0
  sdObjectInit(&SD1, NULL, notify1);
  SD1.usart = AT91C_BASE_US0;
  AT91C_BASE_PIOA->PIO_PDR   = SAM7_USART0_RX | SAM7_USART0_TX;
  AT91C_BASE_PIOA->PIO_ASR   = SAM7_USART0_RX | SAM7_USART0_TX;
  AT91C_BASE_PIOA->PIO_PPUDR = SAM7_USART0_RX | SAM7_USART0_TX;
  AIC_ConfigureIT(AT91C_ID_US0,
                  AT91C_AIC_SRCTYPE_HIGH_LEVEL | SAM7_USART0_PRIORITY,
                  USART0IrqHandler);
#endif

#if USE_SAM7_USART1
  sdObjectInit(&SD2, NULL, notify2);
  SD2.usart = AT91C_BASE_US1;
  AT91C_BASE_PIOA->PIO_PDR   = SAM7_USART1_RX | SAM7_USART1_TX;
  AT91C_BASE_PIOA->PIO_ASR   = SAM7_USART1_RX | SAM7_USART1_TX;
  AT91C_BASE_PIOA->PIO_PPUDR = SAM7_USART1_RX | SAM7_USART1_TX;
  AIC_ConfigureIT(AT91C_ID_US1,
                  AT91C_AIC_SRCTYPE_HIGH_LEVEL | SAM7_USART1_PRIORITY,
                  USART1IrqHandler);
#endif
}

/**
 * @brief Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp pointer to a @p SerialDriver object
 */
void sd_lld_start(SerialDriver *sdp) {

  if (sdp->config == NULL)
    sdp->config = &default_config;

  if (sdp->state == SD_STOP) {
#if USE_SAM7_USART0
    if (&SD1 == sdp) {
      /* Starts the clock and clears possible sources of immediate interrupts.*/
      AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_US0);
      /* Enables associated interrupt vector.*/
      AIC_EnableIT(AT91C_ID_US0);
    }
#endif
#if USE_SAM7_USART1
    if (&SD2 == sdp) {
      /* Starts the clock and clears possible sources of immediate interrupts.*/
      AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_US1);
      /* Enables associated interrupt vector.*/
      AIC_EnableIT(AT91C_ID_US1);
    }
#endif
  }
  usart_init(sdp);
}

/**
 * @brief Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp pointer to a @p SerialDriver object
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    usart_deinit(sdp->usart);
#if USE_SAM7_USART0
    if (&SD1 == sdp) {
      AT91C_BASE_PMC->PMC_PCDR = (1 << AT91C_ID_US0);
      AIC_DisableIT(AT91C_ID_US0);
      return;
    }
#endif
#if USE_SAM7_USART1
    if (&SD2 == sdp) {
      AT91C_BASE_PMC->PMC_PCDR = (1 << AT91C_ID_US1);
      AIC_DisableIT(AT91C_ID_US1);
      return;
    }
#endif
  }
}

#endif /* CH_HAL_USE_SERIAL */

/** @} */
