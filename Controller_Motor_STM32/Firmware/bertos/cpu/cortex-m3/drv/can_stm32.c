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
 * \brief CAN hardware-specific implementation
 *
 * \author Nicolas Dandrimont <Nicolas.Dandrimont@crans.org>
 *
 * $WIZ$
 */

#include "can_stm32.h"

#include <cpu/irq.h>

#include "cfg/cfg_can.h"
#include "cfg/cfg_proc.h"

#include <cfg/macros.h>
#include <cfg/compiler.h>
#include <cfg/debug.h>

// Define log settings for cfg/log.h.
#define LOG_LEVEL         CAN_LOG_LEVEL
#define LOG_FORMAT        CAN_LOG_FORMAT
#include <cfg/log.h>

#include <drv/can.h>
#include <drv/timer.h>
#include <drv/clock_stm32.h>
#include <drv/gpio_stm32.h>

#include <io/stm32.h>
#include <io/stm32_can.h>

#include <cfg/module.h>

#include <drv/irq_cm3.h>

/**
 * CAN Remapping values for STM32
 *
 * $WIZ$ can_stm32_remaps = "CAN_STM32_PORTA", "CAN_STM32_PORTB", "CAN_STM32_PORTD"
 * \{
 */
#define CAN_STM32_PORTA   0
#define CAN_STM32_PORTB   1
#define CAN_STM32_PORTD   2
/* \} */

static can_driver _cand1;
can_driver *CAND1 = &_cand1;

CAN *CAN1 = (CAN *)CAN1_BASE;

/**
 * CAN IRQ Handlers
 */

/**
 * Transmit Done IRQ Handler
 */
static DECLARE_ISR(can1_tx_irqhandler)
{
    // Clear the transmit done events until we handled them...
    CAN1->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;
    // Allow processes to send CAN messages again.
    event_do(&CAND1->tx_empty_event);
}

/**
 * Message received (on RX0) IRQ Handler
 */
static DECLARE_ISR(can1_rx0_irqhandler)
{
    if ((CAN1->RF0R & CAN_RF0R_FMP0) > 0)
    {
        // Disable further message received interrupts.
        CAN1->IER &= ~CAN_IER_FMPIE0;
        event_do(&CAND1->rx_available_event);
    }
    if ((CAN1->RF0R & CAN_RF0R_FOVR0) > 0)
    {
        // The RX0 queue is overflowing. Handle this.
        CAN1->RF0R = CAN_RF0R_FOVR0;
        event_do(&CAND1->error_event);
    }
}

/**
 * Status Change / Error IRQ Handler
 */
static DECLARE_ISR(can1_sce_irqhandler)
{
    uint32_t msr;

    msr = CAN1->MSR;
    // Clear Error Interrupt, Wakeup Interrupt and Sleep Ack Interrupt bits
    CAN1->MSR = CAN_MSR_ERRI | CAN_MSR_WKUI | CAN_MSR_SLAKI;

    if ((msr & CAN_MSR_WKUI) == CAN_MSR_WKUI)
    {
        event_do(&CAND1->wakeup_event);
    }
    if ((msr & CAN_MSR_ERRI) == CAN_MSR_ERRI)
    {
        event_do(&CAND1->error_event);
    }
    if ((msr & CAN_MSR_SLAKI) == CAN_MSR_SLAKI)
    {
        event_do(&CAND1->sleep_event);
    }
}

/**
 * Init CAN hardware.
 */
void can_hw_init(void)
{

    // Initialize the driver structure...
    can_drv_init(CAND1);
    CAND1->can = CAN1;

    // Enable the clocks
    RCC->APB2ENR |= RCC_APB2_AFIO;

#if CAN_STM32_REMAP == CAN_STM32_PORTA
    RCC->APB2ENR |= RCC_APB2_GPIOA;
#elif CAN_STM32_REMAP == CAN_STM32_PORTB
    RCC->APB2ENR |= RCC_APB2_GPIOB;
#elif CAN_STM32_REMAP == CAN_STM32_PORTD
    RCC->APB2ENR |= RCC_APB2_GPIOD;
#else
    #error "CAN remapping not supported for stm32"
#endif

    // Set the pins to the right mode for CAN.
#if CAN_STM32_REMAP == CAN_STM32_PORTA
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOA_BASE, BV(11), GPIO_MODE_IPU, GPIO_SPEED_50MHZ);
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOA_BASE, BV(12), GPIO_MODE_AF_PP, GPIO_SPEED_50MHZ);
#elif CAN_STM32_REMAP == CAN_STM32_PORTB
    stm32_gpioRemap(GPIO_REMAP1_CAN1, GPIO_REMAP_ENABLE);
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOB_BASE, BV(8), GPIO_MODE_IPU, GPIO_SPEED_50MHZ);
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOB_BASE, BV(9), GPIO_MODE_AF_PP, GPIO_SPEED_50MHZ);
#elif CAN_STM32_REMAP == CAN_STM32_PORTD
    stm32_gpioRemap(GPIO_REMAP2_CAN1, GPIO_REMAP_ENABLE);
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOD_BASE, BV(0), GPIO_MODE_IPU, GPIO_SPEED_50MHZ);
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOD_BASE, BV(1), GPIO_MODE_AF_PP, GPIO_SPEED_50MHZ);
#else
    #error "CAN remapping not supported for stm32"
#endif

}

void can_hw_start(can_driver *drv) {
    // Reset the CAN controller, to avoid trouble with JTAGs
    RCC->APB1RSTR |= RCC_APB1_CAN;
    RCC->APB1RSTR &= ~RCC_APB1_CAN;

    // Make sure the USB device is disabled
    RCC->APB1ENR &= ~RCC_APB1_USB;
    RCC->APB1ENR &= ~RCC_APB1_CAN;

    // Clock enable
    RCC->APB1ENR |= RCC_APB1_CAN;

    // Initialization mode
    drv->state = CAN_STARTING;

    drv->can->MCR = CAN_MCR_INRQ;

    while ((drv->can->MSR & CAN_MSR_INAK) == 0)
        timer_delayTicks(1);

    // Initialize registers
    drv->can->BTR = drv->config->btr;
    drv->can->MCR = drv->config->mcr;

    // Initialize filters
    drv->can->FMR |= CAN_FMR_FINIT;
    if (drv->config->n_filters > 0) {
        uint32_t i, mask;
        CAN_FilterRegister *cur_filter;
       
        drv->can->FA1R = 0;
        drv->can->FM1R = 0;
        drv->can->FS1R = 0;
        drv->can->FFA1R = 0;
        cur_filter = drv->can->sFilterRegister;
        mask = 1;

        for (i = 0; i < CAN_MAX_FILTERS; i++) {
            if (i < drv->config->n_filters) {
                if (drv->config->filters[i].mode)
                    drv->can->FM1R |= mask;
                if (drv->config->filters[i].scale)
                    drv->can->FS1R |= mask;
                if (drv->config->filters[i].assignment)
                    drv->can->FFA1R |= mask;
                cur_filter->FR1 = drv->config->filters[i].register1;
                cur_filter->FR2 = drv->config->filters[i].register2;
                drv->can->FA1R |= mask;
            }
            else {
                cur_filter->FR1 = 0;
                cur_filter->FR2 = 0;
            }
            // This loop may be big; allow preemption
            proc_permit();
            cur_filter++;
            mask <<= 1;
            proc_forbid();
        }
    }
    else {
        // Default filter
        drv->can->sFilterRegister[0].FR1 = 0;
        drv->can->sFilterRegister[0].FR2 = 0;
        drv->can->FM1R = 0;
        drv->can->FFA1R = 0;
        drv->can->FS1R = 1;
        drv->can->FA1R = 1;
    }
    drv->can->FMR &= ~CAN_FMR_FINIT;
   
    // Register IRQ handlers
    sysirq_setHandler(USB_HP_CAN_TX_IRQHANDLER, can1_tx_irqhandler);
    sysirq_setHandler(USB_LP_CAN_RX0_IRQHANDLER, can1_rx0_irqhandler);
    //sysirq_setHandler(CAN_RX1_IRQHANDLER, can1_rx1_irqhandler);
    sysirq_setHandler(CAN_SCE_IRQHANDLER, can1_sce_irqhandler);

    // Enable interrupts in the CAN registers
    drv->can->IER = CAN_IER_TMEIE  | CAN_IER_FMPIE0 | CAN_IER_FMPIE1 |
                    CAN_IER_WKUIE  | CAN_IER_ERRIE  | CAN_IER_LECIE  |
                    CAN_IER_BOFIE  | CAN_IER_EPVIE  | CAN_IER_EWGIE  |
                    CAN_IER_FOVIE0 | CAN_IER_FOVIE1;    

}

void can_hw_stop(can_driver *drv) {
    if (drv->state == CAN_READY) {
        if (drv == CAND1) {
            drv->can->MCR = 0x00010002;
            drv->can->IER = 0x00000000;
            sysirq_freeHandler(USB_HP_CAN_TX_IRQHANDLER);
            sysirq_freeHandler(USB_LP_CAN_RX0_IRQHANDLER);
            //sysirq_freeHandler(CAN_RX1_IRQHANDLER);
            sysirq_freeHandler(CAN_SCE_IRQHANDLER);
            RCC->APB1ENR &= ~RCC_APB1_CAN;
        }
    }
}

void can_hw_sleep(can_driver *drv) {
    drv->can->MCR |= CAN_MCR_SLEEP;
}
void can_hw_wakeup(can_driver *drv) {
    drv->can->MCR &= ~CAN_MCR_SLEEP;
}
bool can_hw_can_transmit(can_driver *drv) {
    return (drv->can->TSR & CAN_TSR_TME) != 0;
}
bool can_hw_can_receive(can_driver *drv) {
    return (drv->can->RF0R & CAN_RF0R_FMP0) > 0;
}
void can_hw_transmit(can_driver *drv, const can_tx_frame *frame) {
    uint32_t id_reg;
    CAN_TxMailBox *mailbox;

    // Get the address of a free transmit mailbox
    mailbox = &drv->can->sTxMailBox[(drv->can->TSR & CAN_TSR_CODE) >> 24];

    if (frame->ide)
        id_reg = ((uint32_t)frame->eid << 3) | ((uint32_t)frame->rtr << 1) | CAN_TI0R_IDE;
    else
        id_reg = ((uint32_t)frame->sid << 21) | ((uint32_t)frame->rtr << 1);

    mailbox->TDTR = frame->dlc;
    mailbox->TDLR = frame->data32[0];
    mailbox->TDHR = frame->data32[1];
    mailbox->TIR = id_reg | CAN_TI0R_TXRQ;
}
void can_hw_receive(can_driver *drv, can_rx_frame *frame) {
    uint32_t reg;

    reg = drv->can->sFIFOMailBox[0].RIR;
    frame->rtr = (reg & CAN_RI0R_RTR) >> 1;
    frame->ide = (reg & CAN_RI0R_IDE) >> 2;
    if (frame->ide) {
        frame->eid = reg >> 3;
    }
    else {
        frame->sid = reg >> 21;
    }

    reg = drv->can->sFIFOMailBox[0].RDTR;
    frame->dlc = reg & CAN_RDT0R_DLC;
    frame->fmi = (uint8_t)(reg >> 8);
    frame->time = (uint16_t)(reg >> 16);
    frame->data32[0] = drv->can->sFIFOMailBox[0].RDLR;
    frame->data32[1] = drv->can->sFIFOMailBox[0].RDHR;

    // Release the mailbox
    drv->can->RF0R = CAN_RF0R_RFOM0;

    // Reenable the IRQ if the queue is empty
    if ((drv->can->RF0R & CAN_RF0R_FMP0) == 0)
        drv->can->IER |= CAN_IER_FMPIE0;
}
