/**
 * USB-CAN converter
 *
 * This file contains the interface for a very simple serial driver for STM32
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

#include "serial.h"

#define ENDLINE '\r'
#define RX_BUF_SIZE 4096

static char receive_buffer[RX_BUF_SIZE];
static uint64_t read;
static uint64_t write;

static DECLARE_ISR(uart_irq_handler) {
    struct stm32_usart *base = (struct stm32_usart *)USART3_BASE;

    base->CR1 &= ~(BV(CR1_RXNEIE));
    while (base->SR & (BV(SR_RXNE) | BV(SR_ORE))) {

        receive_buffer[write % RX_BUF_SIZE] = base->DR;
        write++;
    }
    base->CR1 |= BV(CR1_RXNEIE);
}

// Hardcoded to port 3
struct stm32_usart *serial_init(unsigned long baudrate) {
    struct stm32_usart *base = (struct stm32_usart *)USART3_BASE;

    read = 0;
    write = 0;

    // Enable clocks
    RCC->APB2ENR |= RCC_APB2_AFIO | RCC_APB2_GPIOB;
    RCC->APB1ENR |= RCC_APB1_USART3;

    // Enable pins
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOB_BASE, BV(10),
                        GPIO_MODE_AF_PP, GPIO_SPEED_50MHZ);
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOB_BASE, BV(11),
                        GPIO_MODE_IN_FLOATING, GPIO_SPEED_50MHZ);

    // Clear registers
    base->CR2 = 0;
    base->CR1 = 0;
    base->CR3 = 0;
    base->SR = 0;

    // Set baudrate
    base->BRR = evaluate_brr(base, CPU_FREQ, baudrate);

    // No parity, 8 bits, 1 stop bit : no further configuration needed

    // Enable TX and RX
    base->CR1 |= BV(CR1_TE) | BV(CR1_RE);

    // Enable the RX interrupt
    sysirq_setHandler(USART3_IRQHANDLER, uart_irq_handler);

    base->CR1 |= BV(CR1_RXNEIE);

    // Start up
    base->CR1 |= BV(CR1_UE);

    return base;
}

size_t serial_readline(char *buffer, size_t max_len) {
    size_t i = 0;
    char c;

    while (i < max_len-1) {
        if (read == write) {
            cpu_relax();
            continue;
        }
        c = receive_buffer[read % RX_BUF_SIZE];
        read++;
        if (c == ENDLINE) {
            break;
        }
        buffer[i] = c;
        i++;
    }
    buffer[i] = '\0';

    return i;
}

void serial_transmit(const char *buffer, size_t len) {
    struct stm32_usart *base = (struct stm32_usart *)USART3_BASE;

    size_t i;

    for(i=0; i < len; i++) {
        while (!(base->SR & BV(SR_TXE)))
            cpu_relax();
        base->DR = buffer[i];
    }
}
