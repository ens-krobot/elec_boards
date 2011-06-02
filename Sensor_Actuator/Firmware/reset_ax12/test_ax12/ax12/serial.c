/**
 * AX12 Library
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

#define RX_BUF_SIZE 4096

#define MY_USART USART1_BASE

static char receive_buffer[RX_BUF_SIZE];
static uint64_t read;
static uint64_t write;

static DECLARE_ISR(uart_irq_handler) {
    struct stm32_usart *base = (struct stm32_usart *)MY_USART;

    base->CR1 &= ~(BV(CR1_RXNEIE));
    while (base->SR & (BV(SR_RXNE) | BV(SR_ORE))) {

        receive_buffer[write % RX_BUF_SIZE] = base->DR;
        write++;
    }
    base->CR1 |= BV(CR1_RXNEIE);
}

// Hardcoded to port 1
struct stm32_usart *serial_init(unsigned long baudrate) {
    struct stm32_usart *base = (struct stm32_usart *)MY_USART;

    read = 0;
    write = 0;

    // Enable clocks
    RCC->APB2ENR |= RCC_APB2_USART1;
    RCC->APB2ENR |= RCC_APB2_AFIO | RCC_APB2_GPIOA;

    //stm32_gpioRemap(GPIO_PARTIALREMAP_USART3, GPIO_REMAP_ENABLE);

    // Enable pins
    // TX output open drain
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOA_BASE, BV(9),
                        GPIO_MODE_AF_OD, GPIO_SPEED_50MHZ);

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
    sysirq_setHandler(USART1_IRQHANDLER, uart_irq_handler);

    base->CR1 |= BV(CR1_RXNEIE);

    // Let it be Half-Duplex
    base->CR3 |= CR3_HDSEL_SET;

    // Start up
    base->CR1 |= BV(CR1_UE);

    return base;
}

void serial_deinit(void) {
    struct stm32_usart *base = (struct stm32_usart *)MY_USART;

    base->CR1 &= ~BV(CR1_RXNEIE);

    sysirq_freeHandler(USART1_IRQHANDLER);

    // Finish him.
    base->CR1 &= ~BV(CR1_UE);

    RCC->APB2ENR &= ~RCC_APB2_USART1;
}

int serial_getchar(void) {
    ticks_t start = timer_clock();

    for (;;) {
        if (read == write) {
            if (timer_clock() > start + ms_to_ticks(500))
                return -1;
            cpu_relax();
            continue;
        }
        return receive_buffer[(read++) % RX_BUF_SIZE];
    }
}

void serial_putchar(char ch) {
    struct stm32_usart *base = (struct stm32_usart *)MY_USART;

    while (!(base->SR & BV(SR_TXE)))
        cpu_relax();
    base->DR = ch;
}

void serial_transmit(const char *buffer, size_t len) {
    size_t i;

    for(i=0; i < len; i++) {
        serial_putchar(buffer[i]);
    }
}
