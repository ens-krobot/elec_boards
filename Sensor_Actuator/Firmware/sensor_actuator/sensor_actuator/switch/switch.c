/**
 * Sensor-Actuator board : Switches
 *
 * Header file for the switches management.
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

#include "switch.h"

void switch_init(void) {

    // Clock the GPIOs.
    RCC->APB2ENR |= RCC_APB2_GPIOA | RCC_APB2_GPIOB | RCC_APB2_GPIOC;

    // Initialize D1_1 PC(4) and D1_2 PC(5)
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOC_BASE,
                        BV(4) | BV(5),
                        GPIO_MODE_IN_FLOATING,
                        GPIO_SPEED_50MHZ);
    // Initialize D1_3 PB(0), D1_4 PB(1), D1_5 PB(12), D1_6 PB(15)
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOB_BASE,
                        BV(0) | BV(1) | BV(12) | BV(15),
                        GPIO_MODE_IN_FLOATING,
                        GPIO_SPEED_50MHZ);

    // Initialize D2_1 PC(6), D2_2 PC(7), D2_3 PC(8), D2_4 PC(9), D2_6 PC(12)
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOC_BASE,
                        BV(6) | BV(7) | BV(8) | BV(9) | BV(12),
                        GPIO_MODE_IN_FLOATING,
                        GPIO_SPEED_50MHZ);

    // Initialize D2_5 PA(8)
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOA_BASE,
                        BV(8),
                        GPIO_MODE_IN_FLOATING,
                        GPIO_SPEED_50MHZ);
}

#define BOOL(a) ((a)?1:0)

void get_switch_state(switch_status *pkt1, switch_status *pkt2) {

    pkt1->p.sw1 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(4)));
    pkt1->p.sw2 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(5)));
    pkt1->p.sw3 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOB_BASE, BV(0)));
    pkt1->p.sw4 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOB_BASE, BV(1)));
    pkt1->p.sw5 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOB_BASE, BV(12)));
    pkt1->p.sw6 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOB_BASE, BV(15)));

    pkt2->p.sw1 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(6)));
    pkt2->p.sw2 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(7)));
    pkt2->p.sw3 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(8)));
    pkt2->p.sw4 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(9)));
    pkt2->p.sw5 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOA_BASE, BV(8)));
    pkt2->p.sw6 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(12)));

}
