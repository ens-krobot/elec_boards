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

static uint32_t pins[][2] = {
    //{GPIOC_BASE, 4},
    //{GPIOC_BASE, 5},
    //{GPIOB_BASE, 0},
    //{GPIOB_BASE, 1},
    //{GPIOB_BASE, 12},
    //{GPIOB_BASE, 15},
    //{GPIOC_BASE, 9},
    //{GPIOA_BASE, 8},
    //{GPIOC_BASE, 12},
    {GPIOA_BASE, 4},
    {GPIOA_BASE, 5},
    {GPIOA_BASE, 6},
    {GPIOA_BASE, 7},
    {GPIOB_BASE, 5},
    {GPIOC_BASE, 6},
    {GPIOC_BASE, 7},
    {GPIOC_BASE, 8},
};

void switch_init(void) {

    // Clock the GPIOs.
    RCC->APB2ENR |= RCC_APB2_GPIOA | RCC_APB2_GPIOB | RCC_APB2_GPIOC;

    // Initialize D1_1 PC(4) and D1_2 PC(5) as pull-up inputs
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOC_BASE,
                        BV(4) | BV(5),
                        GPIO_MODE_IPU,
                        GPIO_SPEED_50MHZ);
    // Initialize D1_3 PB(0), D1_4 PB(1), D1_5 PB(12), D1_6 PB(15) as pull-up inputs
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOB_BASE,
                        BV(0) | BV(1) | BV(12) | BV(15),
                        GPIO_MODE_IPU,
                        GPIO_SPEED_50MHZ);

    // Initialize D2_4 PC(9), D2_6 PC(12) as floating inputs
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOC_BASE,
                        BV(9) | BV(12),
                        GPIO_MODE_IN_FLOATING,
                        GPIO_SPEED_50MHZ);

    // Initialize D2_5 PA(8) as floating input
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOA_BASE,
                        BV(8),
                        GPIO_MODE_IN_FLOATING,
                        GPIO_SPEED_50MHZ);

    // Initialize SW1..4 PA(4..7) as outputs open drain
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOA_BASE,
                        BV(4) | BV(5) | BV(6) | BV(7),
                        GPIO_MODE_OUT_OD,
                        GPIO_SPEED_50MHZ);

    // Initialize BUZ PB(5) as output push-pull
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOB_BASE,
                        BV(5),
                        GPIO_MODE_OUT_PP,
                        GPIO_SPEED_50MHZ);

    // Initialize D2_1..3 PC(6..8) as output push-pull
    stm32_gpioPinConfig((struct stm32_gpio *)GPIOC_BASE,
                        BV(6) | BV(7) | BV(8),
                        GPIO_MODE_OUT_PP,
                        GPIO_SPEED_50MHZ);

}

#define BOOL(a) ((a)?1:0)

void get_switch_status(switch_status *pkt1, switch_status *pkt2) {

    pkt1->p.sw1 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(4)));
    pkt1->p.sw2 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(5)));
    pkt1->p.sw3 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOB_BASE, BV(0)));
    pkt1->p.sw4 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOB_BASE, BV(1)));
    pkt1->p.sw5 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOB_BASE, BV(12)));
    pkt1->p.sw6 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOB_BASE, BV(15)));

    pkt2->p.sw1 = 0; //BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(6)));
    pkt2->p.sw2 = 0; //BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(7)));
    pkt2->p.sw3 = 0; //BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(8)));
    pkt2->p.sw4 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(9)));
    pkt2->p.sw5 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOA_BASE, BV(8)));
    pkt2->p.sw6 = BOOL(stm32_gpioPinRead((struct stm32_gpio *)GPIOC_BASE, BV(12)));

}

void set_switch(switch_request *pkt) {

    struct stm32_gpio *base = NULL;
    int32_t pin;

    if(pkt->p.num >= sizeof(pins)/sizeof(pins[0]))
        return;

    base = (struct stm32_gpio *)pins[pkt->p.num][0];
    pin = BV(pins[pkt->p.num][1]);

    stm32_gpioPinWrite(base, pin, pkt->p.state);
}
