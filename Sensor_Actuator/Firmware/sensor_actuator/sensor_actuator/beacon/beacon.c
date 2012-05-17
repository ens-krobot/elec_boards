/**
 * Rotary beacon
 *
 * This file contains the logic for the Rotary beacon.
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

#include "beacon.h"

static struct Event got_capture;
static struct Event updated_beacon;

static uint16_t beacon_idx = 0;
static uint32_t beacon_start[MAX_BEACONS] = {0};
static uint32_t beacon_stop[MAX_BEACONS] = {0};
static uint32_t period = 0;


static float calibration_data[10][2] = {
    {0.370, 160.},
    {0.288, 220.},
    {0.247, 240.},
    {0.206, 370.},
    {0.165, 470.},
    {0.124, 700.},
    {0.083, 1000.},
    {0.042, 1440.},
    {0.0, 1500.},
};

/**
 *  ______________________       _____________
 *          :             |_____|
 *          ^             ^     ^
 *         RAZ           IC1   IC2
 *                             IRQ
*/
static DECLARE_ISR(tim1_cc_irq) {
    /// Rising edge
    // Clear TIM8 Capture compare interrupt pending bit
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);

    if (beacon_idx < MAX_BEACONS) {
        beacon_start[beacon_idx] = TIM_GetCapture1(TIM1);
        beacon_stop[beacon_idx] = TIM_GetCounter(TIM8);
        ++beacon_idx;
    }
}

static DECLARE_ISR(tim8_cc_irq) {
    // Clear TIM8 Capture compare interrupt pending bit
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);

    // Get the Input Capture value
    period = TIM_GetCapture1(TIM8);

    if (beacon_idx > 0)
        event_do(&got_capture);

    beacon_idx = 0;

    TIM_SetCounter(TIM8, 0);
    TIM_SetCounter(TIM1, 0);
}

int get_beacon_positions(int beacon_id, beacon_position *pos, beacon_lowlevel_position *pos_ll) {

    float beacon_pos;
    float beacon_width;

    static float distance_smooth[N_SMOOTH] = {0};
    //static float angle_smooth[N_SMOOTH] = {0};        // Issue at the 2*PI <-> 0 transition!

    float angular_width;
    //float t_period;
    float angle;
    //float angle_avg = 0.0;
    float distance_avg = 0.0;

    float ang, angn, dis, disn;

    int i;
    static int n = 0;
    static int index = 0;

    if (!event_waitTimeout(&got_capture, 0))
        return -1;

    if (beacon_stop[beacon_id] >= beacon_start[beacon_id])
        beacon_width = beacon_stop[beacon_id] - beacon_start[beacon_id];
    else
        beacon_width = beacon_stop[beacon_id] + period - beacon_start[beacon_id];

    beacon_pos = beacon_start[beacon_id] + beacon_width / 2.0;
    angle = 2.0 * M_PI - (beacon_pos * 2.0 * M_PI) / (float) period;
    angular_width = (beacon_width * 2.0 * M_PI) / (float) period;

    if (angular_width != 0.0) {
        //angle_smooth[n % N_SMOOTH] =  fmod(angle + BEACON_ANGLE_OFFSET, 2.0 * M_PI);
        angle = fmod(angle + BEACON_ANGLE_OFFSET, 2.0 * M_PI);

        for (i = 0; calibration_data[i][0] != 0.0; ++i) {
            angn = calibration_data[i+1][0];
            if (angular_width > angn) {
                ang = calibration_data[i][0];
                dis = calibration_data[i][1];
                disn = calibration_data[i+1][1];
                distance_smooth[n % N_SMOOTH] = disn - (disn - dis) / (angn - ang) * (angn - angular_width);
                break;
            }
        }

        index = MIN(n+1, N_SMOOTH);

        for (i = 0; i < index; i++) {
            //angle_avg+= angle_smooth[i];
            distance_avg+= distance_smooth[i];
        }

        //angle_avg/= index;
        distance_avg/= index;
        ++n;
    }
    else
        n = 0;

    // Compute the real time period (in s)
    //t_period = period * PRESCALER_VALUE / (float) CPU_FREQ;

    //pos->p.angle = (uint16_t)(angle_avg * 10000.);
    pos->p.angle[beacon_id] = (uint16_t)(angle * 10000.);
    pos->p.distance[beacon_id] = (uint16_t)(distance_avg);
    //pos->p.period = (uint16_t)(t_period * 10000.);

    pos_ll->p.angle = (uint16_t)(angle * 10000.);
    pos_ll->p.width = (uint16_t)(angular_width * 10000.);
    pos_ll->p.period = period;

/*
    pos_ll->p.angle = (uint16_t)(beacon_start[beacon_id]);
    pos_ll->p.width = (uint16_t)(beacon_stop[beacon_id]);
    pos_ll->p.period = period;
*/
    return 0;
}

void beacon_init(void) {

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef       TIM_ICInitStructure;

    event_initGeneric(&got_capture);
    event_initGeneric(&updated_beacon);

    sysirq_setHandler(TIM1_CC_IRQHANDLER, tim1_cc_irq);
    sysirq_setHandler(TIM8_CC_IRQHANDLER, tim8_cc_irq);

    RCC->APB2ENR |= RCC_APB2_GPIOA | RCC_APB2_TIM1 | RCC_APB2_GPIOC | RCC_APB2_TIM8;

    // Enable TIM1_CH1 (E3Z-R61, Open collector with pull-up + Light-ON switch config => detection at falling edge)
    stm32_gpioPinConfig(((struct stm32_gpio *)GPIOA_BASE), BV(8),
                        GPIO_MODE_IN_FLOATING, GPIO_SPEED_50MHZ);


    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER_VALUE;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // PWM input on TIM1_CH1
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;        // /!\ *Falling*
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 10;

    TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);

    /* Select the TIM1 Input Trigger: TI2FP2 */
    TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);

    /* Select the slave Mode: Reset Mode */
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);

    /* Enable the Master/Slave Mode */
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);

    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);

    /* TIM enable counter */
    TIM_Cmd(TIM1, ENABLE);

    // Enable TIM8_CH1
    stm32_gpioPinConfig(((struct stm32_gpio *)GPIOC_BASE), BV(6),
                        GPIO_MODE_IN_FLOATING, GPIO_SPEED_50MHZ);


    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER_VALUE;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    // PWM input on TIM8_CH1
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 10;

    TIM_PWMIConfig(TIM8, &TIM_ICInitStructure);

    /* Enable the CC1 Interrupt Request */
    TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);

    /* TIM enable counter */
    TIM_Cmd(TIM8, ENABLE);
}
