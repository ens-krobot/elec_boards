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

static uint16_t cur_period = 0;
static uint16_t cur_width = 0;

static uint32_t period = 0;
static uint16_t index_width = 0;
static uint16_t beacon_width = 0;
static uint32_t beacon_pos = 0;

PROC_DEFINE_STACK(stack_update, KERN_MINSTACKSIZE * 4);

static float calibration_data[10][2] = {
    {0.20, 300.},
    {0.15, 470.},
    {0.11, 700.},
    {0.095, 810.},
    {0.087, 970.},
    {0.050, 1200.},
    {0.0, 3000.},
};

static DECLARE_ISR(tim1_cc_irq) {
    static bool i;

    // Clear TIM1 Capture compare interrupt pending bit
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);

    // Get the Input Capture value
    cur_period = TIM_GetCapture2(TIM1);

    if (cur_period != 0) {
        cur_width = TIM_GetCapture1(TIM1);
        TIM_SetCounter(TIM1, 0);
        event_do(&got_capture);
    } else {
        cur_width = 0;
    }
    i = !i;
    if (i)
        LED_ON();
    else
        LED_OFF();
}


static void NORETURN beacon_update_process(void)
{

    bool got_index;

    for (;;) {
        event_wait(&got_capture);
        if(abs(index_width - cur_width) < index_width / 4 ||
           cur_width > index_width ||
           unlikely(index_width == 0)) {
            index_width = cur_width;
            period = cur_period;
            if (got_index == true) {
                beacon_width = 0;
                beacon_pos = 0;
                event_do(&updated_beacon);
            } else {
                got_index = true;
            }
        } else {
            if (got_index) {
                beacon_width = cur_width;
                // Position : center of index to center of beacon
                beacon_pos = period - index_width/2 + beacon_width/2;
                period += cur_period;
                got_index = false;
                event_do(&updated_beacon);
            } else {
                // reset : we detected two beacons in a row ?!
                index_width = 0;
            }
        }
    }
}

int get_beacon_positions(beacon_position *pos, beacon_lowlevel_position *pos_ll) {

    float angle;
    float angular_width;
    float t_period;
    float distance;

    float ang, angn, dis, disn;

    float distance_smooth[N_SMOOTH];
    float angle_smooth[N_SMOOTH];

    int i;
    int n = 0;
    int index;

    angle = 0.0;
    distance = 0.0;

    if (!event_waitTimeout(&updated_beacon, 0))
        return -1;

    angle_smooth[n % N_SMOOTH] = (beacon_pos * 2.0 * M_PI) / (period * 1.0);
    //angle = (beacon_pos * 2.0 * M_PI) / (period * 1.0);
    angular_width = (beacon_width * 2.0 * M_PI) / (period * 1.0);

    t_period = period * PRESCALER_VALUE / (CPU_FREQ * 1.0);

    if (angular_width != 0.0) {
        for (i = 0; calibration_data[i][0] != 0.0; i++) {
            angn = calibration_data[i+1][0];
            if (angular_width > angn) {
                ang = calibration_data[i][0];
                dis = calibration_data[i][1];
                disn = calibration_data[i+1][1];
                distance_smooth[n % N_SMOOTH] = disn - (disn - dis) / (angn - ang) * (angn - angular_width);
                //distance = disn - (disn - dis) / (angn - ang) * (angn - angular_width);
                break;
            }
        }
        index = MIN(n+1, N_SMOOTH);
        for (i = 0; i < index; i++) {
            angle = angle + angle_smooth[i];
            distance = distance + distance_smooth[i];
        }
        angle /= index;
        distance /= index;
        n++;
    } else {
        n = 0;
    }

    pos->p.angle = (uint16_t)(angle * 10000.);
    pos->p.distance = (uint16_t)distance;
    pos->p.period = (uint16_t)(t_period * 10000.);

    pos_ll->p.angle = angle == 0.0 ? 0 : (uint16_t)(angle_smooth[(n-1) % N_SMOOTH] * 10000.);
    pos_ll->p.width = (uint16_t)(angle * 100000.);
    pos_ll->p.period = period;

    return 0;
}

void beacon_init(void) {

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef       TIM_ICInitStructure;

    event_initGeneric(&got_capture);
    event_initGeneric(&updated_beacon);

    sysirq_setHandler(TIM1_CC_IRQHANDLER, tim1_cc_irq);

    RCC->APB2ENR |= RCC_APB2_GPIOA | RCC_APB2_TIM1;

    // Enable TIM1_CH1
    stm32_gpioPinConfig(((struct stm32_gpio *)GPIOA_BASE), BV(8),
                        GPIO_MODE_IN_FLOATING, GPIO_SPEED_50MHZ);


    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER_VALUE;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // PWM input on TIM1_CH1
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
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

    /* TIM enable counter */
    TIM_Cmd(TIM1, ENABLE);

    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);

    /* Create a new child process */
    proc_new(beacon_update_process, NULL, sizeof(stack_update), stack_update);
}
