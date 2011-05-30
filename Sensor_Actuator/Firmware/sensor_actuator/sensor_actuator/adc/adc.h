/**
 * Sensor and Actuator Board : ADC Interfacing
 *
 * This file contains the interface for the ADCs
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

#ifndef ADC_H__
#define ADC_H__

#include <stdlib.h>
#include <stdint.h>

#include <drv/adc.h>
#include <drv/clock_stm32.h>
#include <drv/gpio_stm32.h>

#include "../can/can_messages.h"

/**
 * Initialize the ADCs
 */
void sa_adc_init(void);

/**
 * Fetch the ADC data in the CAN packets
 */
void get_adc_values(adc_values *pkt1, adc_values *pkt2);


#endif
