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
 * Copyright 2010 Develer S.r.l. (http://www.develer.com/)
 *
 * -->
 *
 * \brief STM32 GPIO control interface.
 *
 * \author Andrea Righi <arighi@develer.com>
 */

#include "gpio_stm32.h"

#include <cfg/compiler.h>
#include <cfg/debug.h>

#include <io/stm32.h>

#define LSB_MASK                    ((uint16_t)0xFFFF)
#define DBGAFR_POSITION_MASK        ((uint32_t)0x000F0000)
#define DBGAFR_SWJCFG_MASK          ((uint32_t)0xF0FFFFFF)
#define DBGAFR_LOCATION_MASK        ((uint32_t)0x00200000)
#define DBGAFR_NUMBITS_MASK         ((uint32_t)0x00100000)

/**
 * Configure a GPIO pin
 *
 * \param base Base address of the GPIO port
 * \param pins Bit-packed representation of the pin(s)
 * \param mode Pin(s) configuration mode
 * \param speed Output drive speed
 *
 * Return 0 on success, otherwise a negative value.
 */
int stm32_gpioPinConfig(struct stm32_gpio *base,
			uint16_t pins, uint8_t mode, uint8_t speed)
{
	uint32_t reg_mode = mode & 0x0f;
	int i;

	if (mode & 0x10)
		reg_mode |= speed;

	if (pins & 0xff)
	{
		uint32_t reg = base->CRL;

		for (i = 0; i < 8; i++)
		{
			uint32_t pos = 1 << i;

			if (pins & pos)
			{
				pos = i << 2;
				reg &= ~(0x0f << pos);
				reg |= reg_mode << pos;

				if (mode == GPIO_MODE_IPD)
					base->BRR = 0x01 << i;
				if (mode == GPIO_MODE_IPU)
					base->BSRR = 0x01 << i;
			}
		}
		base->CRL = reg;
	}
	if (pins > 0xff)
	{
		uint32_t reg = base->CRH;

		for (i = 0; i < 8; i++)
		{
			uint32_t pos = 1 << (i + 8);

			if (pins & pos)
			{
				pos = i << 2;
				reg &= ~(0x0f << pos);
				reg |= reg_mode << pos;

				if (mode == GPIO_MODE_IPD)
					base->BRR = 0x01 << (i + 8);
				if (mode == GPIO_MODE_IPU)
					base->BSRR = 0x01 << (i + 8);
			}
		}
		base->CRH = reg;
	}
	return 0;
}

void stm32_gpioRemap(stm32_afio_remap remap, stm32_afio_remap_state newstate)
{
    uint32_t tmp = 0x00, tmp1 = 0x00, tmpreg = 0x00, tmpmask = 0x00;
    
    struct stm32_afio *AFIO = (struct stm32_afio *)AFIO_BASE;
    
    tmpreg = AFIO->MAPR;
    
    tmpmask = (remap & DBGAFR_POSITION_MASK) >> 0x10;
    tmp = remap & LSB_MASK;
    
    if ((remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK))
    {
        tmpreg &= DBGAFR_SWJCFG_MASK;
        AFIO->MAPR &= DBGAFR_SWJCFG_MASK;
    }
    else if ((remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK)
    {
        tmp1 = ((uint32_t)0x03) << tmpmask;
        tmpreg &= ~tmp1;
        tmpreg |= ~DBGAFR_SWJCFG_MASK;
    }
    else
    {
        tmpreg &= ~(tmp << ((remap >> 0x15)*0x10));
        tmpreg |= ~DBGAFR_SWJCFG_MASK;
    }

    if (newstate == GPIO_REMAP_ENABLE)
    {
        tmpreg |= (tmp << ((remap >> 0x15)*0x10));
    }

    AFIO->MAPR = tmpreg;
}
