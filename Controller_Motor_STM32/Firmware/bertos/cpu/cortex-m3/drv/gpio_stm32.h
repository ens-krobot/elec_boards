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
 */

#ifndef GPIO_STM32_H
#define GPIO_STM32_H

#include <io/stm32.h>

/**
 * GPIO mode
 * \{
 */
enum
{
	GPIO_MODE_AIN = 0x0,
	GPIO_MODE_IN_FLOATING = 0x04,
	GPIO_MODE_IPD = 0x28,
	GPIO_MODE_IPU = 0x48,
	GPIO_MODE_OUT_OD = 0x14,
	GPIO_MODE_OUT_PP = 0x10,
	GPIO_MODE_AF_OD = 0x1C,
	GPIO_MODE_AF_PP = 0x18,
};
/*\}*/

/**
 * GPIO speed
 *\{
 */
enum
{
	GPIO_SPEED_10MHZ = 1,
	GPIO_SPEED_2MHZ,
	GPIO_SPEED_50MHZ,
};
/*\}*/

/**
 * AFIO function remap
 *\{
 */
typedef enum
{
    GPIO_REMAP_SPI1             = ((uint32_t)0x00000001),  /*!< SPI1 Alternate Function mapping */
    GPIO_REMAP_I2C1             = ((uint32_t)0x00000002),  /*!< I2C1 Alternate Function mapping */
    GPIO_REMAP_USART1           = ((uint32_t)0x00000004),  /*!< USART1 Alternate Function mapping */
    GPIO_REMAP_USART2           = ((uint32_t)0x00000008),  /*!< USART2	Alternate Function mapping */
    GPIO_PARTIALREMAP_USART3    = ((uint32_t)0x00140010),  /*!< USART3	Partial Alternate Function mapping */
    GPIO_FULLREMAP_USART3       = ((uint32_t)0x00140030),  /*!< USART3 Full Alternate Function mapping */
    GPIO_PARTIALREMAP_TIM1      = ((uint32_t)0x00160040),  /*!< TIM1 Partial Alternate Function mapping */
    GPIO_FULLREMAP_TIM1         = ((uint32_t)0x001600C0),  /*!< TIM1 Full Alternate Function mapping */
    GPIO_PARTIALREMAP1_TIM2     = ((uint32_t)0x00180100),  /*!< TIM2 Partial1 Alternate Function mapping */
    GPIO_PARTIALREMAP2_TIM2     = ((uint32_t)0x00180200),  /*!< TIM2 Partial2 Alternate Function mapping */
    GPIO_FULLREMAP_TIM2         = ((uint32_t)0x00180300),  /*!< TIM2 Full Alternate Function mapping */
    GPIO_PARTIALREMAP_TIM3      = ((uint32_t)0x001A0800),  /*!< TIM3 Partial Alternate Function mapping */
    GPIO_FULLREMAP_TIM3         = ((uint32_t)0x001A0C00),  /*!< TIM3 Full Alternate Function mapping */
    GPIO_REMAP_TIM4             = ((uint32_t)0x00001000),  /*!< TIM4 Alternate Function mapping */
    GPIO_REMAP1_CAN1            = ((uint32_t)0x001D4000),  /*!< CAN1 Alternate Function mapping */
    GPIO_REMAP2_CAN1            = ((uint32_t)0x001D6000),  /*!< CAN1 Alternate Function mapping */
    GPIO_REMAP_PD01             = ((uint32_t)0x00008000),  /*!< PD01 Alternate Function mapping */
    GPIO_REMAP_TIM5CH4_LSI      = ((uint32_t)0x00200001),  /*!< LSI connected to TIM5 Channel4 input capture for calibration */
    GPIO_REMAP_ADC1_ETRGINJ     = ((uint32_t)0x00200002),  /*!< ADC1 External Trigger Injected Conversion remapping */
    GPIO_REMAP_ADC1_ETRGREG     = ((uint32_t)0x00200004),  /*!< ADC1 External Trigger Regular Conversion remapping */
    GPIO_REMAP_ADC2_ETRGINJ     = ((uint32_t)0x00200008),  /*!< ADC2 External Trigger Injected Conversion remapping */
    GPIO_REMAP_ADC2_ETRGREG     = ((uint32_t)0x00200010),  /*!< ADC2 External Trigger Regular Conversion remapping */
    GPIO_REMAP_ETH              = ((uint32_t)0x00200020),  /*!< Ethernet remapping (only for Connectivity line devices) */
    GPIO_REMAP_CAN2             = ((uint32_t)0x00200040),  /*!< CAN2 remapping (only for Connectivity line devices) */
    GPIO_REMAP_SWJ_NoJTRST      = ((uint32_t)0x00300100),  /*!< Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST */
    GPIO_REMAP_SWJ_JTAGDisable  = ((uint32_t)0x00300200),  /*!< JTAG-DP Disabled and SW-DP Enabled */
    GPIO_REMAP_SWJ_Disable      = ((uint32_t)0x00300400),  /*!< Full SWJ Disabled (JTAG-DP + SW-DP) */
    GPIO_REMAP_SPI3             = ((uint32_t)0x00201000),  /*!< SPI3 Alternate Function mapping (only for Connectivity line devices) */
    GPIO_REMAP_TIM2ITR1_PTP_SOF = ((uint32_t)0x00202000),  /*!< Ethernet PTP output or USB OTG SOF (Start of Frame) connected
                                                                to TIM2 Internal Trigger 1 for calibration
                                                                (only for Connectivity line devices) */
    GPIO_REMAP_PTP_PPS          = ((uint32_t)0x00204000),  /*!< Ethernet MAC PPS_PTS output on PB05 (only for Connectivity line devices) */ 
} stm32_afio_remap;
/*\}*/

/**
 * AFIO remap state
 */
typedef enum {
    GPIO_REMAP_DISABLE = 0,
    GPIO_REMAP_ENABLE,
} stm32_afio_remap_state;

/**
 * Write a value to the specified pin(s)
 *
 * \param base gpio register address
 * \param pins mask of pins that we want set or clear
 * \param val true to set selected pins of false to clear they.
 */
INLINE void stm32_gpioPinWrite(struct stm32_gpio *base, uint16_t pins, bool val)
{
	if (val)
		base->BSRR |= pins;
	else
		base->BRR  |= pins;
}

/**
 * Read a value from the specified pin(s)
 *
 * \param base gpio register address
 * \param pins mask of pins that we want read
 */
INLINE uint16_t stm32_gpioPinRead(struct stm32_gpio *base, uint16_t pins)
{
	return (base->IDR & pins);
}

/**
 * Initialize a GPIO peripheral configuration
 *
 * \param base gpio register address
 * \param pins mask of pins that we want to configure
 * \param mode select the behaviour of selected pins
 * \param speed clock frequency for selected gpio ports
 */
int stm32_gpioPinConfig(struct stm32_gpio *base, uint16_t pins, uint8_t mode, uint8_t speed);

/**
 * Remap a pin to the specified function
 *
 * \param remap GPIO function to remap
 * \param newstate State of the new mapping.
 */
void stm32_gpioRemap(stm32_afio_remap remap, stm32_afio_remap_state newstate);

#endif /* GPIO_STM32_H */
