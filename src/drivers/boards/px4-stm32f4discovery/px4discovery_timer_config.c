/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file px4discovery_timer_config.c
 *
 * Configuration data for the stm32 pwm_servo driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#include "board_config.h"

#include <drivers/drv_pwm_output.h>
#include <drivers/stm32/drv_io_timer.h>


/* IO Timers normally free-run at 1MHz
 * Here we make adjustments to the Frequency that sets the timer's prescale
 * so that the prescale is set to 0
 */

#define TIM2_CLKIN 1000000
#define TIM4_CLKIN 1000000

__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
	{
		.base = STM32_TIM2_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM2EN,
		.clock_freq = TIM2_CLKIN,
		.first_channel_index = 0,
		.last_channel_index = 2,
		.handler = io_timer_handler0,
		.vectorno =  STM32_IRQ_TIM2,
	},
	{
		.base = STM32_TIM4_BASE,
		.clock_register = STM32_RCC_APB1ENR,
		.clock_bit = RCC_APB1ENR_TIM4EN,
		.clock_freq = TIM4_CLKIN,
		.first_channel_index = 3,
		.last_channel_index = 3,
		.handler = io_timer_handler1,
		.vectorno =  STM32_IRQ_TIM4,
	}
};

__EXPORT const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	{
		// M1
		.gpio_out = GPIO_TIM2_CH2OUT,
		.gpio_in = GPIO_TIM2_CH2IN,
		.timer_index = 0,
		.timer_channel = 2,
		.ccr_offset = STM32_GTIM_CCR2_OFFSET,
		.masks  = GTIM_SR_CC2IF | GTIM_SR_CC2OF
	},
	{
		// M3
		.gpio_out = GPIO_TIM2_CH1OUT,
		.gpio_in = GPIO_TIM2_CH1IN,
		.timer_index = 0,
		.timer_channel = 1,
		.ccr_offset = STM32_GTIM_CCR1_OFFSET,
		.masks  = GTIM_SR_CC1IF | GTIM_SR_CC1OF
	},
	{
		// M4
		.gpio_out = GPIO_TIM4_CH4OUT,
		.gpio_in = GPIO_TIM4_CH4IN,
		.timer_index = 1,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
	{
		// M2
		.gpio_out = GPIO_TIM2_CH4OUT,
		.gpio_in = GPIO_TIM2_CH4IN,
		.timer_index = 0,
		.timer_channel = 4,
		.ccr_offset = STM32_GTIM_CCR4_OFFSET,
		.masks  = GTIM_SR_CC4IF | GTIM_SR_CC4OF
	},
};
