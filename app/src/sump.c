/*
Copyright (c) 2022, YuLong Yao <feilongphone@gmail.com>
	this code is based on:
	    https://github.com/ddrown/stm32-sump
	    https://github.com/gillham/logic_analyzer/blob/master/logic_analyzer.ino

	modified to work under zephyr RTOS
*/

/* Comment from origin author */
/*
 *
 * SUMP Protocol Implementation for Arduino boards.
 *
 * Copyright (c) 2011,2012,2013,2014,2015 Andrew Gillham
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY ANDREW GILLHAM ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANDREW GILLHAM BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/* Comment from secondary author: Dan Drown */
// this code is based on:
// https://github.com/gillham/logic_analyzer/blob/master/logic_analyzer.ino
// modified to work under stm32 HAL

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(sump, CONFIG_SUMP_LOG_LEVEL);

#include <stdio.h>
#include <string.h>

#include "sump.h"
#include <drivers/sump_port/sump_port.h>

K_SEM_DEFINE(sample_sem, 0, 1);
unsigned char sample_buffer[CONFIG_SAMPLE_BUFFER_SIZE];

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static void setupDelay(uint32_t divider)
{
	// todo:
	if (divider >= 11 && divider < 65536) {
		// HAL_TIM_Base_Stop(&htim1);
		// htim1.Init.Period = divider;
		// if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		// Error_Handler();
		// }
		// HAL_TIM_Base_Start(&htim1);
	} else {
		LOG_ERR("invalid divider for sample clock:  %u\n", divider);
	}
}

static void get_metadata()
{
#define PRINT_METADATA(id, fmt, ...)                                                               \
	uart_poll_out(uart_dev, id);                                                               \
	printf(fmt, __VA_ARGS__);                                                                  \
	uart_poll_out(uart_dev, 0x00)

#define PRINT_METADATA_SAMPLE_MEMORY(size)                                                         \
	uart_poll_out(uart_dev, 0x21);                                                             \
	uart_poll_out(uart_dev, 0xFF & (size >> 24));                                              \
	uart_poll_out(uart_dev, 0xFF & (size >> 16));                                              \
	uart_poll_out(uart_dev, 0xFF & (size >> 8));                                               \
	uart_poll_out(uart_dev, 0xFF & (size))

#define PRINT_METADATA_SAMPLE_RATE(rate)                                                           \
	uart_poll_out(uart_dev, 0x23);                                                             \
	uart_poll_out(uart_dev, 0xFF & (rate >> 24));                                              \
	uart_poll_out(uart_dev, 0xFF & (rate >> 16));                                              \
	uart_poll_out(uart_dev, 0xFF & (rate >> 8));                                               \
	uart_poll_out(uart_dev, 0xFF & (rate))

#define PRINT_METADATA_PROBE_NUMS(n)                                                               \
	uart_poll_out(uart_dev, 0x40);                                                             \
	uart_poll_out(uart_dev, n)

#define PRINT_METADATA_PROTOCOL_VERSION(version)                                                   \
	uart_poll_out(uart_dev, 0x41);                                                             \
	uart_poll_out(uart_dev, version)

#define PRINT_METADATA_END() uart_poll_out(uart_dev, 0x00)

	// device name
	PRINT_METADATA(0x01, "ZephyrSUMP-%s", CONFIG_BOARD);

	// firmware version
	PRINT_METADATA(0x02, "%d.%02d", CONFIG_SUMP_VERSION_MAJOR, CONFIG_SUMP_VERSION_MINOR);

	// sample memory with >> 8
	PRINT_METADATA_SAMPLE_MEMORY(CONFIG_SAMPLE_BUFFER_SIZE);

	// sample rate
	PRINT_METADATA_SAMPLE_RATE(CONFIG_SAMPLE_RATE);

	// number of probes
	PRINT_METADATA_PROBE_NUMS(8);

	// protocol version
	PRINT_METADATA_PROTOCOL_VERSION(2);

	// end of data
	PRINT_METADATA_END();
}

static uint8_t trigger, trigger_values, rleEnabled;
static uint32_t readCount = CONFIG_SAMPLE_BUFFER_SIZE, delayCount, divider = 11;

void extended_sump_command(char last_cmd, uint8_t *extended_cmd_arg)
{
	switch (last_cmd) {
	case SUMP_TRIGGER_MASK:
		/*
		 * the trigger mask byte has a '1' for each enabled trigger so
		 * we can just use it directly as our trigger mask.
		 */
		trigger = extended_cmd_arg[0];
		LOG_DBG("trigger %u\n", trigger);
		break;
	case SUMP_TRIGGER_VALUES:
		/*
		 * trigger_values can be used directly as the value of each bit
		 * defines whether we're looking for it to be high or low.
		 */
		trigger_values = extended_cmd_arg[0];
		LOG_DBG("t_values %u\n", trigger_values);
		break;
	case SUMP_TRIGGER_CONFIG:
		LOG_DBG("t_config = ?\n");
		/* read the rest of the command bytes, but ignore them. */
		break;
	case SUMP_SET_DIVIDER:
		/*
		 * the shifting needs to be done on the 32bit unsigned long variable
		 * so that << 16 doesn't end up as zero.
		 */
		divider = extended_cmd_arg[2];
		divider = divider << 8;
		divider += extended_cmd_arg[1];
		divider = divider << 8;
		divider += extended_cmd_arg[0];
		LOG_DBG("divider %u\n", divider);
		setupDelay(divider);
		break;
	case SUMP_SET_READ_DELAY_COUNT:
		/*
		 * this just sets up how many samples there should be before
		 * and after the trigger fires.  The readCount is total samples
		 * to return and delayCount number of samples after the trigger.
		 * this sets the buffer splits like 0/100, 25/75, 50/50
		 * for example if readCount == delayCount then we should
		 * return all samples starting from the trigger point.
		 * if delayCount < readCount we return (readCount - delayCount) of
		 * samples from before the trigger fired.
		 */
		readCount = 4 * (((extended_cmd_arg[1] << 8) | extended_cmd_arg[0]) + 1);
		if (readCount > CONFIG_SAMPLE_BUFFER_SIZE)
			readCount = CONFIG_SAMPLE_BUFFER_SIZE;
		LOG_DBG("read# %u\n", readCount);
		delayCount = 4 * (((extended_cmd_arg[3] << 8) | extended_cmd_arg[2]) + 1);
		if (delayCount > CONFIG_SAMPLE_BUFFER_SIZE)
			delayCount = CONFIG_SAMPLE_BUFFER_SIZE;
		LOG_DBG("delay# %u\n", delayCount);

		if (readCount == 0)
			readCount = CONFIG_SAMPLE_BUFFER_SIZE;
		readCount = CONFIG_SAMPLE_BUFFER_SIZE;
		break; // TODO
	case SUMP_SET_FLAGS:
		/* read the rest of the command bytes and check if RLE is enabled. */
		rleEnabled = ((extended_cmd_arg[1] & 0b1000000) != 0);
		LOG_DBG("rle %u\n", rleEnabled);
		break; // TODO
	}
}

void sump_read_command(sump_cmd_state *read_state, char c)
{
	switch (c) {
	case SUMP_RESET:
		/*
		 * We don't do anything here as some unsupported extended commands have
		 * zero bytes and are mistaken as resets.  This can trigger false resets
		 * so we don't erase the data or do anything for a reset.
		 */
		break;
	case SUMP_QUERY:
		/* return the expected bytes. */
		printf("1ALS");
		break;
	case SUMP_ARM:
		/*
		 * Zero out any previous samples before arming.
		 * Done here instead via reset due to spurious resets.
		 */
		memset(sample_buffer, '\0', CONFIG_SAMPLE_BUFFER_SIZE);
		k_sem_give(&sample_sem);
		break;
	case SUMP_TRIGGER_MASK:
	case SUMP_TRIGGER_VALUES:
	case SUMP_TRIGGER_CONFIG:
	case SUMP_SET_DIVIDER:
	case SUMP_SET_READ_DELAY_COUNT:
	case SUMP_SET_FLAGS:
		// extended commands have a 4 byte argument
		*read_state = STATE_READ4;
		break;
	case SUMP_GET_METADATA:
		/*
		 * We return a description of our capabilities.
		 * Check the function's comments below.
		 */
		get_metadata();
		break;
	case SUMP_SELF_TEST:
		/* ignored. */
		break;
	default:
		/* ignore any unrecognized bytes. */
		break;
	}
}

void sump_commander(void)
{
	uint8_t c;

	static sump_cmd_state read_state = STATE_CMD;
	static char last_cmd = 0;
	static uint8_t extended_cmd_arg[4];

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not found!\n");
	}

	while (true) {
		int ret = uart_poll_in(uart_dev, &c);
		if (ret < 0) {
			/* nothing recv */
			k_sleep(Z_TIMEOUT_TICKS(1));
			continue;
		}

		if (read_state == STATE_CMD) {
			last_cmd = c;
			sump_read_command(&read_state, c);
		} else if (read_state >= STATE_READ4 && read_state <= STATE_READ1) {
			extended_cmd_arg[read_state] = c;
			read_state = read_state + 1;
			if (read_state == STATE_EXTENDED_CMD) {
				extended_sump_command(last_cmd, extended_cmd_arg);
				read_state = STATE_CMD;
			}
		} else {
			LOG_ERR("unknown state %u\n", read_state);
		}
	}
}

void sump_main(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		LOG_ERR("led error\n");
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("led error\n");
	}

	while (true) {
		if (k_sem_take(&sample_sem, K_FOREVER) != 0) {
			LOG_ERR("Input data not available!\n");
			k_msleep(1);
			continue;
		}

		ret = gpio_pin_toggle_dt(&led);
		SumpPortSample(sample_buffer, readCount);

		// give pc some time to prepare recv
		ret = gpio_pin_toggle_dt(&led);
		k_msleep(200);
		ret = gpio_pin_toggle_dt(&led);

		for (size_t i = 0; i < readCount; i++)
			uart_poll_out(uart_dev, sample_buffer[i]);
		ret = gpio_pin_toggle_dt(&led);
	}
}

#define PRIORITY_APP_SUMP  7
#define STACKSIZE_APP_SUMP 1024

K_THREAD_DEFINE(sump_main_id, STACKSIZE_APP_SUMP, sump_main, NULL, NULL, NULL, PRIORITY_APP_SUMP, 0,
		0);

#define PRIORITY_APP_COMMANDER	8
#define STACKSIZE_APP_COMMANDER 1024
K_THREAD_DEFINE(sump_commander_id, STACKSIZE_APP_COMMANDER, sump_commander, NULL, NULL, NULL,
		PRIORITY_APP_COMMANDER, 0, 0);
