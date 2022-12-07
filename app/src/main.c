/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));

#define PERIOD PWM_MSEC(1U)

void main(void)
{
	if (!device_is_ready(pwm_led0.dev)) {
		return;
	}

	pwm_set_dt(&pwm_led0, 2 * PERIOD, PERIOD);
}
