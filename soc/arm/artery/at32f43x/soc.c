/*
 * Copyright (c) 2022 YuLong Yao <feilongphone@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <soc.h>

static int at32f4xx_soc_init(const struct device *dev)
{
	uint32_t key;

	ARG_UNUSED(dev);

	key = irq_lock();

	SystemInit();
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(at32f4xx_soc_init, PRE_KERNEL_1, 0);
