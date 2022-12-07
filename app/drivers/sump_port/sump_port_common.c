
#include "sump_port.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(sump_port, CONFIG_SUMP_LOG_LEVEL);

BUILD_ASSERT(DT_N_S_sump_mode_P_ports_LEN == 1, "multi-port sampling is not support at this time.");

// todo: move to dts later
#define GPIO_PORT DT_NODELABEL(gpioc)

int SumpPortSample(unsigned char *buf, unsigned long long size)
{
	const struct device *dev = DEVICE_DT_GET(GPIO_PORT);
	uint32_t p;
	uint64_t tick;

	if (!device_is_ready(dev)) {
		printk("%s: device not ready.\n", dev->name);
		return -EPERM;
	}

	// this may reconfig uart port or leds
	for (size_t i = 0; i < 32; i++) {
		if (gpio_pin_configure(dev, i, GPIO_INPUT) < 0)
			break;
	}

	for (unsigned long long i = 0; i < size; i++) {
		// store port data
		gpio_port_get_raw(dev, &p);
		*buf++ = (p & 0xff);

		// sleep to next trig
		k_msleep(1);
	}

	return 0;
}
