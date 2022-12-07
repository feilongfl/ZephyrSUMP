
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "sump_port.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(sump_port, CONFIG_SUMP_LOG_LEVEL);

BUILD_ASSERT(DT_N_S_sump_mode_P_ports_LEN == 1, "multi-port sampling is not support at this time.");

// todo: move to dts later
#define GPIO_PORT DT_NODELABEL(gpioc)

#define GPIO_IDT(gpiox) ((gpiox) + 0x00000010U) /*!< GPIO port input status register */

struct gpio_at32_config {
	struct gpio_driver_config common;
	uint32_t reg;
};

// todo: add rate ctrl later
static inline void SumpPortByASM(unsigned char *buf, unsigned char *buf_end)
{
	const struct device *port = DEVICE_DT_GET(GPIO_PORT);
	const struct gpio_driver_config *const cfg =
		(const struct gpio_driver_config *)port->config;

	__asm__("ldr     r1, [%1]\n"
		"poll:"
		"strb    r1, [%0, #0]\n"
		"adds    %0, #1\n"
		"cmp     %2, %0\n"
		"ldr     r1, [%1]\n" // moving this before the cmp slows the loop down to 6MHz
		"bhi.n   poll\n"
		: /* no outputs */
		: "r"(buf), "r"(GPIO_IDT(((struct gpio_at32_config *)cfg)->reg)), "r"(buf_end)
		: "r1");
}

int SumpPortSample(unsigned char *buf, unsigned long long size)
{
	const struct device *dev = DEVICE_DT_GET(GPIO_PORT);

	if (!device_is_ready(dev)) {
		printk("%s: device not ready.\n", dev->name);
		return -EPERM;
	}

	// wait trigger later

	// this may reconfig uart port or leds
	for (size_t i = 0; i < 32; i++) {
		if (gpio_pin_configure(dev, i, GPIO_INPUT) < 0)
			break;
	}

	SumpPortByASM(buf, buf + size);

	return 0;
}
