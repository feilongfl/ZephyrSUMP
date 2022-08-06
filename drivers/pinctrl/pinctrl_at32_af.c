/*
 * Copyright (c) 2022 YuLong Yao <feilongphone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

BUILD_ASSERT((AT32_PULL_NONE == AT32_GPIO_PULL_NONE) &&
	     (AT32_PULL_UP == AT32_GPIO_PULL_UP) &&
	     (AT32_PULL_DOWN == AT32_GPIO_PULL_DOWN),
	     "pinctrl pull-up/down definitions != HAL definitions");

BUILD_ASSERT((AT32_OUTPUT_PUSH_PULL == AT32_GPIO_OUTPUT_PUSH_PULL) &&
	     (AT32_OUTPUT_OPEN_DRAIN == AT32_GPIO_OUTPUT_OPEN_DRAIN),
	     "pinctrl output type definitions != HAL definitions");

/** Utility macro that expands to the GPIO port address if it exists */
#define AT32_PORT_ADDR_OR_NONE(nodelabel)		     \
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)), \
		    (DT_REG_ADDR(DT_NODELABEL(nodelabel)), ), ())

/** Utility macro that expands to the GPIO RCU if it exists */
#define AT32_PORT_RCU_OR_NONE(nodelabel)		     \
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)), \
		    (DT_PROP(DT_NODELABEL(nodelabel), rcu_periph_clock), ), ())

/** AT32 port addresses */
static const uint32_t at32_port_addrs[] = {
	AT32_PORT_ADDR_OR_NONE(gpioa)
	AT32_PORT_ADDR_OR_NONE(gpiob)
	AT32_PORT_ADDR_OR_NONE(gpioc)
	AT32_PORT_ADDR_OR_NONE(gpiod)
	AT32_PORT_ADDR_OR_NONE(gpioe)
	AT32_PORT_ADDR_OR_NONE(gpiof)
	AT32_PORT_ADDR_OR_NONE(gpiog)
	AT32_PORT_ADDR_OR_NONE(gpioh)
	AT32_PORT_ADDR_OR_NONE(gpioi)
};

/** AT32 port RCUs */
static const uint32_t at32_port_rcus[] = {
	AT32_PORT_RCU_OR_NONE(gpioa)
	AT32_PORT_RCU_OR_NONE(gpiob)
	AT32_PORT_RCU_OR_NONE(gpioc)
	AT32_PORT_RCU_OR_NONE(gpiod)
	AT32_PORT_RCU_OR_NONE(gpioe)
	AT32_PORT_RCU_OR_NONE(gpiof)
	AT32_PORT_RCU_OR_NONE(gpiog)
	AT32_PORT_RCU_OR_NONE(gpioh)
	AT32_PORT_RCU_OR_NONE(gpioi)
};

/**
 * @brief Configure a pin.
 *
 * @param pin The pin to configure.
 */
static void pinctrl_configure_pin(pinctrl_soc_pin_t pin)
{
	uint8_t port_idx;
	uint32_t rcu, port, pin_num, af, mode;
	gpio_init_type gpio_type;

	port_idx = AT32_PORT_GET(pin);
	__ASSERT_NO_MSG(port_idx < ARRAY_SIZE(at32_port_addrs));

	rcu = at32_port_rcus[port_idx];
	port = at32_port_addrs[port_idx];
	pin_num = AT32_PIN_GET(pin);
	af = AT32_AF_GET(pin);

	crm_periph_clock_enable(rcu, TRUE);

	if (af != AT32_ANALOG) {
		mode = AT32_GPIO_MODE_MUX;
		gpio_pin_mux_config((void *)port, pin_num, af);
        } else {
		mode = AT32_GPIO_MODE_ANALOG;
	}

	gpio_type.gpio_pins = BIT(pin_num);
	gpio_type.gpio_out_type = AT32_OTYPE_GET(pin);
	gpio_type.gpio_pull = AT32_PUPD_GET(pin);
	gpio_type.gpio_mode = mode;
	gpio_type.gpio_drive_strength = AT32_GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_init((void *)port, &gpio_type);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(pins[i]);
	}

	return 0;
}
