/*
 * Copyright (c) 2022 YuLong Yao <feilongphone@gmail.com>
 * Copyright (c) 2021 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT artery_at32_gpio

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/atery.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/interrupt_controller/exti_at32.h>
#include <zephyr/drivers/reset.h>

#include <soc.h>

/** SYSCFG DT node */
#define SYSCFG_NODE DT_NODELABEL(syscfg)

/** EXTISS mask */
#define EXTISS_MSK	       0xFU
/** EXTISS line step size */
#define EXTISS_STEP	       4U
/** EXTISS line shift */
#define EXTISS_LINE_SHIFT(pin) (EXTISS_STEP * ((pin) % EXTISS_STEP))

/* registers definitions */
#define GPIO_CTL(gpiox)	  REG32((gpiox) + 0x00000000U) /*!< GPIO port control register */
#define GPIO_OMODE(gpiox) REG32((gpiox) + 0x00000004U) /*!< GPIO port output mode register */
#define GPIO_OSPD0(gpiox) REG32((gpiox) + 0x00000008U) /*!< GPIO port output speed register 0 */
#define GPIO_PUD(gpiox)	  REG32((gpiox) + 0x0000000CU) /*!< GPIO port pull-up/pull-down register */
#define GPIO_ISTAT(gpiox) REG32((gpiox) + 0x00000010U) /*!< GPIO port input status register */
#define GPIO_OCTL(gpiox)  REG32((gpiox) + 0x00000014U) /*!< GPIO port output control register */
#define GPIO_BOP(gpiox)	  REG32((gpiox) + 0x00000018U) /*!< GPIO port bit operation register */
#define GPIO_LOCK(gpiox)                                                                           \
	REG32((gpiox) + 0x0000001CU) /*!< GPIO port configuration lock register                    \
				      */
#define GPIO_AFSEL0(gpiox)                                                                         \
	REG32((gpiox) + 0x00000020U) /*!< GPIO alternate function selected register 0 */
#define GPIO_AFSEL1(gpiox)                                                                         \
	REG32((gpiox) + 0x00000024U) /*!< GPIO alternate function selected register 1 */
#define GPIO_BC(gpiox)	  REG32((gpiox) + 0x00000028U) /*!< GPIO bit clear register */
#define GPIO_TG(gpiox)	  REG32((gpiox) + 0x0000002CU) /*!< GPIO port bit toggle register */
#define GPIO_OSPD1(gpiox) REG32((gpiox) + 0x0000003CU) /*!< GPIO port output speed register 1 */

/* GPIO mode configuration values */
#define GPIO_MODE_SET(n, mode) ((uint32_t)((uint32_t)(mode) << (2U * (n))))
#define GPIO_MODE_MASK(n)      ((uint32_t)((uint32_t)0x00000003U << (2U * (n))))

/* GPIO pull-up/pull-down values */
#define GPIO_PUPD_SET(n, pupd) ((uint32_t)((uint32_t)(pupd) << (2U * (n))))
#define GPIO_PUPD_MASK(n)      ((uint32_t)((uint32_t)0x00000003U << (2U * (n))))

struct gpio_at32_config {
	struct gpio_driver_config common;
	uint32_t reg;
	uint32_t rcu_periph_clock;
	uint32_t clkid;
	uint32_t clkid_exti;
	struct reset_dt_spec reset;
};

struct gpio_at32_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

/**
 * @brief EXTI ISR callback.
 *
 * @param line EXTI line (equals to GPIO pin number).
 * @param arg GPIO port instance.
 */
static void gpio_at32_isr(uint8_t line, void *arg)
{
	const struct device *dev = arg;
	struct gpio_at32_data *data = dev->data;

	gpio_fire_callbacks(&data->callbacks, dev, BIT(line));
}

/**
 * @brief Configure EXTI source selection register.
 *
 * @param port GPIO port instance.
 * @param pin GPIO pin number.
 *
 * @retval 0 on success.
 * @retval -EINVAL if pin is not valid.
 */
static int gpio_at32_configure_extiss(const struct device *port, gpio_pin_t pin)
{
	const struct gpio_at32_config *config = port->config;
	uint8_t port_index, shift;
	volatile uint32_t *extiss;

	switch (pin / EXTISS_STEP) {
	case 0U:
		extiss = &SCFG_EXINTC1;
		break;
	case 1U:
		extiss = &SCFG_EXINTC2;
		break;
	case 2U:
		extiss = &SCFG_EXINTC3;
		break;
	case 3U:
		extiss = &SCFG_EXINTC4;
		break;

	default:
		return -EINVAL;
	}

	port_index = (config->reg - (uint32_t)GPIOA) / ((uint32_t)GPIOB - (uint32_t)GPIOA);
	shift = EXTISS_LINE_SHIFT(pin);

	*extiss &= ~(EXTISS_MSK << shift);
	*extiss |= port_index << shift;

	return 0;
}

static inline int gpio_at32_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_at32_config *config = port->config;

	uint32_t ctl, pupd;

	ctl = GPIO_CTL(config->reg);
	ctl &= ~GPIO_MODE_MASK(pin);

	pupd = GPIO_PUD(config->reg);
	pupd &= ~GPIO_PUPD_MASK(pin);

	if ((flags & GPIO_OUTPUT) != 0U) {
		ctl |= GPIO_MODE_SET(pin, AT32_GPIO_MODE_OUTPUT);

		if ((flags & GPIO_SINGLE_ENDED) != 0U) {
			if ((flags & GPIO_LINE_OPEN_DRAIN) != 0U) {
				GPIO_OMODE(config->reg) |= BIT(pin);
			} else {
				return -ENOTSUP;
			}
		} else {
			GPIO_OMODE(config->reg) &= ~BIT(pin);
		}

		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			GPIO_BOP(config->reg) = BIT(pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			GPIO_BC(config->reg) = BIT(pin);
		}
	} else if ((flags & GPIO_INPUT) != 0U) {
		ctl |= GPIO_MODE_SET(pin, AT32_GPIO_MODE_INPUT);
	} else {
		ctl |= GPIO_MODE_SET(pin, AT32_GPIO_MODE_ANALOG);
	}

	if ((flags & GPIO_PULL_UP) != 0U) {
		pupd |= GPIO_PUPD_SET(pin, AT32_GPIO_PULL_UP);
	} else if ((flags & GPIO_PULL_DOWN) != 0U) {
		pupd |= GPIO_PUPD_SET(pin, AT32_GPIO_PULL_DOWN);
	} else {
		pupd |= GPIO_PUPD_SET(pin, AT32_GPIO_PULL_NONE);
	}

	GPIO_PUD(config->reg) = pupd;
	GPIO_CTL(config->reg) = ctl;

	return 0;
}

static int gpio_at32_port_get_raw(const struct device *port, uint32_t *value)
{
	const struct gpio_at32_config *config = port->config;

	*value = GPIO_ISTAT(config->reg);

	return 0;
}

static int gpio_at32_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_at32_config *config = port->config;

	GPIO_OCTL(config->reg) = (GPIO_OCTL(config->reg) & ~mask) | (value & mask);

	return 0;
}

static int gpio_at32_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_at32_config *config = port->config;

	GPIO_BOP(config->reg) = pins;

	return 0;
}

static int gpio_at32_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_at32_config *config = port->config;

	GPIO_BC(config->reg) = pins;

	return 0;
}

static int gpio_at32_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_at32_config *config = port->config;

#ifdef CONFIG_GD32_HAS_AF_PINMUX
	GPIO_TG(config->reg) = pins;
#else
	GPIO_OCTL(config->reg) ^= pins;
#endif /* CONFIG_GD32_HAS_AF_PINMUX */

	return 0;
}

static int gpio_at32_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
					     enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	if (mode == GPIO_INT_MODE_DISABLED) {
		exti_at32_disable(pin);
		(void)exti_at32_configure(pin, NULL, NULL);
		exti_at32_trigger(pin, EXTI_AT32_TRIG_NONE);
	} else if (mode == GPIO_INT_MODE_EDGE) {
		int ret;

		ret = exti_at32_configure(pin, gpio_at32_isr, (void *)port);
		if (ret < 0) {
			return ret;
		}

		ret = gpio_at32_configure_extiss(port, pin);
		if (ret < 0) {
			return ret;
		}

		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			exti_at32_trigger(pin, EXTI_AT32_TRIG_FALLING);
			break;
		case GPIO_INT_TRIG_HIGH:
			exti_at32_trigger(pin, EXTI_AT32_TRIG_RISING);
			break;
		case GPIO_INT_TRIG_BOTH:
			exti_at32_trigger(pin, EXTI_AT32_TRIG_BOTH);
			break;
		default:
			return -ENOTSUP;
		}

		exti_at32_enable(pin);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_at32_manage_callback(const struct device *dev, struct gpio_callback *callback,
				     bool set)
{
	struct gpio_at32_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static const struct gpio_driver_api gpio_at32_api = {
	.pin_configure = gpio_at32_configure,
	.port_get_raw = gpio_at32_port_get_raw,
	.port_set_masked_raw = gpio_at32_port_set_masked_raw,
	.port_set_bits_raw = gpio_at32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_at32_port_clear_bits_raw,
	.port_toggle_bits = gpio_at32_port_toggle_bits,
	.pin_interrupt_configure = gpio_at32_pin_interrupt_configure,
	.manage_callback = gpio_at32_manage_callback,
};

static int gpio_at32_init(const struct device *port)
{
	const struct gpio_at32_config *config = port->config;

	(void)clock_control_on(ARTERY_CLOCK_CONTROLLER, (clock_control_subsys_t *)&config->clkid);

	/* enable access to SYSCFG_EXTISSn registers */
	(void)clock_control_on(ARTERY_CLOCK_CONTROLLER,
			       (clock_control_subsys_t *)&config->clkid_exti);

	return 0;
}

#define GPIO_AT32_DEFINE(n)                                                                        \
	static const struct gpio_at32_config gpio_at32_config##n = {                               \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.reg = DT_INST_REG_ADDR(n),                                                        \
		.clkid = DT_INST_CLOCKS_CELL(n, id),                                               \
		.clkid_exti = DT_CLOCKS_CELL(SYSCFG_NODE, id),                                     \
		.rcu_periph_clock = DT_INST_PROP(n, rcu_periph_clock),                             \
	};                                                                                         \
                                                                                                   \
	static struct gpio_at32_data gpio_at32_data##n;                                            \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &gpio_at32_init, NULL, &gpio_at32_data##n, &gpio_at32_config##n,  \
			      PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, &gpio_at32_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_AT32_DEFINE)
