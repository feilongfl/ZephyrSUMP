/*
 * Copyright (c) 2022 YuLong Yao <feilongphone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT artery_at32_pwm

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/atery.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>

#include <at32f43x_crm.h>
#include <at32f43x_gpio.h>

LOG_MODULE_REGISTER(pwm_at32, CONFIG_PWM_LOG_LEVEL);

/** PWM data. */
struct pwm_at32_data {
	/** Timer clock (Hz). */
	uint32_t tim_clk;
};

/** PWM configuration. */
struct pwm_at32_config {
	/** Timer register. */
	uint32_t reg;
	/** Number of channels */
	uint8_t channels;
	/** Flag to indicate if timer has 32-bit counter */
	bool is_32bit;
	/** Flag to indicate if timer is advanced */
	bool is_advanced;
	/** Prescaler. */
	uint16_t prescaler;
	/** RCU peripheral clock. */
	uint32_t clkid;
	/** RCU peripheral reset. */
	uint32_t rcu_periph_reset;
	uint32_t reset;
	/** pinctrl configurations. */
	const struct pinctrl_dev_config *pcfg;
};

/** Obtain channel enable bit for the given channel */
#define TIMER_CHCTL2_CHXEN(ch) BIT(4U * (ch))
/** Obtain polarity bit for the given channel */
#define TIMER_CHCTL2_CHXP(ch)  BIT(1U + (4U * (ch)))
/** Obtain CHCTL0/1 mask for the given channel (0 or 1) */
#define TIMER_CHCTLX_MSK(ch)   (0xFU << (8U * (ch)))

/** Obtain RCU register offset from RCU clock value */
#define CRM_CLOCK_OFFSET(rcu_clock) ((rcu_clock) >> 16U)
#define APB1EN_REG_OFFSET	    0x40
#define APB2EN_REG_OFFSET	    0x44

static int pwm_at32_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			       uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_at32_config *config = dev->config;
	tmr_type *tmr = (tmr_type *)config->reg;

	if (channel >= config->channels) {
		return -EINVAL;
	}

	/* 16-bit timers can count up to UINT16_MAX */
	if (!config->is_32bit && (period_cycles > UINT16_MAX)) {
		return -ENOTSUP;
	}

	/* disable channel output if period is zero */
	if (period_cycles == 0U) {
		tmr->cctrl &= ~TIMER_CHCTL2_CHXEN(channel);

		return 0;
	}

	/* update polarity */
	if ((flags & PWM_POLARITY_INVERTED) != 0U) {
		tmr->cctrl |= TIMER_CHCTL2_CHXEN(channel);
	} else {
		tmr->cctrl &= ~TIMER_CHCTL2_CHXEN(channel);
	}

	/* update pulse */
	switch (channel) {
	case 0U:
		tmr->c1dt = pulse_cycles;
		break;
	case 1U:
		tmr->c2dt = pulse_cycles;
		break;
	case 2U:
		tmr->c3dt = pulse_cycles;
		break;
	case 3U:
		tmr->c4dt = pulse_cycles;
		break;
	default:
		__ASSERT_NO_MSG(NULL);
		break;
	}

	/* update period */
	tmr->pr = period_cycles;

	/* channel not enabled: configure it */
	if ((tmr->cctrl & TIMER_CHCTL2_CHXEN(channel)) == 0U) {
		volatile uint32_t *chctl;

		/* select PWM1 mode, enable OC shadowing */
		if (channel < 2U) {
			chctl = &tmr->cm1;
		} else {
			chctl = &tmr->cm2;
		}

		*chctl &= ~TIMER_CHCTLX_MSK(channel);
		*chctl |= (0x70 | 0x08) << (8U * (channel % 2U));

		/* enable channel output */
		tmr->cctrl |= TIMER_CHCTL2_CHXEN(channel);

		/* generate update event (to load shadow values) */
		tmr->swevt_bit.ovfswtr = TRUE;
	}

	return 0;
}

static int pwm_at32_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycles)
{
	struct pwm_at32_data *data = dev->data;
	const struct pwm_at32_config *config = dev->config;

	*cycles = (uint64_t)(data->tim_clk / (config->prescaler + 1U));

	return 0;
}

static const struct pwm_driver_api pwm_at32_driver_api = {
	.set_cycles = pwm_at32_set_cycles,
	.get_cycles_per_sec = pwm_at32_get_cycles_per_sec,
};

static int pwm_at32_init(const struct device *dev)
{
	const struct pwm_at32_config *config = dev->config;
	struct pwm_at32_data *data = dev->data;
	tmr_type *tmr = (tmr_type *)config->reg;
	int ret;

	(void)clock_control_on(ARTERY_CLOCK_CONTROLLER, (clock_control_subsys_t *)&config->clkid);

	/* reset timer to its default state */
	crm_periph_reset(config->rcu_periph_reset, TRUE);
	crm_periph_reset(config->rcu_periph_reset, FALSE);

	/* apply pin configuration */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* cache timer clock value */
	(void)clock_control_get_rate(ARTERY_CLOCK_CONTROLLER,
				     (clock_control_subsys_t *)&config->clkid, &data->tim_clk);

	/* basic timer operation: edge aligned, up counting, shadowed CAR */
	tmr->ctrl1_bit.clkdiv = TMR_CLOCK_DIV1;
	tmr->ctrl1_bit.cnt_dir = TMR_COUNT_UP;
	tmr->ctrl1_bit.prben = TRUE;
	tmr->div = config->prescaler;

	/* enable primary output for advanced timers */
	if (config->is_advanced) {
		tmr->brk_bit.oen = TRUE;
	}

	/* enable timer counter */
	tmr->ctrl1_bit.tmren = TRUE;

	return 0;
}

#define PWM_GD32_DEFINE(i)                                                                         \
	static struct pwm_at32_data pwm_at32_data_##i;                                             \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(i);                                                                 \
                                                                                                   \
	static const struct pwm_at32_config pwm_at32_config_##i = {                                \
		.reg = DT_REG_ADDR(DT_INST_PARENT(i)),                                             \
		.rcu_periph_reset = DT_PROP(DT_INST_PARENT(i), rcu_periph_reset),                  \
		.clkid = DT_CLOCKS_CELL(DT_INST_PARENT(i), id),                                    \
		.prescaler = DT_PROP(DT_INST_PARENT(i), prescaler),                                \
		.channels = DT_PROP(DT_INST_PARENT(i), channels),                                  \
		.is_32bit = DT_PROP(DT_INST_PARENT(i), is_32bit),                                  \
		.is_advanced = DT_PROP(DT_INST_PARENT(i), is_advanced),                            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(i),                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(i, &pwm_at32_init, NULL, &pwm_at32_data_##i, &pwm_at32_config_##i,   \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                     \
			      &pwm_at32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_GD32_DEFINE)
