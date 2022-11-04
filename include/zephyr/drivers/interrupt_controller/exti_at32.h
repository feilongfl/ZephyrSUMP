/*
 * Copyright (c) 2022 YuLong Yao <feilongphone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_EXTI_AT32_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_EXTI_AT32_H_

#include <stdint.h>

#include <zephyr/sys/util_macro.h>

/**
 * @name EXTI registers.
 * @anchor SCFG_
 * @{
 */

#define SCFG_CFG1 SCFG_REG(0x00)
#define SCFG_CFG2 SCFG_REG(0x04)
#define SCFG_EXINTC1 SCFG_REG(0x08)
#define SCFG_EXINTC2 SCFG_REG(0x0c)
#define SCFG_EXINTC3 SCFG_REG(0x10)
#define SCFG_EXINTC4 SCFG_REG(0x14)
#define SCFG_UHDRV SCFG_REG(0x2c)

/**
 * @name EXTI trigger modes.
 * @anchor EXTI_AT32_TRIG
 * @{
 */

/** No trigger */
#define EXTI_AT32_TRIG_NONE 0U
/** Trigger on rising edge */
#define EXTI_AT32_TRIG_RISING BIT(0)
/** Trigger on falling edge */
#define EXTI_AT32_TRIG_FALLING BIT(1)
/** Trigger on rising and falling edge */
#define EXTI_AT32_TRIG_BOTH (EXTI_AT32_TRIG_RISING | EXTI_AT32_TRIG_FALLING)

/** @} */

/** Callback for EXTI interrupt. */
typedef void (*exti_at32_cb_t)(uint8_t line, void *user);

/**
 * @brief Enable EXTI interrupt for the given line.
 *
 * @param line EXTI line.
 */
void exti_at32_enable(uint8_t line);

/**
 * @brief Disable EXTI interrupt for the given line.
 *
 * @param line EXTI line.
 */
void exti_at32_disable(uint8_t line);

/**
 * @brief Configure EXTI interrupt trigger mode for the given line.
 *
 * @param line EXTI line.
 * @param trigger Trigger mode (see @ref exti_at32_TRIG).
 */
void exti_at32_trigger(uint8_t line, uint8_t trigger);

/**
 * @brief Configure EXTI interrupt callback.
 *
 * @param line EXTI line.
 * @param cb Callback (NULL to disable).
 * @param user User data (optional).
 *
 * @retval 0 On success.
 * @retval -EALREADY If callback is already set and @p cb is not NULL.
 */
int exti_at32_configure(uint8_t line, exti_at32_cb_t cb, void *user);

#endif /* ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_EXTI_AT32_H_ */
