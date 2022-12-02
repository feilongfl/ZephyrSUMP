/*
 * Copyright (c) 2022 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ARTERY_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ARTERY_H_

#include <zephyr/device.h>

/**
 * @brief Obtain a reference to the ARTERY clock controller.
 *
 * There is a single clock controller in the ARTERY: cctl. The device can be
 * used without checking for it to be ready since it has no initialization
 * code subject to failures.
 */
#define ARTERY_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(cctl))

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_ARTERY_H_ */
