/*
 * Copyright (c) 2022, Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_ARM_ARTERY_AT32F43x_AT32_REGS_H_
#define SOC_ARM_ARTERY_AT32F43x_AT32_REGS_H_

#include <zephyr/sys/util_macro.h>

/* CRM */
#define CRM_CFG0_OFFSET	  0x08U
#define CRM_AHB1EN_OFFSET 0x30U
#define CRM_AHB2EN_OFFSET 0x34U
#define CRM_AHB3EN_OFFSET 0x38U
#define CRM_APB1EN_OFFSET 0x40U
#define CRM_APB2EN_OFFSET 0x44U

#define CRM_CFG0_AHBDIV_POS  4U
#define CRM_CFG0_AHBDIV_MSK  (BIT_MASK(4) << CRM_CFG0_AHBDIV_POS)
#define CRM_CFG0_APB1DIV_POS 10U
#define CRM_CFG0_APB1DIV_MSK (BIT_MASK(3) << CRM_CFG0_APB1DIV_POS)
#define CRM_CFG0_APB2DIV_POS 13U
#define CRM_CFG0_APB2DIV_MSK (BIT_MASK(3) << CRM_CFG0_APB2DIV_POS)

#endif /* SOC_ARM_ARTERY_AT32F43x_AT32_REGS_H_ */
