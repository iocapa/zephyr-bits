/*
 * Copyright (c) 2023 Ionut Catalin Pavel
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs for the CDC ACM passthrough class driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_CDC_ACM_PT_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_CDC_ACM_PT_H_

#include <errno.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

enum cdc_acm_pt_act_code {
	CDC_ACM_PT_ACT_RX,
	CDC_ACM_PT_ACT_TX,
};

typedef void (*cdc_acm_pt_act_callback_t)(const struct device *dev,
					  enum cdc_acm_pt_act_code code,
					  bool status, void *user);

int cdc_acm_pt_act_callback_set(const struct device *dev,
				cdc_acm_pt_act_callback_t callback,
				void *user);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_CDC_ACM_PT_H_ */
