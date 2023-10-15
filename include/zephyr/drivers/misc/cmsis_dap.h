/*
 * Copyright (c) 2023 Ionut Catalin Pavel
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public APIs for the CMSIS-DAP driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_CMSIS_DAP_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_CMSIS_DAP_H_

#include <errno.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef void (*cmsis_dap_vendor_callback_t)(const struct device *dev,
					    uint8_t *buffer,
					    void *user_data);

int cmsis_dap_vendor_callback_set(const struct device *dev,
				  cmsis_dap_vendor_callback_t callback,
				  void *data);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_CMSIS_DAP_H_ */
