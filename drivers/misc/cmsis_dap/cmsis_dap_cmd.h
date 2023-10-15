/*
 * Copyright (c) 2023 Ionut Catalin Pavel
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Command APIs for the CMSIS-DAP driver
 */

#ifndef ZEPHYR_DRIVERS_MISC_CMSIS_DAP_CMD_H_
#define ZEPHYR_DRIVERS_MISC_CMSIS_DAP_CMD_H_

#include <zephyr/device.h>
#include <zephyr/drivers/misc/cmsis_dap.h>
#include <zephyr/drivers/jtag.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define CMSIS_DAP_SWD_EN	BIT(0)
#define CMSIS_DAP_JTAG_EN	BIT(1)

struct cmsis_dap_cmd_data {
	const struct device *const jtag_dev;
	cmsis_dap_vendor_callback_t vendor_cb;
	void *vendor_data;
	const uint16_t packet_size;
	uint16_t wait_retry;
	uint16_t match_retry;
	const uint8_t packet_count;
	uint8_t idle_cycles;
	uint8_t dap_caps;
	uint8_t dap_port;
	uint8_t swd_turnaround;
	bool swd_data_phase;
	struct jtag_instruction instr_buf[CONFIG_USB_CMIS_DAP_MAX_JTAG_INSTR];
};

#define CMSIS_DAP_CMD_INITIALIZER(x)								\
	{											\
		.jtag_dev = DEVICE_DT_GET(DT_INST_PHANDLE(x, jtag)),				\
		.dap_caps = (DT_INST_PROP(x, disable_swd) ? 0 : CMSIS_DAP_SWD_EN) |		\
			    (DT_INST_PROP(x, disable_jtag) ? 0 : CMSIS_DAP_JTAG_EN),	\
		.packet_size = CONFIG_USB_CMSIS_DAP_PACKET_SIZE,				\
		.packet_count = DT_INST_PROP_OR(x, packet_count,				\
					CONFIG_USB_CMSIS_DAP_PACKET_COUNT),			\
	}

int cmsis_dap_cmd_handler(struct cmsis_dap_cmd_data *data, uint8_t *buffer, int rdlen);
int cmsis_dap_cmd_init(struct cmsis_dap_cmd_data *data);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_DRIVERS_MISC_CMSIS_DAP_CMD_H_ */
