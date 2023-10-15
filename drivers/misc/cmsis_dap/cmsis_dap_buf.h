/*
 * Copyright (c) 2023 Ionut Catalin Pavel
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Buffer APIs for the CMSIS-DAP driver
 */

#ifndef ZEPHYR_DRIVERS_MISC_CMSIS_DAP_BUF_H_
#define ZEPHYR_DRIVERS_MISC_CMSIS_DAP_BUF_H_

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct cmsis_dap_buf_pkt {
	uint8_t data[CONFIG_USB_CMSIS_DAP_PACKET_SIZE];
	int length;
};

struct cmsis_dap_buf {
	struct cmsis_dap_buf_pkt *const buffer;
	int8_t put_head;
	int8_t put_tail;
	int8_t put_base;
	int8_t get_head;
	int8_t get_tail;
	int8_t get_base;
	const uint8_t size;
};

#define CMSIS_DAP_BUF_MAX_SIZE			0x80U
#define CMSIS_DAP_BUF_ASSERT_MSG		"invalid size"

#define CMSIS_DAP_BUF_DECLARE(name, pcnt)						\
	BUILD_ASSERT(pcnt < CMSIS_DAP_BUF_MAX_SIZE, CMSIS_DAP_BUF_ASSERT_MSG);		\
	static struct cmsis_dap_buf_pkt __noinit _cmsis_dap_pkt_##name[pcnt];		\
	static struct cmsis_dap_buf name = {						\
		.buffer = _cmsis_dap_pkt_##name,					\
		.size = pcnt								\
	};

static inline void cmsis_dap_buf_reset(struct cmsis_dap_buf *buf)
{
	buf->put_head = buf->put_tail = buf->put_base = 0;
	buf->get_head = buf->get_tail = buf->get_base = 0;
}

static inline uint8_t cmsis_dap_buf_capacity_get(struct cmsis_dap_buf *buf)
{
	return buf->size;
}

bool cmsis_dap_buf_put_claim(struct cmsis_dap_buf *buf, struct cmsis_dap_buf_pkt **packet);
bool cmsis_dap_buf_put_inspect(struct cmsis_dap_buf *buf, struct cmsis_dap_buf_pkt **packet);
bool cmsis_dap_buf_put_release(struct cmsis_dap_buf *buf);
bool cmsis_dap_buf_get_claim(struct cmsis_dap_buf *buf, struct cmsis_dap_buf_pkt **packet);
bool cmsis_dap_buf_get_release(struct cmsis_dap_buf *buf);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZEPHYR_DRIVERS_MISC_CMSIS_DAP_BUF_H_ */
