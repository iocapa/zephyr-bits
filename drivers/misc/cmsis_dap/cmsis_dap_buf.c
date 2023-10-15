#include <zephyr/kernel.h>
#include "cmsis_dap_buf.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cmsis_dap_buf, CONFIG_USB_CMSIS_DAP_LOG_LEVEL);

static inline uint8_t cmsis_dap_buf_space_get(struct cmsis_dap_buf *buf)
{
	return buf->size - (buf->put_head - buf->get_tail);
}

static inline uint8_t cmsis_dap_buf_size_get(struct cmsis_dap_buf *buf)
{
	return buf->put_tail - buf->get_head;
}

bool cmsis_dap_buf_put_claim(struct cmsis_dap_buf *buf,
			     struct cmsis_dap_buf_pkt **packet)
{
	uint8_t free_space, wrap_size, size;
	int8_t base;

	base = buf->put_base;
	wrap_size = buf->put_head - base;
	if (unlikely(wrap_size >= buf->size)) {
		/* put_base is not yet adjusted */
		wrap_size -= buf->size;
		base += buf->size;
	}
	wrap_size = buf->size - wrap_size;

	free_space = cmsis_dap_buf_space_get(buf);
	size = MIN(1, free_space);
	size = MIN(size, wrap_size);

	if (unlikely(size == 0)) {
		return false;
	}
	
	*packet = &buf->buffer[buf->put_head - base];
	buf->put_head += 1;

	return true;
}

bool cmsis_dap_buf_put_inspect(struct cmsis_dap_buf *buf,
			       struct cmsis_dap_buf_pkt **packet)
{
	uint8_t finish_space;
	int8_t base;

	base = buf->put_base;
	finish_space = buf->put_head - buf->put_tail;
	if (unlikely(finish_space == 0)) {
		return false;
	}

	*packet = &buf->buffer[buf->put_tail - base];

	return true;
}

bool cmsis_dap_buf_put_release(struct cmsis_dap_buf *buf)
{
	uint8_t finish_space, wrap_size;

	finish_space = buf->put_head - buf->put_tail;

	if (unlikely(finish_space == 0)) {
		return false;
	}

	buf->put_tail += 1;

	wrap_size = buf->put_tail - buf->put_base;
	if (unlikely(wrap_size >= buf->size)) {
		/* we wrapped: adjust put_base */
		buf->put_base += buf->size;
	}

	return true;
}

bool cmsis_dap_buf_get_claim(struct cmsis_dap_buf *buf,
			     struct cmsis_dap_buf_pkt **packet)
{
	uint8_t available_size, wrap_size, size;
	int8_t base;

	base = buf->get_base;
	wrap_size = buf->get_head - base;
	if (unlikely(wrap_size >= buf->size)) {
		/* get_base is not yet adjusted */
		wrap_size -= buf->size;
		base += buf->size;
	}
	wrap_size = buf->size - wrap_size;

	available_size = cmsis_dap_buf_size_get(buf);
	size = MIN(1, available_size);
	size = MIN(size, wrap_size);

	if (unlikely(size == 0)) {
		return false;
	}

	*packet = &buf->buffer[buf->get_head - base];
	buf->get_head += 1;

	return true;
}

bool cmsis_dap_buf_get_release(struct cmsis_dap_buf *buf)
{
	uint8_t finish_space, wrap_size;

	finish_space = buf->get_head - buf->get_tail;
	if (unlikely(finish_space == 0)) {
		return false;
	}

	buf->get_tail += 1;

	wrap_size = buf->get_tail - buf->get_base;
	if (unlikely(wrap_size >= buf->size)) {
		/* we wrapped: adjust get_base */
		buf->get_base += buf->size;
	}

	return true;
}