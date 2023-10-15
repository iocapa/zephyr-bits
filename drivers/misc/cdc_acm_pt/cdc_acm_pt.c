
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/misc/cdc_acm_pt.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/class/usb_cdc.h>
#include <zephyr/usb/usb_device.h>
#include <usb_descriptor.h>
#include <usb_work_q.h>

#ifndef CONFIG_UART_INTERRUPT_DRIVEN
#error "CONFIG_UART_INTERRUPT_DRIVEN must be set for CDC ACM passthrough driver"
#endif

/* definitions */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_cdc_acm_pt, CONFIG_USB_CDC_ACM_PT_LOG_LEVEL);

/* 115200bps, no parity, 1 stop bit, 8bit char */
#define CDC_ACM_PT_DEFAULT_BAUDRATE {sys_cpu_to_le32(115200), 0, 0, 8}

#define ACM_INT_EP_IDX			0
#define ACM_OUT_EP_IDX			1
#define ACM_IN_EP_IDX			2

struct usb_cdc_acm_pt_config {
#if (CONFIG_USB_COMPOSITE_DEVICE || CONFIG_CDC_ACM_PT_IAD)
	struct usb_association_descriptor iad_cdc;
#endif
	struct usb_if_descriptor if0;
	struct cdc_header_descriptor if0_header;
	struct cdc_cm_descriptor if0_cm;
	struct cdc_acm_descriptor if0_acm;
	struct cdc_union_descriptor if0_union;
	struct usb_ep_descriptor if0_int_ep;

	struct usb_if_descriptor if1;
	struct usb_ep_descriptor if1_in_ep;
	struct usb_ep_descriptor if1_out_ep;
} __packed;

/* Device data structure */
struct cdc_acm_pt_data {
	const struct device *uart_dev;

#if defined(CONFIG_CDC_ACM_PT_ACT_CALLBACK_SUPPORT)
	cdc_acm_pt_act_callback_t act_cb;
	void *act_data;
#endif
	struct k_work_delayable tx_work;
	uint8_t rx_buf[CONFIG_CDC_ACM_PT_BULK_EP_MPS];	/* Internal RX buffer */
	struct ring_buf *rx_ringbuf;
	struct ring_buf *tx_ringbuf;
	/* CDC ACM line coding properties. LE order */
	struct cdc_acm_line_coding line_coding;
	/* CDC ACM line state bitmap, DTE side */
	uint8_t line_state;
	/* CDC ACM configured flag */
	bool configured;
	/* CDC ACM suspended flag */
	bool suspended;

	struct usb_dev_data common;
};

static sys_slist_t cdc_acm_pt_devlist;

static int cdc_acm_pt_line_to_uart_config(struct cdc_acm_line_coding *line_coding,
					  struct uart_config *uart_cfg)
{
	uart_cfg->baudrate = sys_le32_to_cpu(line_coding->dwDTERate);

	switch (line_coding->bCharFormat) {
	case USB_CDC_LINE_CODING_STOP_BITS_1:
		uart_cfg->stop_bits = UART_CFG_STOP_BITS_1;
		break;
	case USB_CDC_LINE_CODING_STOP_BITS_1_5:
		uart_cfg->stop_bits = UART_CFG_STOP_BITS_1_5;
		break;
	case USB_CDC_LINE_CODING_STOP_BITS_2:
	default:
		uart_cfg->stop_bits = UART_CFG_STOP_BITS_2;
		break;
	};

	switch (line_coding->bParityType) {
	case USB_CDC_LINE_CODING_PARITY_NO:
	default:
		uart_cfg->parity = UART_CFG_PARITY_NONE;
		break;
	case USB_CDC_LINE_CODING_PARITY_ODD:
		uart_cfg->parity = UART_CFG_PARITY_ODD;
		break;
	case USB_CDC_LINE_CODING_PARITY_EVEN:
		uart_cfg->parity = UART_CFG_PARITY_EVEN;
		break;
	case USB_CDC_LINE_CODING_PARITY_MARK:
		uart_cfg->parity = UART_CFG_PARITY_MARK;
		break;
	case USB_CDC_LINE_CODING_PARITY_SPACE:
		uart_cfg->parity = UART_CFG_PARITY_SPACE;
		break;
	};

	switch (line_coding->bDataBits) {
	case USB_CDC_LINE_CODING_DATA_BITS_5:
		uart_cfg->data_bits = UART_CFG_DATA_BITS_5;
		break;
	case USB_CDC_LINE_CODING_DATA_BITS_6:
		uart_cfg->data_bits = UART_CFG_DATA_BITS_6;
		break;
	case USB_CDC_LINE_CODING_DATA_BITS_7:
		uart_cfg->data_bits = UART_CFG_DATA_BITS_7;
		break;
	case USB_CDC_LINE_CODING_DATA_BITS_8:
	default:
		uart_cfg->data_bits = UART_CFG_DATA_BITS_8;
		break;
	};

	/* USB CDC has no notion of flow control */
	uart_cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	return 0;
}

static int cdc_acm_pt_class_handle_req(struct usb_setup_packet *setup,
				       int32_t *len, uint8_t **data)
{
	struct cdc_acm_pt_data *dev_data;
	struct usb_dev_data *common;
	struct uart_config uart_cfg;
	struct cdc_acm_line_coding *line_coding;
	int ret;

	common = usb_get_dev_data_by_iface(&cdc_acm_pt_devlist, (uint8_t)setup->wIndex);
	if (common == NULL) {
		LOG_WRN("Device data not found for interface %u", setup->wIndex);
		return -ENODEV;
	}

	dev_data = CONTAINER_OF(common, struct cdc_acm_pt_data, common);

	if (usb_reqtype_is_to_device(setup)) {
		switch (setup->bRequest) {
		case SET_LINE_CODING:
			line_coding = (struct cdc_acm_line_coding *)(*data);

			cdc_acm_pt_line_to_uart_config(line_coding, &uart_cfg);
			ret = uart_configure(dev_data->uart_dev, &uart_cfg);
			if (ret != 0) {
				LOG_ERR("error: CDC_SET_LINE_CODING invalid line coding");
				return -ENOTSUP;
			}

			memcpy(&dev_data->line_coding, *data, sizeof(dev_data->line_coding));

			LOG_DBG("CDC_SET_LINE_CODING %d %d %d %d",
				sys_le32_to_cpu(dev_data->line_coding.dwDTERate),
				dev_data->line_coding.bCharFormat,
				dev_data->line_coding.bParityType,
				dev_data->line_coding.bDataBits);

			return 0;

		case SET_CONTROL_LINE_STATE:
			dev_data->line_state = (uint8_t)setup->wValue;
			LOG_DBG("CDC_SET_CONTROL_LINE_STATE 0x%x",
				dev_data->line_state);

			/* Only enable RX on DTR */
			if (dev_data->line_state & SET_CONTROL_LINE_STATE_DTR) {
				uart_irq_rx_enable(dev_data->uart_dev);
			} else {
				uart_irq_rx_disable(dev_data->uart_dev);
			}

			return 0;

		default:
			break;
		}
	} else {
		if (setup->bRequest == GET_LINE_CODING) {
			*data = (uint8_t *)(&dev_data->line_coding);
			*len = sizeof(dev_data->line_coding);
			LOG_DBG("CDC_GET_LINE_CODING %d %d %d %d",
				sys_le32_to_cpu(dev_data->line_coding.dwDTERate),
				dev_data->line_coding.bCharFormat,
				dev_data->line_coding.bParityType,
				dev_data->line_coding.bDataBits);
			return 0;
		}
	}

	LOG_DBG("CDC ACM bmRequestType 0x%02x bRequest 0x%02x unsupported",
		setup->bmRequestType, setup->bRequest);
	return -ENOTSUP;
}

static void cdc_acm_pt_write_cb(uint8_t ep, int size, void *priv)
{
	struct cdc_acm_pt_data *dev_data = priv;

	LOG_DBG("ep %x: written %d bytes dev_data %p", ep, size, dev_data);

	if (ring_buf_is_empty(dev_data->tx_ringbuf)) {
		LOG_DBG("tx_ringbuf is empty");

#if defined(CONFIG_CDC_ACM_PT_ACT_CALLBACK_SUPPORT)
		dev_data->act_cb(dev_data->common.dev, CDC_ACM_PT_ACT_RX,
				 false, dev_data->act_data);
#endif

		return;
	}

	k_work_schedule_for_queue(&USB_WORK_Q, &dev_data->tx_work, K_NO_WAIT);
}

static void cdc_acm_pt_tx_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct cdc_acm_pt_data *dev_data = CONTAINER_OF(dwork, struct cdc_acm_pt_data, tx_work);
	const struct device *dev = dev_data->common.dev;
	struct usb_cfg_data *cfg = (void *)dev->config;
	uint8_t ep = cfg->endpoint[ACM_IN_EP_IDX].ep_addr;
	uint8_t *data;
	size_t len;

	if (usb_transfer_is_busy(ep)) {
		LOG_DBG("Transfer is ongoing");
		return;
	}

	len = ring_buf_get_claim(dev_data->tx_ringbuf, &data, dev_data->tx_ringbuf->size);

	if (!len) {
		LOG_DBG("Nothing to send");
		return;
	}

	/*
	 * Transfer less data to avoid zero-length packet. The application
	 * running on the host may conclude that there is no more data to be
	 * received (i.e. the transaction has completed), hence not triggering
	 * another I/O Request Packet (IRP).
	 */
	if (!(len % CONFIG_CDC_ACM_PT_BULK_EP_MPS)) {
		len -= 1;
	}

	LOG_DBG("Got %zd bytes from ringbuffer send to ep %x", len, ep);

	usb_transfer(ep, data, len, USB_TRANS_WRITE, cdc_acm_pt_write_cb, dev_data);

	ring_buf_get_finish(dev_data->tx_ringbuf, len);
}

static void cdc_acm_pt_read_cb(uint8_t ep, int size, void *priv)
{
	struct cdc_acm_pt_data *dev_data = priv;
	size_t wrote;

	LOG_DBG("ep %x size %d dev_data %p rx_ringbuf space %u",
		ep, size, dev_data, ring_buf_space_get(dev_data->rx_ringbuf));

	if (size <= 0) {
		goto done;
	}

	wrote = ring_buf_put(dev_data->rx_ringbuf, dev_data->rx_buf, size);
	if (wrote < size) {
		LOG_ERR("Ring buffer full, drop %zd bytes", size - wrote);
	}

	uart_irq_tx_enable(dev_data->uart_dev);

done:
	usb_transfer(ep, dev_data->rx_buf, sizeof(dev_data->rx_buf),
		     USB_TRANS_READ, cdc_acm_pt_read_cb, dev_data);
}

static void cdc_acm_pt_reset_port(struct cdc_acm_pt_data *dev_data)
{
	dev_data->configured = false;
	dev_data->suspended = false;
	dev_data->line_coding = (struct cdc_acm_line_coding)CDC_ACM_PT_DEFAULT_BAUDRATE;
	dev_data->line_state = 0;
	memset(&dev_data->rx_buf, 0, CONFIG_CDC_ACM_PT_BULK_EP_MPS);
}

static void cdc_acm_pt_do_cb(struct cdc_acm_pt_data *dev_data, enum usb_dc_status_code status,
			     const uint8_t *param)
{
	const struct device *dev = dev_data->common.dev;
	struct usb_cfg_data *cfg = (void *)dev->config;

	/* Check the USB status and do needed action if required */
	switch (status) {
	case USB_DC_ERROR:
		LOG_DBG("Device error");
		break;
	case USB_DC_RESET:
		LOG_DBG("Device reset detected");
		cdc_acm_pt_reset_port(dev_data);
		break;
	case USB_DC_CONNECTED:
		LOG_DBG("Device connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_INF("Device configured");
		if (!dev_data->configured) {
			cdc_acm_pt_read_cb(cfg->endpoint[ACM_OUT_EP_IDX].ep_addr, 0, dev_data);
			dev_data->configured = true;
		}
		break;
	case USB_DC_DISCONNECTED:
		LOG_INF("Device disconnected");
		cdc_acm_pt_reset_port(dev_data);
		break;
	case USB_DC_SUSPEND:
		LOG_INF("Device suspended");
		dev_data->suspended = true;
		break;
	case USB_DC_RESUME:
		LOG_INF("Device resumed");
		if (dev_data->suspended) {
			LOG_INF("from suspend");
			dev_data->suspended = false;
		} else {
			LOG_DBG("Spurious resume event");
		}
		break;
	case USB_DC_SOF:
	case USB_DC_INTERFACE:
		break;
	case USB_DC_UNKNOWN:
	default:
		LOG_DBG("Unknown event");
		break;
	}
}

static void cdc_acm_pt_dev_status_cb(struct usb_cfg_data *cfg, enum usb_dc_status_code status,
				     const uint8_t *param)
{
	struct cdc_acm_pt_data *dev_data;
	struct usb_dev_data *common;

	LOG_DBG("cfg %p status %d", cfg, status);

	common = usb_get_dev_data_by_cfg(&cdc_acm_pt_devlist, cfg);
	if (common == NULL) {
		LOG_WRN("Device data not found for cfg %p", cfg);
		return;
	}

	dev_data = CONTAINER_OF(common, struct cdc_acm_pt_data, common);

	cdc_acm_pt_do_cb(dev_data, status, param);
}

static void cdc_acm_pt_interface_config(struct usb_desc_header *head, uint8_t bInterfaceNumber)
{
	struct usb_if_descriptor *if_desc = (struct usb_if_descriptor *) head;
	struct usb_cdc_acm_pt_config *desc =
		CONTAINER_OF(if_desc, struct usb_cdc_acm_pt_config, if0);

	desc->if0.bInterfaceNumber = bInterfaceNumber;
	desc->if0_union.bControlInterface = bInterfaceNumber;
	desc->if1.bInterfaceNumber = bInterfaceNumber + 1;
	desc->if0_union.bSubordinateInterface0 = bInterfaceNumber + 1;
#if (CONFIG_USB_COMPOSITE_DEVICE || CONFIG_CDC_ACM_PT_IAD)
	desc->iad_cdc.bFirstInterface = bInterfaceNumber;
#endif
}

static void cdc_acm_pt_uart_rx_handle(const struct device *dev, const struct device *uart_dev)
{
	struct cdc_acm_pt_data * const dev_data = uart_dev->data;
	uint8_t *data;
	uint32_t len;
	uint32_t rd_len;
	bool new_data = false;

	do {
		len = ring_buf_put_claim(dev_data->tx_ringbuf, &data, dev_data->tx_ringbuf->size);

		if (len > 0) {
			rd_len = uart_fifo_read(dev, data, len);

			/* Start TX work */
			if (rd_len > 0) {
				new_data = true;
			}

			int err = ring_buf_put_finish(dev_data->tx_ringbuf, rd_len);
	
			__ASSERT_NO_MSG(err == 0);
			ARG_UNUSED(err);
		} else {
			uint8_t dummy;

			LOG_WRN("RX ring buffer full.");

			rd_len = uart_fifo_read(dev, &dummy, 1);
		}
	} while (rd_len && (rd_len == len));

	if (new_data) {
		k_work_schedule_for_queue(&USB_WORK_Q, &dev_data->tx_work, K_MSEC(1));
#if defined(CONFIG_CDC_ACM_PT_ACT_CALLBACK_SUPPORT)
		dev_data->act_cb(dev_data->common.dev, CDC_ACM_PT_ACT_RX,
				 true, dev_data->act_data);
#endif
	}
}

static void cdc_acm_pt_uart_tx_handle(const struct device *dev, const struct device *uart_dev)
{
	struct cdc_acm_pt_data * const dev_data = uart_dev->data;
	uint32_t len;
	const uint8_t *data;

	len = ring_buf_get_claim(dev_data->rx_ringbuf, (uint8_t **)&data,
				 dev_data->rx_ringbuf->size);

	if (len) {
		int err;

#if defined(CONFIG_CDC_ACM_PT_ACT_CALLBACK_SUPPORT)
		dev_data->act_cb(dev_data->common.dev, CDC_ACM_PT_ACT_TX,
				 true, dev_data->act_data);
#endif
		len = uart_fifo_fill(dev, data, len);
		err = ring_buf_get_finish(dev_data->rx_ringbuf, len);

		__ASSERT_NO_MSG(err == 0);
		ARG_UNUSED(err);
	} else {
		uart_irq_tx_disable(dev);

#if defined(CONFIG_CDC_ACM_PT_ACT_CALLBACK_SUPPORT)
		dev_data->act_cb(dev_data->common.dev, CDC_ACM_PT_ACT_TX,
				 false, dev_data->act_data);
#endif
	}
}

static void cdc_acm_pt_uart_irq_cb(const struct device *dev, void *user_data)
{
	const struct device *uart_dev = user_data;

	uart_irq_update(dev);

	if (uart_irq_rx_ready(dev)) {
		cdc_acm_pt_uart_rx_handle(dev, uart_dev);
	}

	if (uart_irq_tx_ready(dev)) {
		cdc_acm_pt_uart_tx_handle(dev, uart_dev);
	}
}

static int cdc_acm_pt_init(const struct device *dev)
{
	struct cdc_acm_pt_data * const dev_data = dev->data;
	struct uart_config uart_cfg;
	int ret = 0;

	if (!device_is_ready(dev_data->uart_dev)) {
		return -ENODEV;
	}

	dev_data->common.dev = dev;
	sys_slist_append(&cdc_acm_pt_devlist, &dev_data->common.node);

	LOG_DBG("Device dev %p dev_data %p cfg %p added to devlist %p",
		dev, dev_data, dev->config, &cdc_acm_pt_devlist);

	k_work_init_delayable(&dev_data->tx_work, cdc_acm_pt_tx_work_handler);

	/* Set the initial config */
	cdc_acm_pt_line_to_uart_config(&dev_data->line_coding, &uart_cfg);
	ret = uart_configure(dev_data->uart_dev, &uart_cfg);
	if (ret < 0) {
		return ret;
	}

	/* Setup IRQs */
	ret = uart_irq_callback_user_data_set(dev_data->uart_dev,
					      cdc_acm_pt_uart_irq_cb,
					      (void *)dev);

	return ret;
}

#if defined(CONFIG_CDC_ACM_PT_ACT_CALLBACK_SUPPORT)
int cdc_acm_pt_act_callback_set(const struct device *dev,
				cdc_acm_pt_act_callback_t callback,
				void *data)
{
	struct cdc_acm_pt_data * const dev_data = dev->data;

	dev_data->act_cb = callback;
	dev_data->act_data = data;

	return 0;
}
#endif

#if (CONFIG_USB_COMPOSITE_DEVICE || CONFIG_CDC_ACM_PT_IAD)
#define INITIALIZER_IAD										\
	.iad_cdc = {										\
		.bLength = sizeof(struct usb_association_descriptor),				\
		.bDescriptorType = USB_DESC_INTERFACE_ASSOC,					\
		.bFirstInterface = 0,								\
		.bInterfaceCount = 0x02,							\
		.bFunctionClass = USB_BCC_CDC_CONTROL,						\
		.bFunctionSubClass = ACM_SUBCLASS,						\
		.bFunctionProtocol = 0,								\
		.iFunction = 0,									\
	},
#else
#define INITIALIZER_IAD
#endif

#define INITIALIZER_IF(iface_num, num_ep, class, subclass)					\
	{											\
		.bLength = sizeof(struct usb_if_descriptor),					\
		.bDescriptorType = USB_DESC_INTERFACE,						\
		.bInterfaceNumber = iface_num,							\
		.bAlternateSetting = 0,								\
		.bNumEndpoints = num_ep,							\
		.bInterfaceClass = class,							\
		.bInterfaceSubClass = subclass,							\
		.bInterfaceProtocol = 0,							\
		.iInterface = 0,								\
	}

#define INITIALIZER_IF_HDR									\
	{											\
		.bFunctionLength = sizeof(struct cdc_header_descriptor),			\
		.bDescriptorType = USB_DESC_CS_INTERFACE,					\
		.bDescriptorSubtype = HEADER_FUNC_DESC,						\
		.bcdCDC = sys_cpu_to_le16(USB_SRN_1_1),						\
	}

#define INITIALIZER_IF_CM									\
	{											\
		.bFunctionLength = sizeof(struct cdc_cm_descriptor),				\
		.bDescriptorType = USB_DESC_CS_INTERFACE,					\
		.bDescriptorSubtype = CALL_MANAGEMENT_FUNC_DESC,				\
		.bmCapabilities = 0x02,								\
		.bDataInterface = 1,								\
	}

#define INITIALIZER_IF_ACM									\
	{											\
		.bFunctionLength = sizeof(struct cdc_acm_descriptor),				\
		.bDescriptorType = USB_DESC_CS_INTERFACE,					\
		.bDescriptorSubtype = ACM_FUNC_DESC,						\
		.bmCapabilities = 0x02,								\
	}

#define INITIALIZER_IF_UNION									\
	{											\
		.bFunctionLength = sizeof(struct cdc_union_descriptor),				\
		.bDescriptorType = USB_DESC_CS_INTERFACE,					\
		.bDescriptorSubtype = UNION_FUNC_DESC,						\
		.bControlInterface = 0,								\
		.bSubordinateInterface0 = 1,							\
	}

#define INITIALIZER_IF_EP(addr, attr, mps, interval)						\
	{											\
		.bLength = sizeof(struct usb_ep_descriptor),					\
		.bDescriptorType = USB_DESC_ENDPOINT,						\
		.bEndpointAddress = addr,							\
		.bmAttributes = attr,								\
		.wMaxPacketSize = sys_cpu_to_le16(mps),						\
		.bInterval = interval,								\
	}

#define CDC_ACM_PT_CFG_AND_DATA_DEFINE(x)							\
	USBD_CLASS_DESCR_DEFINE(primary, x)							\
	struct usb_cdc_acm_pt_config cdc_acm_pt_cfg_##x = {					\
		INITIALIZER_IAD									\
		.if0 = INITIALIZER_IF(0, 1,							\
				USB_BCC_CDC_CONTROL,						\
				ACM_SUBCLASS),							\
		.if0_header = INITIALIZER_IF_HDR,						\
		.if0_cm = INITIALIZER_IF_CM,							\
		.if0_acm = INITIALIZER_IF_ACM,							\
		.if0_union = INITIALIZER_IF_UNION,						\
		.if0_int_ep = INITIALIZER_IF_EP(AUTO_EP_IN,					\
				USB_DC_EP_INTERRUPT,						\
				CONFIG_CDC_ACM_PT_INTERRUPT_EP_MPS,				\
				0x0A),								\
		.if1 = INITIALIZER_IF(1, 2,							\
				USB_BCC_CDC_DATA,						\
				0),								\
		.if1_in_ep = INITIALIZER_IF_EP(AUTO_EP_IN,					\
				USB_DC_EP_BULK,							\
				CONFIG_CDC_ACM_PT_BULK_EP_MPS,					\
				0x00),								\
		.if1_out_ep = INITIALIZER_IF_EP(AUTO_EP_OUT,					\
				USB_DC_EP_BULK,							\
				CONFIG_CDC_ACM_PT_BULK_EP_MPS,					\
				0x00),								\
	};											\
												\
	static struct usb_ep_cfg_data cdc_acm_pt_ep_data_##x[] = {				\
		{										\
			.ep_cb = NULL,								\
			.ep_addr = AUTO_EP_IN,							\
		},										\
		{										\
			.ep_cb = usb_transfer_ep_callback,					\
			.ep_addr = AUTO_EP_OUT,							\
		},										\
		{										\
			.ep_cb = usb_transfer_ep_callback,					\
			.ep_addr = AUTO_EP_IN,							\
		},										\
	};											\
												\
	USBD_DEFINE_CFG_DATA(cdc_acm_pt_config_##x) = {						\
		.usb_device_description = NULL,							\
		.interface_config = cdc_acm_pt_interface_config,				\
		.interface_descriptor = &cdc_acm_pt_cfg_##x.if0,				\
		.cb_usb_status = cdc_acm_pt_dev_status_cb,					\
		.interface = {									\
			.class_handler = cdc_acm_pt_class_handle_req,				\
			.custom_handler = NULL,							\
		},										\
		.num_endpoints = ARRAY_SIZE(cdc_acm_pt_ep_data_##x),				\
		.endpoint = cdc_acm_pt_ep_data_##x,						\
	};											\
												\
	BUILD_ASSERT(DT_INST_PROP(x, rx_fifo_size) >= CONFIG_CDC_ACM_PT_BULK_EP_MPS,		\
		     "rx-fifo-size for node " DT_NODE_PATH(DT_DRV_INST(x))			\
		     " needs to be higher than " STRINGIFY(CONFIG_CDC_ACM_PT_BULK_EP_MPS));	\
	RING_BUF_DECLARE(cdc_acm_pt_rx_rb_##x,							\
			 DT_INST_PROP(x, rx_fifo_size));					\
												\
	BUILD_ASSERT(DT_INST_PROP(x, tx_fifo_size) >= CONFIG_CDC_ACM_PT_BULK_EP_MPS,		\
		     "tx-fifo-size for node " DT_NODE_PATH(DT_DRV_INST(x)) 			\
		     " needs to be higher than " STRINGIFY(CONFIG_CDC_ACM_PT_BULK_EP_MPS));	\
	RING_BUF_DECLARE(cdc_acm_pt_tx_rb_##x,							\
			 DT_INST_PROP(x, tx_fifo_size));					\
												\
	static struct cdc_acm_pt_data cdc_acm_pt_data_##x = {					\
		.uart_dev = DEVICE_DT_GET(DT_INST_PHANDLE(x, uart)),				\
		.line_coding = CDC_ACM_PT_DEFAULT_BAUDRATE,					\
		.rx_ringbuf = &cdc_acm_pt_rx_rb_##x,						\
		.tx_ringbuf = &cdc_acm_pt_tx_rb_##x,						\
	};

#define DT_DRV_COMPAT zephyr_cdc_acm_uart_pt

#define CDC_ACM_PT_DT_DEVICE_DEFINE(idx)							\
	BUILD_ASSERT(DT_INST_ON_BUS(idx, usb),							\
		     "node " DT_NODE_PATH(DT_DRV_INST(idx))					\
		     " is not assigned to a USB device controller");				\
	CDC_ACM_PT_CFG_AND_DATA_DEFINE(idx)							\
												\
	DEVICE_DT_INST_DEFINE(idx, cdc_acm_pt_init, NULL,					\
		&cdc_acm_pt_data_##idx, &cdc_acm_pt_config_##idx,				\
		POST_KERNEL, CONFIG_SERIAL_INIT_PRIORITY,					\
		NULL);

DT_INST_FOREACH_STATUS_OKAY(CDC_ACM_PT_DT_DEVICE_DEFINE);
