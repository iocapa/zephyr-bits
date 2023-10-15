#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/bos.h>
#include <usb_descriptor.h>
#include <zephyr/sys/byteorder.h>

#include "cmsis_dap_cmd.h"
#include "cmsis_dap_buf.h"

#define DT_DRV_COMPAT zephyr_cmsis_dap

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cmsis_dap_usb, CONFIG_USB_CMSIS_DAP_LOG_LEVEL);

#define DAP_OUT_EP_IDX		0
#define DAP_IN_EP_IDX		1

struct cmsis_dap_usb_config {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_out_ep;
	struct usb_ep_descriptor if0_in_ep;
} __packed;

struct cmsis_dap_usb_data {
	struct cmsis_dap_cmd_data cmd_data;
	struct usb_dev_data common;
	uint8_t rx_buffer[CONFIG_USB_CMSIS_DAP_PACKET_SIZE];
	struct cmsis_dap_buf *packet_buffer;
	struct k_thread thread;
	struct k_sem packet_sem;
	bool configured;
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_USB_CMSIS_DAP_THREAD_STACK_SIZE);
};

#if CONFIG_USB_CMSIS_DAP_MSOS20_BOS_SUPPORT

/* Byte accessors */
#define __U16_B0(u)				((uint8_t)((u) & 0x00FFu))
#define __U16_B1(u)				((uint8_t)(((u) >> 8) & 0xFFu))
#define __U32_B0(u)				((uint8_t)((u) & 0xFFu))
#define __U32_B1(u)				((uint8_t)(((u) >> 8) & 0xFFu))
#define __U32_B2(u)				((uint8_t)(((u) >> 16) & 0xFFu))
#define __U32_B3(u)				((uint8_t)(((u) >> 24) & 0xFFu))
#define __U16_U8S_LE(u)				__U16_B0(u), __U16_B1(u)
#define __U32_U8S_LE(u)				__U32_B0(u), __U32_B1(u), __U32_B2(u), __U32_B3(u)

/* MS-OS 2.0 definition (function + props) */
#define MSOS20_DEFINE_FUNCTION(idx) \
	__U16_U8S_LE(8), __U16_U8S_LE(0x02), idx, 0, __U16_U8S_LE(8 + (20 + (10 + 40 + 78))),	\
												\
	__U16_U8S_LE(20), __U16_U8S_LE(0x03),							\
	'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,						\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,						\
												\
	__U16_U8S_LE(10 + 40 + 78), __U16_U8S_LE(0x04), __U16_U8S_LE(7), __U16_U8S_LE(40),	\
	'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00,	\
	't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00,	\
	'U', 0x00, 'I', 0x00, 'D', 0x00, 0x00, 0x00,						\
	__U16_U8S_LE(78),									\
	'{', 0x00, 'C', 0x00, 'D', 0x00, 'B', 0x00, '3', 0x00, 'B', 0x00, '5', 0x00, 'A', 0x00,	\
	'D', 0x00, '-', 0x00, '2', 0x00, '9', 0x00, '3', 0x00, 'B', 0x00, '-', 0x00, '4', 0x00,	\
	'6', 0x00, '6', 0x00, '3', 0x00, '-', 0x00, 'A', 0x00, 'A', 0x00, '3', 0x00, '6', 0x00,	\
	'-', 0x00, '1', 0x00, 'A', 0x00, 'A', 0x00, 'E', 0x00, '4', 0x00, '6', 0x00, '4', 0x00,	\
	'6', 0x00, '3', 0x00, '7', 0x00, '7', 0x00, '6', 0x00, '}', 0x00, 0x00, 0x00,		\

/* MS-OS 2.0 definition (header + configuration) */
#define MSOS20_DEFINE_HEADER	\
	__U16_U8S_LE(10), __U16_U8S_LE(0x00), __U32_U8S_LE(0x06030000), 			\
	__U16_U8S_LE(10 + 8 + ((8 + (20 + (10 + 40 + 78))) 					\
						* DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT))),	\
												\
	__U16_U8S_LE(8), __U16_U8S_LE(0x01), 0, 0,						\
	__U16_U8S_LE(8 + ((8 + (20 + (10 + 40 + 78))) 						\
						* DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT))),	\

/* Predefined response to control commands related to MS OS 2.0 descriptors */
static const uint8_t msos20_descriptor[] = {
	MSOS20_DEFINE_HEADER
	DT_INST_FOREACH_STATUS_OKAY(MSOS20_DEFINE_FUNCTION)
};

/* BOS capability */
USB_DEVICE_BOS_DESC_DEFINE_CAP struct usb_bos_msos20_desc {
	struct usb_bos_platform_descriptor platform;
	struct usb_bos_capability_msos cap;
} __packed cmsis_dap_bos_cap_msos20 = {
	.platform = {
		.bLength = sizeof(struct usb_bos_platform_descriptor)
			 + sizeof(struct usb_bos_capability_msos),
		.bDescriptorType = USB_DESC_DEVICE_CAPABILITY,
		.bDevCapabilityType = USB_BOS_CAPABILITY_PLATFORM,
		.bReserved = 0,
		.PlatformCapabilityUUID = {
			/**
			 * MS OS 2.0 Platform Capability ID
			 * D8DD60DF-4589-4CC7-9CD2-659D9E648A9F
			 */
			0xDF, 0x60, 0xDD, 0xD8,
			0x89, 0x45,
			0xC7, 0x4C,
			0x9C, 0xD2,
			0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
		},
	},
	.cap = {
		/* Windows version (8.1) (0x06030000) */
		.dwWindowsVersion = sys_cpu_to_le32(0x06030000),
		.wMSOSDescriptorSetTotalLength =
			sys_cpu_to_le16(sizeof(msos20_descriptor)),
		.bMS_VendorCode = 0x01,
		.bAltEnumCode = 0x00
	},
};
#endif

static sys_slist_t cmsis_dap_devlist;

#if !CONFIG_USB_CMSIS_DAP_MSOS20_BOS_SUPPORT
#define cmsis_dap_vendor_handle_req		NULL
#else
static int cmsis_dap_vendor_handle_req(struct usb_setup_packet *pSetup,
			     int32_t *len, uint8_t **data)
{
	LOG_DBG("vendor request: bmRequestType: %x, bRequest: %x, wValue: %x, wIndex: %x, wLength: %x",
				pSetup->bmRequestType, pSetup->bRequest, pSetup->wValue, 
				pSetup->wIndex, pSetup->wLength);

	
	if (usb_reqtype_is_to_device(pSetup)) {
		return -ENOTSUP;
	}

	/* Get MS OS 2.0 Descriptors request */
	/* 0x07 means "MS_OS_20_DESCRIPTOR_INDEX" */
	if (pSetup->bRequest == 0x01 && pSetup->wIndex == 0x07) {

		*data = (uint8_t *)(&msos20_descriptor);
		*len = sizeof(msos20_descriptor);

		LOG_DBG("MS-OS 2.0 Descriptors fetched");

		return 0;
	}

	return -ENOTSUP;
}
#endif

static void cmsis_dap_interface_config(struct usb_desc_header *head,
				       uint8_t bInterfaceNumber)
{
	struct usb_if_descriptor *if_desc = (struct usb_if_descriptor *)head;
	struct cmsis_dap_usb_config *desc =
			CONTAINER_OF(if_desc, struct cmsis_dap_usb_config, if0);

	LOG_DBG("setting bInterfaceNumber from %d to %d", 
		desc->if0.bInterfaceNumber, bInterfaceNumber);
	desc->if0.bInterfaceNumber = bInterfaceNumber;
}

static void cmsis_dap_read_cb(uint8_t ep, int size, void *priv)
{
	struct cmsis_dap_usb_data *dev_data = priv;
	struct cmsis_dap_buf_pkt *packet;

	LOG_DBG("ep %x size %d", ep, size);

	if (size <= 0) {
		/* Nothing to do, retry */
		goto done;
	}

	if (!cmsis_dap_buf_put_claim(dev_data->packet_buffer, &packet)) {
		LOG_ERR("buffer full, packet dropped");
	} else {
		packet->length = size;
		memcpy(packet->data, dev_data->rx_buffer, size);

		/* Signal the task */
		k_sem_give(&dev_data->packet_sem);
	}

done:
	usb_transfer(ep, dev_data->rx_buffer, sizeof(dev_data->rx_buffer),
		     USB_TRANS_READ, cmsis_dap_read_cb, dev_data);
}

static void cmsis_dap_write_cb(uint8_t ep, int size, void *priv)
{
	struct cmsis_dap_usb_data *dev_data = priv;
	struct cmsis_dap_buf_pkt *packet;

	if (usb_transfer_is_busy(ep)) {
		LOG_DBG("ransfer is ongoing");
		return;
	}

	cmsis_dap_buf_get_release(dev_data->packet_buffer);
	if (!cmsis_dap_buf_get_claim(dev_data->packet_buffer, &packet)) {
		LOG_DBG("nothing to send");
		return;
	}

	LOG_DBG("transferring %zd bytes to ep %x", packet->length, ep); 

	usb_transfer(ep, packet->data, packet->length,
		     USB_TRANS_WRITE, cmsis_dap_write_cb, dev_data);
}

static void cmsis_dap_thread(void *arg1, void *arg2, void *arg3)
{
	struct cmsis_dap_usb_data *dev_data = arg1;
	struct usb_cfg_data *cfg = arg2;
	struct cmsis_dap_buf_pkt *packet;
	int ret;

	ARG_UNUSED(arg3);

	/* Initialize the command handler */
	ret = cmsis_dap_cmd_init(&dev_data->cmd_data);
	if (ret < 0) {
		LOG_ERR("handler initialization failed");
		return;
	}

	while (1) {
		k_sem_take(&dev_data->packet_sem, K_FOREVER);

		while (cmsis_dap_buf_put_inspect(dev_data->packet_buffer, &packet)) {

			LOG_HEXDUMP_DBG(packet->data, packet->length, "request buffer");

			/* Call the handler */
			ret = cmsis_dap_cmd_handler(&dev_data->cmd_data,
						    packet->data,
						    packet->length);

			/* Release and transmit the response */
			packet->length = ret;
			if (!cmsis_dap_buf_put_release(dev_data->packet_buffer)) {
				LOG_ERR("cannot release packet");
			}

			LOG_HEXDUMP_DBG(packet->data, packet->length, "response buffer");

			/* Write */
			cmsis_dap_write_cb(cfg->endpoint[DAP_IN_EP_IDX].ep_addr, 0, dev_data);
		}
	}
}

static void cmsis_dap_dev_status_cb(struct usb_cfg_data *cfg,
				    enum usb_dc_status_code status,
				    const uint8_t *param)
{
	struct cmsis_dap_usb_data *dev_data;
	struct usb_dev_data *common;

	LOG_DBG("cfg %p status %d", cfg, status);

	common = usb_get_dev_data_by_cfg(&cmsis_dap_devlist, cfg);
	if (common == NULL) {
		LOG_WRN("Device data not found for cfg %p", cfg);
		return;
	}

	dev_data = CONTAINER_OF(common, struct cmsis_dap_usb_data, common);

	/* Check the USB status and do needed action if required */
	switch (status) {
	case USB_DC_ERROR:
		LOG_DBG("USB device error");
		break;
	case USB_DC_RESET:
		LOG_DBG("USB device reset detected");
		dev_data->configured = false;
		break;
	case USB_DC_CONNECTED:
		LOG_DBG("USB device connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_DBG("USB device configured");
		if (!dev_data->configured) {
			cmsis_dap_read_cb(cfg->endpoint[DAP_OUT_EP_IDX].ep_addr, 0, dev_data);
			dev_data->configured = true;
		}
		break;
	case USB_DC_DISCONNECTED:
		LOG_DBG("USB device disconnected");
		dev_data->configured = false;
		break;
	case USB_DC_SUSPEND:
		LOG_DBG("USB device suspended");
		break;
	case USB_DC_RESUME:
		LOG_DBG("USB device resumed");
		break;
	case USB_DC_UNKNOWN:
	default:
		LOG_DBG("USB unknown state");
		break;
	}
}

static int cmsis_dap_init(const struct device *dev)
{
	struct cmsis_dap_usb_data *dev_data = dev->data;
	struct cmsis_dap_cmd_data *cmd_data = &dev_data->cmd_data;
	k_tid_t tid;

#if CONFIG_USB_CMSIS_DAP_MSOS20_BOS_SUPPORT
	static bool msos20_registered = false;

	/* Only register once */
	if (!msos20_registered) {
		msos20_registered = true;
		usb_bos_register_cap((void *)&cmsis_dap_bos_cap_msos20);
		LOG_DBG("BOS capability for MS OS v2 registered");
	}
#endif

	dev_data->common.dev = dev;
	sys_slist_append(&cmsis_dap_devlist, &dev_data->common.node);

	if (!device_is_ready(cmd_data->jtag_dev)) {
		return -ENODEV;
	}

	LOG_DBG("Device dev %p dev_data %p cfg %p added to devlist %p",
		dev, dev_data, dev->config, &cmsis_dap_devlist);

	tid = k_thread_create(&dev_data->thread, dev_data->thread_stack,
			      K_KERNEL_STACK_SIZEOF(dev_data->thread_stack),
			      cmsis_dap_thread, dev_data, (void *)dev->config, NULL,
			      CONFIG_USB_CMSIS_DAP_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(tid, dev->name);
	k_sem_init(&dev_data->packet_sem, 0, 1);
	cmsis_dap_buf_reset(dev_data->packet_buffer);

	return 0;
}

int cmsis_dap_vendor_callback_set(const struct device *dev,
				  cmsis_dap_vendor_callback_t callback,
				  void *data)
{
	struct cmsis_dap_usb_data *dev_data = dev->data;
	struct cmsis_dap_cmd_data *cmd_data = &dev_data->cmd_data;

	cmd_data->vendor_cb = callback;
	cmd_data->vendor_data = data;

	return 0;
}

#define INITIALIZER_IF(iface_num, i_iface)							\
	{											\
		.bLength = sizeof(struct usb_if_descriptor),					\
		.bDescriptorType = USB_DESC_INTERFACE,						\
		.bInterfaceNumber = iface_num,							\
		.bAlternateSetting = 0,								\
		.bNumEndpoints = 2,								\
		.bInterfaceClass = USB_BCC_VENDOR,						\
		.bInterfaceSubClass = 0,							\
		.bInterfaceProtocol = 0,							\
		.iInterface = i_iface + 4,							\
	}

#define INITIALIZER_IF_EP(addr, mps, interval)							\
	{											\
		.bLength = sizeof(struct usb_ep_descriptor),					\
		.bDescriptorType = USB_DESC_ENDPOINT,						\
		.bEndpointAddress = addr,							\
		.bmAttributes = USB_DC_EP_BULK,							\
		.wMaxPacketSize = sys_cpu_to_le16(mps),						\
		.bInterval = interval,								\
	}

#define CMSIS_DAP_IF_NAME(x)									\
	DT_INST_PROP_OR(x, iface_name, "CMSIS-DAPv2 #"#x)

#define CMSIS_DAP_STRING_DESCR_DEFINE(p, instance)						\
	static __in_section(usb, descriptor_##p.5, instance) __used __aligned(1)

#define CMSIS_DAP_CFG_AND_DATA_DEFINE(x)							\
	CMSIS_DAP_STRING_DESCR_DEFINE(primary, x)						\
	struct {										\
		uint8_t bLength;								\
		uint8_t bDescriptorType;							\
		uint8_t bString[USB_BSTRING_LENGTH(CMSIS_DAP_IF_NAME(x))];			\
	} __packed cmsis_dap_usb_string_desc##x = {						\
		.bLength = USB_STRING_DESCRIPTOR_LENGTH(CMSIS_DAP_IF_NAME(x)),			\
		.bDescriptorType = USB_DESC_STRING,						\
		.bString = CMSIS_DAP_IF_NAME(x)							\
	};											\
												\
	USBD_CLASS_DESCR_DEFINE(primary, -1##x)							\
	struct cmsis_dap_usb_config cmsis_dap_usb_cfg_##x = {					\
		.if0 = INITIALIZER_IF(0, x),							\
		.if0_out_ep = INITIALIZER_IF_EP(AUTO_EP_OUT,					\
				CONFIG_USB_CMSIS_DAP_BULK_EP_MPS,				\
				0x00),								\
		.if0_in_ep = INITIALIZER_IF_EP(AUTO_EP_IN,					\
				CONFIG_USB_CMSIS_DAP_BULK_EP_MPS,				\
				0x00),								\
	};											\
												\
	static struct usb_ep_cfg_data cmsis_dap_usb_ep_data_##x[] = {				\
		[DAP_OUT_EP_IDX] = {								\
			.ep_cb = usb_transfer_ep_callback,					\
			.ep_addr = AUTO_EP_OUT,							\
		},										\
		[DAP_IN_EP_IDX] = {								\
			.ep_cb = usb_transfer_ep_callback,					\
			.ep_addr = AUTO_EP_IN,							\
		},										\
	};											\
												\
	USBD_DEFINE_CFG_DATA(cmsis_dap_usb_config_##x) = {					\
		.usb_device_description = NULL,							\
		.interface_config = cmsis_dap_interface_config,					\
		.interface_descriptor = &cmsis_dap_usb_cfg_##x.if0,				\
		.cb_usb_status = cmsis_dap_dev_status_cb,					\
		.interface = {									\
			.custom_handler = NULL,							\
			.vendor_handler = cmsis_dap_vendor_handle_req,				\
		},										\
		.num_endpoints = ARRAY_SIZE(cmsis_dap_usb_ep_data_##x),				\
		.endpoint = cmsis_dap_usb_ep_data_##x,						\
	};											\
												\
	CMSIS_DAP_BUF_DECLARE(cmsis_dap_packet_buffer_##x,					\
		DT_INST_PROP_OR(x, packet_count, CONFIG_USB_CMSIS_DAP_PACKET_COUNT));		\
												\
	static struct cmsis_dap_usb_data cmsis_dap_usb_data_##x = {				\
		.cmd_data = CMSIS_DAP_CMD_INITIALIZER(x),					\
		.packet_buffer = &cmsis_dap_packet_buffer_##x,					\
	};

#define CMSIS_DAP_DT_DEVICE_DEFINE(idx)								\
	BUILD_ASSERT(DT_INST_ON_BUS(idx, usb),							\
		     "node " DT_NODE_PATH(DT_DRV_INST(idx))					\
		     " is not assigned to a USB device controller");				\
												\
	CMSIS_DAP_CFG_AND_DATA_DEFINE(idx)							\
												\
	DEVICE_DT_INST_DEFINE(idx, cmsis_dap_init, NULL,					\
		&cmsis_dap_usb_data_##idx, &cmsis_dap_usb_config_##idx,				\
		APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY,				\
		NULL);

DT_INST_FOREACH_STATUS_OKAY(CMSIS_DAP_DT_DEVICE_DEFINE);
