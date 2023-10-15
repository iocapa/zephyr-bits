#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/jtag.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "cmsis_dap_cmd.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cmsis_dap_cmd, CONFIG_USB_CMSIS_DAP_LOG_LEVEL);

/* Error codes */
#define CMSIS_DAP_RSP_OK			0x00U
#define CMSIS_DAP_RSP_NOT_OK			0xFFU

/* Command codes */
#define CMSIS_DAP_CMD_INFO			0x00U
#define CMSIS_DAP_CMD_HOST_STATUS		0x01U
#define CMSIS_DAP_CMD_CONNECT			0x02U
#define CMSIS_DAP_CMD_DISCONNECT		0x03U
#define CMSIS_DAP_CMD_TRANSFER_CONFIGURE	0x04U
#define CMSIS_DAP_CMD_TRANSFER			0x05U
#define CMSIS_DAP_CMD_TRANSFER_BLOCK		0x06U
// #define CMSIS_DAP_CMD_TRANSFER_ABORT		0x07U
// #define CMSIS_DAP_CMD_WRITE_ABORT		0x08U
#define CMSIS_DAP_CMD_DELAY			0x09U
// #define CMSIS_DAP_CMD_RESET_TARGET		0x0AU
#define CMSIS_DAP_CMD_SWJ_PINS			0x10U
#define CMSIS_DAP_CMD_SWJ_CLOCK			0x11U
#define CMSIS_DAP_CMD_SWJ_SEQUENCE		0x12U
#define CMSIS_DAP_CMD_SWD_CONFIGURE		0x13U
#define CMSIS_DAP_CMD_SWD_SEQUENCE		0x1DU
#define CMSIS_DAP_CMD_JTAG_SEQUENCE		0x14U
// #define CMSIS_DAP_CMD_JTAG_CONFIGURE		0x15U
// #define CMSIS_DAP_CMD_JTAG_IDCODE		0x16U
// #define CMSIS_DAP_CMD_SWO_TRANSPORT		0x17U
// #define CMSIS_DAP_CMD_SWO_MODE			0x18U
// #define CMSIS_DAP_CMD_SWO_BAUDRATE		0x19U
// #define CMSIS_DAP_CMD_SWO_CONTROL		0x1AU
// #define CMSIS_DAP_CMD_SWO_STATUS		0x1BU
// #define CMSIS_DAP_CMD_SWO_EXTENDED_STATUS	0x1EU
// #define CMSIS_DAP_CMD_SWO_DATA			0x1CU
// #define CMSIS_DAP_CMD_QUEUE_COMMANDS		0x7EU
// #define CMSIS_DAP_CMD_EXECUTE_COMMANDS		0x7FU

/* Sub-codes for CMSIS_DAP_CMD_INFO */
#define CMSIS_DAP_CMD_INFO_VENDOR_NAME		0x01U
#define CMSIS_DAP_CMD_INFO_PRODUCT_NAME		0x02U
#define CMSIS_DAP_CMD_INFO_SERIAL_NUMBER	0x03U
#define CMSIS_DAP_CMD_INFO_PROTOCOL_VERSION	0x04U
#define CMSIS_DAP_CMD_INFO_DEVICE_VENDOR	0x05U
#define CMSIS_DAP_CMD_INFO_DEVICE_NAME		0x06u
#define CMSIS_DAP_CMD_INFO_BOARD_VENDOR		0x07u
#define CMSIS_DAP_CMD_INFO_BOARD_NAME		0x08u
#define CMSIS_DAP_CMD_INFO_FIRMWARE_VERSION	0x09u
#define CMSIS_DAP_CMD_INFO_CAPABILITIES		0xF0U
#define CMSIS_DAP_CMD_INFO_PACKET_COUNT		0xFEU
#define CMSIS_DAP_CMD_INFO_PACKET_SIZE		0xFFU

/* Flags for CMSIS_DAP_CMD_INFO_CAPABILITIES */
#define CMSIS_DAP_CMD_INFO_CAPABILITIES_SWD	BIT(0)
#define CMSIS_DAP_CMD_INFO_CAPABILITIES_JTAG	BIT(1)

/* Flags for CMSIS_DAP_CMD_CONNECT */
#define CMSIS_DAP_CMD_CONNECT_DEFAULT		0
#define CMSIS_DAP_CMD_CONNECT_SWD		1
#define CMSIS_DAP_CMD_CONNECT_JTAG		2

#define CMSIS_DAP_TRANSFER_APnDP		BIT(0)
#define CMSIS_DAP_TRANSFER_RnW			BIT(1)
#define CMSIS_DAP_TRANSFER_A2			BIT(2)
#define CMSIS_DAP_TRANSFER_A3			BIT(3)
#define CMSIS_DAP_TRANSFER_MATCH_VALUE		BIT(4)
#define CMSIS_DAP_TRANSFER_MATCH_MASK		BIT(5)

#define CMSIS_DAP_TRANSFER_OK			BIT(0)
#define CMSIS_DAP_TRANSFER_WAIT			BIT(1)
#define CMSIS_DAP_TRANSFER_FAULT		BIT(2)

typedef void (*cmsis_dap_cmd_generic_instr)(struct jtag_instruction *instr,
					    const uint8_t **out_buf,
					    uint8_t **in_buf);

static void cmsis_dap_cmd_swd_instr(struct jtag_instruction *instr,
				    const uint8_t **out_buf,
				    uint8_t **in_buf)
{
	uint8_t bit_count, byte_count;
	bool is_input;

	/* Get info */
	is_input = (*(*out_buf) & BIT(7)) != 0;
	bit_count = *(*out_buf) & BIT_MASK(6);
	(*out_buf)++;

	/* Wrap bit count */
	if (bit_count == 0) {
		bit_count = 64;
	}

	/* Compute byte count */
	byte_count = (bit_count + 7) / 8;

	/* Format instruction */
	instr->shift_count = bit_count;

	if (is_input) {
		instr->op_code = JTAG_INSTRUCTION_TMS_IN;
		instr->shift_in = *in_buf;
		*in_buf += byte_count;
	} else {
		instr->op_code = JTAG_INSTRUCTION_TMS_OUT;
		instr->shift_out = *out_buf;
		*out_buf += byte_count;
	}

	LOG_ERR("swd sequence instruction");
}

static void cmsis_dap_cmd_jtag_instr(struct jtag_instruction *instr,
				     const uint8_t **out_buf,
				     uint8_t **in_buf)
{
	uint8_t bit_count, byte_count;
	bool is_input, tms_value;

	/* Get info */
	is_input = (*(*out_buf) & BIT(7)) != 0;
	tms_value = (*(*out_buf) & BIT(6)) != 0;
	bit_count = (*(*out_buf)) & BIT_MASK(6);
	(*out_buf)++;

	/* Wrap bit count */
	if (bit_count == 0) {
		bit_count = 64;
	}

	/* Compute byte count */
	byte_count = (bit_count + 7) / 8;

	/* Format instruction */
	instr->shift_count = bit_count;
	instr->op_code = JTAG_INSTRUCTION_TDX_INOUT;
	instr->tms_value = tms_value;
	instr->shift_out = *out_buf;
	*out_buf += byte_count;

	if (is_input) {
		instr->shift_in = *in_buf;
		*in_buf += byte_count;
	} else {
		instr->shift_in = NULL;
	}
}

static uint8_t cmsis_dap_cmd_parity(uint32_t value)
{
	value ^= value >> 16;
	value ^= value >> 8;
	value ^= value >> 4;
	value &= 0x0f;

	return (0x6996 >> value) & 1;
}

static int cmsis_dap_cmd_swd_transfer(struct cmsis_dap_cmd_data *data,
				      uint8_t request,
				      uint32_t *value)
{
	struct jtag_instruction *instr = data->instr_buf;
	uint8_t header, response = 0, buf[5];
	int rc;

	/* Mask unused bits */
	request &= CMSIS_DAP_TRANSFER_APnDP
		 | CMSIS_DAP_TRANSFER_RnW
		 | CMSIS_DAP_TRANSFER_A2
		 | CMSIS_DAP_TRANSFER_A3;

	header = 0x81 | (cmsis_dap_cmd_parity(request) << 5) | (request << 1);

	uint8_t address = (header >> 1) & (BIT_MASK(2) << 2);
	//LOG_ERR("addr %02x", address << 2);

	/* Header */
	instr[0].op_code = JTAG_INSTRUCTION_TMS_OUT;
	instr[0].shift_count = 8;
	instr[0].shift_out = &header;

	/* Read status */
	instr[1].op_code = JTAG_INSTRUCTION_TMS_IN;
	instr[1].shift_count = 3 + data->swd_turnaround;
	instr[1].shift_in = &response;

	/* Execute */
	rc = jtag_exec(data->jtag_dev, instr, 2);
	if (rc < 0) {
		/* TODO */
		LOG_ERR("transfer failed %d", rc);
	}

	/* Format response */
	response = (response >> data->swd_turnaround) & BIT_MASK(3);

	//LOG_ERR("resp %02x", response);

	if (response == CMSIS_DAP_TRANSFER_OK) {
		if (request & CMSIS_DAP_TRANSFER_RnW) {
			/* Read rest */
			instr[0].op_code = JTAG_INSTRUCTION_TMS_IN;
			instr[0].shift_count = 33;
			instr[0].shift_in = buf;

			instr[1].op_code = JTAG_INSTRUCTION_TMS_IN;
			instr[1].shift_count = data->swd_turnaround;
			instr[1].shift_in = NULL;

			instr[2].op_code = JTAG_INSTRUCTION_TMS_OUT;
			instr[2].shift_count = 10;
			instr[2].shift_out = NULL;
			instr[2].tms_value = false;

			rc = jtag_exec(data->jtag_dev, instr, 3);
			if (rc < 0) {
				/* TODO */
				LOG_ERR("transfer failed %d", rc);
			}

			/* Check parity */
			if (cmsis_dap_cmd_parity(sys_get_le32(buf)) != (buf[4] & BIT_MASK(1))) {
				LOG_ERR("parity error");
			}

			*value = sys_get_le32(buf);
			LOG_DBG("<--: APnDP: %u, addr: %x, value: %08x", (header & BIT(1)) ? 1 : 0, address, *value);
		} else {

			/* Turnaround */
			instr[0].op_code = JTAG_INSTRUCTION_TMS_IN;
			instr[0].shift_count = data->swd_turnaround;
			instr[0].shift_in = NULL;

			/* Data phase */
			sys_put_le32(*value, buf);
			buf[4] = cmsis_dap_cmd_parity(*value);

			instr[1].op_code = JTAG_INSTRUCTION_TMS_OUT;
			instr[1].shift_count = 33;
			instr[1].shift_out = buf;

			instr[2].op_code = JTAG_INSTRUCTION_TMS_OUT;
			instr[2].shift_count = 10;
			instr[2].shift_out = NULL;
			instr[2].tms_value = false;

			rc = jtag_exec(data->jtag_dev, instr, 3);
			if (rc < 0) {
				/* TODO */
				LOG_ERR("transfer failed %d", rc);
			}
			LOG_DBG("-->: APnDP: %u, addr: %x, value: %08x", (request & CMSIS_DAP_TRANSFER_APnDP) ? 1 : 0, address, *value);
		}

		/* Idle cycles */
		instr[0].op_code = JTAG_INSTRUCTION_TMS_OUT;
		instr[0].shift_count = data->idle_cycles;
		instr[0].tms_value = true;
		instr[0].shift_out = NULL;

		rc = jtag_exec(data->jtag_dev, instr, 1);
		if (rc < 0) {
			/* TODO */
			LOG_ERR("transfer failed %d", rc);
		}


	} else if ((response == CMSIS_DAP_TRANSFER_WAIT) 
	        || (response == CMSIS_DAP_TRANSFER_FAULT)) {

		LOG_DBG("transfer wait");

		if (data->swd_data_phase && (request & CMSIS_DAP_TRANSFER_RnW)) {

			instr[0].op_code = JTAG_INSTRUCTION_TMS_IN;
			instr[0].shift_count = 33;
			instr[0].shift_in = NULL;

			rc = jtag_exec(data->jtag_dev, instr, 1);
			if (rc < 0) {
				/* TODO */
				LOG_ERR("transfer failed %d", rc);
			}
		}

		/* Turnaround */
		instr[0].op_code = JTAG_INSTRUCTION_TMS_IN;
		instr[0].shift_count = data->swd_turnaround;
		instr[0].shift_in = NULL;

		rc = jtag_exec(data->jtag_dev, instr, 1);
		if (rc < 0) {
			/* TODO */
			LOG_ERR("transfer failed %d", rc);
		}

		if (data->swd_data_phase && ((request & CMSIS_DAP_TRANSFER_RnW) ==0)) {

			instr[0].op_code = JTAG_INSTRUCTION_TMS_OUT;
			instr[0].shift_count = 33;
			instr[0].shift_out = NULL;
			instr[0].tms_value = false;

			rc = jtag_exec(data->jtag_dev, instr, 1);
			if (rc < 0) {
				/* TODO */
				LOG_ERR("transfer failed %d", rc);
			}
		}
	} else {
		LOG_DBG("transfer error");

		/* Turnaround */
		instr[0].op_code = JTAG_INSTRUCTION_TMS_IN;
		instr[0].shift_count = data->swd_turnaround + 33;
		instr[0].shift_in = NULL;

		rc = jtag_exec(data->jtag_dev, instr, 1);
		if (rc < 0) {
			/* TODO */
			LOG_ERR("transfer failed %d", rc);
		}
	}

	return -((int)response);
}

static int cmsis_dap_cmd_info(struct cmsis_dap_cmd_data *data,
			      uint8_t *buffer, int rdlen)
{
	switch (buffer[0]) {

	case CMSIS_DAP_CMD_INFO_VENDOR_NAME:
	case CMSIS_DAP_CMD_INFO_PRODUCT_NAME:
	case CMSIS_DAP_CMD_INFO_SERIAL_NUMBER:
	case CMSIS_DAP_CMD_INFO_PROTOCOL_VERSION:
	case CMSIS_DAP_CMD_INFO_DEVICE_VENDOR:
	case CMSIS_DAP_CMD_INFO_DEVICE_NAME:
	case CMSIS_DAP_CMD_INFO_BOARD_VENDOR:
	case CMSIS_DAP_CMD_INFO_BOARD_NAME:
	case CMSIS_DAP_CMD_INFO_FIRMWARE_VERSION:
		buffer[0] = 0;
		return 1;

	case CMSIS_DAP_CMD_INFO_CAPABILITIES:
		buffer[0] = sizeof(uint8_t);
		buffer[1] = data->dap_caps;
		return 2;

	case CMSIS_DAP_CMD_INFO_PACKET_COUNT:
		buffer[0] = sizeof(uint8_t);
		buffer[1] = data->packet_count;
		return 2;

	case CMSIS_DAP_CMD_INFO_PACKET_SIZE:
		buffer[0] = sizeof(uint16_t);
		sys_put_le16(data->packet_size, &buffer[1]);
		return 3;

	default:
		LOG_ERR("unknown DAP_INFO command %02x", buffer[0]);
		buffer[0] = CMSIS_DAP_RSP_NOT_OK;
		return 1;
	}
}

static int cmsis_dap_cmd_connect(struct cmsis_dap_cmd_data *data,
				 uint8_t *buffer, int rdlen)
{
	uint8_t port = buffer[0];
	int rc;

	LOG_DBG("cmsis_dap_cmd_connect port %u", port);

	/* Validate port */
	if (port > CMSIS_DAP_CMD_CONNECT_JTAG) {
		LOG_ERR("invalid port %u", port);
		buffer[0] = CMSIS_DAP_RSP_NOT_OK;
		return 1;
	}

	/* Determine port based on caps. Preffer SWD */
	if (port == 0) {
		if (data->dap_caps & CMSIS_DAP_CMD_INFO_CAPABILITIES_SWD) {
			port = CMSIS_DAP_CMD_CONNECT_SWD;
		} else if (data->dap_caps & CMSIS_DAP_CMD_INFO_CAPABILITIES_JTAG) {
			port = CMSIS_DAP_CMD_CONNECT_JTAG;
		} else {
			LOG_ERR("no default port available");
			return 1;
		}
	}

	/* Save connection info */
	data->dap_port = port;

	/* Try to connect */
	rc = jtag_connect(data->jtag_dev, port == CMSIS_DAP_CMD_CONNECT_JTAG);
	if (rc < 0) {
		LOG_ERR("cannot connect to port %u", port);
		buffer[0] = CMSIS_DAP_RSP_NOT_OK;
		return 1;
	}

	buffer[0] = port;
	return 1;
}

static int cmsis_dap_cmd_disconnect(struct cmsis_dap_cmd_data *data,
				    uint8_t *buffer, int rdlen)
{
	int rc;

	/* Try to disconnect */
	rc = jtag_disconnect(data->jtag_dev);
	if (rc < 0) {
		LOG_ERR("cannot disconnect");
		buffer[0] = CMSIS_DAP_RSP_NOT_OK;
	} else {
		buffer[0] = CMSIS_DAP_RSP_OK;
	}

	return 1;
}

static int cmsis_dap_cmd_delay(struct cmsis_dap_cmd_data *data,
			       uint8_t *buffer, int rdlen)
{
	uint16_t delay = sys_get_le16(buffer);

	LOG_DBG("delay %u us", delay);

	/* Just delay a number of microseconds (precision not required) */
	k_usleep((int)delay);

	buffer[0] = CMSIS_DAP_RSP_OK;
	return 1;
}

static int cmsis_dap_cmd_swj_pins(struct cmsis_dap_cmd_data *data,
				  uint8_t *buffer, int rdlen)
{
	// TODO
	LOG_ERR("setting pins mask %02x, value %02x", buffer[1], buffer[0]);
	jtag_set_pins(data->jtag_dev, buffer[1], buffer[0]);
	jtag_get_pins(data->jtag_dev, &buffer[0]);
	return 1;
}

static int cmsis_dap_cmd_swj_clock(struct cmsis_dap_cmd_data *data,
				   uint8_t *buffer, int rdlen)
{
	uint32_t freq_hz = sys_get_le32(buffer);
	int rc;

	LOG_DBG("setting clock to: %u", freq_hz);

	rc = jtag_set_frequency(data->jtag_dev, freq_hz);
	if (rc < 0) {
		LOG_ERR("cannot set frequency");
		buffer[0] = CMSIS_DAP_RSP_NOT_OK;
	} else {
		buffer[0] = CMSIS_DAP_RSP_OK;
	}

	return 1;
}

static int cmsis_dap_cmd_swj_sequence(struct cmsis_dap_cmd_data *data,
				      uint8_t *buffer, int rdlen)
{
	struct jtag_instruction *instr = data->instr_buf;
	uint16_t bit_count = buffer[0];
	int rc;

	if (bit_count == 0) {
		bit_count = 256;
	}

	/* Setup and execute */
	instr[0].op_code = JTAG_INSTRUCTION_TMS_OUT;
	instr[0].shift_count = bit_count;
	instr[0].shift_out = &buffer[1];

	rc = jtag_exec(data->jtag_dev, instr, 1);
	if (rc < 0) {
		LOG_ERR("cannot execute swj_sequence");
		buffer[0] = CMSIS_DAP_RSP_NOT_OK;
	} else {
		buffer[0] = CMSIS_DAP_RSP_OK;
	}

	//LOG_ERR("swj sequence");

	return 1;
}

static int cmsis_dap_cmd_swd_configure(struct cmsis_dap_cmd_data *data,
				       uint8_t *buffer, int rdlen)
{
	data->swd_turnaround = (buffer[0] & BIT_MASK(2)) + 1;
	LOG_DBG("swd_turnaround set to %u", data->swd_turnaround);

	data->swd_data_phase = (buffer[0] & BIT(2)) != 0;
	LOG_DBG("swd_data_phase set to %s", data->swd_data_phase ? "true" : "false");

	buffer[0] = CMSIS_DAP_RSP_OK;
	return 1;
}

static int cmsis_dap_cmd_generic_sequence(struct cmsis_dap_cmd_data *data,
					  uint8_t *buffer, int rdlen,
					  cmsis_dap_cmd_generic_instr f_instr)
{
	struct jtag_instruction *instr_buf = data->instr_buf;
	const uint8_t *out_buf = &buffer[1];
	uint8_t *in_buf = &buffer[1];
	uint16_t seq_count, rem_count, instr_ind;
	int rc;

	seq_count = buffer[0];
	if (seq_count == 0) {
		LOG_ERR("invalid sequence count %u", seq_count);
		buffer[0] = CMSIS_DAP_RSP_NOT_OK;
		return 1;
	}

	while (seq_count) {
		/* Remove the run from total */
		rem_count = MIN(seq_count, ARRAY_SIZE(data->instr_buf));
		seq_count -= rem_count;

		/* Create the buffer */
		for (instr_ind = 0; instr_ind < rem_count; instr_ind++) {

			/* Create instruction */
			f_instr(&instr_buf[instr_ind], &out_buf, &in_buf);
		}

		/* Execute the buffer */
		rc = jtag_exec(data->jtag_dev, instr_buf, instr_ind);
		if (rc < 0) {
			LOG_ERR("failed to execute swd_sequence");
			buffer[0] = CMSIS_DAP_RSP_NOT_OK;
			return 1;
		}
	}

	buffer[0] = CMSIS_DAP_RSP_OK;

	/* Include the response */
	return (in_buf - &buffer[0]);
}

static int cmsis_dap_cmd_transfer_configure(struct cmsis_dap_cmd_data *data,
					    uint8_t *buffer, int rdlen)
{
	/* Just push the values */
	data->idle_cycles = buffer[0];
	data->wait_retry = sys_get_le16(&buffer[1]);
	data->match_retry = sys_get_le16(&buffer[3]);

	LOG_DBG("idle cycles: %u", data->idle_cycles);
	LOG_DBG("wait retry: %u", data->wait_retry);
	LOG_DBG("match retry: %u", data->match_retry);

	buffer[0] = CMSIS_DAP_RSP_OK;
	return 1;
}

static int cmsis_dap_cmd_transfer(struct cmsis_dap_cmd_data *data,
				  uint8_t *buffer, int rdlen)
{
	const uint8_t *out_buf = &buffer[2];
	uint8_t *in_buf = &buffer[2];
	uint8_t xfer_info;
	uint8_t xfer_done = 0;
	uint8_t xfer_count;
	uint16_t xfer_size = 0;
	uint32_t xfer_data;

	if (buffer[0] != 0) {
		LOG_ERR("not supported");
		sys_put_le16(0, buffer);
		return 2;
	}

	/* Get number of transfers */
	xfer_count = buffer[1];

	while (xfer_count--) {
		xfer_info = *out_buf;
		out_buf++;
		
		if (xfer_info & BIT(7)) {
			LOG_ERR("crapstamp");
		}

		/* Read */
		if (xfer_info & CMSIS_DAP_TRANSFER_RnW) {
			if (xfer_info & CMSIS_DAP_TRANSFER_MATCH_VALUE) {
				LOG_ERR("match value not supported");
			} else {
				cmsis_dap_cmd_swd_transfer(data, xfer_info, &xfer_data);
				if (xfer_info & CMSIS_DAP_TRANSFER_APnDP) {
					cmsis_dap_cmd_swd_transfer(data, xfer_info, &xfer_data);
				}
				sys_put_le32(xfer_data, in_buf);
				in_buf += 4;
				xfer_size += 4;
			}
		} else { /* Write */
			if (xfer_info & CMSIS_DAP_TRANSFER_MATCH_MASK) {
				LOG_ERR("match mask not supported");
			} else {
				xfer_data = sys_get_le32(out_buf);
				cmsis_dap_cmd_swd_transfer(data, xfer_info, &xfer_data);
				out_buf += 4;
			}
		}

		xfer_done += 1;
	}

	buffer[0] = xfer_done;
	buffer[1] = BIT(0);

	//LOG_HEXDUMP_ERR(buffer, xfer_size + 2, "response buffer");

	/* TODO */
	return xfer_size + 2;
}

static int cmsis_dap_cmd_transfer_block(struct cmsis_dap_cmd_data *data,
					uint8_t *buffer, int rdlen)
{
	/* TODO */
	LOG_ERR("cmsis_dap_cmd_transfer_block not supported");
	buffer[0] = CMSIS_DAP_RSP_NOT_OK;
	return 1;
}

static int cmsis_dap_cmd_host_status(struct cmsis_dap_cmd_data *data,
				     uint8_t *buffer,
				     int rdlen)
{
	// TODO
	buffer[0] = CMSIS_DAP_RSP_OK;
	return 1;
}

int cmsis_dap_cmd_handler(struct cmsis_dap_cmd_data *data,
			  uint8_t *buffer, int rdlen)
{
	uint8_t *fbuffer = &buffer[1];
	int wrlen = 1;

	/* Subtract command code */
	rdlen -= 1;

	switch (buffer[0]) {

	/* General commands */
	case CMSIS_DAP_CMD_INFO:
		wrlen += cmsis_dap_cmd_info(data, fbuffer, rdlen);
		break;

	case CMSIS_DAP_CMD_HOST_STATUS:
		wrlen += cmsis_dap_cmd_host_status(data, fbuffer, rdlen);
		break;

	case CMSIS_DAP_CMD_CONNECT:
		wrlen += cmsis_dap_cmd_connect(data, fbuffer, rdlen);
		break;

	case CMSIS_DAP_CMD_DISCONNECT:
		wrlen += cmsis_dap_cmd_disconnect(data, fbuffer, rdlen);
		break;

	case CMSIS_DAP_CMD_DELAY:
		wrlen += cmsis_dap_cmd_delay(data, fbuffer, rdlen);
		break;

	/* Common commands */
	case CMSIS_DAP_CMD_SWJ_PINS:
		wrlen += cmsis_dap_cmd_swj_pins(data, fbuffer, rdlen);
		break;

	case CMSIS_DAP_CMD_SWJ_CLOCK:
		wrlen += cmsis_dap_cmd_swj_clock(data, fbuffer, rdlen);
		break;

	case CMSIS_DAP_CMD_SWJ_SEQUENCE:
		wrlen += cmsis_dap_cmd_swj_sequence(data, fbuffer, rdlen);
		break;

	/* SWD commands */
	case CMSIS_DAP_CMD_SWD_CONFIGURE:
		wrlen += cmsis_dap_cmd_swd_configure(data, fbuffer, rdlen);
		break;

	case CMSIS_DAP_CMD_SWD_SEQUENCE:
		wrlen += cmsis_dap_cmd_generic_sequence(data, fbuffer, rdlen,
							cmsis_dap_cmd_swd_instr);
		break;

	/* JTAG commands */
	case CMSIS_DAP_CMD_JTAG_SEQUENCE:
		wrlen += cmsis_dap_cmd_generic_sequence(data, fbuffer, rdlen,
							cmsis_dap_cmd_jtag_instr);
		break;

	/* Transfer commands */
	case CMSIS_DAP_CMD_TRANSFER_CONFIGURE:
		wrlen += cmsis_dap_cmd_transfer_configure(data, fbuffer, rdlen);
		break;

	case CMSIS_DAP_CMD_TRANSFER:
		wrlen += cmsis_dap_cmd_transfer(data, fbuffer, rdlen);
		break;

	case CMSIS_DAP_CMD_TRANSFER_BLOCK:
		wrlen += cmsis_dap_cmd_transfer_block(data, fbuffer, rdlen);
		break;

	default:
		/* Unknown command */
		LOG_ERR("command not supported %02x", buffer[0]);
		buffer[1] = CMSIS_DAP_RSP_NOT_OK;
		wrlen += 1;
		break;
	}

	return wrlen;
}

int cmsis_dap_cmd_init(struct cmsis_dap_cmd_data *data)
{
	uint32_t dev_specs;
	uint8_t dap_caps = data->dap_caps;
	int rc;

	/* Get device specs */
	rc = jtag_get_specs(data->jtag_dev, &dev_specs);
	if (rc < 0) {
		return rc;
	}

	/* TMS IN/OUT required for SWD */
	if (!((dap_caps & CMSIS_DAP_SWD_EN)
	&& (dev_specs & JTAG_SPEC_TMS_IN)
	&& (dev_specs & JTAG_SPEC_TMS_OUT)))
	{
		dap_caps &= ~CMSIS_DAP_CMD_INFO_CAPABILITIES_SWD;
	}

	/* TMS OUT and TDX IN/OUT required for JTAG */
	if (!((dap_caps & CMSIS_DAP_JTAG_EN)
	&& (dev_specs & JTAG_SPEC_TDX_INOUT)
	&& (dev_specs & JTAG_SPEC_TMS_OUT)))
	{
		dap_caps &= ~CMSIS_DAP_CMD_INFO_CAPABILITIES_JTAG;
	}

	data->dap_caps = dap_caps;

	return 0;
}