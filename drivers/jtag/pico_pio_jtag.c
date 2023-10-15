#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/jtag.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/dma/rpi_pico_dma.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico_util.h>

/* PIO registers */
#include <hardware/structs/pio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pico_pio_jtag, CONFIG_JTAG_LOG_LEVEL);

#define DT_DRV_COMPAT	raspberrypi_pico_pio_jtag

/* Devicetree pin macros */
#define JTAG_PINMUX_EXISTS_BY_IDX(node_id, idx)							\
	IS_ENABLED(DT_CAT6(node_id, _P_, pinmux, _IDX_, idx, _EXISTS))

#define JTAG_PIN_EXISTS(node_id, p_name, p_idx, g_name, g_idx)					\
	JTAG_PINMUX_EXISTS_BY_IDX(DT_CHILD(DT_PINCTRL_BY_NAME(node_id, p_name, p_idx), g_name),	\
				  g_idx)

#define JTAG_PIN_BY_NAME_OR(node_id, p_name, p_idx, g_name, g_idx, d_val)			\
	COND_CODE_1(JTAG_PIN_EXISTS(node_id, p_name, p_idx, g_name, g_idx),			\
	(RP2_GET_PIN_NUM(DT_PROP_BY_IDX(DT_CHILD(DT_PINCTRL_BY_NAME(node_id, p_name, p_idx),	\
					g_name), pinmux, g_idx))), (d_val))

#define JTAG_INST_PIN_BY_NAME_OR(inst, p_name, p_idx, g_name, g_idx, d_val)			\
	JTAG_PIN_BY_NAME_OR(DT_DRV_INST(inst), p_name, p_idx, g_name, g_idx, d_val)

/* Device configuration structure */
struct pio_jtag_config {
	const struct pinctrl_dev_config *pcfg;
	const struct device *parent;
	struct gpio_dt_spec srst_gpio;
	struct gpio_dt_spec trst_gpio;
	pio_hw_t *pio_regs;
	uint32_t *q_data;
	uint32_t clock_frequency;
	uint16_t q_size;
	uint8_t irq_idx;
	int8_t tck_gpio;
	int8_t tms_gpio;
	int8_t tdi_gpio;
	int8_t tdo_gpio;
};

/* DMA data structure */
struct pio_jtag_dma_data {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
};

/* Device data structure */
struct pio_jtag_data {
	struct pio_jtag_dma_data tx_dma_data;
	struct pio_jtag_dma_data rx_dma_data;
	struct pio_rpi_pico_irq_cfg irq_cfg;
	struct k_sem lock;
	struct k_sem sem;

	uint32_t clkdiv;
	uint8_t sm_index;
	uint8_t sm_mask;
	uint8_t instr;
	bool connected;
	bool tdx_active;
};

/* DSS macro */
#define PIO_JTAG_SS(opt, ss, delay)								\
	PIO_ASM_SIDE(1, opt, 2, ss, delay)

/* Set TMS pin direction */
#define PIO_JTAG_TMS_DIR(dir)									\
	PIO_ASM_SET(PIO_ASM_SET_DST_PINDIRS, dir, PIO_JTAG_SS(0, 0, 0))

/* Set TMS pin value */
#define PIO_JTAG_TMS_VAL(val)									\
	PIO_ASM_SET(PIO_ASM_SET_DST_PINS, val, PIO_JTAG_SS(0, 0, 0))

#define PIO_JTAG_SWAP_IRQ									\
	PIO_ASM_IRQ(0, 1, PIO_ASM_INDEX(true, 0), PIO_JTAG_SS(0, 0, 0))

/* NOP instruction */
#define PIO_JTAG_NOP										\
	PIO_ASM_MOV(PIO_ASM_MOV_DST_Y, 0, PIO_ASM_MOV_SRC_Y, PIO_JTAG_SS(0, 0, 0))

#ifdef JTAG_RPI_PICO_MINIMAL_INSTR
static const uint16_t pio_jtag_prg[] = {
            //     .wrap_target
    0x80a0, //  0: pull   block                      
    0x6030, //  1: out    x, 16                      
    0x60f0, //  2: out    exec, 16                   
    0x60f0, //  3: out    exec, 16                   
    0x60f0, //  4: out    exec, 16                   
    0x7701, //  5: out    pins, 1         side 0 [7] 
    0x5e01, //  6: in     pins, 1         side 1 [6] 
    0x0045, //  7: jmp    x--, 5                     
    0x8000, //  8: push   noblock                    
            //     .wrap
};

#define PIO_JTAG_WRAP_SIZE		(ARRAY_SIZE(pio_jtag_prg) - 1)
#define PIO_JTAG_CLK_DIV		16

#else

static const uint16_t pio_jtag_prg[] = {
            //     .wrap_target
    0x80a0, //  0: pull   block                      
    0x6030, //  1: out    x, 16                      
    0x60f0, //  2: out    exec, 16                   
    0x60f0, //  3: out    exec, 16                   
    0x60f0, //  4: out    exec, 16                   
    0x7701, //  5: out    pins, 1         side 0 [7] 
    0xa242, //  6: nop                           [2] 
    0x5801, //  7: in     pins, 1         side 1     
    0x004a, //  8: jmp    x--, 10                    
    0x8000, //  9: push   noblock                    
            //     .wrap
    0xa042, // 10: nop                               
    0x0705, // 11: jmp    5                      [7] 
};

#define PIO_JTAG_WRAP_SIZE		(ARRAY_SIZE(pio_jtag_prg) - 3)
#define PIO_JTAG_CLK_PREDIV		22

#endif

/* Frequency macros */
#define PIO_JTAG_MAX_FREQ(ref)		((ref) / PIO_JTAG_CLK_PREDIV)
#define PIO_JTAG_MIN_FREQ(ref)		((ref) / (PIO_JTAG_CLK_PREDIV * PIO_SM0_CLKDIV_RESET))

static int pio_jtag_get_specs(const struct device *dev,
			      uint32_t *specs)
{
	const struct pio_jtag_config *dev_config = dev->config;
	uint32_t supp = JTAG_SPEC_TMS_IN | JTAG_SPEC_TMS_OUT;

	if (!specs) {
		return -EINVAL;
	}

	/* Only when tdx pins are present */
	if (!((dev_config->tdi_gpio < 0) || (dev_config->tdi_gpio < 0))) {
		supp |= JTAG_SPEC_TDX_INOUT;
	}

	*specs = supp;

	return 0;
}

static int pio_jtag_set_pins(const struct device *dev,
			     uint8_t mask, uint8_t value)
{
	const struct pio_jtag_config *dev_config = dev->config;

	/* Check for pins we cannot control */
	if (mask & (JTAG_PIN_TCK | JTAG_PIN_TMS | JTAG_PIN_TDI | JTAG_PIN_TDO)) {
		LOG_ERR("setting TCK, TMS, TDI and TDO not supported");
		return -EIO;
	}

	/* nSRST */
	if (mask & JTAG_PIN_NSRST) {
		if (!gpio_is_ready_dt(&dev_config->srst_gpio)) {
			return -EIO;
		}
		gpio_pin_set_dt(&dev_config->srst_gpio, (value & JTAG_PIN_NSRST) ? 0 : 1);
	}

	/* nTRST */
	if (mask & JTAG_PIN_NTRST) {
		if (!gpio_is_ready_dt(&dev_config->trst_gpio)) {
			return -EIO;
		}
		gpio_pin_set_dt(&dev_config->trst_gpio, (value & JTAG_PIN_NTRST) ? 0 : 1);
	}

	return 0;
}

static int pio_jtag_get_pins(const struct device *dev,
			     uint8_t *value)
{
	const struct pio_jtag_config *dev_config = dev->config;
	pio_hw_t *pio_hw = dev_config->pio_regs;
	uint8_t pins = 0;

	if (!value) {
		return -EIO;
	}

	/* All default to 1 */
	pins = JTAG_PIN_TCK | JTAG_PIN_TMS
	     | JTAG_PIN_TDI | JTAG_PIN_TDO
	     | JTAG_PIN_NSRST | JTAG_PIN_NTRST;

	/* nSRST */
	if (gpio_is_ready_dt(&dev_config->srst_gpio)) {
		if (!gpio_pin_get_dt(&dev_config->srst_gpio)) {
			pins &= ~JTAG_PIN_NSRST;
		}
	}

	/* nTRST */
	if (gpio_is_ready_dt(&dev_config->trst_gpio)) {
		if (!gpio_pin_get_dt(&dev_config->trst_gpio)) {
			pins &= ~JTAG_PIN_NTRST;
		}
	}

	/* Cannot read others */
	*value = pins;

	return 0;
}

static int pio_jtag_set_frequency(const struct device *dev,
				  uint32_t freq_hz)
{
	const struct pio_jtag_config *dev_config = dev->config;
	struct pio_jtag_data *dev_data = dev->data;
	pio_hw_t *pio_hw = dev_config->pio_regs;
	pio_sm_hw_t *sm = &pio_hw->sm[dev_data->sm_index];
	const uint32_t min_freq = PIO_JTAG_MIN_FREQ(dev_config->clock_frequency);
	const uint32_t max_freq = PIO_JTAG_MAX_FREQ(dev_config->clock_frequency);

	/* Snap to valid range */
	if (freq_hz < min_freq) {
		LOG_WRN("frequency: %u too low, setting to: %u", freq_hz, min_freq);
		freq_hz = min_freq;
	} else if (freq_hz > max_freq) {
		LOG_WRN("frequency: %u too high, setting to: %u", freq_hz, max_freq);
		freq_hz = max_freq;
	}

	dev_data->clkdiv = PIO_SM_CLKDIV(max_freq, freq_hz);

	/* Update only if connected */
	if (dev_data->connected) {
		sm->clkdiv = dev_data->clkdiv;
	}

	return 0;
}

static int pio_jtag_connect(const struct device *dev,
			    bool connect_tdx)
{
	const struct pio_jtag_config *dev_config = dev->config;
	struct pio_jtag_data *dev_data = dev->data;
	pio_hw_t *pio_hw = dev_config->pio_regs;
	pio_sm_hw_t *sm = &pio_hw->sm[dev_data->sm_index];

	/* Do nothing if already connected */
	if (dev_data->connected) {
		return 0;
	}

	/* Check if we need TDI / TDO and if they exist */
	if (connect_tdx) {
		if ((dev_config->tdo_gpio < 0) || (dev_config->tdi_gpio < 0)) {
			return -EINVAL;
		}

		/* TDI out and high */
		sm->pinctrl = dev_config->tck_gpio << PIO_SM0_PINCTRL_SET_BASE_LSB;
		sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINS, 1, PIO_JTAG_SS(0, 0, 0));
		sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINDIRS, 1, PIO_JTAG_SS(0, 0, 0));

		/* TDO in */
		sm->pinctrl = dev_config->tck_gpio << PIO_SM0_PINCTRL_SET_BASE_LSB;
		sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINDIRS, 0, PIO_JTAG_SS(0, 0, 0));
	}

	/* TCK out and high */
	sm->pinctrl = dev_config->tck_gpio << PIO_SM0_PINCTRL_SET_BASE_LSB;
	sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINS, 1, PIO_JTAG_SS(0, 0, 0));
	sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINDIRS, 1, PIO_JTAG_SS(0, 0, 0));

	/* TMS out and high */
	sm->pinctrl = dev_config->tms_gpio << PIO_SM0_PINCTRL_SET_BASE_LSB;
	sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINS, 1, PIO_JTAG_SS(0, 0, 0));
	sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINDIRS, 1, PIO_JTAG_SS(0, 0, 0));

	/* Reset all and go to start */
	PIO_ATOMIC_SET(pio_hw->ctrl, dev_data->sm_mask << PIO_CTRL_SM_RESTART_LSB);
	PIO_ATOMIC_SET(pio_hw->ctrl, dev_data->sm_mask << PIO_CTRL_CLKDIV_RESTART_LSB);
	PIO_ATOMIC_SET(pio_hw->ctrl, dev_data->sm_mask << PIO_CTRL_SM_ENABLE_LSB);

	dev_data->connected = true;

	return 0;
}

static int pio_jtag_disconnect(const struct device *dev)
{
	const struct pio_jtag_config *dev_config = dev->config;
	struct pio_jtag_data *dev_data = dev->data;

	if (!dev_data->connected) {
		return -EALREADY;
	}

	/* TODO */
	dev_data->connected = false;

	return 0;
}

static void pio_jtag_irq(const struct device *dev)
{
	const struct pio_jtag_config *dev_config = dev->config;
	struct pio_jtag_data *dev_data = dev->data;
	pio_hw_t *pio_hw = dev_config->pio_regs;
	pio_sm_hw_t *sm = &pio_hw->sm[dev_data->sm_index];
	uint32_t pinctrl;
	uint8_t in_gpio, out_gpio;

	/* Swap */
	dev_data->tdx_active = !dev_data->tdx_active;
	if (dev_data->tdx_active) {
		in_gpio = dev_config->tdo_gpio;
		out_gpio = dev_config->tdi_gpio;
	} else {
		in_gpio = dev_config->tms_gpio;
		out_gpio = dev_config->tms_gpio;
	}

	/* Clear and update */
	pinctrl = sm->pinctrl;
	pinctrl &= ~(PIO_SM0_PINCTRL_OUT_BASE_BITS | PIO_SM0_PINCTRL_IN_BASE_BITS);
	pinctrl |= (out_gpio << PIO_SM0_PINCTRL_OUT_BASE_LSB) 
	         | (in_gpio << PIO_SM0_PINCTRL_IN_BASE_LSB);
	sm->pinctrl = pinctrl;

	/* Clear IRQ */
	pio_hw->irq = dev_data->sm_mask;
}

static void pio_jtag_dma_callback(const struct device *dev, void *arg,
				  uint32_t channel, int status)
{
	const struct device *jtag_dev = (const struct device *)arg;
	struct pio_jtag_data *dev_data = jtag_dev->data;

	/* All ops are done */
	k_sem_give(&dev_data->sem);

	return;
}

static int pio_jtag_exec(const struct device *dev,
			 const struct jtag_instruction *instr, size_t count)
{
	const struct pio_jtag_config *dev_config = dev->config;
	struct pio_jtag_data *dev_data = dev->data;
	const struct jtag_instruction *op;
	pio_hw_t *pio_hw = dev_config->pio_regs;
	uint32_t wr_size = 0;
	uint32_t rd_size = 0;
	uint32_t bytes, bits;
	size_t index;
	uint32_t *buf = dev_config->q_data;
	bool tdx_active = dev_data->tdx_active;

	/* Don't allow in disconnected state */
	if (!dev_data->connected) {
		return -EPERM;
	}

	/* Pack commands and values */
	for (index = 0; index < count; index++) {

		op = &instr[index];

		switch (op->op_code) {
		case JTAG_INSTRUCTION_TMS_OUT:

			if (op->shift_count == 0) {
				*buf++ = (PIO_JTAG_TMS_VAL(op->tms_value) << 16) | (0);
				*buf++ = (0x0000) | (PIO_JTAG_TMS_DIR(1));
				bytes = 0;
				rd_size -= 1;
				wr_size += 2;
				break;
			}

			*buf++ = (PIO_JTAG_TMS_DIR(1) << 16) | (op->shift_count - 1);

			if (tdx_active) {
				*buf++ = (PIO_JTAG_SWAP_IRQ << 16) | PIO_JTAG_NOP;
				tdx_active = false;
			} else {
				*buf++ = (PIO_JTAG_NOP << 16) | PIO_JTAG_NOP;
			}

			wr_size += 2;
			bytes = (op->shift_count + 7) / 8;
			if (op->shift_out) {
				memcpy(buf, op->shift_out, bytes);
			} else {
				memset(buf, op->tms_value ? 0xFF : 0x00, bytes);
			}

			break;

		case JTAG_INSTRUCTION_TMS_IN:
			*buf++ = (PIO_JTAG_TMS_DIR(0) << 16) | (op->shift_count - 1);

			if (tdx_active) {
				*buf++ = (PIO_JTAG_SWAP_IRQ << 16) | PIO_JTAG_NOP;
				tdx_active = false;
			} else {
				*buf++ = (PIO_JTAG_NOP << 16) | PIO_JTAG_NOP;
			}

			wr_size += 2;
			bytes = (op->shift_count + 7) / 8;

			break;

		case JTAG_INSTRUCTION_TDX_INOUT:

			if (!tdx_active) {
				*buf++ = (PIO_JTAG_SWAP_IRQ << 16) | (op->shift_count - 1);
				tdx_active = true;
			} else {
				*buf++ = (PIO_JTAG_NOP << 16) | (op->shift_count - 1);
			}

			*buf++ = (PIO_JTAG_TMS_DIR(1) << 16) | (PIO_JTAG_TMS_VAL(op->tms_value));
			
			wr_size += 2;
			bytes = (op->shift_count + 7) / 8;
			memcpy(buf, op->shift_out, bytes);

			break;

		default:
			break;
		}

		buf += (bytes + 3) / 4;
		wr_size += (bytes + 3) / 4;
		rd_size += (op->shift_count + 32) / 32;
	}

	/* Rx config */
	dma_reload(dev_data->rx_dma_data.dma_dev, dev_data->rx_dma_data.channel,
		   (uint32_t)(&(pio_hw->rxf[dev_data->sm_index])), (uint32_t)dev_config->q_data,
		   rd_size);

	dma_start(dev_data->rx_dma_data.dma_dev, dev_data->rx_dma_data.channel);

	/* Tx config */
	dma_reload(dev_data->tx_dma_data.dma_dev, dev_data->tx_dma_data.channel,
		   (uint32_t)dev_config->q_data, (uint32_t)(&(pio_hw->txf[dev_data->sm_index])),
		   wr_size);


	k_sem_take(&dev_data->sem, K_FOREVER);

	buf = dev_config->q_data;

	/* Unpack */
	for (index = 0; index < count; index++) {
		op = &instr[index];
		rd_size = (op->shift_count + 32) / 32;
		bits = 32 - (op->shift_count % 32);
		bytes = (op->shift_count + 7) / 8;

		switch (op->op_code) {

		case JTAG_INSTRUCTION_TMS_IN:
			buf[rd_size - 1] >>= bits;
			if (op->shift_in) {
				memcpy(op->shift_in, buf, bytes);
			}
			break;

		case JTAG_INSTRUCTION_TDX_INOUT:
			buf[rd_size - 1] >>= bits;
			if (op->shift_in) {
				memcpy(op->shift_in, buf, bytes);
			}
			break;

		default:
			break;
		}

		buf += rd_size;
	}

	return 0;
}

static int pio_jtag_pio_setup(const struct device *dev)
{
	const struct pio_jtag_config *dev_config = dev->config;
	struct pio_jtag_data *dev_data = dev->data;
	pio_hw_t *pio_hw = dev_config->pio_regs;
	pio_sm_hw_t *sm;
	uint32_t mask;
	int rc;

	/* Allocate SMs */
	rc = pio_rpi_pico_alloc_sm(dev_config->parent, 1, &dev_data->sm_index);
	if (rc < 0) {
		return rc;
	}

	dev_data->sm_mask = BIT(dev_data->sm_index);
	sm = &pio_hw->sm[dev_data->sm_index];

	/* Load program */
	rc = pio_rpi_pico_alloc_shared_instr(dev_config->parent, STRINGIFY(DT_DRV_COMPAT),
					     ARRAY_SIZE(pio_jtag_prg), &dev_data->instr);
	if ((rc < 0) && (rc != -EALREADY)) {
		return rc;
	}

	if (rc != -EALREADY) {
		/* Load initial program */
		pio_rpi_pico_util_load_prg(pio_hw->instr_mem, dev_data->instr,
					   pio_jtag_prg, ARRAY_SIZE(pio_jtag_prg));
	}

	/* TODO */
	sm->pinctrl = (2u << PIO_SM0_PINCTRL_SIDESET_COUNT_LSB)
		    | (1u << PIO_SM0_PINCTRL_SET_COUNT_LSB)
		    | (1u << PIO_SM0_PINCTRL_OUT_COUNT_LSB)
		    | (dev_config->tck_gpio << PIO_SM0_PINCTRL_SIDESET_BASE_LSB)
		    | (dev_config->tdi_gpio << PIO_SM0_PINCTRL_SET_BASE_LSB)
		    | (dev_config->tms_gpio << PIO_SM0_PINCTRL_OUT_BASE_LSB)
		    | (dev_config->tms_gpio << PIO_SM0_PINCTRL_IN_BASE_LSB);

	sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINDIRS, 1, PIO_JTAG_SS(0, 0, 0));

	/* Configure */
	sm->pinctrl = (2u << PIO_SM0_PINCTRL_SIDESET_COUNT_LSB)
		    | (1u << PIO_SM0_PINCTRL_SET_COUNT_LSB)
		    | (1u << PIO_SM0_PINCTRL_OUT_COUNT_LSB)
		    | (dev_config->tck_gpio << PIO_SM0_PINCTRL_SIDESET_BASE_LSB)
		    | (dev_config->tms_gpio << PIO_SM0_PINCTRL_SET_BASE_LSB)
		    | (dev_config->tms_gpio << PIO_SM0_PINCTRL_OUT_BASE_LSB)
		    | (dev_config->tms_gpio << PIO_SM0_PINCTRL_IN_BASE_LSB);

	sm->execctrl = PIO_SM0_EXECCTRL_OUT_STICKY_BITS 
		     | PIO_SM0_EXECCTRL_SIDE_PINDIR_BITS
		     | PIO_SM0_EXECCTRL_SIDE_EN_BITS
		     | (dev_data->instr << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB)
		     | ((dev_data->instr + PIO_JTAG_WRAP_SIZE) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB);

	sm->shiftctrl = PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS
		      | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS
		      | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS
		      | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS;

	sm->clkdiv = PIO_SM_CLKDIV(dev_config->clock_frequency, (1000000 * 22));

	sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINS, 1, PIO_JTAG_SS(1, 1, 0));
	sm->execctrl ^= PIO_SM0_EXECCTRL_SIDE_PINDIR_BITS;
	sm->instr = PIO_ASM_SET(PIO_ASM_SET_DST_PINDIRS, 1, PIO_JTAG_SS(1, 1, 0));

	mask = dev_data->sm_mask;
	PIO_ATOMIC_SET(pio_hw->ctrl, mask << PIO_CTRL_SM_RESTART_LSB);
	PIO_ATOMIC_SET(pio_hw->ctrl, mask << PIO_CTRL_CLKDIV_RESTART_LSB);
	PIO_ATOMIC_SET(pio_hw->ctrl, mask << PIO_CTRL_SM_ENABLE_LSB);

	return 0;
}

static int pio_jtag_init(const struct device *dev)
{
	const struct pio_jtag_config *dev_config = dev->config;
	struct pio_jtag_data *dev_data = dev->data;
	struct pio_irq_hw *irq_hw = PIO_IRQ_HW_INDEX(dev_config->pio_regs, dev_config->irq_idx);
	int rc;

	if (!device_is_ready(dev_config->parent)) {
		return -ENODEV;
	}

	rc = pio_jtag_pio_setup(dev);
	if (rc < 0) {
		return rc;
	}

	/* Rx DMA */
	rc = dma_config(dev_data->rx_dma_data.dma_dev, dev_data->rx_dma_data.channel,
			&dev_data->rx_dma_data.dma_cfg);
	if (rc < 0) {
		return rc;
	}

	/* Tx DMA */
	rc = dma_config(dev_data->tx_dma_data.dma_dev, dev_data->tx_dma_data.channel,
			&dev_data->tx_dma_data.dma_cfg);
	if (rc < 0) {
		return rc;
	}

	if (gpio_is_ready_dt(&dev_config->srst_gpio)) {
		gpio_pin_configure_dt(&dev_config->srst_gpio, GPIO_OUTPUT_INACTIVE);
	}

	if (gpio_is_ready_dt(&dev_config->trst_gpio)) {
		gpio_pin_configure_dt(&dev_config->trst_gpio, GPIO_OUTPUT_INACTIVE);
	}

	rc = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (rc < 0) {
		return rc;
	}

	// pio_rpi_pico_irq_register

	/* Configure the IRQ */
	pio_rpi_pico_irq_register(dev_config->parent, &dev_data->irq_cfg);
	pio_rpi_pico_irq_enable(dev_config->parent, &dev_data->irq_cfg);

	/* Enable IRQ */
	// mask = (uint32_t)dev_data->sm_mask << PIO_IRQ0_INTE_SM0_LSB;
	// PIO_ATOMIC_SET(irq_hw->inte, mask);

	k_sem_init(&dev_data->lock, 1, 1);
	k_sem_init(&dev_data->sem, 0, 1);

	return 0;
}

static const struct jtag_driver_api pio_jtag_driver_api = {
	.get_specs = pio_jtag_get_specs,
	.connect = pio_jtag_connect,
	.disconnect = pio_jtag_disconnect,
	.set_pins = pio_jtag_set_pins,
	.get_pins = pio_jtag_get_pins,
	.set_frequency = pio_jtag_set_frequency,
	.exec = pio_jtag_exec,
};

#define PIO_JTAG_DEFINE_DMA_QUEUE(name, size)							\
	BUILD_ASSERT(size < 65636, "invalid size");						\
	static uint32_t __noinit name[size];

#define PIO_JTAG_DMA_CONFIG(inst)								\
	.tx_dma_data = {									\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, tx)),			\
		.channel =  DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel),			\
		.dma_blk_cfg = {								\
			.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,				\
			.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,				\
		},										\
		.dma_cfg = {									\
			.channel_direction = MEMORY_TO_PERIPHERAL,				\
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, tx, slot),			\
			.source_data_size = 4,							\
			.dest_data_size = 4,							\
			.block_count = 1,							\
			.head_block = &pio_jtag_data##inst.tx_dma_data.dma_blk_cfg,		\
			.channel_priority = 1,							\
		},										\
	},											\
	.rx_dma_data = {									\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, rx)),			\
		.channel =  DT_INST_DMAS_CELL_BY_NAME(inst, rx, channel),			\
		.dma_blk_cfg = {								\
			.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,				\
			.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,				\
		},										\
		.dma_cfg = {									\
			.channel_direction = PERIPHERAL_TO_MEMORY,				\
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, rx, slot),			\
			.source_data_size = 4,							\
			.dest_data_size = 4,							\
			.block_count = 1,							\
			.head_block = &pio_jtag_data##inst.rx_dma_data.dma_blk_cfg,		\
			.channel_priority = 0,							\
			.user_data = (void *)DEVICE_DT_GET(DT_DRV_INST(inst)),			\
			.dma_callback = pio_jtag_dma_callback,					\
		},										\
	}

// #define PIO_JTAG_IRQ_CONFIG(inst)								\
// static void pio_jtag_irq_config##inst(const struct device *dev)					\
// {												\
// 	ARG_UNUSED(dev);									\
// 	static struct pio_rpi_pico_irq_cfg pio_jtag_irq_cfg##inst = {				\
// 		.irq_func = pio_jtag_irq,							\
// 		.irq_param = DEVICE_DT_GET(DT_DRV_INST(inst)),					\
// 		.irq_idx = DT_INST_PROP_BY_IDX(inst, pio_interrupts, 0),			\
// 	};											\
// 	pio_rpi_pico_irq_register(DEVICE_DT_GET(DT_INST_PARENT(inst)), &pio_jtag_irq_cfg##inst);\
// 	pio_rpi_pico_irq_enable(DEVICE_DT_GET(DT_INST_PARENT(inst)), &pio_jtag_irq_cfg##inst);	\
// }

#define PIO_JTAG_IRQ_CONFIG(inst)								\
	.irq_cfg = {										\
		.irq_func = pio_jtag_irq,							\
		.irq_param = DEVICE_DT_GET(DT_DRV_INST(inst)),					\
		.irq_idx = DT_INST_PROP_BY_IDX(inst, pio_interrupts, 0),			\
	}

#define PIO_JTAG_INIT(inst) \
	BUILD_ASSERT(DT_INST_ON_BUS(inst, pio_rpi_pico),					\
		"node " DT_NODE_PATH(DT_DRV_INST(inst)) " is not assigned to a PIO instance");	\
												\
	PINCTRL_DT_INST_DEFINE(inst);								\
												\
	PIO_JTAG_DEFINE_DMA_QUEUE(pio_jtag_queue##inst, DT_INST_PROP(inst, queue_size));	\
												\
	static struct pio_jtag_data pio_jtag_data##inst = {					\
		PIO_JTAG_DMA_CONFIG(inst),							\
		PIO_JTAG_IRQ_CONFIG(inst),							\
		.clkdiv = PIO_SM_CLKDIV(PIO_JTAG_MAX_FREQ(					\
					DT_INST_PIO_RPI_PICO_CLOCK_FREQ_HZ(inst)),		\
					DT_INST_PROP_OR(inst, default_frequency, 1000000)),	\
	};											\
												\
	static const struct pio_jtag_config pio_jtag_config##inst = {				\
		.parent = DEVICE_DT_GET(DT_INST_PARENT(inst)),					\
		.pio_regs = (pio_hw_t *)DT_INST_PIO_RPI_PICO_REG_ADDR(inst),			\
		.srst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, srst_gpios, {0}),			\
		.trst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, srst_gpios, {0}),			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),					\
		.q_data = pio_jtag_queue##inst,							\
		.q_size = DT_INST_PROP(inst, queue_size),					\
		.clock_frequency = DT_INST_PIO_RPI_PICO_CLOCK_FREQ_HZ(inst),			\
		.tck_gpio = JTAG_INST_PIN_BY_NAME_OR(inst, default, 0, tck_gpio, 0, -1),	\
		.tms_gpio = JTAG_INST_PIN_BY_NAME_OR(inst, default, 0, tms_gpio, 0, -1),	\
		.tdi_gpio = JTAG_INST_PIN_BY_NAME_OR(inst, default, 0, tdi_gpio, 0, -1),	\
		.tdo_gpio = JTAG_INST_PIN_BY_NAME_OR(inst, default, 0, tdo_gpio, 0, -1),	\
		.irq_idx = DT_INST_PROP_BY_IDX(inst, pio_interrupts, 0),			\
	};											\
												\
	DEVICE_DT_INST_DEFINE(inst, &pio_jtag_init,						\
		NULL,										\
		&pio_jtag_data##inst,								\
		&pio_jtag_config##inst,								\
		POST_KERNEL,									\
		CONFIG_JTAG_INIT_PRIORITY,							\
		&pio_jtag_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIO_JTAG_INIT)
