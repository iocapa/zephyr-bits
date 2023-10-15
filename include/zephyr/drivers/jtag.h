#ifndef ZEPHYR_INCLUDE_JTAG_H_
#define ZEPHYR_INCLUDE_JTAG_H_

#include <zephyr/device.h>

/* Supported capability flags */

/* Supports TMS_OUT instructions */
#define JTAG_SPEC_TMS_OUT		BIT(0)
/* Supports TMS_IN instructions */
#define JTAG_SPEC_TMS_IN		BIT(1)
/* Supports TDX_INOUT instructions */
#define JTAG_SPEC_TDX_INOUT		BIT(2)

/* Pins */

#define JTAG_PIN_TCK			BIT(0)
#define JTAG_PIN_TMS			BIT(1)
#define JTAG_PIN_TDI			BIT(2)
#define JTAG_PIN_TDO			BIT(3)
#define JTAG_PIN_NTRST			BIT(5)
#define JTAG_PIN_NSRST			BIT(7)

/* Instruction codes */
enum jtag_instruction_code {
	JTAG_INSTRUCTION_TMS_OUT,
	JTAG_INSTRUCTION_TMS_IN,
	JTAG_INSTRUCTION_TDX_INOUT,
};

/**
 * @brief Callback function for asynchronous operation
 */
typedef void (*jtag_async_callback_t)(const struct device *dev, int result, void *data);

struct jtag_instruction {
	/**
	 * Bits to shift out. Applies to TMS_IN and TDX_INOUT.
	 * LSB transmitted first, padded to byte boundary.
	 */
	const uint8_t *shift_out;

	/**
	 * Bits to shift in. Applies to TMS_IN and TDX_INOUT.
	 * Can be NULL if not required.
	 * LSB received first, padded to byte boundary.
	 */
	uint8_t *shift_in;

	/**
	 * Number of bits (clock cycles) to shift.
	 * Required for ALL operations.
	 * Setting this to 0 suppresses all clock cycles
	 * therefore executing only the optional state change.
	 */
	uint16_t shift_count;

	/**
	 * Instruction code.
	 * Required for ALL instructions
	 */
	enum jtag_instruction_code op_code;

	/**
	 * TMS value.
	 * Applies to TMS_OUT and TDX_INOUT.
	 */
	bool tms_value;
};

/* Driver API */
struct jtag_driver_api {
	int (*get_specs)(const struct device *dev,
			 uint32_t *specs);

	int (*connect)(const struct device *dev,
		       bool connect_tdx);

	int (*disconnect)(const struct device *dev);

	int (*set_pins)(const struct device *dev,
			uint8_t mask, uint8_t value);

	int (*get_pins)(const struct device *dev,
			uint8_t *value);

	int (*set_frequency)(const struct device *dev,
			     uint32_t freq_hz);

#ifdef CONFIG_JTAG_ASYNC_API
	int (*exec_async)(const struct device *dev,
		       const struct jtag_instruction *instr, size_t count,
		       jtag_async_callback_t cb, void *userdata);
#endif

	int (*exec)(const struct device *dev,
			  const struct jtag_instruction *instr, size_t count);
};

static inline int jtag_get_specs(const struct device *dev,
				 uint32_t *specs)
{
	const struct jtag_driver_api *api =
		(const struct jtag_driver_api *)dev->api;

	return api->get_specs(dev, specs);
}

static inline int jtag_connect(const struct device *dev,
			       bool connect_tdx)
{
	const struct jtag_driver_api *api =
		(const struct jtag_driver_api *)dev->api;

	return api->connect(dev, connect_tdx);
}

static inline int jtag_disconnect(const struct device *dev)
{
	const struct jtag_driver_api *api =
		(const struct jtag_driver_api *)dev->api;

	return api->disconnect(dev);
}

static inline int jtag_set_pins(const struct device *dev,
				uint8_t mask, uint8_t value)
{
	const struct jtag_driver_api *api =
		(const struct jtag_driver_api *)dev->api;

	return api->set_pins(dev, mask, value);
}

static inline int jtag_get_pins(const struct device *dev,
				uint8_t *value)
{
	const struct jtag_driver_api *api =
		(const struct jtag_driver_api *)dev->api;

	return api->get_pins(dev, value);
}

static inline int jtag_set_frequency(const struct device *dev,
				     uint32_t freq_hz)
{
	const struct jtag_driver_api *api =
		(const struct jtag_driver_api *)dev->api;

	return api->set_frequency(dev, freq_hz);
}

static inline int jtag_exec_async(const struct device *dev,
				  const struct jtag_instruction *instr, size_t count,
				  jtag_async_callback_t cb, void *userdata)
{
#ifdef CONFIG_JTAG_ASYNC_API_API
	const struct jtag_driver_api *api =
		(const struct jtag_driver_api *)dev->api;

	return api->exec_async(dev, instr, count, cb, userdata);
#endif
	ARG_UNUSED(dev);
	ARG_UNUSED(instr);
	ARG_UNUSED(count);
	ARG_UNUSED(cb);
	ARG_UNUSED(userdata);
	return -ENOTSUP;
}

static inline int jtag_exec(const struct device *dev,
			    const struct jtag_instruction *instr, size_t count)
{
	const struct jtag_driver_api *api =
		(const struct jtag_driver_api *)dev->api;

	return api->exec(dev, instr, count);
}

#endif /* ZEPHYR_INCLUDE_JTAG_H_ */
