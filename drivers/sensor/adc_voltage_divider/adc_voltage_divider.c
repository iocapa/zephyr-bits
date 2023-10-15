#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/voltage_divider.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_voltage_divider, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT zephyr_adc_voltage_divider

struct adc_voltage_divider_data {
	struct adc_sequence sequence;
	int32_t sample;
};

static int adc_voltage_divider_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct voltage_divider_dt_spec *config = dev->config;
	struct adc_voltage_divider_data *data = dev->data;

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOLTAGE) {
		return adc_read(config->port.dev, &data->sequence);
	}

	return -ENOTSUP;
}

static int adc_voltage_divider_channel_get(const struct device *dev, enum sensor_channel chan,
					   struct sensor_value *val)
{
	const struct voltage_divider_dt_spec *config = dev->config;
	struct adc_voltage_divider_data *data = dev->data;
	int32_t mv = 0;
	int rc;

	if (chan != SENSOR_CHAN_VOLTAGE) {
		return -ENOTSUP;
	}

	mv = data->sample;
	rc = adc_raw_to_millivolts_dt(&config->port, &mv);
	if (rc) {
		LOG_DBG("adc_raw_to_millivolts() failed %d", rc);
		return rc;
	}

	/* Scale to microvolts */
	mv *= 1000;

	/* Apply resistor divider ratio */
	if (config->full_ohms > 0) {
		mv = (mv * 1000) / ((config->output_ohms * 1000) / config->full_ohms);
	}

	val->val1 = mv / 1000000;
	val->val2 = mv % 1000000;

	return 0;
}

static const struct sensor_driver_api adc_voltage_divider_driver_api = {
	.sample_fetch = adc_voltage_divider_sample_fetch,
	.channel_get = adc_voltage_divider_channel_get,
};

static int adc_voltage_divider_init(const struct device *dev)
{
	const struct voltage_divider_dt_spec *config = dev->config;
	struct adc_voltage_divider_data *data = dev->data;
	int rc;

	if (!device_is_ready(config->port.dev)) {
		LOG_ERR("ADC is not ready");
		return -ENODEV;
	}

	rc = adc_channel_setup_dt(&config->port);
	if (rc != 0) {
		LOG_ERR("setup: %d", rc);
		return rc;
	}

	rc = adc_sequence_init_dt(&config->port, &data->sequence);
	if (rc != 0) {
		LOG_ERR("sequence: %d", rc);
		return rc;
	}

	data->sequence.buffer = &data->sample;
	data->sequence.buffer_size = sizeof(data->sample);

	return 0;
}

#define ADC_VOLTAGE_DIVIDER_DEFINE(inst)							\
	static struct adc_voltage_divider_data adc_voltage_divider_dev_data_##inst;		\
												\
	static const struct voltage_divider_dt_spec adc_voltage_divider_dev_config_##inst =	\
					VOLTAGE_DIVIDER_DT_SPEC_GET(DT_DRV_INST(inst));		\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, adc_voltage_divider_init, NULL,			\
				     &adc_voltage_divider_dev_data_##inst,			\
				     &adc_voltage_divider_dev_config_##inst,			\
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,			\
				     &adc_voltage_divider_driver_api);				\

DT_INST_FOREACH_STATUS_OKAY(ADC_VOLTAGE_DIVIDER_DEFINE)
