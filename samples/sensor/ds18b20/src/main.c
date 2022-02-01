/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>

#if 1

struct ds18b20_scratchpad {
	int16_t temp;
	uint8_t alarm_temp_high;
	uint8_t alarm_temp_low;
	uint8_t config;
	uint8_t res[3];
	uint8_t crc;
};

struct ds18b20_config {
	const struct device *bus;
	uint8_t family;
	uint8_t resolution;
};

struct ds18b20_data {
	uint8_t rom[8];
	struct ds18b20_scratchpad scratchpad;
	uint8_t power_supply;
	bool lazy_loaded;
};
#endif

#define DS_1 DT_INST(0, maxim_ds18b20)
#define DS_2 DT_INST(1, maxim_ds18b20)
#define DS_3 DT_INST(2, maxim_ds18b20)
/*
 * Get a device structure from a devicetree node with compatible
 * "maxim,ds18b20". (If there are multiple, just pick one.)
 */
static const struct device *get_ds18b20_device(int id)
{
	struct device *dev = 0;
	if(id == 0) {
		dev = DEVICE_DT_GET(DS_1);
	}
	else if(id == 1) {
		dev = DEVICE_DT_GET(DS_2);
	} else
	if(id == 2) {
		dev = DEVICE_DT_GET(DS_3);
	}

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	struct ds18b20_data *data = dev->data;
	if(id == 0) {
		data->rom[0] = 0x28;
		data->rom[1] = 0xA0;
		data->rom[2] = 0x60;
		data->rom[3] = 0x86;
		data->rom[4] = 0x04;
		data->rom[5] = 0x00;
		data->rom[6] = 0x00;
		data->rom[7] = 0x1d;
	} else
	if(id == 1) {
		data->rom[0] = 0x28;
		data->rom[1] = 0x46;
		data->rom[2] = 0x91;
		data->rom[3] = 0x86;
		data->rom[4] = 0x04;
		data->rom[5] = 0x00;
		data->rom[6] = 0x00;
		data->rom[7] = 0xed;
	} else
	if(id == 2) {
		data->rom[0] = 0x28;
		data->rom[1] = 0x0f;
		data->rom[2] = 0x28;
		data->rom[3] = 0x87;
		data->rom[4] = 0x04;
		data->rom[5] = 0x00;
		data->rom[6] = 0x00;
		data->rom[7] = 0x3d;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

void main(void)
{
	const struct device *dev = get_ds18b20_device(0);
	const struct device *dev2 = get_ds18b20_device(1);
	const struct device *dev3 = get_ds18b20_device(2);

	if (dev == NULL) {
		return;
	}

	while (1) {
		struct sensor_value temp;

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		printk("Dev Temp: %d.%06d\n", temp.val1, temp.val2);
		sensor_sample_fetch(dev2);
		sensor_channel_get(dev2, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		printk("Dev2 Temp: %d.%06d\n", temp.val1, temp.val2);
		sensor_sample_fetch(dev3);
		sensor_channel_get(dev3, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		printk("Dev3 Temp: %d.%06d\n", temp.val1, temp.val2);

		k_sleep(K_MSEC(5000));
	}
}
