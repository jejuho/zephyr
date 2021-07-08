/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/w1.h>
#include <zephyr.h>
#include <ztest.h>

#define W1_DUMMY_DEVICE_1 DT_INST(0, test_w1_dummy_device)
#define W1_DUMMY_DEVICE_2 DT_INST(1, test_w1_dummy_device)

#define ZEPHYR_W1_SERIAL DT_INST(0, zephyr_w1_serial)

#if DT_NODE_HAS_STATUS(ZEPHYR_W1_SERIAL, okay)
#define W1_DEVICE      ZEPHYR_W1_SERIAL
#else
#error Your devicetree has no enabled nodes with a compatible w1-driver
#endif

const struct device *get_w1_device(void)
{
	const struct device *dev = DEVICE_DT_GET(W1_DEVICE);

	zassert_true(device_is_ready(dev), "W1 controller not found");

	return dev;
}

const struct device *get_w1_dummy_device_1(void)
{
	const struct device *dev = DEVICE_DT_GET(W1_DUMMY_DEVICE_1);

	zassert_true(device_is_ready(dev), "W1 dummy device 1 not found");

	return dev;
}

const struct device *get_w1_dummy_device_2(void)
{
	const struct device *dev = DEVICE_DT_GET(W1_DUMMY_DEVICE_2);

	zassert_true(device_is_ready(dev), "W1 dummy device 2 not found");

	return dev;
}

/* test vectors: */
const uint8_t rom_01_bytes[] = {0x2d, 0x18, 0x08, 0xf5, 0x2d, 0x00, 0x00, 0x67 };
const uint8_t rom_02_bytes[] = {0x2d, 0x2d, 0xfc, 0xf4, 0x2d, 0x00, 0x00, 0x57 };
const uint8_t rom_03_bytes[] = {0x2d, 0xa8, 0xdc, 0xf2, 0x2d, 0x00, 0x00, 0xa7 };

const uint64_t rom_01_64 = 0x2d1808f52d000067;
const uint64_t rom_02_64 = 0x2d2dfcf42d000057;
const uint64_t rom_03_64 = 0x2da8dcf22d0000a7;

const struct w1_rom rom_01 = {
	.family = 0x2d,
	.serial = {0x18, 0x08, 0xf5, 0x2d, 0x00, 0x00},
	.crc = 0x67,
};
const struct w1_rom rom_02 = {
	.family = 0x2d,
	.serial = {0x2d, 0xfc, 0xf4, 0x2d, 0x00, 0x00},
	.crc = 0x57,
};
const struct w1_rom rom_03 = {
	.family = 0x2d,
	.serial = {0xa8, 0xdc, 0xf2, 0x2d, 0x00, 0x00},
	.crc = 0xa7,
};

void test_w1_basic(void)
{
	const struct device *w1_dev = get_w1_device();
	size_t peripheral_count;

	w1_lock_bus(w1_dev);
	w1_lock_bus(w1_dev);
	w1_unlock_bus(w1_dev);
	w1_unlock_bus(w1_dev);

	peripheral_count = w1_get_peripheral_count(w1_dev);
	zassert_true(peripheral_count == 2,
		     "w1_get_peripheral_count() does not dt definitions: %u/2",
		     peripheral_count);
}

void test_w1_crc(void)
{
	uint8_t crc8_result;

	crc8_result = w1_crc8(rom_01_bytes, 8);
	zassert_equal(crc8_result, 0, "crc1: crc over complete rom not 0");

	crc8_result = w1_crc8(rom_02_bytes, 8);
	zassert_equal(crc8_result, 0, "crc2: crc over complete rom not 0");

	crc8_result = w1_crc8(rom_03_bytes, 7);
	zassert_equal(crc8_result, rom_03_bytes[7], "crc3 does not match");
}

void test_w1_rom(void)
{
	struct w1_rom rom_x;
	uint64_t rom_x_64 = -1;

	rom_x_64 = w1_rom_to_uint64(&rom_01);
	zassert_equal(rom_01_64, rom_x_64,
		      "rom_01_struct converted to uint64 does not match");
	rom_x_64 = w1_rom_to_uint64(&rom_02);
	zassert_equal(rom_02_64, rom_x_64,
		      "rom_02_struct converted to uint64 does not match");

	w1_uint64_to_rom(rom_01_64, &rom_x);
	zassert_mem_equal(&rom_x, &rom_01, sizeof(rom_01),
		      "rom_01_64 converted to rom struct does not match");
	w1_uint64_to_rom(rom_03_64, &rom_x);
	zassert_mem_equal(&rom_x, &rom_03, sizeof(rom_03),
		      "rom_03_64 converted to rom struct does not match");
}

void test_w1_reset_empty(void)
{
	int ret;
	const struct device *w1_dev = get_w1_device();

	ret = w1_reset_bus(w1_dev);
	zassert_false(ret, "In case no devices are connected should return 0");
}

int found_w1_devices;

void w1_test_search_callback(struct w1_rom found_rom, void *callback_arg)
{
	ARG_UNUSED(found_rom);
	ARG_UNUSED(callback_arg);
	found_w1_devices++;
}

void test_w1_search_empty(void)
{
	int ret;
	const struct device *w1_dev = get_w1_device();

	ret = w1_search_rom(w1_dev, 0, 0);
	zassert_false(ret, "In case no devices are connected should return 0");

	ret = w1_search_alarm(w1_dev, 0, 0);
	zassert_false(ret, "In case no devices are connected should return 0");
}

void test_w1_fire_and_forget(void)
{
	const struct device *w1_dev = get_w1_device();
	const uint8_t block_send[8] = {0x0F, 0x0E, 0x0D, 0x0C, 0xC0, 0xD0, 0xE0, 0xF0};

	w1_write_bit(w1_dev, false);

	w1_write_byte(w1_dev, 0x3b);

	w1_write_block(w1_dev, block_send, sizeof(block_send));
}

void test_w1_receive_nothing(void)
{
	const struct device *w1_dev = get_w1_device();
	bool bool_rcv;
	uint8_t byte_rcv;
	uint8_t block_rcv[8] = {0x0F, 0x0E, 0x0D, 0x0C, 0xC0, 0xD0, 0xE0, 0xF0};
	const uint8_t block_ref[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	/* on idle bus without sender all received bits should be logical ones */

	bool_rcv = w1_read_bit(w1_dev);
	zassert_true(bool_rcv, "bit: empty receive should be logical ones");

	byte_rcv = w1_read_byte(w1_dev);
	zassert_equal(byte_rcv, 0xFF, "byte: empty receive should be logical 0xFF");

	w1_read_block(w1_dev, block_rcv, sizeof(block_rcv));
	zassert_mem_equal(block_rcv, block_ref, sizeof(block_rcv),
			  "block: empty receive should be local all 0xFF");
}

void test_w1_peripheral(void)
{
	int ret;
	struct w1_rom rom_1 = {0};
	const struct device *w1_dev = get_w1_device();
	const uint8_t block_send[8] = {0x0F, 0x0E, 0x0D, 0x0C, 0xC0, 0xD0, 0xE0, 0xF0};
	uint8_t block_rcv[8] = {0x00};

	ret = w1_read_rom(w1_dev, &rom_1);
	zassert_equal(ret, -EIO, "read_rom should fail w/o connected dev");

	ret = w1_match_rom(w1_dev, &rom_1);
	zassert_equal(ret, -EIO, "match_rom should fail w/o connected dev");

	ret = w1_resume_command(w1_dev);
	zassert_equal(ret, -EIO, "resume command should fail w/o connected dev");

	ret = w1_skip_rom(w1_dev);
	zassert_equal(ret, -EIO, "skip_rom should fail w/o connected dev");

	ret = w1_reset_select(w1_dev, &rom_1);
	zassert_equal(ret, -EIO, "reset_select should fail w/o connected dev");

	ret = w1_write_read(w1_dev, &rom_01, block_send, 8, block_rcv, 0);
	zassert_equal(ret, -EIO, "w1_write_read should fail w/o connected dev");
}
