/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <syscall_handler.h>
#include <drivers/w1.h>


static inline bool z_vrfy_w1_reset_bus(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, reset_bus));

	return z_impl_w1_reset_bus((const struct device *)dev);
}
#include <syscalls/w1_reset_bus_mrsh.c>

static inline bool z_vrfy_w1_read_bit(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, read_bit));

	return z_impl_w1_read_bit((const struct device *)dev);
}
#include <syscalls/w1_read_bit_mrsh.c>

static inline void z_vrfy_w1_write_bit(const struct device *dev, bool bit)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, write_bit));

	z_impl_w1_write_bit((const struct device *)dev, (bool)bit);
}
#include <syscalls/w1_write_bit_mrsh.c>

static inline uint8_t z_vrfy_w1_read_byte(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, read_byte));

	return z_impl_w1_read_byte((const struct device *)dev);
}
#include <syscalls/w1_read_byte_mrsh.c>

static inline void z_vrfy_w1_write_byte(const struct device *dev, uint8_t byte)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, write_byte));

	z_impl_w1_write_byte((const struct device *)dev, (uint8_t)byte);
}
#include <syscalls/w1_write_byte_mrsh.c>

static inline void z_vrfy_w1_read_block(const struct device *dev,
					uint8_t *buffer, size_t length)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, read_block));

	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(buffer, length));

	z_impl_w1_read_block((const struct device *)dev, (uint8_t *)buffer,
			     (size_t)length);
}
#include <syscalls/w1_read_block_mrsh.c>

static inline void z_vrfy_w1_write_block(const struct device *dev,
					 const uint8_t *buffer, size_t length)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, write_block));

	Z_OOPS(Z_SYSCALL_MEMORY_READ(buffer, length));

	z_impl_w1_write_block((const struct device *)dev, (uint8_t *)buffer,
			      (size_t)length);
}
#include <syscalls/w1_write_block_mrsh.c>

static inline void z_vrfy_w1_lock_bus(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, lock_bus));

	z_impl_w1_lock_bus((const struct device *)dev);
}
#include <syscalls/w1_lock_bus_mrsh.c>

static inline void z_vrfy_w1_unlock_bus(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_W1(dev, unlock_bus));

	z_impl_w1_unlock_bus((const struct device *)dev);
}
#include <syscalls/w1_unlock_bus_mrsh.c>

static inline size_t z_vrfy_w1_get_peripheral_count(const struct device *dev)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_W1));

	return z_impl_w1_get_peripheral_count((const struct device *)dev);
}
#include <syscalls/w1_get_peripheral_count_mrsh.c>

static inline size_t z_vrfy_w1_search_bus(const struct device *dev,
			       uint8_t command,
			       uint8_t family,
			       w1_search_callback_t callback_isr,
			       void *callback_arg)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_W1));

	Z_OOPS(Z_SYSCALL_VERIFY_MSG(callback_isr == 0,
				    "callbacks may not be set from user mode"));
	/* callback_arg is not dereferenced, no need to check parameter */

	return z_impl_w1_search_bus((const struct device *)dev,
				    (uint8_t)command, (uint8_t)family,
				    (w1_search_callback_t)callback_isr,
				    (void *)callback_arg);
}
#include <syscalls/w1_search_bus_mrsh.c>
