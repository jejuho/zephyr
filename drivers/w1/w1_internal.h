/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Internal 1-Wire Driver definitions
 */

#ifndef ZEPHYR_DRIVERS_W1_INTERNAL_H_
#define ZEPHYR_DRIVERS_W1_INTERNAL_H_

#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Count the number of peripheral devices expected on the bus.
 * This can be used to decide if the bus has a multi-drop topology or
 * only a single device is present.
 * There is a comma after each ordinal (including the last)
 * Hence FOR_EACH adds "+1" once too often which has to be subtracted in the end.
 */
#define F1(x) 1
#define W1_PERIPHERALS_COUNT(node_id)  \
		(FOR_EACH(F1, (+), DT_SUPPORTS_DEP_ORDS(node_id)) - 1)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_W1_INTERNAL_H_ */
