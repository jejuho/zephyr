# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_sources_ifdef(CONFIG_USERSPACE                 w1_handlers.c)

# drivers implementing link and access functions (read, write, reset, lock)
zephyr_library_sources_ifdef(CONFIG_W1_ZEPHYR_SERIAL  w1_zephyr_serial.c)

# network functions:
zephyr_library_sources(w1_net_intern.c)
# network functions that should be callable from user as well as supervisor mode:
remove_definitions(-D__ZEPHYR_SUPERVISOR__)
zephyr_library_sources(w1_net_public.c)
