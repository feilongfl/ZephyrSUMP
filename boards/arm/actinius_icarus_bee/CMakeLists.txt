#
# Copyright (c) 2021-2022 Actinius
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if (CONFIG_BOARD_ACTINIUS_ICARUS_BEE_NS)
	zephyr_library_sources(board.c)
endif()
