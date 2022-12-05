# Copyright (c) 2022, YuLong Yao <feilongphone@gmail.com>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(pyocd "--target=_at32f437zmt7" "--pack=${ZEPHYR_HAL_ARTERY_MODULE_DIR}/${CONFIG_SOC_SERIES}/support/ArteryTek.AT32F435_437_DFP.2.1.2.pack")

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
