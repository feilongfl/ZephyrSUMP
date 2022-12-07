# SPDX-License-Identifier: Apache-2.0
zephyr_library()
zephyr_library_include_directories()

if(CONFIG_SUMP_PORT_FAST_DRIVER)
  zephyr_library_sources_ifdef(CONFIG_SOC_FAMILY_AT32_ARM sump_port_at32.c)
else()
  zephyr_library_sources(sump_port_common.c)
endif()
