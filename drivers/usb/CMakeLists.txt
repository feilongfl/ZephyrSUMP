# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_UDC_DRIVER udc)
add_subdirectory_ifdef(CONFIG_USB_DEVICE_DRIVER device)
