.. _zephyr_1.8:

Zephyr Kernel 1.8.0
####################

We are pleased to announce the release of Zephyr kernel version 1.8.0.

Major enhancements with this release include:

* Tickless kernel
* IP Stack improvements
* Bluetooth 5.0 features
* Ecosystem: Tracing, debugging support through third-party tools (openocd,
  Segger Systemview)
* Improved build support on Mac and Windows development environments
* Xtensa GCC support
* Initial implementation of MMU/MPU support
* Expanded device support

The following sections provide detailed lists of changes by component.

Kernel
******

* Use k_cycle_get_32 instead of sys_cycle_get_32 for Kernel
* Added k_panic() and k_oops() APIs for Kernel
* Added k_thread_create() API for Kernel
* Added k_queue API for Kernel
* Add tickless kernel support

Architectures
*************

* arm: Update core to use struct k_thread
* arm: Added ARM MPU support
* dts: Added ARM CMSDK support
* arm: Added Initial support for NXP MPU
* arm: Added Device Tree Support for nRF52832 SoC based boards
* arm: Fixed nRF52840-QIAA SoC support for device tree
* arm: Added Device Tree Support for nRF52840 SoC & boards
* arm: Added Device Tree Support for nRF51822 SoC & boards
* dts: Introduced st/mem.h for FLASH & SRAM sizes
* dts: Put IRQ priority into the interrupt property
* arm: Support for MKL25Z soc
* arm: Added FPU support
* x86: defined MMU data structures
* Support for ARC EM Starter Kit version 2.3 added



Boards
******

* Added qemu_xtensa board definition
* Added a more informative page fault handler x86 board
* xtensa: build similar to other Zephyr arches
* Define MMU data structures for x86 board
* Added support for board disco_l475_iot1
* Added STM32F413 Nucleo board
* Added support for the CC3220SF_LAUNCHXL board
* Support for new ARM board FRDM-KL25Z
* arduino_101 board enable GPIO by default
* boards: convert to using newly introduced integer sized types
* arm: Added support for Nucleo L432KC board
* arm: Added support for STM32L496G Discovery board
* arm: Added support for STM32F469I-DISCO board
* BBC micro:bit: Added driver & API for the 5x5 LED display

Drivers and Sensors
*******************

* UART interrupt-driver API is better defined
* Support for pull-style console API
* nRF5 IEEE 802.15.4 radio driver added
* KW41Z IEEE 802.15.4 radio driver added
* Added MCUX TRNG driver
* Added support for the SiFive Freedom E310 pinmux driver
* drivers/sensor: Convert formatter strings to use PRI defines
* Added lps22hb sensor driver
* Added lsm6dsl sensor driver
* Added heart rate sensor driver
* Added support for max30101 heart rate sensor
* Added support for lis2dh accelerometer

Networking
**********

* HTTPS server support added
* HTTP Basic-Auth support added
* IPv6 fragmentation support added
* Add block wise support to CoAP for well-known response
* Big refactoring of network buffer handling
* Start to collect TCP statistics if enabled in config
* IEEE 802.15.4 security support added
* DNS resolver sample application added
* IPv6 multicast listener (MLDv2) support added
* NATS protocol sample application added
* HTTP client and server connectivity fixes
* Network samples Coverity fixes
* Network samples llvm compiler warning fixes
* MQTT publisher connectivity fixes
* 6lo IPv6 header compression fixes
* CoAP connectivity fixes
* DHCPv4 connectivity fixes
* TCP connectivity fixes
* DNS documentation and connectivity fixes
* IPv6 connectivity fixes
* IPv4 ARP fixes
* IEEE 802.15.4 configuration tweaking fixes
* Remove ORFD (Overly Reduced Function Device) 802.15.4 support
* Network offloading driver fixes
* Fix various memory leaks
* Properly check TCP and UDP checksum before accepting packet
* Start RX and TX network threads in proper order
* Network samples documentation fixes and clarifications
* RPL mesh routing fixes
* Network link (MAC) address fixes

Bluetooth
*********

* Host: Added ATT and SMP packet tracking for flow control enforcement
* Host: GATT database changed to a linked list in preparation for dynamic allocation
* Bluetooth 5.0: The Controller reports itself as 5.0-capable
* Bluetooth 5.0: Introduced Channel Selection Algorithm #2 support
* Bluetooth 5.0: Added Multiple PHY support, both 2Mbit/s and long-range coded
* Bluetooth 5.0: Integrated Scan Request notifications
* Controller: Added Low Duty Cycle Directed Advertising support
* Controller: Added Scan duplicate filtering support
* Controller: Enforced complete role separation in the controller for smaller builds
* Controller: Introduced Advanced Controller configuration with several new Kconfig options
* Controller: Changed the radio interrupts to direct ISRs to reduce interrupt latency
* Added HCI Controller to Host flow control support in both Host and Controller
* BR/EDR: Added HFP (e)SCO audio channel establishment support
* BR/EDR: Added support for a functional SDP server

Build and Infrastructure
************************

* Support building host tools
* Added separate DTS target
* Added support for MSYS2
* Use -O2 instead of -Os for ARC with SDK 0.9

Libraries
*********

* Added library for software driven I2C
* Created a HTTP library
* Added HTTP server library support
* Added minimal JSON library
* Update TinyCrypt to version 0.2.6
* Added minimal JSON library

HALs
****

* Added Atmel SAM family I2C (TWIHS) driver
* Added Atmel SAM serial (UART) driver
* Added WDT driver for Atmel SAM SoCs
* Added Atmel SAM4S SoC support
* Imported Nordic 802.15.4 radio driver
* Added Initial support for NXP MPU
* Updated QMSI to 1.4 RC4
* Added FPU support
* Added basic support for STM32F413
* Introduced STM32F4x DMA driver
* pinmux: stm32: Added support for Nucleo L432KC
* Added support for STM32L496G Discovery board
* Added dts for STM32F407
* Added support for STM32F4DISCOVERY Board
* Added support for STM32F469XI
* Added support for STM32F469I-DISCO

Documentation
*************

* Board documentation added for new board ports
* Added a board porting guide
* Added security sections to porting and user guides
* Continued migration of wiki.zephyrproject.org material to website and github wiki
* Improved CSS formatting and appearance of generated documents
* Added breadcrumb navigation header with kernel version number
* Updated getting started setup guides for Linux, Windows, and macOS
* Updates and additions to follow new and updated kernel features
* Broken link and spelling check scans
* Removed deprecated kernel documentation (pre 1.6 release) from website (still available in git repo if needed)

Tests and Samples
*****************

* Added test to verify same tick timeout expiry order
* Added clock_test for kernel
* Added tickless tests
* Added a simple CC2520 crypto dev test
* Added combined observer & broadcaster app for Bluetooth samples
* Added support to wait both IPv4 and IPv6
* Enabled tickless kernel option in some apps

JIRA Related Items
******************

.. comment  List derived from Jira query: ...

* ``ZEP-248`` - Add a BOARD/SOC porting guide
* ``ZEP-339`` - Tickless Kernel
* ``ZEP-540`` - add APIs for asynchronous transfer callbacks
* ``ZEP-628`` - Validate RPL Routing node support
* ``ZEP-638`` - feature to consider: flag missing functionality at build time when possible
* ``ZEP-720`` - Add MAX30101 heart rate sensor driver
* ``ZEP-828`` - IPv6 - Multicast Join/Leave Support
* ``ZEP-843`` - Unified assert/unrecoverable error infrastructure
* ``ZEP-888`` - 802.15.4 - Security support
* ``ZEP-932`` - Adapt kernel sample & test projects
* ``ZEP-948`` - Revisit the timeslicing algorithm
* ``ZEP-973`` - Remove deprecated API related to device PM functions and DEVICE\_ and SYS\_* macros
* ``ZEP-1028`` - shrink k_block struct size
* ``ZEP-1032`` - IPSP router role support
* ``ZEP-1169`` - Sample mbedDTLS DTLS client stability on ethernet driver
* ``ZEP-1171`` - Event group kernel APIs
* ``ZEP-1280`` - Provide Event Queues Object
* ``ZEP-1313`` - porting and user guides must include a security section
* ``ZEP-1326`` - Clean up _THREAD_xxx APIs
* ``ZEP-1388`` - Add support for KW40 SoC
* ``ZEP-1391`` - Add support for Hexiwear KW40
* ``ZEP-1392`` - Add FXAS21002 gyroscope sensor driver
* ``ZEP-1435`` - Improve Quark SE C1000 ARC Floating Point Performance
* ``ZEP-1438`` - AIO: AIO Comparator is not stable on D2000 and Arduino101
* ``ZEP-1463`` - Add Zephyr Support in segger SystemView
* ``ZEP-1500`` - net/mqtt: Test case for the MQTT high-level API
* ``ZEP-1528`` - Provide template for multi-core applications
* ``ZEP-1529`` - Unable to exit menuconfig
* ``ZEP-1530`` - Hotkeys for the menu at the bottom of menuconfig sometimes doesn't work
* ``ZEP-1568`` - Replace arm cortex_m scs and scb functionality with direct CMSIS-core calls
* ``ZEP-1586`` - menuconfig: Backspace is broken
* ``ZEP-1599`` - printk() support for the '-' indicator  in format string (left justifier)
* ``ZEP-1607`` - JSON encoding/decoding library
* ``ZEP-1621`` - Stack Monitoring
* ``ZEP-1631`` - Ability to use k_mem_pool_alloc (or similar API) from ISR
* ``ZEP-1684`` - Add Atmel SAM family watchdog (WDT) driver
* ``ZEP-1695`` - Support ADXL362 sensor
* ``ZEP-1698`` - BME280 support for SPI communication
* ``ZEP-1711`` - xtensa build defines Kconfigs with lowercase names
* ``ZEP-1718`` - support for IPv6 fragmentation
* ``ZEP-1719`` - TCP does not work with 6lo
* ``ZEP-1721`` - many TinyCrypt test cases only run on ARM and x86
* ``ZEP-1722`` - xtensa: TinyCrypt does not build
* ``ZEP-1735`` - Controller to Host flow control
* ``ZEP-1759`` - All python scripts needed for build should be moved to python 3 to minimize dependencies
* ``ZEP-1761`` - K_MEM_POOL_DEFINE build error "invalid register name" when built with llvm/icx from ISSM toolchain
* ``ZEP-1769`` - Implement  Set Event Mask and LE Set Event Mask commands
* ``ZEP-1772`` - re-introduce controller to host flow control
* ``ZEP-1776`` - sending LE COC data from RX thread can lead to deadlock
* ``ZEP-1785`` - Tinytile: Flashing not supported with this board
* ``ZEP-1788`` - [REG] bt_enable: No HCI driver registered
* ``ZEP-1800`` - Update external mbed TLS library to latest version (2.4.2)
* ``ZEP-1812`` - Add tickless kernel support in HPET timer
* ``ZEP-1816`` - Add tickless kernel support in LOAPIC timer
* ``ZEP-1817`` - Add tickless kernel support in ARCV2 timer
* ``ZEP-1818`` - Add tickless kernel support in cortex_m_systick timer
* ``ZEP-1821`` - Update PM apps to use mili/micro seconds instead of ticks
* ``ZEP-1823`` - Improved Benchmarks
* ``ZEP-1825`` - Context Switching KPI
* ``ZEP-1836`` - Expose current ecb_encrypt() as bt_encrypt() so host can directly access it
* ``ZEP-1856`` - remove legacy micro/nano kernel APIs
* ``ZEP-1857`` - Build warnings [-Wpointer-sign] with LLVM/icx (bluetooth_handsfree)
* ``ZEP-1866`` - Add Atmel SAM family I2C (TWIHS) driver
* ``ZEP-1880`` - "samples/grove/temperature": warning raised when generating configure file
* ``ZEP-1886`` - Build warnings [-Wpointer-sign] with LLVM/icx (tests/net/nbuf)
* ``ZEP-1887`` - Build warnings [-Wpointer-sign] with LLVM/icx (tests/drivers/spi/spi_basic_api)
* ``ZEP-1893`` - openocd: 'make flash' works with Zephyr SDK only and fails for all other toolchains
* ``ZEP-1896`` - [PTS] L2CAP/LE/CFC/BV-06-C
* ``ZEP-1899`` - Missing board documentation for xtensa/xt-sim
* ``ZEP-1908`` - Missing board documentation for arm/nucleo_96b_nitrogen
* ``ZEP-1910`` - Missing board documentation for arm/96b_carbon
* ``ZEP-1927`` - AIO: AIO_CMP_POL_FALL is triggered immediately after aio_cmp_configure
* ``ZEP-1935`` - Packet loss make RPL mesh more vulnerable
* ``ZEP-1936`` - tests/drivers/spi/spi_basic_api/testcase.ini#test_spi - Assertion Fail
* ``ZEP-1946`` - Time to Next Event
* ``ZEP-1955`` - Nested interrupts crash on Xtensa architecture
* ``ZEP-1959`` - Add Atmel SAM family serial (UART) driver
* ``ZEP-1965`` - net-tools HEAD is broken for QEMU/TAP
* ``ZEP-1966`` - Doesn't seem to be able to both send and receive locally via local address
* ``ZEP-1968`` - "make mrproper" removes top-level dts/ dir, makes ARM builds fail afterwards
* ``ZEP-1980`` - Move app_kernel benchmark to unified kernel
* ``ZEP-1984`` - net_nbuf_append(), net_nbuf_append_bytes() have data integrity problems
* ``ZEP-1990`` - Basic support for the BBC micro:bit LED display
* ``ZEP-1993`` - Flowcontrol Required for CDC_ACM
* ``ZEP-1995`` - samples/subsys/console breaks xtensa build
* ``ZEP-1997`` - Crash during startup if co-processors are present
* ``ZEP-2008`` - Port tickless idle test to unified kernel and cleanup
* ``ZEP-2009`` - Port test_sleep test to unified kernel and cleanup
* ``ZEP-2011`` - Retrieve RPL node information through CoAP requests
* ``ZEP-2012`` - Fault in networking stack for cores that can't access unaligned memory
* ``ZEP-2013`` - dead object monitor code
* ``ZEP-2014`` - Default samples/subsys/shell/shell fails to build on QEMU RISCv32 / NIOS2
* ``ZEP-2019`` - Xtensa port does not compile if CONFIG_TICKLESS_IDLE is enabled
* ``ZEP-2027`` - Bluetooth Peripheral Sample won't pair with certain Android devices
* ``ZEP-2029`` - xtensa: irq_offload() doesn't work on XRC_D2PM
* ``ZEP-2033`` - Channel Selection Algorithm #2
* ``ZEP-2034`` - High Duty Cycle Non-Connectable Advertising
* ``ZEP-2037`` - Malformed echo response
* ``ZEP-2048`` - Change UART "baud-rate" property to "current-speed"
* ``ZEP-2051`` - Move away from C99 types to zephyr defined types
* ``ZEP-2052`` - arm: unhandled exceptions in thread take down entire system
* ``ZEP-2055`` - Add README.rst in the root of the project for github
* ``ZEP-2057`` - crash in tests/net/rpl on qemu_x86 causing intermittent sanitycheck failure
* ``ZEP-2061`` - samples/net/dns_resolve networking setup/README is confusing
* ``ZEP-2064`` - RFC: Making net_shell command handlers reusable
* ``ZEP-2065`` - struct dns_addrinfo has unused fields
* ``ZEP-2066`` - nitpick: SOCK_STREAM/SOCK_DGRAM values swapped compared to most OSes
* ``ZEP-2069`` - samples: net: dhcpv4_client: runs failed on frdm k64f board
* ``ZEP-2070`` - net pkt doesn't full unref after send a data form bluetooth's ipsp
* ``ZEP-2076`` - samples: net: coaps_server: build failed
* ``ZEP-2077`` - Fix IID when using CONFIG_NET_L2_BLUETOOTH_ZEP1656
* ``ZEP-2080`` - No reply from RPL node after 20-30 minutes.
* ``ZEP-2092`` - [NRF][BT] Makefile:946: recipe for target 'include/generated/generated_dts_board.h' failed
* ``ZEP-2114`` - tests/kernel/fatal : Fail for QC1000/arc
* ``ZEP-2125`` - Compilation error when UART1 port is enabled via menuconfig
* ``ZEP-2132`` - Build samples/bluetooth/hci_uart fail
* ``ZEP-2138`` - Static code scan (coverity) issues seen
* ``ZEP-2143`` - Compilation Error on Windows 10 with MSYS2
* ``ZEP-2152`` - Xtensa crashes on startup for cores with coprocessors
* ``ZEP-2178`` - Static code scan (coverity) issues seen
