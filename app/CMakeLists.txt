# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

zephyr_include_directories(.)

add_subdirectory(drivers)

project(zephyr_sump)

target_link_libraries(app PRIVATE app__drivers__sump_port)
target_sources(app PRIVATE src/main.c src/sump.c)
