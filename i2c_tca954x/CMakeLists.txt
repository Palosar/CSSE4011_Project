# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(CONF_FILE prj.conf)
set(DTC_OVERLAY_FILE app.overlay)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i2c_tca954x)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
