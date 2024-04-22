# Copyright (c) 2022 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(SES_TOOLCHAIN_VARIANT "Andes")

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    SET(CMAKE_BUILD_TYPE flash_xip)
endif()
find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(multicore_core0)
sdk_ses_compile_options(-mabi=ilp32d)
sdk_ses_compile_options(-march=rv32gc)
sdk_ses_opt_debug_connection(J-Link)

sdk_app_inc(src)
sdk_app_inc($ENV{HPM_SDK_BASE}/samples/multicore/common)

file(GLOB_RECURSE SRC_FILES src/*.c)
sdk_app_src(${SRC_FILES})
sdk_app_src($ENV{HPM_SDK_BASE}/samples/multicore/common/multicore_common.c)
generate_ses_project()
