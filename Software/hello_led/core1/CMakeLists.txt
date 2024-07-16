# Copyright (c) 2022 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

set(SES_TOOLCHAIN_VARIANT "Andes")

set(HPM_BUILD_TYPE "sec_core_img")
set(BUILD_FOR_SECONDARY_CORE 1)
set(SEC_CORE_IMG_C_ARRAY_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/../core0/src/sec_core_img.c)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(multicore_core1)
sdk_ses_compile_options(-mabi=ilp32d)
sdk_ses_compile_options(-march=rv32gc)
sdk_ses_opt_debug_connection(J-Link)

sdk_inc(src)

file(GLOB_RECURSE SRC_FILES src/*.c)
sdk_app_src(${SRC_FILES})

generate_ses_project()
