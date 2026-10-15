#
# Copyright (c) 2026 Telink Semiconductor
#
# SPDX-License-Identifier: Apache-2.0
#

if (CONFIG_ST25DV)

zephyr_library()

zephyr_include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/src
)

zephyr_library_sources(
	${CMAKE_CURRENT_SOURCE_DIR}/src/st25dv.c
)

endif() # CONFIG_ST25DV
