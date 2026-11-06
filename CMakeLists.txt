# SPDX-License-Identifier: GPL-2.0-or-later
#
# CMake build of NVMEASPI, the NVMe transport for OpenASPI's native layer
# (C, for 32/64-bit kernels such as GonWare's GDOS).
#
# Needs the OpenASPI::native target: a parent project provides it, or
# OPENASPI_SOURCE_DIR points to an OpenASPI checkout (default: ../OpenAspi
# or ../OpenASPI next to this one).

cmake_minimum_required(VERSION 3.20)
project(NVMEASPI LANGUAGES C)

if(NOT TARGET OpenASPI::native)
  set(OPENASPI_SOURCE_DIR "" CACHE PATH "OpenASPI source tree")
  if(NOT OPENASPI_SOURCE_DIR)
    foreach(_d OpenAspi OpenASPI)
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../${_d}/CMakeLists.txt")
        set(OPENASPI_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../${_d}")
      endif()
    endforeach()
  endif()
  if(NOT OPENASPI_SOURCE_DIR)
    message(FATAL_ERROR "NVMEASPI: set OPENASPI_SOURCE_DIR to an OpenASPI checkout")
  endif()
  add_subdirectory("${OPENASPI_SOURCE_DIR}" openaspi EXCLUDE_FROM_ALL)
endif()

add_library(nvmeaspi_native STATIC native/nvme.c)
target_include_directories(nvmeaspi_native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(nvmeaspi_native PUBLIC OpenASPI::native)
set_target_properties(nvmeaspi_native PROPERTIES C_STANDARD 90 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
add_library(NVMEASPI::native ALIAS nvmeaspi_native)

if(PROJECT_IS_TOP_LEVEL)
  target_compile_options(nvmeaspi_native PRIVATE -Wall -Wextra -Werror -pedantic)
endif()
