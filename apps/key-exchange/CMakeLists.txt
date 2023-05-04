#
# Copyright 2018, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.7.2)
set(project_dir "${CMAKE_CURRENT_LIST_DIR}/../..")
file(GLOB project_modules ${project_dir}/projects/*)
list(
    APPEND
        CMAKE_MODULE_PATH
        ${project_dir}/kernel
        ${project_dir}/tools/seL4/cmake-tool/helpers
        ${project_dir}/tools/seL4/elfloader-tool
        ${project_dir}/tools/camkes
        ${project_dir}/GEC-block
        ${project_modules}
)
# set(POLLY_DIR ${project_dir}/tools/polly CACHE INTERNAL "")

include(application_settings)

# Deal with the top level target-triplet variables.
if(NOT TUT_BOARD)
    message(
        FATAL_ERROR
            "Please select a board to compile for. Choose either pc or zynq7000\n\t`-DTUT_BOARD=<PREFERENCE>`"
    )
endif()

# Set arch and board specific kernel parameters here.
if(${TUT_BOARD} STREQUAL "zynq7000")
    # Do a quick check and warn the user if they haven't set
    # -DARM/-DAARCH32/-DAARCH64.
    if(
        (NOT ARM)
        AND (NOT AARCH32)
        AND ((NOT CROSS_COMPILER_PREFIX) OR ("${CROSS_COMPILER_PREFIX}" STREQUAL ""))
    )
        message(
            WARNING
                "The target machine is an ARM machine. Unless you've defined -DCROSS_COMPILER_PREFIX, you may need to set one of:\n\t-DARM/-DAARCH32/-DAARCH64"
        )
    endif()

    set(KernelArch "arm" CACHE STRING "" FORCE)
    set(KernelSel4Arch "aarch32" CACHE STRING "" FORCE)
    set(KernelPlatform "zynq7000" CACHE STRING "" FORCE)
    ApplyData61ElfLoaderSettings(${KernelPlatform} ${KernelSel4Arch})
elseif(${TUT_BOARD} STREQUAL "rpi4")
    set(KernelArch "arm" CACHE STRING "" FORCE)
    set(KernelSel4Arch "aarch64" CACHE STRING "" FORCE)
    set(KernelPlatform "bcm2711" CACHE STRING "" FORCE)
    ApplyData61ElfLoaderSettings(${KernelPlatform} ${KernelSel4Arch})
elseif(${TUT_BOARD} STREQUAL "qemu-arm-virt")
    set(KernelArch "arm" CACHE STRING "" FORCE)
    set(KernelSel4Arch "aarch64" CACHE STRING "" FORCE)
    set(KernelPlatform "qemu-arm-virt" CACHE STRING "" FORCE)
    ApplyData61ElfLoaderSettings(${KernelPlatform} ${KernelSel4Arch})
elseif(${TUT_BOARD} STREQUAL "ls1046a")
    set(KernelArch "arm" CACHE STRING "" FORCE)
    set(KernelSel4Arch "aarch64" CACHE STRING "" FORCE)
    set(KernelPlatform "ls1046a" CACHE STRING "" FORCE)
    set(KernelPlatformOK1046AC2 ON CACHE BOOL "" FORCE)
    ApplyData61ElfLoaderSettings(${KernelPlatform} ${KernelSel4Arch})
else()
    message(FATAL_ERROR "Unsupported board ${TUT_BOARD}.")
endif()

# include(${project_dir}/kernel/configs/seL4Config.cmake)
set(CapDLLoaderMaxObjects 20000 CACHE STRING "" FORCE)
set(KernelRootCNodeSizeBits 16 CACHE STRING "")

set(LibSel4PlatSupportUseDebugPutChar TRUE CACHE BOOL "" FORCE)

# Just let the regular abort spin without calling DebugHalt to prevent needless
# confusing output from the kernel for a tutorial
set(LibSel4MuslcSysDebugHalt FALSE CACHE BOOL "" FORCE)

# Only configure a single domain for the domain scheduler
set(KernelNumDomains 1 CACHE STRING "" FORCE)

# Enable SMP support
set(KernelMaxNumNodes 4 CACHE STRING "" FORCE)

# Serial output relies on Debug settings
ApplyCommonReleaseVerificationSettings(FALSE FALSE)

project(comm C)

find_package(camkes-tool REQUIRED)
camkes_tool_setup_camkes_build_environment()

# Use components in `global-components`
find_file(GLOBAL_COMPONENTS_PATH global-components.cmake PATHS ${project_dir}/projects/global-components/ CMAKE_FIND_ROOT_PATH_BOTH)
mark_as_advanced(FORCE GLOBAL_COMPONENTS_PATH)
if("${GLOBAL_COMPONENTS_PATH}" STREQUAL "GLOBAL_COMPONENTS_PATH-NOTFOUND")
    message(FATAL_ERROR "Failed to find global-components.cmake. Consider cmake -DGLOBAL_COMPONENTS_PATH=/path/to/global-components.cmake")
endif()
include(${GLOBAL_COMPONENTS_PATH})

# GEC crypto library
find_package(GEC REQUIRED)
GEC_import_library()

message(STATUS "GEC_DIR: ${GEC_DIR}")
set(mavlink_include_path "${project_dir}/mavlink/include" CACHE STRING "" FORCE)
set(gec_include_path "${GEC_DIR}/include" CACHE STRING "" FORCE)


DeclareCAmkESComponent(
    Pixhawk
    INCLUDES include ${gec_include_path}
    SOURCES components/Pixhawk/src/Pixhawk.c
)

DeclareCAmkESComponent(
    Telemetry
    INCLUDES include ${gec_include_path}
    SOURCES components/Telemetry/src/Telemetry.c
)

DeclareCAmkESComponent(
    Decrypt
    INCLUDES include ${mavlink_include_path} ${gec_include_path}
    SOURCES components/Decrypt/src/Decrypt.c
    LIBS GEC
)

DeclareCAmkESComponent(
    Encrypt
    INCLUDES include ${mavlink_include_path} ${gec_include_path}
    SOURCES components/Encrypt/src/Encrypt.c
    LIBS GEC
)

DeclareCAmkESComponent(
    KeyExchange
    INCLUDES include ${mavlink_include_path} ${gec_include_path}
    SOURCES components/KeyExchange/src/KeyExchange.c
    LIBS GEC
)

DeclareCAmkESRootserver(comm.camkes)
GenerateCAmkESRootserver()
