
cmake_minimum_required(VERSION 3.8.2)
project(vm-linux-rpi4 C CXX ASM)

# import camkes_vm declare function
include(${CAMKES_ARM_VM_HELPERS_PATH})

# use custom dtb file
set(VmDtbFile ON CACHE BOOL "" FORCE)

if("${PLATFORM}" STREQUAL "rpi4")
    set(cpp_flags "-DKERNELARMPLATFORM_RPI4")
    
    AddToFileServer("linux" "${CMAKE_CURRENT_LIST_DIR}/images/linux")
    AddToFileServer("matrix653" "${CMAKE_CURRENT_LIST_DIR}/images/bspbcm2711.bin")

    set(dtb_file "${CMAKE_CURRENT_LIST_DIR}/images/rpi4.dtb")
    set(dts_file "${CMAKE_CURRENT_LIST_DIR}/images/rpi4.dts")
    find_program(DTC_TOOL dtc)
    if("${DTC_TOOL}" STREQUAL "DTC_TOOL-NOTFOUND")
        message(FATAL_ERROR "Cannot find 'dtc' program.")
    endif()
    execute_process(
        COMMAND
            ${DTC_TOOL} -q -I dts -O dtb -o ${dtb_file} ${dts_file}
        RESULT_VARIABLE error
    )

    AddToFileServer("linux-dtb" "${dtb_file}")
elseif()
    message(FATAL_ERROR "unsupported platform ${PLATFORM}.")
endif()

DefineCAmkESVMFileServer()

CAmkESAddImportPath(${VM_LINUX_APP})

