
set(support_plat "rpi4;")

if(NOT ${PLATFORM} IN_LIST support_plat)
    message(FATAL_ERROR "PLATFORM: ${PLATFORM} not supported.
            Supported: ${supported}")
endif()

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

include(application_settings)
correct_platform_strings()

set(RELEASE OFF CACHE BOOL "Performance optimized build")
ApplyCommonReleaseVerificationSettings(${RELEASE} FALSE)

find_package(seL4 REQUIRED)
sel4_configure_platform_settings()

ApplyData61ElfLoaderSettings(${KernelPlatform} ${KernelSel4Arch})

if(${VM_LINUX})
    if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/vm-linux/${VM_LINUX_APP}")
        message(FATAL_ERROR "VM App: ${VM_LINUX_APP} not supported.")
    endif()

    set(KernelArmHypervisorSupport ON CACHE BOOL "" FORCE)
    set(KernelRootCNodeSizeBits 18 CACHE STRING "" FORCE)
    set(KernelArmVtimerUpdateVOffset OFF CACHE BOOL "" FORCE)
    set(KernelArmDisableWFIWFETraps ON CACHE BOOL "" FORCE)

    # capDL settings
    set(CapDLLoaderMaxObjects 90000 CACHE STRING "" FORCE)

    set(LibUSB OFF CACHE BOOL "" FORCE)
else()
    set(KernelRootCNodeSizeBits 16 CACHE STRING "")

    set(CapDLLoaderMaxObjects 20000 CACHE STRING "" FORCE)
endif()

set(LibSel4PlatSupportUseDebugPutChar TRUE CACHE BOOL "" FORCE)

set(KernelNumDomains 1 CACHE STRING "" FORCE)
set(KernelMaxNumNodes 4 CACHE STRING "" FORCE)