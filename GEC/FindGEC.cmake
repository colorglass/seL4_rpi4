set(GEC_DIR "${CMAKE_CURRENT_LIST_DIR}" CACHE STRING "")
mark_as_advanced(GEC_DIR)

macro(GEC_import_library)
    add_subdirectory(${GEC_DIR} GEC)
endmacro()
