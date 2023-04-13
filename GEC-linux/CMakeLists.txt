cmake_minimum_required(VERSION 3.7.2)

project(GEC C)

file(
    GLOB
        deps
        src/*.c
)

add_library(
    GEC ${deps}
)

target_include_directories(GEC PUBLIC include)

# target_link_libraries(GEC muslc)
