cmake_minimum_required(VERSION 3.10)

project(CPU_Ray_Tracing_Implementation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB_RECURSE SOURCES "src/*.cc" "src/*.h")
add_subdirectory(third_party/tinyexr)

add_executable(CPU_Ray_Tracing_Implementation ${SOURCES})

add_custom_command(
    TARGET CPU_Ray_Tracing_Implementation POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_SOURCE_DIR}/assets
        ${CMAKE_BINARY_DIR}/assets
    COMMENT "Creating symbolic link to assets directory in build directory"
)

target_link_libraries(CPU_Ray_Tracing_Implementation PRIVATE tinyexr)
target_include_directories(CPU_Ray_Tracing_Implementation PRIVATE third_party/tinyexr)