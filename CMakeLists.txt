cmake_minimum_required(VERSION 3.10)

project(mc_h1)

set(CMAKE_CXX_STANDARD 17)

find_package(mc_rtc REQUIRED)
find_description_package(h1_description)

message("-- H1_DESCRIPTION_PATH: ${H1_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

macro(add_module NAME)
  add_robot(${NAME} src/${NAME}.cpp src/${NAME}.h)
  target_include_directories(${NAME}
                             PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
endmacro()

add_module(h1)

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
