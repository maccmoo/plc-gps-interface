cmake_minimum_required(VERSION 2.8.9)
project (plc-gps-interface)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_C_FLAGS "-pthread -g -Wall -Wextra -Wno-strict-prototypes -Wno-unknown-warning-option -Werror -std=gnu99 ${CMAKE_C_FLAGS}")

add_library(sbp_callback_functions sbp_callback_functions.c)
add_library(cJSON cJSON.c)

add_executable(plc-gps-interface plc-gps-interface.c)
target_link_libraries (plc-gps-interface sbp_callback_functions slog cJSON)

find_package(PkgConfig)
#pkg_check_modules(LIBSERIALPORT libserialport)
#link_directories(${LIBSERIALPORT_LIBRARY_DIRS})
#include_directories(${LIBSERIALPORT_INCLUDE_DIRS})

link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")

#target_link_libraries(plc-gps-interface serialport sbp modbus slog)
target_link_libraries(plc-gps-interface sbp modbus slog)
