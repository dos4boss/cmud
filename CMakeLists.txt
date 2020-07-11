cmake_minimum_required(VERSION 3.10)

project(cmud)

add_subdirectory(gsl-lite)
add_subdirectory(cli)
add_subdirectory(spdlog)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

set(BINARY_NAME cmud)
add_executable(${BINARY_NAME} src/cmud.cpp src/hw_interface.cpp src/hw_interface_helpers.cpp src/board_interface.cpp src/correction_processor_interface.cpp)
target_link_libraries(${BINARY_NAME} cli::cli gsl::gsl-lite spdlog::spdlog_header_only)
