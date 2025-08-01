cmake_minimum_required(VERSION 3.10.0)
project(momentum_sdk VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

#pybind11 requirement
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Include momentum_driver C sources
set(MOMENTUM_DRIVER_SOURCES
    momentum_driver/momentum_can_driver.c
    momentum_driver/momentum_can_dbc.c
)

set(MOMENTUM_SDK_SOURCES
    src/momentum_sdk.cpp 
    src/can/can_interface.cpp 
    src/can/can_utilities.cpp  
    src/protocol/message_parser.cpp
)

add_library(momentum_sdk STATIC 
    ${MOMENTUM_SDK_SOURCES} 
    ${MOMENTUM_DRIVER_SOURCES}
)

set_target_properties(momentum_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(momentum_sdk 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/momentum_driver>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/momentum_driver
)

#Python bindings
pybind11_add_module(momentum_sdk_py MODULE python/momentum_bindings.cpp)
target_link_libraries(momentum_sdk_py PRIVATE momentum_sdk)
set_target_properties(momentum_sdk_py EXPORT_PROPERTIES
    OUTPUT_NAME "momentum_sdk"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/
)

add_executable(momentum_basic examples/momentum_cpp/momentum_basic.cpp)
target_link_libraries(momentum_basic momentum_sdk)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

ament_export_dependencies(pybind11)
ament_package()