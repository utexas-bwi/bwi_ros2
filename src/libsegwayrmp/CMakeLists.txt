## Project Setup
cmake_minimum_required(VERSION 3.8)

project(libsegwayrmp VERSION 0.2.8 LANGUAGES CXX)

## Configurations
include(cmake/libsegwayrmp_settings.cmake)

## Configure the build system

# Find Boost
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(ament_cmake REQUIRED)

if(DEFINED ENV{COLCON_WS})
    set(COLCON_WS $ENV{COLCON_WS})
else()
    message(FATAL_ERROR "Environment variable COLCON_WS is not set!")
endif()

# Find Serial
# ----->
# Set serial include directory
include_directories(${COLCON_WS}/serial/include)

# Find Serial Library (adjust the path to your actual library location)
find_library(SERIAL_LIBRARY NAMES serial HINTS ${COLCON_WS}/serial/build)

if (NOT SERIAL_LIBRARY)
    message(FATAL_ERROR "Serial library not found!")
endif()
# <-----

# Find Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Include Boost directories if found
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

# Set the source files, headers, and link libraries
set(LIBSEGWAYRMP_SRCS
  src/segwayrmp.cc
  src/impl/rmp_io.cc
  src/impl/rmp_ftd2xx.cc
  src/impl/rmp_serial.cc
)

set(LIBSEGWAYRMP_HEADERS
  include/libsegwayrmp/segwayrmp.h
)

set(LIBSEGWAYRMP_LINK_LIBS ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

## Build the segwayrmp project

# Compile the Library
add_library(libsegwayrmp SHARED ${LIBSEGWAYRMP_SRCS})

# Add the include folder to the include path
# include_directories(${PROJECT_SOURCE_DIR}/include)
target_include_directories(libsegwayrmp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${COLCON_WS}/serial/include  # Add the serial include directory
)

# Link the libraries to segwayrmp using keywords
target_link_libraries(libsegwayrmp  
  PRIVATE Qt5::Core            # Link Qt5 Core
  PRIVATE Qt5::Widgets         # Link Qt5 Widgets
  ${SERIAL_LIBRARY} 
  ${LIBSEGWAYRMP_LINK_LIBS}       # Link Boost libraries
)

set(BUILD_SHARED_LIBS OFF)

# Configure FTD2XX support *after* the library has been defined
include(cmake/libsegwayrmp_ftd2xx.cmake)

# segwayrmp serial
include(cmake/libsegwayrmp_serial.cmake)

## Build Examples
set(LIBSEGWAYRMP_EXAMPLE_SRCS examples/segwayrmp_example.cc)
set(LIBSEGWAYRMP_EXAMPLE_LINK_LIBS segwayrmp)

include(cmake/libsegwayrmp_examples.cmake)

## Build Tests
set(LIBSEGWAYRMP_TEST_SRCS tests/segwayrmp_tests.cc)
set(LIBSEGWAYRMP_TEST_LINK_LIBS libsegwayrmp)
include(cmake/libsegwayrmp_tests.cmake)

## Setup Install/Uninstall Targets
include(cmake/libsegwayrmp_targets.cmake)

## Install Configuration
install(TARGETS libsegwayrmp
  EXPORT libsegwayrmpTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(EXPORT libsegwayrmpTargets
  FILE libsegwayrmpTargets.cmake
  NAMESPACE libsegwayrmp::
  DESTINATION lib/cmake/libsegwayrmp
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
