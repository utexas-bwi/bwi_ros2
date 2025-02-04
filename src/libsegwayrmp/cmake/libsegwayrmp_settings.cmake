## This cmake file contains different settings for the project

# Should the examples be built?
option(LIBSEGWAYRMP_BUILD_EXAMPLES "Build the examples?" OFF)

# Should the tests be built?
option(LIBSEGWAYRMP_BUILD_TESTS "Build the tests?" OFF)

# Should support for control via Serial be built?
option(LIBSEGWAYRMP_USE_SERIAL "Build with Serial (RS-232) Support?" ON)

# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Uncomment the line below to set the build type
set(CMAKE_BUILD_TYPE "RELWITHDEBINFO")

# Specify the install directory for binaries and libraries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Ensure CMake's install commands use the correct directories
include(GNUInstallDirs)

