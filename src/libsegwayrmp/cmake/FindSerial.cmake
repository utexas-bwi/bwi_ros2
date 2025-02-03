# Find the Serial library

find_path(SERIAL_INCLUDE_DIR
  NAMES SerialStream.h
  PATHS /usr/include /usr/local/include
)

find_library(SERIAL_LIBRARY
  NAMES serial
  PATHS /usr/lib /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Serial DEFAULT_MSG SERIAL_LIBRARY SERIAL_INCLUDE_DIR)

if(SERIAL_FOUND)
  set(SERIAL_LIBRARIES ${SERIAL_LIBRARY})
  set(SERIAL_INCLUDE_DIRS ${SERIAL_INCLUDE_DIR})
endif()

