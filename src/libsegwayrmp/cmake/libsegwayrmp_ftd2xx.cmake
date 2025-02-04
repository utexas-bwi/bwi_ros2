# This file configures the library to build against one of the provided
# ftd2xx libraries shipped with this library.

include_directories(${PROJECT_SOURCE_DIR}/ftd2xx/include)

# Add the source file that uses the ftd2xx library
list(APPEND libsegwayrmp_SRCS src/impl/rmp_ftd2xx.cc)

# Determine 32-bit or 64-bit system
set(bitness x32)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(bitness x64)  
endif()

# For Unix-like systems
if(UNIX)
  # Linux-specific configuration
  if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    list(APPEND libsegwayrmp_LINK_LIBS
      ${PROJECT_SOURCE_DIR}/ftd2xx/linux/x64/libftd2xx.a
      dl
      rt
      pthread
    )
    # Copy the static library for easy access
    file(
      COPY ${PROJECT_SOURCE_DIR}/ftd2xx/linux/x64/libftd2xx.a
      DESTINATION ${CMAKE_BINARY_DIR}/lib/
    )
  endif()

  # Mac OS X-specific configuration
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    message("")
    message("-- **********************************************************")
    message("-- On macOS, the ftd2xx library needs to be installed manually before running applications:")
    message("-- http://www.ftdichip.com/Drivers/D2XX/MacOSX/D2XX1.1.12.dmg")
    message("-- **********************************************************")
    message("")
    list(APPEND libsegwayrmp_LINK_LIBS
      ${PROJECT_SOURCE_DIR}/ftd2xx/osx/libftd2xx.1.1.12.dylib
    )
  endif()
endif()

# For Windows
if(WIN32)
  list(APPEND libsegwayrmp_LINK_LIBS
    ${PROJECT_SOURCE_DIR}/ftd2xx/win/${bitness}/ftd2xx.lib
  )
endif()

# Ensure the libraries are correctly linked to libsegwayrmp target
target_link_libraries(libsegwayrmp PRIVATE ${libsegwayrmp_LINK_LIBS})  # Use PRIVATE for consistency

