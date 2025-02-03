# Find Serial if support requested
if(LIBSEGWAYRMP_USE_SERIAL)
  find_package(serial QUIET)

  # Set to FALSE initially, and enable if found
  # Nikunj
  set(LIBSEGWAYRMP_USE_SERIAL TRUE)
  
  if(TRUE)
    set(LIBSEGWAYRMP_USE_SERIAL TRUE)
    # Include serial library headers
    include_directories(${serial_INCLUDE_DIRS})
    # Add serial libraries to link with
    list(APPEND libsegwayrmp_LINK_LIBS ${serial_LIBRARIES})  # Update variable name
  else()
    # Could not find serial either through find_package or ROS 2
    message("--")
    message("-- Serial support disabled: Serial library not found.")
    message("--")
  endif()

  # If serial support is enabled, add source files and define macros
  if(LIBSEGWAYRMP_USE_SERIAL)
    message("-- Building libsegwayrmp with serial support")  # Update message
    list(APPEND libsegwayrmp_SRCS src/impl/rmp_serial.cc)  # Update variable name
    # Define a macro for conditional compilation
    add_definitions(-DLIBSEGWAYRMP_USE_SERIAL)
  endif()
endif()

