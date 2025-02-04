## This file sets up the targets like install and uninstall

set(${PROJECT_NAME}_targets_to_install libsegwayrmp)

# If the GUI is built, install it
if (TARGET libsegwayrmp_gui)
  list(APPEND ${PROJECT_NAME}_targets_to_install libsegwayrmp_gui)
endif()

install(
  TARGETS ${${PROJECT_NAME}_targets_to_install}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  FILES include/libsegwayrmp/segwayrmp.h
  DESTINATION include/libsegwayrmp
)

configure_file(
  "cmake/libsegwayrmpConfig.cmake.in"
  "cmake/libsegwayrmpConfig.cmake"
  @ONLY
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/libsegwayrmpConfig.cmake
  DESTINATION share/libsegwayrmp
)

install(
  FILES package.xml
  DESTINATION share/libsegwayrmp/
)

# For now, enable installing the FTDI library on Linux
if(UNIX AND ${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  install(
    FILES lib/libftd2xx.a
    DESTINATION lib/
  )
  set(libsegwayrmp_ADDITIONAL_LIBRARIES "-ldl")
endif()

# Enable pkg-config file generation for Linux
if(UNIX AND ${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  configure_file("cmake/libsegwayrmp.pc.in" "cmake/libsegwayrmp.pc" @ONLY)
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/libsegwayrmp.pc 
    DESTINATION lib/pkgconfig/
  )
endif()

# Configure make uninstall
# add_custom_target(uninstall
#   COMMAND ${CMAKE_COMMAND} -E echo "Uninstalling package"
# )

if (UNIX)
  add_custom_command(
    TARGET uninstall
    COMMAND ${CMAKE_COMMAND} -E remove -f < install_manifest.txt
    COMMENT "Uninstall package"
  )
else()
  add_custom_command(
    TARGET uninstall
    COMMAND ${CMAKE_COMMAND} -E echo "Uninstall only implemented in UNIX"
    COMMENT "Uninstall only implemented in UNIX"
  )
endif()

