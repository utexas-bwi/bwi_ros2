# If asked to and there are some example src files
if(LIBSEGWAYRMP_BUILD_EXAMPLES AND DEFINED LIBSEGWAYRMP_EXAMPLE_SRCS)
  message("-- Building libsegwayrmp Examples")

  # Compile the libsegwayrmp examples
  add_executable(libsegwayrmp_example ${LIBSEGWAYRMP_EXAMPLE_SRCS})

  # Link the examples to libraries (ensure libraries are available)
  target_link_libraries(libsegwayrmp_example
    ${ament_LIBRARIES}
    ${LIBSEGWAYRMP_EXAMPLE_LINK_LIBS}
    libsegwayrmp  # Link to the libsegwayrmp library
  )

  # Install the executable
  install(TARGETS libsegwayrmp_example
    DESTINATION lib/${PROJECT_NAME}
  )
endif(LIBSEGWAYRMP_BUILD_EXAMPLES AND DEFINED LIBSEGWAYRMP_EXAMPLE_SRCS)

