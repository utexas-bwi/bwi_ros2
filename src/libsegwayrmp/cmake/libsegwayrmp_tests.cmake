# If asked to and there are some test source files
if(LIBSEGWAYRMP_BUILD_TESTS AND DEFINED LIBSEGWAYRMP_TEST_SRCS)
  # Find GTest if it's available
  find_package(GTest QUIET)

  if(GTEST_FOUND)
    message("-- Building libsegwayrmp Tests")
    
    # Add GTest to the include path
    include_directories(${GTEST_INCLUDE_DIRS})

    # Compile the libsegwayrmp tests
    add_executable(libsegwayrmp_tests ${LIBSEGWAYRMP_TEST_SRCS})

    # Link the tests to the libsegwayrmp library and GTest
    target_link_libraries(libsegwayrmp_tests
      ${LIBSEGWAYRMP_TEST_LINK_LIBS}
      ${GTEST_LIBRARIES}
      gtest_main  # Optional: link with gtest_main for convenience
    )

    # Add CMake test support
    include(GTest)
    add_test(NAME libsegwayrmp_tests COMMAND libsegwayrmp_tests)
  else()
    message("-- Skipping libsegwayrmp Tests - GTest not Found!")
  endif()
endif()

