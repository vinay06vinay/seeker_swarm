# Any C++ source files needed to build this target (my_model_tester).
add_executable(my_model_tester
  # list of source cpp files:
  main.cpp
  test.cpp
  )

# Any include directories needed to build this target.
# Note: we do not need to specify the include directories for the
# dependent libraries, they are automatically included.
target_include_directories(my_model_tester PUBLIC
  # list of include directories:
  ${CMAKE_SOURCE_DIR}/include
  )

# Any dependent libraires needed to build this target.
target_link_libraries(my_model_tester PUBLIC
  # list of libraries:
  gtest
  my_model
  )

# Enable CMake’s test runner to discover the tests included in the
# binary, using the GoogleTest CMake module.
gtest_discover_tests(my_model_tester)


# # Build executable:
# add_executable (my_model_tester
#   test/main.cpp
#   )
# target_link_libraries(my_model_tester
#   # list of libraries
#   my_model
#   gtest
#   )
