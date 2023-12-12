cmake_minimum_required(VERSION 3.14)
project(my_model)

#This package allows us to write config files that can be relocated, paths are not hard-coded.
include ( CMakePackageConfigHelpers )

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###################################################
# Create the compilation database for clangd and move it out of the build dir.
###################################################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")


###################################
# find dependencies
###################################
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)
find_package(Doxygen REQUIRED)
find_package(rclcpp REQUIRED)   # allow ROScpp library 


############################################
# Set up unit test and code coverage generation.
############################################
# Must compile with debug, e.g,
#   cmake -S ./ -B build/ -D CMAKE_BUILD_TYPE=Debug
#
# below sets up code coverage flags
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()

option(WANT_COVERAGE "this option enable coverage" ON)
if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  set(WANT_COVERAGE OFF)
endif()

if(WANT_COVERAGE)
  message("Enabling coverage")
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules")
  include(CodeCoverage)
  # The following line come from cmake-modules/CodeCoverage.cmake
  append_coverage_compiler_flags() # will add "--coverage" compilation flag
  # Exclude certain directories from coverage analysis
  set(COVERAGE_EXCLUDES
    ${PROJECT_BINARY_DIR}
    )
endif()

# create targets for building code coverage reports
if(WANT_COVERAGE)
  if(UNIT_TEST_ALREADY_RAN)
    unset(EXECUTABLE)          # don't run unit test again, just generate the coverage report
  else()
    set(EXECUTABLE ctest test/)   # what to run insdie the build directory?
  endif()
  # unit test coverage:
  setup_target_for_coverage_lcov ( # use lcov to generate coverage report
    NAME test_coverage             # target name
    EXECUTABLE ${EXECUTABLE}
    EXCLUDE
      "/usr/include/*"          # Don't analyze system headers
      "/opt/*"                  # Don't analyze system headers
    )
endif()

# Enables testing for this directory and below
enable_testing()
include(GoogleTest)

# Sanity check:
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
message(STATUS "WANT_COVERAGE    = ${WANT_COVERAGE}")



###################################################
# Doxygen Rules
# ref: https://cmake.org/cmake/help/latest/module/FindDoxygen.html
###################################################
if (DOXYGEN_FOUND)
  set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_EXTRACT_ALL          YES )
  set( DOXYGEN_CLASS_DIAGRAMS       YES )
  set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
  set( DOXYGEN_HAVE_DOT             YES )
  set( DOXYGEN_CLASS_GRAPH          YES )
  set( DOXYGEN_CALL_GRAPH           YES )
  set( DOXYGEN_CALLER_GRAPH         YES )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
  set( DOXYGEN_EXTRACT_PRIVATE      YES )
  set( DOXYGEN_EXTRACT_PACKAGE      YES )
  set( DOXYGEN_EXTRACT_STATIC       YES )
  set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
  set( DOXYGEN_UML_LOOK             YES )
  set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
  set( DOXYGEN_TEMPLATE_RELATIONS   YES )
  set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
  set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
  set( DOXYGEN_DOT_TRANSPARENT      YES )
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()


###################################
# Specify Targets
###################################
# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test
  )

# Build libraries:
add_library (my_model
  src/my_dummy_lib_funct1.cpp
  src/my_dummy_lib_funct2.cpp
  )

# Specify where the associated include files are
target_include_directories (my_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include> 
    PUBLIC ${OpenCV_INCLUDE_DIRS}
    PUBLIC ${rclcpp_INCLUDE_DIRS}
    ) 

# Specify where the associated libraries are
# Any dependent libraires needed to build this target.
target_link_libraries(my_model
  # list of libraries
  ${OpenCV_LIBRARIES}
  ${rclcpp_LIBRARIES}
  )


###################################
# Installation
###################################

# Install executables and libraries:
install (TARGETS
  my_model
  EXPORT my_model-export      # Add targets to a export group named "my_model-export"
  DESTINATION lib/${PROJECT_NAME}
  )

# Install the header files
install(FILES
  include/my_dummy_lib_funct1.hpp
  include/my_dummy_lib_funct2.hpp
  DESTINATION include 
  )

###################################
# Export config so we can use find_package()
###################################

# Export the export group. CMake will write a configuration file of the current build tree:
export (TARGETS
  my_model
  FILE my_model-config.cmake)   # must by <PACKAGE>-config.cmake

# Export build tree to the package registry (~/.cmake/packages/<packag>/)
set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)
export(PACKAGE my_model)

# Install the export group
install(EXPORT my_model-export
  DESTINATION share/${PROJECT_NAME}/cmake/
  )


###################################
# Sub-Modules
###################################
add_subdirectory(test)
