cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcl-transforms)

if(NOT CMAKE_BUILD_TYPE)
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

string(REPLACE "." "-" COMPILER_VERSION ${CMAKE_CXX_COMPILER_VERSION})
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(COMPILER "clang-${COMPILER_VERSION}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(COMPILER "gcc-${COMPILER_VERSION}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  set(COMPILER "intel-${COMPILER_VERSION}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  set(COMPILER "mvvc-${COMPILER_VERSION}")
endif()

# Benchmarks and tests will be compiled with different compiler flags pertaining to CPU instruction set.
# Clang and Celero do not play well with "-mno-sse2" flag, so add it only for non-Clang compilers.
set(FLAGS none native sse2)
if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  list(APPEND FLAGS no-sse2)
endif()

# Helper macro to add a compiler option corresponding to a flag to a given target.
macro(add_flag target flag)
  set(_opt "")
  if(${flag} STREQUAL "native")
    set(_opt "-march=native")
  elseif(NOT ${flag} STREQUAL "none")
    set(_opt "-m${flag}")
  endif()
  target_compile_options(${target} PRIVATE "${_opt}")
endmacro()

add_subdirectory(third-party)

add_library(transforms INTERFACE)
set_property(
  TARGET transforms
  PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_subdirectory(bench)
add_subdirectory(test)

message("-- Compiler: ${COMPILER}")
message("-- Flags: ${FLAGS}")
