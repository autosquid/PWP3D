cmake_minimum_required( VERSION 2.8 )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")



set(3rd_root  "${CMAKE_CURRENT_SOURCE_DIR}/3rd")



set(ARMADILLO_DIR ${3rd_root}/armadillo-6.700.6)



find_package(Armadillo REQUIRED)

add_definitions("-DARMA_DONT_USE_CXX11")
include_directories(
  ${ARMADILLO_INCLUDE_DIRS}
  )


# Build Configures
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall ")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()


SET( CMAKE_SHARED_LINKER_FLAGS ${CMAKE_SHARED_LINKER_FLAGS} "/NODEFAULTLIBS:LIBCMT" )

option(CUDPP_BUILD_SHARED_LIBS
  "On to build shared libraries, off for static libraries."
  ON
  )

# Platform configuration vars
include(SetPlatformVars)

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
if( NOT CMAKE_BUILD_TYPE )
   set( CMAKE_BUILD_TYPE Release)
endif()

option(BUILD_SHARED_LIBS "Build Shared Library" ON)
add_subdirectory(PerseusLib)

