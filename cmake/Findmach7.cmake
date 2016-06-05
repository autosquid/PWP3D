# - Find MACH7
# Intel Threading Building Blocks offers a rich and complete approach to expressing parallelism in a C++ program
# www.threadingbuildingblocks.org
#
# The module defines the following variables:
#  MACH7_FOUND - the system has MACH7
#  MACH7_INCLUDE_DIR - where to find MACH7.h
#  MACH7_INCLUDE_DIRS - MACH7 includes
#  MACH7_LIBRARY - where to find the MACH7 library
#  MACH7_LIBRARIES - aditional libraries

set(MACH7_ROOT_DIR ${MACH7_DIR})

# set MACH7_INCLUDE_DIR
find_path (MACH7_INCLUDE_DIR
  NAMES
    Mach7/match.hpp
  PATHS
    "${MACH7_DIR}/include"
  DOC
    "MACH7 include directory"
)

# set MACH7_INCLUDE_DIRS
set (MACH7_INCLUDE_DIRS ${MACH7_INCLUDE_DIR})


IF(MACH7_INCLUDE_DIR)
  SET(MACH7_FOUND TRUE)
ENDIF(MACH7_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args (MACH7 DEFAULT_MSG
  MACH7_INCLUDE_DIR
  MACH7_INCLUDE_DIRS
  MACH7_ROOT_DIR
)


mark_as_advanced (
  MACH7_INCLUDE_DIR
  MACH7_INCLUDE_DIRS
  MACH7_ROOT_DIR
)
