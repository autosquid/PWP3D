# Copyright (c) 2008, 2014 OpenCog.org (http://opencog.org)
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# - Try to find Guile; Once done this will define
#
# GUILE_FOUND - system has the GUILE library
# GUILE_INCLUDE_DIRS - the GUILE include directory
# GUILE_LIBRARIES - The libraries needed to use GUILE

# - Try to find Guile2
# Once done this will define
#  GUILE2_FOUND - System has Guile2
#  GUILE2_INCLUDE_DIRS - The Guile2 include directories
#  GUILE2_LIBRARIES - The libraries needed to use Guile2
#  GUILE2_DEFINITIONS - Compiler switches required for using Guile2

find_package(PkgConfig)
pkg_check_modules(PC_GUILE QUIET guile-2.0)
set(GUILE2_DEFINITIONS ${PC_GUILE_CFLAGS_OTHER})

find_path(GUILE2_INCLUDE_DIR libguile.h
          HINTS ${PC_GUILE_INCLUDEDIR} ${PC_GUILE_INCLUDE_DIRS})

find_library(GUILE2_LIBRARY NAMES guile-2.0
             HINTS ${PC_GUILE_LIBDIR} ${PC_GUILE_LIBRARY_DIRS} )

set(GUILE2_LIBRARIES ${GUILE2_LIBRARY} )
set(GUILE2_INCLUDE_DIRS ${GUILE2_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set GUILE2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Guile2  DEFAULT_MSG
                                  GUILE2_LIBRARY GUILE2_INCLUDE_DIR)

mark_as_advanced(GUILE2_INCLUDE_DIR GUILE2_LIBRARY )
