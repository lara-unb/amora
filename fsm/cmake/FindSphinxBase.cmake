# - Find SphinxBase on the development system.
# This module finds if SphinxBase is installed and determines where the
# include files and libraries are. It also determines what the name of
# the library is. This code sets the following variables:
#
#  SPHINXBASE_LIBRARIES           - path to the SphinxBase library
#  SPHINXAD_LIBRARIES			  - path to the Sphinxad library
#  SPHINXBASE_INCLUDE_DIRS        - path to where sphinxbase.h is found
#
#=============================================================================
# Copyright (c) 2012 Jacky Alcine <jacky.alcine@thesii.org>
#
# This module is free software; you can redistribute it and/or
# modify it under the terms of the GNU Library General Public
# License as published by the Free Software Foundation; either
# version 2 of the License, or (at your option) any later version.
#
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

find_package(PkgConfig QUIET)

pkg_check_modules(PC_SPHINXBASE sphinxbase)

find_path(SPHINXBASE_INCLUDE_DIRS cmd_ln.h
    HINTS ${PC_SPHINXBASE_INCLUDEDIR} ${PC_SPHINXBASE_INCLUDE_DIRS})
find_library(SPHINXBASE_LIBRARIES sphinxbase
    HINTS ${PC_SPHINXBASE_LIBRARY_DIRS} ${PC_SPHINXBASE_LIBDIR})
find_library(SPHINXAD_LIBRARIES sphinxad
    HINTS ${PC_SPHINXBASE_LIBRARY_DIRS} ${PC_SPHINXBASE_LIBDIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SphinxBase DEFAULT_MSG
    SPHINXBASE_LIBRARIES SPHINXBASE_INCLUDE_DIRS)

list(APPEND SPHINXBASE_INCLUDE_DIRS ${PC_SPHINXBASE_INCLUDE_DIRS})

set(SPHINXBASE_VERSION ${PC_SPHINXBASE_VERSION})

mark_as_advanced(SPHINXBASE_INCLUDE_DIRS SPHINXBASE_LIBRARIES SPHINXAD_LIBRARIES)