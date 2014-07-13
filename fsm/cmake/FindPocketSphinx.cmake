# - Find PocketSphinx on the development system.
# This module finds if PocketSphinx is installed and determines where the
# include files and libraries are. It also determines what the name of
# the library is. This code sets the following variables:
#
#  POCKETSPHINX_LIBRARIES           - path to the PocketSphinx library
#  POCKETSPHINX_INCLUDE_DIRS        - path to where pocketsphinx.h is found
#  POCKETSPHINX_MODELDIR            - path to where preinstalled models would
#                                      be found
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
pkg_check_modules(PC_POCKETSPHINX pocketsphinx)

find_path(POCKETSPHINX_INCLUDE_DIRS pocketsphinx.h
    HINTS ${PC_POCKETSPHINX_INCLUDEDIR} ${PC_POCKETSPHINX_INCLUDE_DIRS}
    PATH_SUFFIXES pocketsphinx)
find_library(POCKETSPHINX_LIBRARIES pocketsphinx
    HINTS ${PC_POCKETSPHINX_LIBRARY_DIRS} ${PC_POCKETSPHINX_LIBDIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PocketSphinx DEFAULT_MSG
    POCKETSPHINX_LIBRARIES POCKETSPHINX_INCLUDE_DIRS)

list(APPEND POCKETSPHINX_INCLUDE_DIRS ${PC_POCKETSPHINX_INCLUDE_DIRS})

mark_as_advanced(POCKETSPHINX_INCLUDE_DIRS POCKETSPHINX_LIBRARIES)

execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} pocketsphinx --variable=modeldir
                OUTPUT_VARIABLE POCKETSPHINX_MODELDIR
                OUTPUT_STRIP_TRAILING_WHITESPACE)

set(POCKETSPHINX_VERSION ${PC_POCKETSPHINX_VERSION})