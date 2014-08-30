#		Find SpeechTools on the development system.
# This module finds if SpeechTools is installed and determines where the
# include files and libraries are. It also determines what the name of
# the library is. This code sets the following variables:
#
#	SPEECHTOOLS_INCLUDE_DIRS	- path where festival.h is found
#	SPEECHTOOLS_LIBRARIES		- path to the Festival library
#
#======================================================================
# Copyright (c) 2014 Gabriel Araujo <gabriel.araujo.5000@gmail.com>
#
# This module is free software; you can redistribute it and/or
# modify it under the terms of the GNU Library General Public
# License as published by the Free Software Foundation; either
# version 2 of the License, or (at your option) any later version.
#
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

find_path(SPEECHTOOLS_INCLUDE_DIRS speech_tools/EST.h
		  HINTS /usr/include /usr/local/include
		  PATH_SUFFIXES speech_tools)

find_library(ESTBASE_LIBRARIES estbase
		   HINTS /usr/lib/ usr/local/lib)
find_library(ESTOOLS_LIBRARIES estools
		   HINTS /usr/lib/ usr/local/lib)
find_library(ESTSTRING_LIBRARIES eststring
		   HINTS /usr/lib/ usr/local/lib)

SET(SPEECHTOOLS_LIBRARIES ${ESTOOLS_LIBRARIES} ${ESTBASE_LIBRARIES} ${ESTSTRING_LIBRARIES})
SET(SPEECHTOOLS_INCLUDE_DIR ${SPEECHTOOLS_INCLUDE_DIRS}/speech_tools)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Festival DEFAULT_MSG
    SPEECHTOOLS_LIBRARIES SPEECHTOOLS_INCLUDE_DIRS)


mark_as_advanced(SPEECHTOOLS_INCLUDE_DIRS SPEECHTOOLS_LIBRARIES)