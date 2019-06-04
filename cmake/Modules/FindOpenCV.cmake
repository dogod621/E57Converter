###########################################################
#                  Find OpenCV Library
# See http://sourceforge.net/projects/opencvlibrary/
#----------------------------------------------------------
#
## 1: Setup:
# The following variables are optionally searched for defaults
#  OpenCV_DIR:            Base directory of OpenCv tree to use.
#
## 2: Variable
# The following are set after configuration is done: 
#  
#  OpenCV_FOUND
#  OpenCV_LIBS
#  OpenCV_INCLUDE_DIR
#  OpenCV_VERSION (OpenCV_VERSION_MAJOR, OpenCV_VERSION_MINOR, OpenCV_VERSION_PATCH)
#
#
# Deprecated variable are used to maintain backward compatibility with
# the script of Jan Woetzel (2006/09): www.mip.informatik.uni-kiel.de/~jw
#  OpenCV_INCLUDE_DIRS
#  OpenCV_LIBRARIES
#  OpenCV_LINK_DIRECTORIES
# 
## 3: Version
#
# 2010/04/07 Benoit Rat, Correct a bug when OpenCVConfig.cmake is not found.
# 2010/03/24 Benoit Rat, Add compatibility for when OpenCVConfig.cmake is not found.
# 2010/03/22 Benoit Rat, Creation of the script.
#
#
# tested with:
# - OpenCV 2.1:  MinGW, MSVC2008
# - OpenCV 2.0:  MinGW, MSVC2008, GCC4
#
#
## 4: Licence:
#
# LGPL 2.1 : GNU Lesser General Public License Usage
# Alternatively, this file may be used under the terms of the GNU Lesser

# General Public License version 2.1 as published by the Free Software
# Foundation and appearing in the file LICENSE.LGPL included in the
# packaging of this file.  Please review the following information to
# ensure the GNU Lesser General Public License version 2.1 requirements
# will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
# 
#----------------------------------------------------------


find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV")

##====================================================
## Find OpenCV libraries
##----------------------------------------------------
if(EXISTS "${OpenCV_DIR}")

	#When its possible to use the Config script use it.
	if(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

		## Include the standard CMake script
		include("${OpenCV_DIR}/OpenCVConfig.cmake")

		## Search for a specific version
		set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")

	#Otherwise it try to guess it.
	else(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")
		set(ERR_MSG "OpenCV_DIR does not contain OpenCVConfig.cmake.")
	endif(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

else(EXISTS "${OpenCV_DIR}")
	set(ERR_MSG "Please specify OpenCV directory using OpenCV_DIR env. variable")
endif(EXISTS "${OpenCV_DIR}")
##====================================================


##====================================================
## Print message
##----------------------------------------------------
if(NOT OpenCV_FOUND)
  # make FIND_PACKAGE friendly
  if(NOT OpenCV_FIND_QUIETLY)
		if(OpenCV_FIND_REQUIRED)
		  message(FATAL_ERROR "OpenCV required but some headers or libs not found. ${ERR_MSG}")
		else(OpenCV_FIND_REQUIRED)
		  message(STATUS "WARNING: OpenCV was not found. ${ERR_MSG}")
		endif(OpenCV_FIND_REQUIRED)
  endif(NOT OpenCV_FIND_QUIETLY)
endif(NOT OpenCV_FOUND)
##====================================================

