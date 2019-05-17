cmake_minimum_required( VERSION 3.2 )

# Create Project
project( E57Converter )

file(GLOB srcs ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB hdrs ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_executable( E57Converter  ${srcs} ${hdrs})

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set ( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT OFF )
	set(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME})
	#SET(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../install/" CACHE PATH "${CMAKE_CURRENT_SOURCE_DIR}/../install/" FORCE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# E57
if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libE57Format")
  message(FATAL_ERROR "The libE57Format submodule directory is missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by running \"git submodule update --init --recursive\"")
endif()

#set ( XercesC_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Dependency/Xerces-C/install/include" )
#set ( XercesC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../Dependency/Xerces-C/install/lib/xerces-c_3.lib" )
#set ( XercesC_VERSION 3 )

add_subdirectory (libE57Format)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libE57Format/include)
target_link_libraries(E57Converter ${XercesC_LIBRARY})
target_link_libraries(E57Converter E57Format)

# PCL
find_package( PCL 1.9 REQUIRED )

if( PCL_FOUND )
	# Additional Include Directories
	# [C/C++]>[General]>[Additional Include Directories]
	include_directories( ${PCL_INCLUDE_DIRS} )

	# Preprocessor Definitions
	# [C/C++]>[Preprocessor]>[Preprocessor Definitions]
	add_definitions( ${PCL_DEFINITIONS} )
	#add_definitions( -DPCL_NO_PRECOMPILE )

	# Additional Library Directories
	# [Linker]>[General]>[Additional Library Directories]
	link_directories( ${PCL_LIBRARY_DIRS} )

	# Additional Dependencies
	# [Linker]>[Input]>[Additional Dependencies]
	target_link_libraries( E57Converter ${PCL_LIBRARIES} )
endif()

#
install(TARGETS E57Converter
		RUNTIME DESTINATION bin)