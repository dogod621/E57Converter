cmake_minimum_required( VERSION 3.2 )

# Create Project
project( E57Converter )

#
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set ( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT OFF )
	set(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME})
	#SET(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../install/" CACHE PATH "${CMAKE_CURRENT_SOURCE_DIR}/../install/" FORCE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

# Use folders (for IDE project grouping)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#
if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Xerces-C")
  message(FATAL_ERROR "The Xerces-C submodule directory is missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by running \"git submodule update --init --recursive\"")
endif()
if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libE57Format")
  message(FATAL_ERROR "The libE57Format submodule directory is missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by running \"git submodule update --init --recursive\"")
endif()

# XercesC
add_subdirectory (Xerces-C)

# XercesC Documentation
set(XercesC_Documentation_TARGETS)
list(APPEND XercesC_Documentation_TARGETS doc-xml doc-style)
if(TARGET createdocs)
	list(APPEND XercesC_Documentation_TARGETS createdocs)
endif()
if(TARGET createapidocs)
	list(APPEND XercesC_Documentation_TARGETS createapidocs)
endif()
set_target_properties(${XercesC_Documentation_TARGETS} PROPERTIES COMPILE_FLAGS "/wd4819" FOLDER "XercesC/Documentation")

# XercesC Library
set(XercesC_Library_TARGETS)
list(APPEND XercesC_Library_TARGETS xerces-c-headers xerces-c-missing xerces-c)
if(TARGET genicu)
	list(APPEND XercesC_Library_TARGETS genicu)
endif()
if(TARGET gencat)
	list(APPEND XercesC_Library_TARGETS gencat)
endif()
set_target_properties(${XercesC_Library_TARGETS} PROPERTIES COMPILE_FLAGS "/wd4819" FOLDER "XercesC/Library")

# XercesC Tests
set(XercesC_Tests_TARGETS)
list(APPEND XercesC_Tests_TARGETS 
	DOMTest
	DOMMemTest
	Normalizer
	RangeTest
	DOMTraversalTest
	DOMTypeInfoTest
	EncodingTest
	InitTermTest
	MemHandlerTest
	NetAccessorTest
	XSerializerTest
	XSTSHarness
	XSValueTest)
if(TARGET ThreadTest)
	list(APPEND XercesC_Tests_TARGETS ThreadTest)
endif()
if(TARGET Char16Test)
	list(APPEND XercesC_Tests_TARGETS Char16Test)
endif()
set_target_properties(${XercesC_Tests_TARGETS} PROPERTIES COMPILE_FLAGS "/wd4819" FOLDER "XercesC/Tests")

# XercesC Samples
set(XercesC_Samples_TARGETS)
list(APPEND XercesC_Samples_TARGETS 
	CreateDOMDocument
	DOMCount
	DOMPrint
	EnumVal
	MemParse
	PParse
	PSVIWriter
	Redirect
	SAX2Count
	SAX2Print
	SAXCount
	SAXPrint
	SCMPrint
	SEnumVal
	StdInParse
	XInclude)
set_target_properties(${XercesC_Samples_TARGETS} PROPERTIES COMPILE_FLAGS "/wd4819" FOLDER "XercesC/Samples")

# E57
if ( WIN32 )
	set(USING_STATIC_XERCES OFF CACHE BOOL " " FORCE)
endif()
add_subdirectory (libE57Format)
set(libE57Format_TARGETS)
list(APPEND libE57Format_TARGETS E57Format)
set_target_properties(${libE57Format_TARGETS} PROPERTIES COMPILE_FLAGS "/wd4819" FOLDER "libE57Format")

# E57Converter
add_subdirectory("E57Converter")

