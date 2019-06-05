cmake_minimum_required( VERSION 3.2 )

#
option(POINT_E57_WITH_RGB "E57 per-point data can contain scanned RGB or not" ON)
option(POINT_E57_WITH_INTENSITY "E57 per-point data can contain scanned intensity or not" ON)
option(POINT_E57_WITH_LABEL "E57 per-point data can contain scan index or not" ON)
option(POINT_E57_WITH_HDR "E57 per-point data can contain scanned HDR RGB or not" ON)
option(POINT_PCD_WITH_RGB "PCD per-point data can contain RGB or not" ON)
option(POINT_PCD_WITH_INTENSITY "PCD per-point data can contain intensity or not" ON)
option(POINT_PCD_WITH_NORMAL "PCD per-point data can contain normal or not" ON)
option(POINT_PCD_WITH_LABEL "PCD per-point data can contain label or not" ON)

# Create Project
file(GLOB e57Converter_srcs ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB e57Converter_hpps ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB e57Converter_hdrs ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_executable( E57Converter  ${e57Converter_srcs} ${e57Converter_hpps} ${e57Converter_hdrs})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libE57Format/include/)
target_link_libraries(E57Converter E57Format)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Xerces-C/src/)
target_link_libraries(E57Converter xerces-c)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Json/include/)
target_link_libraries(E57Converter nlohmann_json::nlohmann_json)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Half/half/)
target_link_libraries(E57Converter half)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../E57Converter/)

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

# OpenCV
find_package( OpenCV 4.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( E57Converter ${OpenCV_LIBS} )

#
if ( ${POINT_E57_WITH_RGB} )
	add_definitions(-DPOINT_E57_WITH_RGB)
endif()

if ( ${POINT_E57_WITH_INTENSITY} )
	add_definitions(-DPOINT_E57_WITH_INTENSITY)
endif()

if ( ${POINT_E57_WITH_LABEL} )
	add_definitions(-DPOINT_E57_WITH_LABEL)
endif()

if ( ${POINT_E57_WITH_HDR} )
	add_definitions(-DPOINT_E57_WITH_HDR)
endif()

if ( ${POINT_PCD_WITH_RGB} )
	add_definitions(-DPOINT_PCD_WITH_RGB)
endif()

if ( ${POINT_PCD_WITH_INTENSITY} )
	add_definitions(-DPOINT_PCD_WITH_INTENSITY)
endif()

if ( ${POINT_PCD_WITH_NORMAL} )
	add_definitions(-DPOINT_PCD_WITH_NORMAL)
endif()

if ( ${POINT_PCD_WITH_LABEL} )
	add_definitions(-DPOINT_PCD_WITH_LABEL)
endif()

# Install
install(FILES ${e57Converter_hpps} ${e57Converter_hdrs} DESTINATION include/E57Converter/)
install(TARGETS E57Converter
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib)