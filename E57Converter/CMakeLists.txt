cmake_minimum_required( VERSION 3.2 )

# Create Project
file(GLOB e57Converter_srcs ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB e57Converter_hpps ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB e57Converter_hdrs ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_executable( E57Converter  ${e57Converter_srcs} ${e57Converter_hpps} ${e57Converter_hdrs})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libE57Format/include/)
target_link_libraries(E57Converter E57Format)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Xerces-C/src/)
target_link_libraries(E57Converter xerces-c)

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

# Install
install(FILES ${e57Converter_hpps} ${e57Converter_hdrs} DESTINATION include/E57Converter/)
install(TARGETS E57Converter
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib)