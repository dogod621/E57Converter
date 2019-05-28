# Just for correctlly compile libE57Format, replace original FindXercesC

set ( XercesC_FOUND True )
set ( XercesC_INCLUDE_DIR 
	"${CMAKE_SOURCE_DIR}/Xerces-C/src/" 
	"${CMAKE_BINARY_DIR}/Xerces-C/src/")
