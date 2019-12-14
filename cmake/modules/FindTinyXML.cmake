# CMake module to search for TinyXML library and headers. Sets:
#  TinyXML_FOUND - system has TinyXML.
#  TinyXML_INCLUDE_DIRS - the TinyXML include directory.
#  TinyXML_LIBRARIES - libraries to link against to use TinyXML.

find_path(TinyXML_INCLUDE_DIRS
  NAMES
    tinyxml.h
  HINTS
    /usr
    /usr/local
  PATH_SUFFIXES
    include
)

find_library(TinyXML_LIBRARIES
  NAMES
    tinyxml
  HINTS
    /usr
    /usr/local
  PATH_SUFFIXES
    lib64 lib
    lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML DEFAULT_MSG
  TinyXML_INCLUDE_DIRS TinyXML_LIBRARIES)

if (TinyXML_FOUND)
  message(STATUS "TinyXML include folders: ${TinyXML_INCLUDE_DIRS}")
  message(STATUS "TinyXML libraries: ${TinyXML_LIBRARIES}")

  mark_as_advanced(TinyXML_INCLUDE_DIRS TinyXML_LIBRARIES)
endif()