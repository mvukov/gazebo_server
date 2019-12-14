# CMake module to search for TinyXML2 library and headers. Sets:
#  TinyXML2_FOUND - system has TinyXML2.
#  TinyXML2_INCLUDE_DIRS - the TinyXML2 include directory.
#  TinyXML2_LIBRARIES - libraries to link against to use TinyXML2.

find_path(TinyXML2_INCLUDE_DIRS
  NAMES
    tinyxml2.h
  HINTS
    /usr
    /usr/local
  PATH_SUFFIXES
    include
)

find_library(TinyXML2_LIBRARIES
  NAMES
    tinyxml2
  HINTS
    /usr
    /usr/local
  PATH_SUFFIXES
    lib64 lib
    lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML2 DEFAULT_MSG
  TinyXML2_INCLUDE_DIRS TinyXML2_LIBRARIES)

if (TinyXML2_FOUND)
  message(STATUS "TinyXML2 include folders: ${TinyXML2_INCLUDE_DIRS}")
  message(STATUS "TinyXML2 libraries: ${TinyXML2_LIBRARIES}")

  mark_as_advanced(TinyXML2_INCLUDE_DIRS TinyXML2_LIBRARIES)
endif()