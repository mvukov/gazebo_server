# Try to find the pybind11 library and headers. Sets:
#  pybind11_FOUND - system has pybind11
#  pybind11_INCLUDE_DIRS - the pybind11 include directory
#  pybind11_add_module - builds a Python module from C++ code.

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

set(_PYTHON_VERSION "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")

find_path(pybind11_INCLUDE_DIRS
  NAMES pybind11/pybind11.h
  HINTS
    ${Python3_INCLUDE_DIRS}
    /usr/include/python${_PYTHON_VERSION}
    /usr/include/python${_PYTHON_VERSION}m
    /usr/local/include/python${_PYTHON_VERSION}
    /usr/local/include/python${_PYTHON_VERSION}m
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pybind11 DEFAULT_MSG
  pybind11_INCLUDE_DIRS)

if (pybind11_FOUND)
  message(STATUS "pybind11 include folders: ${pybind11_INCLUDE_DIRS}")
  message(STATUS "Python3 include folders: ${Python3_INCLUDE_DIRS}")
  message(STATUS "Python3 libraries: ${Python3_LIBRARIES}")

  mark_as_advanced(pybind11_INCLUDE_DIRS)
else()
  message(STATUS "Failed to find pybind11!")
  return()
endif()

execute_process(
  COMMAND ${Python3_EXECUTABLE} "-c"
      "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'));"
  OUTPUT_VARIABLE PYTHON_MODULE_EXTENSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Builds a Python module from C++ code.
# The first argument is the module name and the rest are the C++ source files.
# Your catkin package' CMake script needs to execute catkin_python_setup() and
# there should be at least an initialization file __init__.py file in the
# package source folder.
function(pybind11_add_module target_name)
  add_library(${target_name} ${ARGN})

  target_include_directories(${target_name}
    PRIVATE ${pybind11_INCLUDE_DIRS}
    PRIVATE ${Python3_INCLUDE_DIRS}
  )
  target_link_libraries(${target_name}
    PRIVATE ${Python3_LIBRARIES}
  )

  set_target_properties(${target_name} PROPERTIES PREFIX "")
  set_target_properties(${target_name}
    PROPERTIES SUFFIX "${PYTHON_MODULE_EXTENSION}"
  )

  set_target_properties(${target_name}
    PROPERTIES CXX_VISIBILITY_PRESET "hidden"
  )

  set_target_properties(${target_name}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  )

  install(TARGETS ${target_name}
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  )
endfunction()
