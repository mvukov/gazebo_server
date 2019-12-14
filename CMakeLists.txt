cmake_minimum_required(VERSION 3.12)

cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0054 NEW)

project(gazebo_server)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(gazebo 9 REQUIRED)
find_package(pybind11 QUIET)

catkin_python_setup()
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    EIGEN3
    GAZEBO
    TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(SERVER_LIBRARIES
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_compile_options(-std=c++17 -Wall -Wextra -Werror)

add_library(${PROJECT_NAME}
  src/gazebo_server.cpp
  src/helpers.cpp
  src/joint.cpp
  src/link.cpp
)
target_link_libraries(${PROJECT_NAME} ${SERVER_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_add_env_hooks(99.gazebo_server
  SHELLS sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

if (pybind11_FOUND)
  pybind11_add_module(py_gazebo_server
    src/py_gazebo_server.cpp
  )
  target_link_libraries(py_gazebo_server PRIVATE
      ${PROJECT_NAME} ${SERVER_LIBRARIES})
endif()

if (CATKIN_ENABLE_TESTING)
  set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test_data")

  catkin_add_gtest(test_gazebo_server test/test_gazebo_server.cpp)
  target_link_libraries(test_gazebo_server
    ${PROJECT_NAME}
    ${SERVER_LIBRARIES}
  )
  target_compile_definitions(test_gazebo_server PRIVATE
    -DTEST_DATA_PATH="${TEST_DATA_PATH}")

  catkin_add_nosetests(test/test_gazebo_server.py)
endif()
