cmake_minimum_required(VERSION 2.8.3)
project(designator_integration_cpp)

find_package(catkin REQUIRED COMPONENTS designator_integration_msgs roscpp)

set(COMPILE_DEBUG 0)

if(CMAKE_COMPILER_IS_GNUCXX)
  if(COMPILE_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pg")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O2")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES designator_integration
  CATKIN_DEPENDS designator_integration_msgs roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(designator_integration
  src/designators/Designator.cpp
  src/designators/KeyValuePair.cpp)

add_dependencies(designator_integration designator_integration_msgs_gencpp)

install(TARGETS designator_integration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/designators/
  DESTINATION include/designators/
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
