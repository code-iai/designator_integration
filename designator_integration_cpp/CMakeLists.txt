cmake_minimum_required(VERSION 2.8.3)
project(designator_integration_cpp)

find_package(catkin REQUIRED COMPONENTS designator_integration_msgs roscpp)

find_package(RapidJSON QUIET)

message(${RapidJSON_VERSION})
if(${RapidJSON_FOUND})
  if(${RapidJSON_VERSION}i EQUAL "1.0.0")
    set(WITH_RAPIDJSON True)
    message("Found good version of RapidJSON(1.0.0)")
   else()
    set(WITH_RAPIDJSON False)
    message("Wrong version of RapidJSON")	  
  endif()
endif()

message(${RAPIDJSON_INCLUDE_DIRS})
set(COMPILE_DEBUG 0)

if(CMAKE_COMPILER_IS_GNUCXX)
  if(COMPILE_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
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

if(${WITH_RAPIDJSON})
  include_directories(${RAPIDJSON_INCLUDE_DIRS})
endif()

configure_file(${PROJECT_SOURCE_DIR}/config.h.in ${PROJECT_SOURCE_DIR}/include/designators/config.h @ONLY)

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
