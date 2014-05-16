cmake_minimum_required(VERSION 2.8.3)
project(designator_integration)

find_package(catkin REQUIRED COMPONENTS designator_integration_msgs roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES designator_integration
  CATKIN_DEPENDS designator_integration_msgs roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(designator_integration
  src/designators/CDesignator.cpp
  src/designators/CKeyValuePair.cpp)

# ## Declare a cpp executable
# add_executable(designator_integration_node
#   src/node.cpp)

# ## Add cmake target dependencies of the executable/library
# ## as an example, message headers may need to be generated before nodes
add_dependencies(designator_integration designator_integration_msgs_gencpp)
# add_dependencies(designator_integration_node designator_integration_msgs_gencpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(designator_integration_node
#   ${catkin_LIBRARIES}
#   DesignatorIntegration)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
install(TARGETS designator_integration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_designator_integration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
