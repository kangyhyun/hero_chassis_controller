cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    controller_interface
    hardware_interface
        pluginlib
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        controller_interface
        hardware_interface
        pluginlib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Add dependencies to exported targets
add_library(${PROJECT_NAME} SHARED
        src/hero_chassis_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark other files for installation
install(
        FILES hero_chassis_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############
## Testing ##
#############

#[[
if (${CATKIN_ENABLE_TESTING})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
      test/test_ros_package_template.cpp
      test/AlgorithmTest.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)

endif ()
]]

##########################
## Static code analysis ##
##########################

