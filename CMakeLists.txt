cmake_minimum_required(VERSION 2.8.3)
project(kjq_navigation)

## Use C++11
add_definitions(-std=c++11)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    grid_map_core
    grid_map_ros
    grid_map_msgs
)

## Find system libraries
find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
   ${EIGEN3_INCLUDE_DIR}
#   LIBRARIES
#     ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    nav_msgs
    grid_map_core
    grid_map_ros
    grid_map_msgs
    roscpp
    sensor_msgs
#  DEPENDS
#    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
#  ${Boost_INCLUDE_DIRS}
)



# Generate Core Cpp library
add_library(${PROJECT_NAME}_core
   src/AStarPlanner.cpp
   src/LocalPlanner.cpp
)

add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)


# Generate nodes
add_executable(random_drive
    src/RandomDriveNode.cpp
)

add_executable(a_star_test
    src/AStarTestNode.cpp
)

add_executable(pi_bridge
    src/PiBridgeNode.cpp
)

add_executable(movement
    src/MovementNode.cpp
)

#
add_dependencies(random_drive
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(a_star_test
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(pi_bridge
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(movement
    ${catkin_EXPORTED_TARGETS}
)

#
target_link_libraries(random_drive
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
)

target_link_libraries(a_star_test
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
)

target_link_libraries(pi_bridge
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
)

target_link_libraries(movement
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# # Mark executables and/or libraries for installation
# install(
#   TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
# )

# # Mark other files for installation
# install(
#   DIRECTORY doc
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# #############
# ## Testing ##
# #############

if(CATKIN_ENABLE_TESTING)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    ## Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test
        test/test_kjq_navigation.cpp)
    #   test/AlgorithmTest.cpp)
endif()




if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif()