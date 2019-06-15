cmake_minimum_required(VERSION 2.8.3)
project(y3space_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
             geometry_msgs
             roscpp
             rospy
             sensor_msgs
             serial
             std_msgs)

find_package(Boost REQUIRED COMPONENTS system)

###################
## catkin Config ##
###################

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs
                   roscpp
                   rospy
                   sensor_msgs
                   serial
                   std_msgs)

###########
## Build ##
###########

include_directories(
    INCLUDE_DIRS include
    ${BOOST_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

add_executable(y3space_driver
               src/y3space_driver/Y3SpaceDriver.cpp
               src/main.cpp)

target_link_libraries(yostlab_driver_node
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})
