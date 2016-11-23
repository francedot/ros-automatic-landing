# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(landing_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS tf roscpp rospy std_msgs sensor_msgs message_generation message_filters)

## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs sensor_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build ar_sys_listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(vel_controller src/vel_controller.cpp)
target_link_libraries(vel_controller ${catkin_LIBRARIES})
add_dependencies(vel_controller landing_pkg_generate_messages_cpp)

add_executable(pose_to_stamped_yaw src/pose_to_stamped_yaw.cpp)
target_link_libraries(pose_to_stamped_yaw ${catkin_LIBRARIES})
add_dependencies(pose_to_stamped_yaw landing_pkg_generate_messages_cpp)

# add_executable(ar_sys_listener src/ar_sys_listener.cpp)
# target_link_libraries(ar_sys_listener ${catkin_LIBRARIES})
# add_dependencies(ar_sys_listener landing_pkg_generate_messages_cpp)

# add_executable(transform_bootstrapper src/transform_bootstrapper.cpp)
# target_link_libraries(transform_bootstrapper ${catkin_LIBRARIES})
# add_dependencies(transform_bootstrapper landing_pkg_generate_messages_cpp)

# %EndTag(FULLTEXT)%
