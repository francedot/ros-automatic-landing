# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(landing_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS tf roscpp rospy std_msgs sensor_msgs message_generation message_filters actionlib_msgs actionlib)

## Declare ROS messages and services
add_message_files(FILES ErrorStamped.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs sensor_msgs)

# Before catkin_package!
# find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES AutoLanding.action)

## Generate added messages and services
generate_messages(DEPENDENCIES actionlib_msgs std_msgs sensor_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS actionlib_msgs)

## Build ar_sys_listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(boot src/boot.cpp)
target_link_libraries(boot ${catkin_LIBRARIES})
add_dependencies(boot landing_pkg_generate_messages_cpp)

add_executable(main_controller src/main_controller.cpp)
target_link_libraries(main_controller ${catkin_LIBRARIES})
add_dependencies(main_controller landing_pkg_generate_messages_cpp)

add_executable(pose_to_stamped_yaw src/pose_to_stamped_yaw.cpp)
target_link_libraries(pose_to_stamped_yaw ${catkin_LIBRARIES})
add_dependencies(pose_to_stamped_yaw landing_pkg_generate_messages_cpp)

add_executable(error_monitor src/error_monitor.cpp)
target_link_libraries(error_monitor ${catkin_LIBRARIES})
add_dependencies(error_monitor landing_pkg_generate_messages_cpp)

add_executable(quota_monitor src/quota_monitor.cpp)
target_link_libraries(quota_monitor ${catkin_LIBRARIES})
add_dependencies(quota_monitor landing_pkg_generate_messages_cpp)

add_executable(mock_data_publisher src/mock_data_publisher.cpp)
target_link_libraries(mock_data_publisher ${catkin_LIBRARIES})
add_dependencies(mock_data_publisher landing_pkg_generate_messages_cpp)

add_executable(lander src/lander.cpp)
target_link_libraries(lander ${catkin_LIBRARIES})
add_dependencies(lander landing_pkg_generate_messages_cpp)

# add_executable(ar_sys_listener src/ar_sys_listener.cpp)
# target_link_libraries(ar_sys_listener ${catkin_LIBRARIES})
# add_dependencies(ar_sys_listener landing_pkg_generate_messages_cpp)

# add_executable(transform_bootstrapper src/transform_bootstrapper.cpp)
# target_link_libraries(transform_bootstrapper ${catkin_LIBRARIES})
# add_dependencies(transform_bootstrapper landing_pkg_generate_messages_cpp)

# %EndTag(FULLTEXT)%
