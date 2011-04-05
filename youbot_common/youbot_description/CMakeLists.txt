cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/include)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(joint_velocity_controller src/joint_velocity_controller.cpp)
rosbuild_add_executable(joint_base_publisher src/joint_base_publisher.cpp)
rosbuild_add_executable(joint_arm_publisher src/joint_arm_publisher.cpp)
rosbuild_add_executable(joint_gripper_publisher src/joint_gripper_publisher.cpp)
rosbuild_add_executable(joint_robot_publisher src/joint_robot_publisher.cpp)
rosbuild_add_executable(arm_trajectory_republisher src/arm_trajectory_republisher.cpp)
rosbuild_add_executable(base_command_republisher src/base_command_republisher.cpp)
rosbuild_add_executable(youbot_arm_test examples/youbot_arm_test.cpp)
rosbuild_add_executable(youbot_arm_velocity_test examples/youbot_arm_velocity_test.cpp)
rosbuild_add_executable(youbot_keyboard_teleop_experiment examples/keyboard_teleop_experiment.cpp)


