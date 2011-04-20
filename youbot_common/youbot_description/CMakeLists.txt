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
include_directories(	${PROJECT_SOURCE_DIR}/include
			${PROJECT_SOURCE_DIR}/src)

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

OPTION(WITH_DEBUG_INFO "Publish debug info" ON)

IF(WITH_DEBUG_INFO)
  ADD_DEFINITIONS(-DDEBUG_INFO)
ENDIF(WITH_DEBUG_INFO)

#simple publishers
rosbuild_add_executable(joint_gripper_publisher src/joint_gripper_publisher.cpp)
rosbuild_add_executable(joint_arm_publisher src/joint_arm_publisher.cpp)
rosbuild_add_executable(joint_base_publisher src/joint_base_publisher.cpp)
rosbuild_add_executable(joint_robot_publisher src/joint_robot_publisher.cpp)

#youbot_base_cartesian_position_controller
rosbuild_add_executable(youbot_keyboard_teleop_experiment examples/keyboard_teleop_experiment.cpp)

#youbot_arm_joint_position_controller
rosbuild_add_executable(youbot_arm_joint_position_controller_test examples/youbot_arm_joint_position_controller_test.cpp)

#youbot_arm_joint_velocity_controller
rosbuild_add_library(youbot_arm_joint_velocity_controller src/youbot_arm_joint_velocity_controller.cpp)
rosbuild_add_executable(youbot_arm_joint_velocity_controller_test examples/youbot_arm_joint_velocity_controller_test.cpp)

#youbot_arm_joint_torque_controller
#rosbuild_add_library(youbot_arm_joint_torque_controller src/youbot_arm_joint_torque_controller.cpp)
#rosbuild_add_executable(youbot_arm_joint_torque_controller_test examples/youbot_arm_joint_torque_controller_test.cpp)

#youbot_arm_cartesian_interaction_controller
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/20_sim_arm_interaction_control)
rosbuild_add_library(youbot_arm_cartesian_interaction_controller src/youbot_arm_cartesian_interaction_controller.cpp)
target_link_libraries(youbot_arm_cartesian_interaction_controller 20_sim_arm_interaction_controller)
rosbuild_add_executable(youbot_arm_cartesian_interaction_controller_test examples/youbot_arm_cartesian_interaction_controller_test.cpp)

#youbot_arm_cartesian_interaction_controller_gravity_compensation
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/20_sim_arm_interaction_control_gravity)
rosbuild_add_library(youbot_arm_cartesian_interaction_controller_gravity_compensation src/youbot_arm_cartesian_interaction_controller_gravity_compensation.cpp)
target_link_libraries(youbot_arm_cartesian_interaction_controller_gravity_compensation 20_sim_arm_interaction_controller_gravity_compensation)

#youbot_interaction_controller
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/20_sim_interaction_control)
rosbuild_add_library(youbot_interaction_controller src/youbot_interaction_controller.cpp)
target_link_libraries(youbot_interaction_controller 20_sim_interaction_controller)

#youbot_ros_controllers (test)
rosbuild_add_executable(youbot_arm_cartesian_pose_controller_test examples/youbot_arm_cartesian_pose_controller_test.cpp)

#depricated
rosbuild_add_executable(arm_trajectory_republisher src/arm_trajectory_republisher.cpp)

