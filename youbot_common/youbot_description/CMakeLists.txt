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

OPTION(WITH_DEBUG_INFO "Publish debug info" ON)	#TODO:: move debug parameter to the yaml file

IF(WITH_DEBUG_INFO)
  ADD_DEFINITIONS(-DDEBUG_INFO)
ENDIF(WITH_DEBUG_INFO)

#youbot base teleoperation experiment 
rosbuild_add_executable(youbot_keyboard_teleop_experiment examples/keyboard_teleop_experiment.cpp)

#test for a joint position control of the youbot arm
rosbuild_add_library(joint_position_control src/joint_position_control.cpp)

#youbot universal controller: joint space position control, joint velocity control, joint torque control
rosbuild_add_library(youbot_universal_control src/youbot_universal_control.cpp src/joint_state_observer_gazebo.cpp)
target_link_libraries(youbot_universal_control youbot_trajectory_action_server)
rosbuild_add_executable(arm_joint_position_control_test examples/arm_joint_position_control_test.cpp)
rosbuild_add_executable(arm_joint_velocity_control_test examples/arm_joint_velocity_control_test.cpp)
rosbuild_add_executable(arm_joint_torque_control_test examples/arm_joint_torque_control_test.cpp)

#cartesian compliance control for the unified youbot arm and base system
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/20_sim_cartesian_compliance_control)
#rosbuild_add_library(cartesian_compliance_control src/cartesian_compliance_control.cpp)
#target_link_libraries(cartesian_compliance_control 20_sim_cartesian_compliance_control)
rosbuild_add_executable(cartesian_compliance_control_test examples/cartesian_compliance_control_test.cpp)

#static joint states publisher
rosbuild_add_executable(static_joint_publisher src/static_joint_publisher.cpp)

#depricated
rosbuild_add_executable(base_command_republisher src/base_command_republisher.cpp)

