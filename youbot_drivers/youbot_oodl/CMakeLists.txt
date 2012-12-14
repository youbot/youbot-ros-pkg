cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## check required prerequisites
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system)

rosbuild_find_ros_package(soem)
rosbuild_find_ros_package(youbot_driver)
#FIND_PACKAGE(Soem REQUIRED)
#FIND_PACKAGE(OODL_YouBot REQUIRED)

## preprocessor definitions
#ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR="${OODL_YOUBOT_CONFIG_DIR}/")
ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR="${youbot_driver_PACKAGE_PATH}/config/")


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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

## Set Compilter flags
INCLUDE_DIRECTORIES(
	${OODL_YOUBOT_INCLUDE_DIR}
	${SOEM_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
)	


rosbuild_add_executable(youbot_oodl 
    src/youbot_oodl.cpp 
    src/YouBotOODLWrapper.cpp
    src/YouBotConfiguration.cpp
    src/joint_state_observer_oodl.cpp
)

target_link_libraries(youbot_oodl ${OODL_YOUBOT_LIBRARIES} ${SOEM_LIBRARIES} ${Boost_LIBRARIES} youbot_trajectory_action_server)

OPTION(USE_SETCAP "Set permissions to access ethernet interface without sudo" ON)

SET(SUDO_COMMAND gksudo)
IF($ENV{USE_NORMAL_SUDO})
    SET(SUDO_COMMAND sudo)
ENDIF($ENV{USE_NORMAL_SUDO})

IF(USE_SETCAP)
add_custom_command(TARGET youbot_oodl POST_BUILD
    #COMMAND sudo setcap cap_net_raw+ep youbot_oodl
    COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep youbot_oodl
    WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
)    
endif(USE_SETCAP)

rosbuild_add_executable(youbot_arm_test 
    examples/youbot_arm_test.cpp 
)

rosbuild_add_executable(youbot_2nd_arm_test 
    examples/youbot_2nd_arm_test.cpp 
)

#rosbuild_add_executable(youbot_arm_velocity_test 
#    examples/youbot_arm_velocity_test.cpp 
#)
