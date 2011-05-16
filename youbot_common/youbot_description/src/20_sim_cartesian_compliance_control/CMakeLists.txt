FILE(GLOB_RECURSE 20SIM_GERNERATED_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp" )
rosbuild_add_library(20_sim_cartesian_compliance_control ${20SIM_GERNERATED_SOURCES})
#rosbuild_add_executable(20_sim_cartesian_compliance_control_test  ${20SIM_GERNERATED_SOURCES})



