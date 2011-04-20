FILE(GLOB_RECURSE 20SIM_GERNERATED_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp" )
#MESSAGE("20Sim generated sources: ${20SIM_GERNERATED_SOURCES}")
rosbuild_add_library(20_sim_interaction_controller ${20SIM_GERNERATED_SOURCES})
rosbuild_add_executable(20_sim_interaction_controller_test  ${20SIM_GERNERATED_SOURCES})



