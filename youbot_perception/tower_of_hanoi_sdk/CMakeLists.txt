cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#Set the path for external CMAKE modules
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")

FIND_PACKAGE(BRICS_3D REQUIRED)

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


rosbuild_genmsg()
rosbuild_gensrv()

ADD_DEFINITIONS(-DEIGEN3)

# add library directories (-L)
LINK_DIRECTORIES(
#    ${BRICS_3D_ALGOTITHM_LIBRARY}
#    ${BRICS_3D_CORE_LIBRARY}
#    ${BRICS_3D_UTIL_LIBRARY}
    ${BRICS_3D_LINK_DIRECTORIES}
    
)

# add include directories (-I)
INCLUDE_DIRECTORIES(
	 ${BRICS_3D_INCLUDE_DIRS}
     ${PROJECT_SOURCE_DIR}/lib
)




#ADD_SUBDIRECTORY(./lib) #compile everything in lib to create the libraries

#executable for color based region of interest extraction
rosbuild_add_executable(colorBasedRoiExtractor src/ColorBasedRoiExtractionNode.cpp 
						src/examples/ColorBasedRoiExtractor)
target_link_libraries(colorBasedRoiExtractor brics3d_util.a brics3d_algorithm brics3d_core)

#executable for hsv configuration finder
rosbuild_add_executable(hsvLimitsFinder src/HSVLimitsFinderNode.cpp 
						src/examples/ColorBasedRoiExtractor)
target_link_libraries(hsvLimitsFinder  brics3d_util brics3d_algorithm brics3d_core)

#executable for object cluster extraction
rosbuild_add_executable(objectClusterExtractor src/ObjectClusterExtractionNode.cpp 
						src/examples/EuclideanClusterExtractor lib/EuclideanClustering)
target_link_libraries(objectClusterExtractor   
										brics3d_util brics3d_algorithm brics3d_core)

#executable for Model Fitting using ICP
rosbuild_add_executable(modelFittingICP src/ModelFittingICPNode.cpp 
						src/examples/ModelFitting lib/IterativeClosestPoint)
target_link_libraries(modelFittingICP   
										brics3d_util brics3d_algorithm brics3d_core)										

										
#executable for 6D Pose Estimation
rosbuild_add_executable(poseEstimator6D src/PoseEstimation6DNode.cpp 
						src/examples/PoseEstimation6D lib/IterativeClosestPoint lib/EuclideanClustering)
target_link_libraries(poseEstimator6D   
										brics3d_util brics3d_algorithm brics3d_core)
										
#executable for controlling the perception system
rosbuild_add_executable(perceptionEngineController src/ControlPerception.cpp)

#executable for configuring the perception system
rosbuild_add_executable(perceptionEngineConfigurator src/ConfigurePerception.cpp)

#executable for 3D world model
rosbuild_add_executable(worldModel src/WorldModelNode.cpp )
target_link_libraries(worldModel brics3d_world_model brics3d_util brics3d_algorithm brics3d_core)

#examples for 3D world model
rosbuild_add_executable(worldModelSampleQuery src/examples/WorldModelSampleQuery.cpp )
rosbuild_add_executable(worldModelVisualizer src/examples/WorldModelVisualizer.cpp )