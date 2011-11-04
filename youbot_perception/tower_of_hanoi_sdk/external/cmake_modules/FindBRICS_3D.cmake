# - Try to find BRICS_3D
# Once done this will define
#
#  BRICS_3D_FOUND - if BRICS_3D was found
#  BRICS_3D_INCLUDE_DIRS - the BRICS_3D include directory (-I)
#  BRICS_3D_LINK_DIRECTORIES - the BRICS_3D linker directories (-L)
#  BRICS_3D_LIBRARIES - BRICS_3D libraries
#
# You can set an environment variable "BRICS_3D_DIR" to help CMake to find the BRICS_3D library,
# in case it is not installed in one of the standard paths.
#

FIND_PATH(BRICS_3D_INCLUDE_DIRS NAMES core/Point3D.h
  PATHS
  $ENV{BRICS_3D_DIR}/include/brics3d
  $ENV{BRICS_3D_DIR}/include
  $ENV{BRICS_3D_DIR}/src  
  $ENV{ROBOTPKG_BASE}/include/brics3d
  ENV CPATH
  /usr/include/brics3d
  /usr/local/include/brics3d
  /opt/local/include/brics3d
  NO_DEFAULT_PATH
)

FIND_LIBRARY(BRICS_3D_CORE_LIBRARY NAMES "brics3d_core" 
  PATHS
  $ENV{BRICS_3D_DIR}/lib/brics3d 
  $ENV{BRICS_3D_DIR}/lib 
  $ENV{ROBOTPKG_BASE}/lib/brics3d
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib/brics3d
  /usr/local/lib/brics3d
  /opt/local/lib/brics3d
  NO_DEFAULT_PATH
)

FIND_LIBRARY(BRICS_3D_ALGORITHM_LIBRARY NAMES "brics3d_algorithm" 
  PATHS
  $ENV{BRICS_3D_DIR}/lib/brics3d 
  $ENV{BRICS_3D_DIR}/lib 
  $ENV{ROBOTPKG_BASE}/lib/brics3d
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib/brics3d
  /usr/local/lib/brics3d
  /opt/local/lib/brics3d
  NO_DEFAULT_PATH
)

FIND_LIBRARY(BRICS_3D_UTIL_LIBRARY NAMES "brics3d_util" 
  PATHS
  $ENV{BRICS_3D_DIR}/lib/brics3d 
  $ENV{BRICS_3D_DIR}/lib 
  $ENV{ROBOTPKG_BASE}/lib/brics3d
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib/brics3d
  /usr/local/lib/brics3d
  /opt/local/lib/brics3d
  NO_DEFAULT_PATH
)

FIND_LIBRARY(BRICS_3D_WORLD_MODEL_LIBRARY NAMES "brics3d_world_model" 
  PATHS
  $ENV{BRICS_3D_DIR}/lib/brics3d 
  $ENV{BRICS_3D_DIR}/lib 
  $ENV{ROBOTPKG_BASE}/lib/brics3d
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib/brics3d
  /usr/local/lib/brics3d
  /opt/local/lib/brics3d
  NO_DEFAULT_PATH
)

IF(BRICS_3D_CORE_LIBRARY)
  GET_FILENAME_COMPONENT( BRICS_3D_LINK_DIRECTORIES ${BRICS_3D_CORE_LIBRARY} PATH CACHE)
ENDIF(BRICS_3D_CORE_LIBRARY)


SET(BRICS_3D_LIBRARIES_TMP
)

IF(BRICS_3D_WORLD_MODEL_LIBRARY)
  LIST(APPEND BRICS_3D_LIBRARIES_TMP brics3d_world_model)
ENDIF(BRICS_3D_WORLD_MODEL_LIBRARY)

IF(BRICS_3D_ALGORITHM_LIBRARY)
  LIST(APPEND BRICS_3D_LIBRARIES_TMP brics3d_algorithm scanlib 6dslam_core newmat_cmake flann_s ANN)
ENDIF(BRICS_3D_ALGORITHM_LIBRARY)

IF(BRICS_3D_UTIL_LIBRARY)
  LIST(APPEND BRICS_3D_LIBRARIES_TMP brics3d_util)
ENDIF(BRICS_3D_UTIL_LIBRARY)

IF(BRICS_3D_CORE_LIBRARY)
  LIST(APPEND BRICS_3D_LIBRARIES_TMP brics3d_core)
ENDIF(BRICS_3D_CORE_LIBRARY)


SET(BRICS_3D_LIBRARIES
    ${BRICS_3D_LIBRARIES_TMP}
    CACHE STRING "Accumulated BRICS_3D libraries."
)


include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set BRICS_3D_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(BRICS_3D  DEFAULT_MSG
                                  BRICS_3D_INCLUDE_DIRS BRICS_3D_LINK_DIRECTORIES BRICS_3D_LIBRARIES)

# show the BRICS_3D_INCLUDE_DIRS and BRICS_3D_LIBRARIES variables only in the advanced view
IF(BRICS_3D_FOUND)
  MARK_AS_ADVANCED(BRICS_3D_INCLUDE_DIRS BRICS_3D_LINK_DIRECTORIES BRICS_3D_LIBRARIES)
ENDIF(BRICS_3D_FOUND)

