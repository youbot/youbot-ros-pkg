# - Try to find soem
# Once done this will define
#
#  SOEM_FOUND - soem found
#  SOEM_INCLUDE_DIR - the soem include directory
#  SOEM_LIBRARIES - soem library
#

FIND_PATH(SOEM_INCLUDE_DIR NAMES ethercatmain.h
  PATHS
  $ENV{YOUBOTDIR}/include/soem/src 
  $ENV{ROBOTPKG_BASE}/include/soem/src 
  ENV CPATH
  /usr/include/
  /usr/include/soem/src
  /usr/local/include/
  /usr/local/include/soem/src
  /opt/local/include/
  /opt/local/include/soem/src
  NO_DEFAULT_PATH
)

FIND_LIBRARY(SOEM_LIBRARIES NAMES soem
  PATHS
  $ENV{YOUBOTDIR}/lib 
  $ENV{ROBOTPKG_BASE}/lib
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OODL_YOUBOT_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Soem  DEFAULT_MSG
                                  SOEM_INCLUDE_DIR SOEM_LIBRARIES)

# show the SOEM_INCLUDE_DIR and SOEM_LIBRARY_DIR variables only in the advanced view
IF (SOEM_FOUND)
  MARK_AS_ADVANCED(SOEM_INCLUDE_DIR SOEM_LIBRARIES)
ENDIF (SOEM_FOUND)


