# - Try to find OODL_YouBot
# Once done this will define
#
#  OODL_YOUBOT_FOUND - if OODL_YouBot was found
#  OODL_YOUBOT_INCLUDE_DIR - the OODL_YouBot include directory
#  OODL_YOUBOT_LIBRARIES - OODL_YouBot libraries
#  OODL_YOUBOT_CONFIG_DIR - OODL_YouBot configuration directory
#
#
# The configuration directory OODL_YOUBOT_CONFIG_DIR will always be
# set, either to a directory containing the "youbout-ethercat.cfg" file,
# or if no such file is found, to the local folder ".". In the last
# case a warning message is  issued.
#

FIND_PATH(OODL_YOUBOT_INCLUDE_DIR NAMES youbot/YouBotBase.hpp
  PATHS
  $ENV{YOUBOTDIR}/ 
  $ENV{ROBOTPKG_BASE}/include/youbot/ 
  ${youbot_driver_PACKAGE_PATH}/
  ENV CPATH
  /usr/include/youbot/
  /usr/local/include/youbot/
  /opt/local/include/youbot/
  NO_DEFAULT_PATH
)

FIND_LIBRARY(OODL_YOUBOT_LIBRARIES NAMES "YouBotDriver"
  PATHS
  $ENV{YOUBOTDIR}/lib 
  $ENV{ROBOTPKG_BASE}/lib
  ${youbot_driver_PACKAGE_PATH}/lib
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

FIND_PATH(OODL_YOUBOT_CONFIG_DIR NAMES youbot-ethercat.cfg
  PATHS
  $ENV{YOUBOTDIR}/config
  $ENV{ROBOTPKG_BASE}/config
  ${youbot_driver_PACKAGE_PATH}/config
  /etc/youbot
  NO_DEFAULT_PATH
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OODL_YOUBOT_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(OODL_YouBot  DEFAULT_MSG
                                  OODL_YOUBOT_INCLUDE_DIR OODL_YOUBOT_LIBRARIES)

# show the OODL_YOUBOT_INCLUDE_DIR, OODL_YOUBOT_LIBRARIES and OODL_YOUBOT_CONFIG_DIR variables only in the advanced view
IF(OODL_YOUBOT_FOUND)
  MARK_AS_ADVANCED(OODL_YOUBOT_INCLUDE_DIR OODL_YOUBOT_LIBRARIES_DIR OODL_YOUBOT_CONFIG_DIR)
ENDIF(OODL_YOUBOT_FOUND)

# show warning that no configuration directory was found; given that OODL itself was found
IF (OODL_YOUBOT_FOUND AND NOT OODL_YOUBOT_CONFIG_DIR)
  SET(OODL_YOUBOT_CONFIG_DIR .)
  IF (NOT OODL_YouBot_FIND_QUIETLY)
    MESSAGE(STATUS "Warning: oodl-youbot configuration directory not found, using default folder \".\"")
  ENDIF (NOT OODL_YouBot_FIND_QUIETLY)
ENDIF (OODL_YOUBOT_FOUND AND NOT OODL_YOUBOT_CONFIG_DIR)

