# Find RANAV library
# Looks for a local or system-wide version of libRANAV.
#
# RANAV                  the RANAV base directory
# RANAV_INCLUDE_DIR      where to find the include files
# RANAV_LIBRARY_DIR      where to find the libraries
# RANAV_LIBRARIES        list of libraries to link
# RANAV_FOUND            true if RANAV was found

find_path(RANAV include/ranav/ekf.h PATHS
  $ENV{RANAV_HOME}
  $ENV{HOME}/code/ranav
  $ENV{HOME}/ranav
  ${CMAKE_SOURCE_DIR}/../../ranav
  )

if(RANAV)
  # GSL
  find_library(LIB_GSL gsl)
  IF (LIB_GSL)
    MESSAGE(STATUS "found library 'gsl': ${LIB_GSL}")
  ELSE (LIB_GSL)
    MESSAGE(FATAL_ERROR "cannot find library 'gsl' - this will not work ...")
  ENDIF (LIB_GSL)
  # NLopt
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${RANAV}/src)
  find_package(NLopt REQUIRED)
  # OpenMP
  find_package(OpenMP)
  if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()

  # set variables
  message(STATUS "Found RANAV at ${RANAV}")
  set(RANAV_INCLUDE_DIR ${RANAV}/include ${NLOPT_INCLUDE_DIR})
  set(RANAV_LIBRARY_DIR ${RANAV}/lib ${NLOPT_LIBRARY_DIR})
  set(RANAV_LIBRARIES ranav_multi_rotor_control ranav_estimation ranav_trajectory ranav_controller ranav_utils gsl gslcblas ${NLOPT_LIBRARIES})
endif(RANAV)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RANAV DEFAULT_MSG RANAV_INCLUDE_DIR RANAV_LIBRARY_DIR)
mark_as_advanced(RANAV_INCLUDE_DIR RANAV_LIBRARY_DIR)
