# Find NLOPT library
# Looks for a local or system-wide version of libNLOPT.
#
# NLOPT                  the NLOPT base directory
# NLOPT_INCLUDE_DIR      where to find the include files
# NLOPT_LIBRARY_DIR      where to find the libraries
# NLOPT_LIBRARIES        list of libraries to link
# NLOPT_FOUND            true if NLOPT was found

find_path(NLOPT include/nlopt.h PATHS
  $ENV{NLOPT_HOME}
  $ENV{HOME}/code/nlopt-2.4.1
  $ENV{HOME}/nlopt-2.4.1
  ${CMAKE_SOURCE_DIR}/../../nlopt-2.4.1
  )
if(NLOPT)
  message(STATUS "Found NLOPT at ${NLOPT}")
  set(NLOPT_INCLUDE_DIR ${NLOPT}/include)
  set(NLOPT_LIBRARY_DIR ${NLOPT}/lib)
  set(NLOPT_LIBRARIES nlopt)
endif(NLOPT)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NLOPT DEFAULT_MSG NLOPT_INCLUDE_DIR NLOPT_LIBRARY_DIR)
mark_as_advanced(NLOPT_INCLUDE_DIR NLOPT_LIBRARY_DIR)
