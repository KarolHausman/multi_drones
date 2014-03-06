project(RANAV)
cmake_minimum_required(VERSION 2.8)

# use "-DCMAKE_BUILD_TYPE=Debug" in cmake for a Debug-build
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
message(STATUS "Build type is " ${CMAKE_BUILD_TYPE})

# COMPILER FLAGS
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb -DDEBUG")


##############################
# find external libraries
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

# Eigen is required
# if Eigen is available, USE_EIGEN is defined which can
# be checked using #ifdef USE_EIGEN within the code
find_package(Eigen3 3.2.0 REQUIRED)
if(EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
  set(USE_EIGEN 1)
endif()

# NLopt
find_package(NLopt REQUIRED)
if(NLOPT_FOUND)
  include_directories(${NLOPT_INCLUDE_DIR})
  link_directories(${NLOPT_LIBRARY_DIR})
  set(USE_NLOPT 1)
endif()

# add core packages to include
include_directories(${CMAKE_SOURCE_DIR}/utils)


##############################
# make targets

# Install
#set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../)
set(INSTALL_HEADERS_PATH DESTINATION include/ranav)
set(INSTALL_TARGETS_DEFAULT_ARGS
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )


add_subdirectory(utils)
add_subdirectory(estimation)
add_subdirectory(multi_rotor_control)
