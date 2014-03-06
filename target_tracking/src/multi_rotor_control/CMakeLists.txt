
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(../utils ../estimation ../trajectory)
add_library(ranav_multi_rotor_control point2dmotionmodel.cpp rel2dsensormodel.cpp targettrajectory.cpp targettrackingcontroller.cpp topology.cpp rotor2dmotionmodel.cpp cam2dsensormodel.cpp)
target_link_libraries(ranav_multi_rotor_control ranav_estimation ${NLOPT_LIBRARIES})

add_executable(test_multi main_test_multi.cpp)
target_link_libraries(test_multi ranav_multi_rotor_control)

install(FILES multiagentsensormodel.h multiagentmotionmodel.h cam2dsensormodel.h rel2dsensormodel.h rotor2dmotionmodel.h point2dmotionmodel.h targettrajectory.h targettrackingcontroller.h topology.h ${INSTALL_HEADERS_PATH})
install(TARGETS ranav_multi_rotor_control ${INSTALL_TARGETS_DEFAULT_ARGS})
