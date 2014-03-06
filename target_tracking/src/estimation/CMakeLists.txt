
add_library(ranav_estimation ekf.cpp motionmodel.cpp sensormodel.cpp)
target_link_libraries(ranav_estimation ranav_utils ${LIB_GSL})

install(FILES ekf.h motionmodel.h sensormodel.h ${INSTALL_HEADERS_PATH})
install(TARGETS ranav_estimation ${INSTALL_TARGETS_DEFAULT_ARGS})
