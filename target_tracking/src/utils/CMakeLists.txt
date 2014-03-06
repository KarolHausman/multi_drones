
add_library(ranav_utils random.cpp time_utils.cpp tparam.cpp variant.cpp)

install(FILES random.h time_utils.h tparam.h variant.h ${INSTALL_HEADERS_PATH})
install(TARGETS ranav_utils ${INSTALL_TARGETS_DEFAULT_ARGS})
