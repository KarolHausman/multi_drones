#include "time_utils.h"
#include <cstdlib>
#include <ctime>
#include <sys/time.h>

namespace ranav {

double Time::getTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec/1000000.0;
}

} /* namespace ranav */
