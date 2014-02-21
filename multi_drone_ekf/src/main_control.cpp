#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <ranav/tparam.h>

using namespace ranav;

int main(int argc, char **argv) {
  std::string paramfile = "multi.ini";
  char c;
  while ((c = getopt(argc, argv, "p:h")) != EOF) {
    switch (c) {
      case 'p':
        paramfile = optarg;
        break;
      case 'h':
      default:
        std::cerr << "Usage: " << argv[0] << " [options]\n";
        std::cerr << "\nOptions:\n";
        std::cerr << "-p <file>:  use the given parameter file\n";
        std::cerr << "-h:         this help\n";
        return 1;
    }
  }

  TParam p;
  p.loadTree(paramfile);

  return 0;
}
