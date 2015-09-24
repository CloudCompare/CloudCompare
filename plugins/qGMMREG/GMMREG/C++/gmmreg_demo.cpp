#include <iostream>

#include "gmmreg_api.h"

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " config_file method" << std::endl;
    print_usage();
    return -1;
  }
  gmmreg_api(argv[1], argv[2]);
  return 0;
}
