#include "misc_utils.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#ifndef WIN32
char *strupr(char *string) {
  char *s;
  if (string) {
    for (s = string;  *s;  ++s) {
      *s = toupper(*s);
    }
  }
  return string;
}

char *strlwr(char *string) {
  char *s;
  if (string) {
    for (s = string;  *s;  ++s) {
      *s = tolower(*s);
    }
  }
  return string;
}
#endif

namespace gmmreg {
namespace utils {

void parse_tokens(char* str, const char delims[],
    std::vector<float>& v_tokens) {
  char* pch = strtok (str, delims);
  while (pch != NULL) {
    v_tokens.push_back(static_cast<float>(atof(pch)));
    pch = strtok (NULL, delims);
  }
}

void parse_tokens(char* str, const char delims[],
    std::vector<int>& v_tokens) {
  char* pch = strtok (str, delims);
  while (pch != NULL) {
    v_tokens.push_back(atoi(pch));
    pch = strtok (NULL, delims);
  }
}

int get_config_fullpath(const char* input_config, char* f_config) {
#ifdef WIN32
  const int BUFSIZE = 1024;
  char* lpPart[BUFSIZE] = {NULL};
  int retval = GetFullPathName(input_config,
      BUFSIZE, f_config, lpPart);

  if (retval == 0) {
    // Handle an error condition.
    printf ("GetFullPathName failed (%d)\n", GetLastError());
    return -1;
  }
#else
  strcpy(f_config, input_config);
#endif
  return 0;
}

}  // namespace utils
}  // namespace gmmreg
