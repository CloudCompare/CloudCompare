#ifndef GMMREG_UTILS_MISC_UTILS_H_
#define GMMREG_UTILS_MISC_UTILS_H_

#include <vector>

#ifdef WIN32
#include <windows.h>
#else
char *strupr(char *string);
char *strlwr(char *string);
#endif

namespace gmmreg {
namespace utils {

void parse_tokens(char* str, const char delims[],
                  std::vector<float>& v_tokens);
void parse_tokens(char* str, const char delims[],
                  std::vector<int>& v_tokens);

int get_config_fullpath(const char* input_config,char* f_config);

}  // namespace utils
}  // namespace gmmreg

#endif  // GMMREG_UTILS_MISC_UTILS_H_
