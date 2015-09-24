#ifndef GMMREG_UTILS_IO_UTILS_H_
#define GMMREG_UTILS_IO_UTILS_H_

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#include <vcl_iostream.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

namespace gmmreg {

template<typename T>
int LoadMatrixFromTxt(const char* filename, vnl_matrix<T>& matrix);

template<typename T>
void SaveMatrixToAsciiFile(const char * filename, const vnl_matrix<T>& x);

template<typename T>
void SaveVectorToAsciiFile(const char * filename, const vnl_vector<T>& x);


template<typename T>
int LoadMatrixFromTxt(const char* filename, vnl_matrix<T>& matrix) {
  std::ifstream infile(filename, std::ios_base::in);
  if (infile.is_open()) {
    if (matrix.read_ascii(infile)) {
      return matrix.rows();
    } else {
      std::cerr << "unable to parse input file " << filename
                << " as a matrix." << std::endl;
      return -1;
    }
  } else {
    std::cerr << "unable to open model file " << filename << std::endl;
    return -1;
  }
}

template<typename T>
void SaveMatrixToAsciiFile(const char * filename, const vnl_matrix<T>& x) {
  if (strlen(filename)>0) {
    std::ofstream outfile(filename,std::ios_base::out);
    x.print(outfile);
  }
}

template<typename T>
void SaveVectorToAsciiFile(const char * filename, const vnl_vector<T>& x) {
  if (strlen(filename)>0) {
    std::ofstream outfile(filename,std::ios_base::out);
    outfile << x;
  }
}

}  // namespace gmmreg

#endif // GMMREG_UTILS_IO_UTILS_H_
