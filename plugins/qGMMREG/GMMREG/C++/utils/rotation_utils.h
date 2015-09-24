#ifndef GMMREG_UTILS_ROTATION_UTILS_H_
#define GMMREG_UTILS_ROTATION_UTILS_H_

#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

namespace gmmreg {

template<typename T>
void Quaternion2Rotation(const vnl_vector<T>& q,
                         vnl_matrix<T>& R,
                         vnl_matrix<T>& g1,
                         vnl_matrix<T>& g2,
                         vnl_matrix<T>& g3,
                         vnl_matrix<T>& g4);

template<typename T>
void Quaternion2Rotation(const vnl_vector<T>& q, vnl_matrix<T>& R);

}  // namespace gmmreg

#include "rotation_utils.cc"

#endif // GMMREG_UTILS_ROTATION_UTILS_H_
