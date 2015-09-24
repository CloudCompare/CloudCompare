#include "rotation_utils.h"

namespace gmmreg {

template<typename T>
void Quaternion2Rotation(const vnl_vector<T>& q,
    vnl_matrix<T>& R, vnl_matrix<T>& g1, vnl_matrix<T>& g2, vnl_matrix<T>& g3, vnl_matrix<T>& g4) {
  T x,y,z,r;
  T x2,y2,z2,r2;
  x = q[0];  y = q[1];  z=q[2];  r = q[3];
  x2 = q[0] * q[0];
  y2 = q[1] * q[1];
  z2 = q[2] * q[2];
  r2 = q[3] * q[3];
  // fill diagonal terms
  R(0,0) = r2 + x2 - y2 - z2;
  R(1,1) = r2 - x2 + y2 - z2;
  R(2,2) = r2 - x2 - y2 + z2;
  // fill off diagonal terms
  R(0,1) = 2 * (x*y + r*z);
  R(0,2) = 2 * (z*x - r*y);
  R(1,2) = 2 * (y*z + r*x);
  R(1,0) = 2 * (x*y - r*z);
  R(2,0) = 2 * (z*x + r*y);
  R(2,1) = 2 * (y*z - r*x);
  T ss = (x2+y2+z2+r2);
  R = R/ss;
  T ssss = ss*ss;

  // derivative of R(0,0) = r2 + x2 - y2 - z2;
  g1(0,0) = 4*x*(y2+z2)/ssss;  g2(0,0) = -4*y*(x2+r2)/ssss;
  g3(0,0) = -4*z*(x2+r2)/ssss;  g4(0,0) = 4*r*(y2+z2)/ssss;
  // derivative of R(1,1) = r2 - x2 + y2 - z2;
  g1(1,1) = -4*x*(y2+r2)/ssss;  g2(1,1) = 4*y*(x2+z2)/ssss;
  g3(1,1) = -4*z*(y2+r2)/ssss;  g4(1,1) = 4*r*(x2+z2)/ssss;
  // derivative of R(2,2) = r2 - x2 - y2 + z2;
  g1(2,2) = -4*x*(z2+r2)/ssss;  g2(2,2) = -4*y*(r2+z2)/ssss;
  g3(2,2) = 4*z*(x2+y2)/ssss;  g4(2,2) = 4*r*(x2+y2)/ssss;

  // fill off diagonal terms
  // derivative of R(0,1) = 2 * (xy + rz);
  g1(0,1) = 2*y/ss - 2*x*R(0,1)/ssss;
  g2(0,1) = 2*x/ss - 2*y*R(0,1)/ssss;
  g3(0,1) = 2*r/ss - 2*z*R(0,1)/ssss;
  g4(0,1) = 2*z/ss - 2*r*R(0,1)/ssss;
  // derivative of R(0,2) = 2 * (zx - ry);
  g1(0,2) = 2*z/ss - 2*x*R(0,2)/ssss;
  g2(0,2) = -2*r/ss - 2*y*R(0,2)/ssss;
  g3(0,2) = 2*x/ss - 2*z*R(0,2)/ssss;
  g4(0,2) = -2*y/ss - 2*r*R(0,2)/ssss;
  // derivative of R(1,2) = 2 * (yz + rx);
  g1(1,2) = 2*r/ss - 2*x*R(1,2)/ssss;
  g2(1,2) = 2*z/ss - 2*y*R(1,2)/ssss;
  g3(1,2) = 2*y/ss - 2*z*R(1,2)/ssss;
  g4(1,2) = 2*x/ss - 2*r*R(1,2)/ssss;
  // derivative of R(1,0) = 2 * (xy - rz);
  g1(1,0) = 2*y/ss - 2*x*R(1,0)/ssss;
  g2(1,0) = 2*x/ss - 2*y*R(1,0)/ssss;
  g3(1,0) = -2*r/ss - 2*z*R(1,0)/ssss;
  g4(1,0) = -2*z/ss - 2*r*R(1,0)/ssss;
  // derivative of R(2,0) = 2 * (zx + ry);
  g1(2,0) = 2*z/ss - 2*x*R(2,0)/ssss;
  g2(2,0) = 2*r/ss - 2*y*R(2,0)/ssss;
  g3(2,0) = 2*x/ss - 2*z*R(2,0)/ssss;
  g4(2,0) = 2*y/ss - 2*r*R(2,0)/ssss;
  // derivative of R(2,1) = 2 * (yz - rx);
  g1(2,1) = -2*r/ss - 2*x*R(2,1)/ssss;
  g2(2,1) = 2*z/ss - 2*y*R(2,1)/ssss;
  g3(2,1) = 2*y/ss - 2*z*R(2,1)/ssss;
  g4(2,1) = -2*x/ss - 2*r*R(2,1)/ssss;
}


template<typename T>
void Quaternion2Rotation(const vnl_vector<T>& q, vnl_matrix<T>& R) {
  T x,y,z,r;
  T x2,y2,z2,r2;
  x = q[0];  y = q[1];  z=q[2];  r = q[3];
  x2 = q[0] * q[0];
  y2 = q[1] * q[1];
  z2 = q[2] * q[2];
  r2 = q[3] * q[3];
  // fill diagonal terms
  R(0,0) = r2 + x2 - y2 - z2;
  R(1,1) = r2 - x2 + y2 - z2;
  R(2,2) = r2 - x2 - y2 + z2;
  // fill off diagonal terms
  R(0,1) = 2 * (x*y + r*z);
  R(0,2) = 2 * (z*x - r*y);
  R(1,2) = 2 * (y*z + r*x);
  R(1,0) = 2 * (x*y - r*z);
  R(2,0) = 2 * (z*x + r*y);
  R(2,1) = 2 * (y*z - r*x);
  T ss = (x2+y2+z2+r2);
  R = R/ss;
}

}  // namespace gmmreg
