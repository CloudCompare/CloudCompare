#include "gmmreg_rigid.h"

#include <assert.h>
#include <fstream>
#include <iostream>

#include <vcl_iostream.h>
#include <vnl/algo/vnl_lbfgsb.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include "gmmreg_utils.h"
#include "utils/io_utils.h"
#include "utils/rotation_utils.h"

namespace gmmreg {

const float kPi = 3.1415926f;

void SetRigidTransformBound(const int d, vnl_lbfgsb* minimizer) {
  // http://public.kitware.com/vxl/doc/release/core/vnl/html/vnl__lbfgsb_8h_source.html
  vnl_vector<long> nbd;
  vnl_vector<double> lower_bound, upper_bound;
  if (d == 2) {
    nbd.set_size(3); // (dx, dy, d\theta)
    nbd[0] = 0;  // not constrained
    nbd[1] = 0;  // not constrained
    nbd[2] = 2;  // has both lower and upper bounds
    lower_bound.set_size(3);
    lower_bound.fill(0);
    lower_bound[2] = -1.0 * kPi;
    upper_bound.set_size(3);
    upper_bound.fill(0);
    upper_bound[2] = kPi;
  } else if (d == 3) {
    nbd.set_size(7); // ((q1, q2, q3, q4), (dx, dy, dz))
    nbd.fill(0);
    for (int i = 0; i < 4; ++i) {
      nbd[i] = 2;  // quaternion part has both lower and upper bounds
    }
    lower_bound.set_size(7);
    lower_bound.fill(0);
    for (int i = 0; i < 4; ++i) {
      lower_bound[i] = -1;
    }
    upper_bound.set_size(7);
    upper_bound.fill(0);
    for (int i = 0; i < 4; ++i) {
      upper_bound[i] = 1;
    }
  }
  minimizer->set_bound_selection(nbd);
  minimizer->set_lower_bound(lower_bound);
  minimizer->set_upper_bound(upper_bound);
}

int RigidRegistration::StartRegistration(vnl_vector<double>& params) {
  vnl_lbfgsb minimizer(*func_);
  SetRigidTransformBound(this->d_, &minimizer);
  //double fxval;
  func_->SetBase(this);
  for (unsigned int k = 0; k < this->level_; ++k) {
    func_->SetScale(this->v_scale_[k]);
    SetParam(params);
    minimizer.set_max_function_evals(this->v_func_evals_[k]);
    // For more options, see
    // http://public.kitware.com/vxl/doc/release/core/vnl/html/vnl__nonlinear__minimizer_8h-source.html
    minimizer.minimize(params);
    if (minimizer.get_failure_code() < 0) {
      /*
      fxval = func->f( params );
      vcl_cout << "break Minimized to " << fxval << vcl_endl
        << "Iterations: " << minimizer.get_num_iterations() << "; "
        << "Evaluations: " << minimizer.get_num_evaluations() << vcl_endl;
      vcl_cout << params << vcl_endl;
      */
      return -1;
    }
    /*
    fxval = func->f( params );
    vcl_cout << "Minimized to " << fxval << vcl_endl
             << "Iterations: " << minimizer.get_num_iterations() << "; "
             << "Evaluations: " << minimizer.get_num_evaluations() << vcl_endl;
    */
  }
  vcl_cout << "Solution: " << params << vcl_endl;

  return 0;
}

int RigidRegistration::SetInitParams(const char* f_config) {
  char f_init_affine[80] = {0}, f_init_rigid[80] = {0};
  GetPrivateProfileString(this->section_, "init_rigid", NULL,
      f_init_rigid, 80, f_config);
  SetInitRigid(f_init_rigid);
  return 0;
}

int RigidRegistration::SetInitRigid(const char* filename) {
  assert((this->d_ == 2) || (this->d_ == 3));
  if (this->d_ == 2) {
    param_rigid_.set_size(3);
    param_rigid_.fill(0);
    param_rigid_[0] = 0;
    param_rigid_[1] = 0;
    param_rigid_[2] = 0;
  } else if (this->d_ == 3) {
    param_rigid_.set_size(7);
    param_rigid_.fill(0);
    param_rigid_[3] = 1;  // q = (0, 0, 0, 1) for eye(3)
  }
  return 0;
}

void RigidRegistration::SetParam(vnl_vector<double>& x0) {
  x0 = this->param_rigid_;
}

void RigidRegistration::PerformTransform(const vnl_vector<double>& x) {
  // assert((d_ == 2) || (d_ == 3));
  vnl_matrix<double> translation;
  vnl_matrix<double> rotation;
  vnl_matrix<double> ones;
  ones.set_size(this->m_, 1);
  ones.fill(1);
  if (this->d_ == 2) {
    rotation.set_size(2, 2);
    double theta = x[2];
    rotation[0][0] = cos(theta);
    rotation[0][1] = -sin(theta);
    rotation[1][0] = sin(theta);
    rotation[1][1] = cos(theta);
    translation.set_size(1, 2);
    translation[0][0] = x[0];
    translation[0][1] = x[1];
  } else if (this->d_ == 3) {
    rotation.set_size(3, 3);
    vnl_vector<double> q;
    q.set_size(4);
    for (int i = 0; i < 4; ++i) {
      q[i] = x[i];
    }
    Quaternion2Rotation<double>(q, rotation);
    translation.set_size(1, 3);
    translation[0][0] = x[4];
    translation[0][1] = x[5];
    translation[0][2] = x[6];
  }
  this->transformed_model_ = this->model_ * rotation.transpose() + ones * translation;
  this->param_rigid_ = x;
}

void RigidRegistration::SaveResults(const char* f_config)
{
  char f_transformed[80] = {0};
  GetPrivateProfileString(this->common_section_, "transformed_model",
      NULL, f_transformed, 80, f_config);
  this->SaveTransformed(f_transformed, f_config);

  char f_final_rigid[80] = {0};
  GetPrivateProfileString(this->common_section_, "final_rigid",
      NULL, f_final_rigid, 80, f_config);
  SaveVectorToAsciiFile(f_final_rigid, this->param_rigid_);
}

int RigidRegistration::PrepareOwnOptions(const char* f_config)
{
  return MultiScaleOptions(f_config);
}

}  // namespace gmmreg
