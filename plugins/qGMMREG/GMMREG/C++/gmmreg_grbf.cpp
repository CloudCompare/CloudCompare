#include "gmmreg_grbf.h"

#include <assert.h>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include <vcl_iostream.h>
#include <vnl/algo/vnl_lbfgs.h>
#include <vnl/algo/vnl_qr.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_trace.h>

#include "gmmreg_utils.h"
#include "utils/io_utils.h"
#include "utils/misc_utils.h"

namespace gmmreg {

int GrbfRegistration::PrepareInput(const vnl_matrix<double>& model, const vnl_matrix<double>& scene, const vnl_matrix<double>* control)
{
	if (Base::PrepareInput(model, scene, control) < 0)
		return -1;

	//TODO: let the caller set this parameter!
	beta_ = 1;

	return 0;
}

int GrbfRegistration::PrepareInput(const char* f_config) {
  Base::PrepareInput(f_config);
  char f_ctrl_pts[80] = {0};
  GetPrivateProfileString(common_section_, "ctrl_pts", NULL,
      f_ctrl_pts, 80, f_config);
  if (SetCtrlPts(f_ctrl_pts) < 0) {
    // TODO(bing-jian): compute the ctrl pts on the fly
    return -1;
  }
  char s_beta[80] = {0};
  GetPrivateProfileString(section_, "beta", "1", s_beta, 60, f_config);
  beta_ = atof(s_beta);
  return 0;
}

int GrbfRegistration::StartRegistration(vnl_vector<double>& params) {
  vnl_lbfgs minimizer(*func_);
  func_->SetBase(this);
  for (unsigned int k = 0; k < level_; ++k) {
    func_->SetScale(v_scale_[k]);
    func_->SetLambda(v_lambda[k]);
    func_->PrepareParamGradient();
    SetParam(params);
    int n_max_func_evals = v_func_evals_[k];
    minimizer.set_max_function_evals(n_max_func_evals);
    // For more options, see
    // http://public.kitware.com/vxl/doc/release/core/vnl/html/vnl__nonlinear__minimizer_8h-source.html
    minimizer.minimize(params);
    if (minimizer.get_failure_code() < 0)
	{
      return -1;
    }
  }
  return 0;
}

int GrbfRegistration::SetInitParams(const char* f_config) {
  char f_init_grbf[80] = {0};
  GetPrivateProfileString(common_section_, "init_grbf", NULL,
      f_init_grbf, 80, f_config);
  SetInitGrbf(f_init_grbf);
  param_all_.set_size(n_, d_);
  return 0;
}

int GrbfRegistration::SetInitGrbf(const char* filename) {
  if (strlen(filename) == 0) {
    assert(n_ > 0);
    assert(d_ > 0);
    param_grbf_.set_size(n_, d_);
    param_grbf_.fill(0);
    return 0;
  } else {
    std::ifstream infile(filename, std::ios_base::in);
    param_grbf_.read_ascii(infile);
    assert(param_grbf_.cols() == d_);
    assert(param_grbf_.rows() == n_);
    return 1;
  }
}

void GrbfRegistration::PrepareBasisKernel() {
  ComputeGaussianKernel(model_, ctrl_pts_, basis_, kernel_, beta_);
}

void GrbfRegistration::PerformTransform(const vnl_vector<double> &x) {
  SetGrbf(x);
  transformed_model_ = model_ + basis_ * param_grbf_;
}

double GrbfRegistration::BendingEnergy() {
  return vnl_trace(param_grbf_.transpose() * kernel_ * param_grbf_);
}

void GrbfRegistration::ComputeGradient(const double lambda, const vnl_matrix<double>& gradient, vnl_matrix<double>& grad_all)
{
  grad_all = basis_.transpose() * gradient;
  if (lambda > 0) {
    grad_all += 2 * lambda * kernel_ * param_grbf_;
  }
}

void GrbfRegistration::SaveResults(const char* f_config) {
  char f_transformed[256] = {0};
  char f_final_grbf[256] = {0};
  GetPrivateProfileString(common_section_, "final_grbf", NULL,
      f_final_grbf, 255, f_config);
  GetPrivateProfileString(common_section_, "transformed_model", NULL,
      f_transformed, 255, f_config);
  SaveTransformed(f_transformed, f_config);
  SaveMatrixToAsciiFile(f_final_grbf, param_grbf_);
}

int GrbfRegistration::PrepareOwnOptions(const std::vector<float>& lambda)
{
	try
	{
		v_lambda = lambda;
	}
	catch (const std::bad_alloc&)
	{
		std::cerr<< " not enough memory " << std::endl;
		return -1;
	}

	if (v_lambda.size() < level_)
	{
		std::cerr<< " too many levels " << std::endl;
		return -1;
	}

	return 0;
}


int GrbfRegistration::PrepareOwnOptions(const char* f_config)
{
	assert(f_config);
	if (MultiScaleOptions(f_config) < 0)
		return -1;

	static char delims[] = " -,;";
	char s_lambda[256] = {0};
	GetPrivateProfileString(section_, "lambda", NULL, s_lambda, 255, f_config);

	std::vector<float> lambda;
	utils::parse_tokens(s_lambda, delims, lambda);

	return PrepareOwnOptions(lambda);
}

void GrbfRegistration::SetParam(vnl_vector<double>& x0) {
  int k = 0;
  x0.set_size(func_->get_number_of_unknowns());
  x0.fill(0);
  for (unsigned int i = 0; i < param_grbf_.rows(); ++i) {
    for (int j = 0; j < d_; ++j, ++k) {
      x0[k] = param_grbf_(i, j);
    }
  }
}

void GrbfRegistration::SetGrbf(const vnl_vector<double>& x) {
  /* reshape x, assuming x is row major; */
  int rows_x = x.size() / d_;
  for (int i = 0, k = 0; i < rows_x; ++i) {
    for (int j = 0; j < d_; ++j, ++k) {
      param_grbf_(i, j) = x[k];
    }
  }
}

}  // namespace gmmreg
