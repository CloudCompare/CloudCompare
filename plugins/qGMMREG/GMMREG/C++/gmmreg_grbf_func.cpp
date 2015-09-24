#include "gmmreg_grbf_func.h"
#include "gmmreg_utils.h"

namespace gmmreg {

double GaussianRadialBasisFunc_L2::Eval(const double f1, const double f2,
    vnl_matrix<double>& g1, vnl_matrix<double>& g2) {
  /* L2 version */
  double f = f1 - 2*f2;
  gradient_ = g1*2 - g2*2;
  return f;
}

double GaussianRadialBasisFunc_KC::Eval(const double f1, const double f2,
    vnl_matrix<double>& g1, vnl_matrix<double>& g2) {
  /* -KC^2 version */
  double f21 = f2/f1;
  double f =  -f2*f21;
  gradient_ = g1*2*f21*f21 - g2*2*f21;
  return f;
}

double GaussianRadialBasisFunc::f(const vnl_vector<double>& x) {
  base_->PerformTransform(x);
  double bending = base_->BendingEnergy();
  double energy1 = GaussTransform(base_->transformed_model_,
      base_->transformed_model_, scale_, gradient1_);
  double energy2 = GaussTransform(base_->transformed_model_,
      base_->scene_, scale_, gradient2_);
  double energy = Eval(energy1, energy2, gradient1_, gradient2_);
  energy += lambda_ * bending;
  return energy;
}

void GaussianRadialBasisFunc::gradf(const vnl_vector<double>& x,
    vnl_vector<double>& g) {
  base_->ComputeGradient(lambda_, gradient_, grad_all_);
  int rows_x = grad_all_.rows();
  int start_row = 0;
  for (int i = start_row, k = 0; i < rows_x; ++i) {
    for (int j = 0; j < d_; ++j, ++k) {
      g[k] = grad_all_(i, j);
    }
  }
}

void GaussianRadialBasisFunc::PrepareParamGradient() {
  gradient_.set_size(m_, d_);
  grad_all_.set_size(n_, d_);
}

}  // namespace gmmreg
