#include "gmmreg_cpd.h"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vcl_iostream.h>
#include <vnl/algo/vnl_determinant.h>
#include <vnl/algo/vnl_qr.h>
#include <vnl/algo/vnl_svd.h>
#include <vnl/vnl_trace.h>

#include "gmmreg_utils.h"
#include "utils/io_utils.h"

namespace gmmreg {

int CoherentPointDrift::PrepareInput(const char* f_config) {
  Base::PrepareInput(f_config);
  char f_ctrl_pts[80]={0};
  GetPrivateProfileString(common_section_, "ctrl_pts", NULL,
      f_ctrl_pts, 80, f_config);
  if (SetCtrlPts(f_ctrl_pts) < 0) {
    //todo: compute the ctrl pts on the fly
    return -1;
  }
  return 0;
}

void CoherentPointDriftGrbf::PrepareBasisKernel() {
  ComputeGaussianKernel(model_, ctrl_pts_, basis_, kernel_, beta_);
  Gtranspose = basis_.transpose();
  dPG.set_size(m_, n_);
  dPY0.set_size(m_, d_);
  //vnl_qr<double> qr(Gtranspose*dPG+lambda*sigma*sigma*kernel); //, 1e-18);
  //invG = qr.inverse()*Gtranspose;
}

double CoherentPointDriftGrbf::UpdateParam() {
  double bending = vnl_trace(param_all_.transpose() * kernel_ * param_all_);
  double row_sum;
  for (int i = 0; i < m_; ++i) {
    row_sum  = P.get_row(i).sum();
    dPG.set_row(i, row_sum * basis_.get_row(i));
    dPY0.set_row(i, row_sum * model_.get_row(i));
  }
  vnl_qr<double> qr(Gtranspose * dPG + lambda_ * sigma_ * sigma_ * kernel_); //, 1e-18);
  param_all_ = qr.solve(Gtranspose * (P * scene_ - dPY0));
  //param_all = invG*(P*scene-dPY0);
  return bending;
}

void CoherentPointDriftTps::PrepareBasisKernel() {
  vnl_matrix<double> K, U;
  ComputeTPSKernel(model_, ctrl_pts_, U, K);

  //n = ctrl_pts.rows();
  vnl_matrix<double> Pn;
  Pn.set_size(n_, d_ + 1);
  Pn.set_column(0, 1);
  Pn.update(ctrl_pts_, 0, 1);
  vnl_qr<double> qr(Pn);
  vnl_matrix<double> V = qr.Q();
  vnl_matrix<double> PP = V.extract(n_, n_ - d_ - 1, 0, d_ + 1);
  kernel_ = PP.transpose() * K * PP;

  //m = model.rows();
  vnl_matrix<double> Pm;
  Pm.set_size(m_, d_ + 1);
  Pm.set_column(0, 1);
  Pm.update(model_, 0, 1);

  basis_.set_size(m_, n_);
  basis_.update(Pm);
  basis_.update(U * PP, 0, d_ + 1);

  G = U * PP;
  vnl_matrix<double> Gtranspose;
  Gtranspose = G.transpose();

  vnl_qr<double> qr_Pm(Pm);
  Q1 = qr_Pm.Q().extract(m_, d_ + 1);
  Q2 = qr_Pm.Q().extract(m_, m_ - d_ - 1, 0, d_ + 1);
  R = qr_Pm.R().extract(d_ + 1, d_ + 1);

  vnl_svd<double> svd(R);
  invR = svd.inverse();
  nP.set_size(m_, s_);

  vnl_matrix<double> A = G.transpose() * Q2 * Q2.transpose() * G + lambda_ * kernel_;
  vnl_qr<double> qr2(A);
  invG = qr2.inverse() * G.transpose() * Q2 * Q2.transpose();
}

double CoherentPointDriftTps::UpdateParam() {
  tps_ = param_all_.extract(n_ - d_ -1, d_, d_ + 1,0);
  //std::cout << "before: tps " << tps.array_two_norm() << std::endl;
  double bending = vnl_trace(tps_.transpose() * kernel_ * tps_);
  //std::cout << "bending = " << bending << std::endl;
  double row_sum;
  for (int i = 0; i < m_; ++i) {
    row_sum  = P.get_row(i).sum();
    if (row_sum > eps_) {
      nP.set_row(i, P.get_row(i) / row_sum);
    }
  }
  //std::cout << "before: nP " << nP.array_two_norm() << std::endl;
  //vnl_qr<double> qr(G.transpose()*Q2*Q2.transpose()*G+lambda*kernel);
  //tps = qr.solve(G.transpose()*Q2*Q2.transpose()*(nP*scene));
  tps_ = invG * (nP * scene_);
  affine_ = invR * Q1.transpose() * (nP * scene_ - model_ - G * tps_);
  param_all_.update(affine_);
  param_all_.update(tps_, d_+1);
  //std::cout << "after: tps" << tps.array_two_norm() << std::endl;
  return bending;
}

int CoherentPointDrift::StartRegistration(vnl_vector<double>& params) {
  int EMiter, iter = 0;
  double Eu, E_old;
  double E = 1;
  //outlier_term = outliers*pow((2*sigma*sigma*3.1415926),0.5*d);
  double ntol = tol + 10;
  vnl_matrix<double> dP;
  vnl_matrix<double> prev_model;
  vnl_matrix<double> moving_model(model_);
  //vnl_matrix<double> eye(n,n);
  //eye.set_identity();
  /*PrepareBasisKernel(); already done */
  P.set_size(model_.rows(), scene_.rows());
  double bending;
  //column_sum.set_size(s);
  while ((iter < max_iter) && (ntol > tol)) {
    std::cout << "iter=" << iter << "\n";
    EMiter = 0;   EMtol = tol + 10;
    prev_model = moving_model;
    while ((EMiter < max_em_iter) && (EMtol > tol)) {
      std::cout << "EMiter="<<EMiter<< "\t";
      std::cout << "E="<<E<<"\t";
      //std::cout << "sigma="<<sigma<<std::endl;
      E_old = E;
      ComputeP(moving_model, scene_, P, Eu, sigma_, outliers);
      bending = UpdateParam();
      moving_model = model_ + basis_ * param_all_;
      E = Eu + (lambda_ / 2) * bending;
      EMtol = fabs(E_old - E) / E_old;
      ++EMiter;
    }
    sigma_ *= anneal;
    ++iter;
    ntol = (moving_model - prev_model).array_two_norm();
  }
  return 0;
}

int CoherentPointDrift::SetInitParams(const char* f_config) {
  char f_init_params[80] = {0};
  GetPrivateProfileString("Files", "init_params", NULL,
      f_init_params, 80, f_config);
  if (strlen(f_init_params) == 0) {
    assert(n_ > 0);
    assert(d_ > 0);
    param_all_.set_size(n_, d_);
    param_all_.fill(0);
    return 0;
  } else {
    std::ifstream infile(f_init_params, std::ios_base::in);
    param_all_.read_ascii(infile);
    assert(param_all_.cols() == d_);
    assert(param_all_.rows() == n_);
    return 1;
  }
}

void CoherentPointDrift::PerformTransform(const vnl_vector<double> &x) {
  transformed_model_ = model_ + basis_ * param_all_;
  //transformed_model = basis*param_all;
}

double CoherentPointDrift::BendingEnergy() {
  return vnl_trace(param_all_.transpose() * kernel_ * param_all_);
}

void CoherentPointDrift::SaveResults(const char* f_config)
{
  char f_transformed[256] = {0};
  GetPrivateProfileString(common_section_, "transformed_model", NULL,
      f_transformed, 255, f_config);
  SaveTransformed(f_transformed, f_config);

  char f_final_params[256] = {0};
  GetPrivateProfileString(common_section_, "final_params", NULL,
      f_final_params, 255, f_config);
  SaveMatrixToAsciiFile(f_final_params, param_all_);
}

int CoherentPointDrift::PrepareOwnOptions(const char* f_config)
{
  char s_EMtol[60] = {0}, s_anneal[60] = {0}, s_beta[60] = {0};
  char s_lambda[60] = {0}, s_outliers[60] = {0}, s_sigma[60] = {0};
  char s_tol[60] = {0}, s_viz[60] = {0};
  GetPrivateProfileString(section_, "emtol", "1e-3", s_EMtol, 60, f_config);
  EMtol = atof(s_EMtol);
  GetPrivateProfileString(section_, "anneal", "0.97", s_anneal, 60, f_config);
  anneal = atof(s_anneal);
  GetPrivateProfileString(section_, "beta", "1", s_beta, 60, f_config);
  beta_ = atof(s_beta);
  GetPrivateProfileString(section_, "lambda", "1", s_lambda, 60, f_config);
  lambda_ = atof(s_lambda);
  GetPrivateProfileString(section_, "outliers", "0", s_outliers, 60, f_config);
  outliers = atoi(s_outliers);
  GetPrivateProfileString(section_, "sigma", "1", s_sigma, 60, f_config);
  sigma_ = atof(s_sigma);
  GetPrivateProfileString(section_, "tol", "1e-5", s_tol, 60, f_config);
  tol = atof(s_tol);
  max_iter = GetPrivateProfileInt(section_, "max_iter", 150, f_config);
  max_em_iter = GetPrivateProfileInt(section_, "max_em_iter", 150, f_config);

  return 0;
}

}  // namespace gmmreg
