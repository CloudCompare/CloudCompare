#ifndef GMMREG_CPD_H_
#define GMMREG_CPD_H_

#include "gmmreg_base.h"

namespace gmmreg {

class CoherentPointDrift : public Base {
 public:
  CoherentPointDrift(): eps_(0.0000000001) {
    strcpy(section_, "GMMREG_EM");
  }
  virtual ~CoherentPointDrift() {}

 private:
  int StartRegistration(vnl_vector<double>&) override;
  void SetParam(vnl_vector<double>&);
  int SetInitParams(const char*) override;
  void SaveResults(const char*) override;
  int PrepareInput(const char*) override;
  void PrepareParamGradient(bool);
  void PerformTransform(const vnl_vector<double>&) override;
  double BendingEnergy() override;
  void ComputeGradient(const double lambda,
      const vnl_matrix<double>& gradient,
      vnl_matrix<double>& grad_all) override {};
  int PrepareOwnOptions(const char*) override;
  virtual void PrepareBasisKernel() = 0;
  virtual double UpdateParam() = 0;

 protected:
  vnl_matrix<double> basis_, param_all_;
  double EMtol, tol, beta_, anneal;
  int max_iter, max_em_iter, outliers;
  //vnl_vector<double> column_sum;
  //double outlier_term;
  vnl_matrix<double> P;
  double eps_;
};

class CoherentPointDriftTps: public CoherentPointDrift {
 private:
  vnl_matrix<double> tps_;
  vnl_matrix<double> affine_;
  vnl_matrix<double> nP;
  vnl_matrix<double> G, Q1, Q2, R, invR, invG;

  void PrepareBasisKernel();
  double UpdateParam();
};

class CoherentPointDriftGrbf: public CoherentPointDrift {
 private:
  vnl_matrix<double> dPG;
  vnl_matrix<double> dPY0;
  vnl_matrix<double> Gtranspose, invG;

  void PrepareBasisKernel();
  double UpdateParam();
};

}  // namespace gmmreg

#endif  // GMMREG_CPD_H_
