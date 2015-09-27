#ifndef GMMREG_RIGID_H_
#define GMMREG_RIGID_H_

#include <assert.h>

#include <iostream>
#include <fstream>

#include "gmmreg_base.h"
#include "gmmreg_rigid_func.h"

namespace gmmreg {

class RigidRegistration: public Base {
 public:
  RigidRegistration() {
    strcpy(this->section_, "GMMREG_OPT");
    func_ = new RigidFunc;
  }
  ~RigidRegistration() {
    delete func_;
  }

 private:
  vnl_vector<double> param_rigid_;
  RigidFunc* func_;

  int StartRegistration(vnl_vector<double>&) override;
  int SetInitRigid(const char*);
  void SetParam(vnl_vector<double>&);
  int SetInitParams(const char*) override;
  void SaveResults(const char*) override;
  void PrepareBasisKernel() {};
  void PrepareParamGradient(bool) {};
  void PerformTransform(const vnl_vector<double>&) override;
  double BendingEnergy() override {
    return 0;
  };
  void ComputeGradient(const double lambda, const vnl_matrix<double>& gradient, vnl_matrix<double>& grad_all) override {};
  int PrepareOwnOptions(const char*) override;
};

}  // namespace gmmreg

#endif  // GMMREG_RIGID_H_
