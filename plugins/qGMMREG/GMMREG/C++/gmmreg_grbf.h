#ifndef GMMREG_GRBF_H_
#define GMMREG_GRBF_H_

#include <vector>

#include "gmmreg_base.h"
#include "gmmreg_grbf_func.h"

namespace gmmreg {

class GrbfRegistration: public Base {
 public:
  GrbfRegistration() {
    strcpy(section_, "GMMREG_OPT");
  }
  virtual ~GrbfRegistration() {
    delete func_;
  }

  int PrepareOwnOptions(const std::vector<float>& v_lambda);

 protected:
  GaussianRadialBasisFunc *func_;

 private:
  vnl_matrix<double> param_grbf_;
  vnl_matrix<double> after_grbf, basis_, param_all_;
  std::vector<float> v_beta;
  double beta_;
  std::vector<float> v_lambda;
  std::vector<int> v_affine;

  int StartRegistration(vnl_vector<double>&) override;
  int SetInitGrbf(const char* filename);
  void SetParam(vnl_vector<double>& x0);
  void SetGrbf(const vnl_vector<double>&);
  int SetInitParams(const char* filename) override;
  void SaveResults(const char* f_config) override;

  int PrepareInput(const vnl_matrix<double>& model, const vnl_matrix<double>& scene, const vnl_matrix<double>* control) override;
  int PrepareInput(const char* input_config) override;
  void PrepareBasisKernel();
  void PrepareParamGradient(bool);
  void PerformTransform(const vnl_vector<double>&) override;
  double BendingEnergy() override;
  void ComputeGradient(const double lambda, const vnl_matrix<double>& gradient,
      vnl_matrix<double>& grad_all) override;
  int PrepareOwnOptions(const char* f_config) override;
};

class GrbfRegistration_L2: public GrbfRegistration {
 public:
  GrbfRegistration_L2(): GrbfRegistration() {
    func_ = new GaussianRadialBasisFunc_L2;
  }
};

class GrbfRegistration_KC: public GrbfRegistration {
 public:
  GrbfRegistration_KC(): GrbfRegistration() {
    func_ = new GaussianRadialBasisFunc_KC;
  }
};

}  // namespace gmmreg

#endif  // GMMREG_GRBF_H_
