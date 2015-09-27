#ifndef GMMREG_TPS_H_
#define GMMREG_TPS_H_

#include <vector>

#include "gmmreg_base.h"
#include "gmmreg_tps_func.h"

namespace gmmreg {

class TpsRegistration: public Base {
 public:
  TpsRegistration() {
    strcpy(section_, "GMMREG_OPT");
  }
  virtual ~TpsRegistration() {
    delete func_;
  }

  int PrepareOwnOptions(const std::vector<float>& v_lambda,
						const std::vector<int>& v_affine);

 protected:
  ThinPlateSplineFunc* func_;

 private:
  vnl_matrix<double> param_affine_, param_tps_;
  vnl_matrix<double> after_tps_, basis_, param_all_;
  std::vector<float> v_lambda_;
  std::vector<int> v_affine_;

  int StartRegistration(vnl_vector<double>&) override;
  int SetInitAffine(const char* filename);
  int SetInitTps(const char* filename);
  void SetParam(vnl_vector<double>& x0);
  void SetAffineAndTps(const vnl_vector<double>&);
  int SetInitParams(const char* filename) override;
  void SaveResults(const char* f_config) override;

  int PrepareInput(const char* input_config) override;
  void PrepareBasisKernel() override;
  void PrepareParamGradient(bool);
  void PerformTransform(const vnl_vector<double>&) override;
  double BendingEnergy() override;
  void ComputeGradient(const double lambda, const vnl_matrix<double>& gradient,
      vnl_matrix<double>& grad_all) override;
  int PrepareOwnOptions(const char* f_config) override;
};

class TpsRegistration_L2: public TpsRegistration {
 public:
  TpsRegistration_L2(): TpsRegistration() {
    func_ = new ThinPlateSplineFunc_L2;
  }
};

class TpsRegistration_KC: public TpsRegistration {
 public:
  TpsRegistration_KC(): TpsRegistration() {
    func_ = new ThinPlateSplineFunc_KC;
  }
};

}  // namespace gmmreg
#endif  // GMMREG_TPS_H_
