#ifndef GMMREG_TPS_FUNC_H_
#define GMMREG_TPS_FUNC_H_

#include <vnl/vnl_cost_function.h>
#include "gmmreg_base.h"

namespace gmmreg {

class ThinPlateSplineFunc : public vnl_cost_function {
 public:
  ThinPlateSplineFunc(): vnl_cost_function() {}
  virtual ~ThinPlateSplineFunc() {}

  virtual double Eval(const double f1, const double f2,
      const vnl_matrix<double>& g1, const vnl_matrix<double>& g2) = 0;
  double f(const vnl_vector<double>& x) override;
  void gradf(const vnl_vector<double>& x, vnl_vector<double>& g) override;

  inline void SetBase(Base* base) {
    this->base_ = base;
    this->m_ = base->m_;
    this->n_ = base->n_;
    this->d_ = base->d_;
    gradient1_.set_size(m_, d_);
    gradient2_.set_size(m_, d_);
  }

  inline void SetScale(const double scale) {
    this->scale_ = scale;
  }
  inline double GetScale() const {
    return this->scale_;
  }
  inline void SetLambda(const double lambda) {
    this->lambda_ = lambda;
  }
  inline double GetLambda() const {
    return this->lambda_;
  }

  void PrepareParamGradient();

  inline void SetFixAffine(const bool fix_affine) {
    this->fix_affine_ = fix_affine;
    if (fix_affine) {
      set_number_of_unknowns((n_ - d_ - 1) * d_); //dim = (n-d-1)*d;
    } else {
      set_number_of_unknowns(n_ * d_); //dim = n*d;
    }
    //gmmreg->dim = get_number_of_unknowns(); //dim;
  }
  inline bool GetFixAffine() const {
    return this->fix_affine_;
  }

  bool fix_affine_;

 protected:
  Base* base_;
  vnl_matrix<double> gradient_;

 private:
  double scale_, lambda_;
  int m_, n_, d_;
  vnl_matrix<double> gradient1_, gradient2_, grad_all_;
};

class ThinPlateSplineFunc_L2 : public ThinPlateSplineFunc {
  double Eval(const double f1, const double f2,
      const vnl_matrix<double>& g1, const vnl_matrix<double>& g2) override;
};


class ThinPlateSplineFunc_KC : public ThinPlateSplineFunc {
  double Eval(const double f1, const double f2,
      const vnl_matrix<double>& g1, const vnl_matrix<double>& g2) override;
};

}  // namespace gmmreg

#endif  // GMMREG_TPS_FUNC_H__
