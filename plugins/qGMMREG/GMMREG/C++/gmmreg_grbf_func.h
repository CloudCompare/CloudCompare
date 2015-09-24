#ifndef GaussianRadialBasisFunc_H_
#define GaussianRadialBasisFunc_H_

#include <vnl/vnl_cost_function.h>
#include "gmmreg_base.h"

namespace gmmreg {

class GaussianRadialBasisFunc : public vnl_cost_function {
 public:
  GaussianRadialBasisFunc(): vnl_cost_function(), m_(0), n_(0), d_(0) {}
  virtual ~GaussianRadialBasisFunc() {}

  virtual double Eval(const double f1, const double f2,
      vnl_matrix<double>& g1, vnl_matrix<double>& g2) = 0;
  double f(const vnl_vector<double>& x);
  void gradf(const vnl_vector<double>& x, vnl_vector<double>& g);

  inline void SetBase(Base* base) {
    this->base_ = base;
    this->m_ = base->m_;
    this->n_ = base->n_;
    this->d_ = base->d_;
    gradient1_.set_size(m_, d_);
    gradient2_.set_size(m_, d_);
    set_number_of_unknowns(n_ * d_);
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
  inline void SetBeta(const double beta) {
    this->beta_ = beta;
  }
  inline double GetBeta() const {
    return this->beta_;
  }
  void PrepareParamGradient();

 protected:
  Base* base_;
  vnl_matrix<double> gradient_;

 private:
  double scale_, lambda_, beta_;
  int m_, n_, d_;
  vnl_matrix<double> gradient1_, gradient2_, grad_all_;
};

class GaussianRadialBasisFunc_L2 : public GaussianRadialBasisFunc {
  double Eval(const double f1, const double f2,
      vnl_matrix<double>& g1, vnl_matrix<double>& g2);
};

class GaussianRadialBasisFunc_KC : public GaussianRadialBasisFunc {
  double Eval(const double f1, const double f2,
      vnl_matrix<double>& g1, vnl_matrix<double>& g2);
};

}  // namespace gmmreg

#endif  // GMMREG_GRBF_FUNC_H_
