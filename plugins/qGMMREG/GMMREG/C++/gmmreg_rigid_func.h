#ifndef GMMREG_RIGID_FUNC_H_
#define GMMREG_RIGID_FUNC_H_

#include <vnl/vnl_cost_function.h>
#include "gmmreg_base.h"

namespace gmmreg {

class RigidFunc : public vnl_cost_function {
 public:
  RigidFunc(): vnl_cost_function() {}
  ~RigidFunc() {}

  double Eval(const double &, vnl_matrix<double>*);

  double f(const vnl_vector<double>& x) override;
  void gradf(const vnl_vector<double>& x, vnl_vector<double>& g) override;

  inline void SetScale(const double scale) {
    this->scale_ = scale;
  }
  inline double GetScale() const {
    return this->scale_;
  }


  inline void SetBase(Base* base) {
    this->base_ = base;
    this->m_ = base->m_;
    this->d_ = base->d_;
    if (d_ == 2) {
      set_number_of_unknowns(3);
    } else if (d_ == 3) {
      set_number_of_unknowns(7);
    }
    gradient_.set_size(m_, d_);
  }

 private:
  Base* base_;
  vnl_matrix<double> gradient_;

 private:
  double scale_;
  int m_, d_;
};

}  // namespace gmmreg

#endif  // #ifndef GMMREG_RIGID_FUNC_H__
