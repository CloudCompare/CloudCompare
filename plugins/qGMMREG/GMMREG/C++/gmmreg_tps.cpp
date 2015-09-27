#include "gmmreg_tps.h"

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

	int TpsRegistration::PrepareInput(const char* f_config) {
		Base::PrepareInput(f_config);
		char f_ctrl_pts[256] = {0};
		GetPrivateProfileString(common_section_, "ctrl_pts", NULL,
			f_ctrl_pts, 256, f_config);
		if (SetCtrlPts(f_ctrl_pts) < 0) {
			//todo: compute the ctrl pts on the fly
			return -1;
		}
		return 0;
	}

	int TpsRegistration::StartRegistration(vnl_vector<double>& params) {
		vnl_lbfgs minimizer(*func_);
		func_->SetBase(this);
		for (unsigned int k = 0; k < level_; ++k)
		{
			func_->SetScale(v_scale_[k]);
			func_->SetLambda(v_lambda_[k]);
			bool b_fix_affine = (v_affine_[k] == 1);
			func_->SetFixAffine(b_fix_affine);
			func_->PrepareParamGradient();
			SetParam(params);
			int n_max_func_evals = v_func_evals_[k];
			minimizer.set_max_function_evals(n_max_func_evals);
			// For more options, see
			// http://public.kitware.com/vxl/doc/release/core/vnl/html/vnl__nonlinear__minimizer_8h-source.html
			minimizer.minimize(params);
			if (minimizer.get_failure_code() < 0) {
				return -1;
			}
		}
		vcl_cout << "registration done" << vcl_endl;
		return 0;
	}

	int TpsRegistration::SetInitParams(const char* f_config) {
		char f_init_affine[256] = {0}, f_init_tps[256] = {0};
		if (f_config)
		{
			GetPrivateProfileString(common_section_, "init_affine", NULL,
				f_init_affine, 256, f_config);
			GetPrivateProfileString(common_section_, "init_tps", NULL,
				f_init_tps, 256, f_config);
		}
		SetInitAffine(f_init_affine);
		SetInitTps(f_init_tps);
		param_all_.set_size(n_, d_);
		param_all_.update(param_affine_);
		param_all_.update(param_tps_, d_ + 1);
		return 0;
	}

	int TpsRegistration::SetInitAffine(const char* filename) {
		if (strlen(filename) == 0) {
			// set default affine parameters from identity transform
			assert(d_ > 0);
			param_affine_.set_size(d_ + 1, d_);
			// the first row is for translation
			param_affine_.fill(0);
			// the next dxd matrix is for affine matrix
			vnl_matrix<double> id;
			id.set_size(d_, d_);
			id.set_identity();
			param_affine_.update(id, 1);
			return 0;
		} else {
			std::ifstream infile(filename);
			param_affine_.read_ascii(infile);
			assert(param_affine_.cols() == d_);
			assert(param_affine_.rows() == (d_ + 1));
			return 1;
		}
	}

	int TpsRegistration::SetInitTps(const char* filename) {
		if (strlen(filename) == 0) {
			assert(n_ - d_ - 1 > 0);
			assert(d_ > 0);
			param_tps_.set_size(n_ - d_ - 1, d_);
			param_tps_.fill(0);
			return 0;
		} else {
			std::ifstream infile(filename, std::ios_base::in);
			param_tps_.read_ascii(infile);
			assert(param_tps_.cols() == d_);
			assert(param_tps_.rows() == (n_ - d_ - 1));
			return 1;
		}
	}

	void TpsRegistration::PrepareBasisKernel() {
		// TODO(bing-jian): detect singularity of the data
		vnl_matrix<double> K, U;
		ComputeTPSKernel(model_, ctrl_pts_, U, K);
		m_ = model_.rows();
		vnl_matrix<double> Pm;
		Pm.set_size(m_, d_ + 1);
		Pm.set_column(0, 1);
		Pm.update(model_, 0, 1);

		vnl_matrix<double> Pn;
		Pn.set_size(n_, d_ + 1);
		Pn.set_column(0, 1);
		Pn.update(ctrl_pts_, 0, 1);
		/* should use SVD(Pn), but vnl's SVD is an ``economy-size'' SVD  */

		/* vnl_svd<double> SVD(Pn.transpose());
		vnl_matrix<double> VV = SVD.V();
		std::cout << VV.rows() << " " << VV.cols() << std::endl;
		SaveMatrixToAsciiFile("./VV.txt", VV);
		*/

		vnl_qr<double> qr(Pn);
		vnl_matrix<double> V = qr.Q();
		vnl_matrix<double> PP = V.extract(n_, n_ - d_ - 1, 0, d_ + 1);
		basis_.set_size(m_, n_);
		basis_.update(Pm);
		basis_.update(U * PP, 0, d_ + 1);
		kernel_ = PP.transpose() * K * PP;
	}

	void TpsRegistration::PerformTransform(const vnl_vector<double> &x) {
		SetAffineAndTps(x);
		transformed_model_ = basis_ * param_all_;
		// SaveMatrixToAsciiFile("./param_all.txt", param_all_);
		// SaveMatrixToAsciiFile("./basis.txt", basis_);
	}

	double TpsRegistration::BendingEnergy() {
		return vnl_trace(param_tps_.transpose() * kernel_ * param_tps_);
	}

	void TpsRegistration::ComputeGradient(const double lambda,
		const vnl_matrix<double>& gradient, vnl_matrix<double>& grad_all) {
			grad_all.fill(0);
			if (lambda > 0) {
				grad_all.update(2 * lambda * kernel_ * param_tps_, d_ + 1);
			}
			grad_all += basis_.transpose() * gradient;
	}

	void TpsRegistration::SaveResults(const char* f_config)
	{
		char f_final_affine[256] = {0};
		GetPrivateProfileString(common_section_, "final_affine", NULL,
			f_final_affine, 255, f_config);

		char f_final_tps[256] = {0};
		GetPrivateProfileString(common_section_, "final_tps", NULL,
			f_final_tps, 255, f_config);

		char f_transformed[256] = {0};
		GetPrivateProfileString(common_section_, "transformed_model", NULL,
			f_transformed, 255, f_config);

		SaveTransformed(f_transformed, f_config);
		SaveMatrixToAsciiFile(f_final_affine, param_affine_);
		SaveMatrixToAsciiFile(f_final_tps, param_tps_);
	}

	int TpsRegistration::PrepareOwnOptions(const char* f_config)
	{
		assert(f_config);
		if (MultiScaleOptions(f_config) < 0)
		{
			return -1;
		}

		static const char delims[] = " -,;";

		std::vector<float> v_lambda;
		char s_lambda[256] = {0};
		GetPrivateProfileString(section_, "lambda", NULL, s_lambda, 255, f_config);
		utils::parse_tokens(s_lambda, delims, v_lambda);

		char s_affine[256] = {0};
		std::vector<int> v_affine;
		GetPrivateProfileString(section_, "fix_affine", NULL, s_affine, 255, f_config);
		utils::parse_tokens(s_affine, delims, v_affine);

		return PrepareOwnOptions(v_lambda, v_affine);
	}

	int TpsRegistration::PrepareOwnOptions(	const std::vector<float>& v_lambda,
		const std::vector<int>& v_affine)
	{
		try
		{
			v_lambda_ = v_lambda;
			v_affine_ = v_affine;
		}
		catch (const std::bad_alloc&)
		{
			std::cerr<< " not enough memory " << std::endl;
			return -1;
		}

		if (v_lambda_.size() < level_ || v_affine_.size() < level_)
		{
			std::cerr<< " too many levels " << std::endl;
			return -1;
		}

		return 0;
	}

	void TpsRegistration::SetParam(vnl_vector<double>& x0) {
		int k = 0;
		x0.set_size(func_->get_number_of_unknowns());
		x0.fill(0);
		if (!func_->fix_affine_) { // x0 includes affine
			for (unsigned i = 0; i < param_affine_.rows(); ++i) {
				for (int j = 0; j < d_; ++j, ++k) {
					x0[k] = param_affine_(i, j);
				}
			}
		}
		for (unsigned i = 0; i < param_tps_.rows(); ++i) {
			for (int j = 0; j < d_; ++j, ++k) {
				x0[k] = param_tps_(i, j);
			}
		}
	}

	void TpsRegistration::SetAffineAndTps(const vnl_vector<double>& x) {
		/* reshape x, assuming x is row major; */
		int rows_x = x.size() / d_;
		if (func_->fix_affine_) { // affine is given, x does not include affine
			param_all_.update(param_affine_);
			for (int i = 0, k = 0; i < rows_x; ++i) {
				for (int j = 0; j < d_; ++j, ++k) {
					param_tps_(i, j) = x[k];
				}
			}
			param_all_.update(param_tps_, d_ + 1);
		} else { // affine is not given, x includes affine already
			for (int i = 0, k = 0; i < rows_x; ++i) {
				for (int j = 0; j < d_; ++j, ++k) {
					param_all_(i, j) = x[k];
				}
			}
			param_affine_ = param_all_.extract(d_ + 1, d_);
			param_tps_ = param_all_.extract(rows_x - d_ - 1, d_, d_ + 1);
		}
	}

}  // namespace gmmreg
