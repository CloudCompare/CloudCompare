#ifndef GMMREG_BASE_H_
#define GMMREG_BASE_H_

#ifdef WIN32
#include <windows.h>
#else
#include "port_ini.h"
#endif

#include <vector>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

namespace gmmreg
{
	class Base
	{
	public:

		Base()
			: m_(0)
			, n_(0)
			, s_(0)
			, d_(0)
			, level_(0)
			, initialized_(false)
		{
			strcpy(common_section_, "FILES");
		}

		virtual ~Base() {}

	public: //without ini file
		int Initialize(	const vnl_matrix<double>& model,
						const vnl_matrix<double>& scene,
						const vnl_matrix<double>* control,
						int b_normalize );

		int MultiScaleOptions(	unsigned int level,
								const std::vector<float>& v_scale,
								const std::vector<int>& v_func_evals);
		int Run();

	public: //with ini file
		int Initialize(const char* f_config);

		int Run(const char* f_config);

	public: //other public methods
		virtual void PerformTransform(const vnl_vector<double>&) = 0;
		virtual double BendingEnergy() = 0;  // serving as a regularization term
		virtual void ComputeGradient(const double lambda, const vnl_matrix<double>& gradient, vnl_matrix<double>& grad_all) = 0;

		// direct access to the transformed model
		const vnl_matrix<double>& transformedModel() { return transformed_model_; }

	protected:
		/* m: # of points in model */
		/* s: # of points in scene */
		/* n: # of points in ctrl_pts */
		/* d: dimensionality, e.g. 2 for 2D points, 3 for 3D points */
		int m_, n_, s_, d_;

		// each row is a sample point
		vnl_matrix<double> model_, scene_, ctrl_pts_, transformed_model_;

		double sigma_, lambda_;
		vnl_matrix<double> kernel_;
		int b_normalize_;
		vnl_vector<double> model_centroid_, scene_centroid_;
		char section_[80], common_section_[80];

		unsigned int level_;
		std::vector<float> v_scale_;
		std::vector<int> v_func_evals_;

		// load input data from files
		virtual int PrepareInput(const vnl_matrix<double>& model, const vnl_matrix<double>& scene, const vnl_matrix<double>* control);
		virtual int PrepareInput(const char* input_config);
		int SetCtrlPts(const char* filename);
		void SaveTransformed(const char* filename, const char* f_config);
		int MultiScaleOptions(const char* f_config);
		void DenormalizeAll();

		friend class RigidFunc;
		friend class ThinPlateSplineFunc;
		friend class GaussianRadialBasisFunc;

	private:

		bool initialized_;
		double model_scale_, scene_scale_;
		int PrepareCommonOptions(int b_normalize);
		int PrepareCommonOptions(const char* f_config);
		virtual int PrepareOwnOptions(const char* f_config) = 0;
		virtual void PrepareBasisKernel() = 0;
		virtual int SetInitParams(const char* filename) = 0;
		virtual void SaveResults(const char* filename) = 0;
		virtual int StartRegistration(vnl_vector<double>& params) = 0;
	};

}  // namespace gmmreg

#endif  // GMMREG_BASE_H_
