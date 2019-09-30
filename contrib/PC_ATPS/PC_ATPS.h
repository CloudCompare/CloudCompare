#ifndef __PC_ATPS_HEADER__
#define __PC_ATPS_HEADER__

/***********************************************************************
PC_Filter
created:	2019/06
author:		xz_zhu
purpose:	PC_ATPS.dll
************************************************************************/

/*!
 * \file PC_ATPS.h
 * \brief plane segmentation dll header file
 *
 */

#include <vector>

#ifndef ATPS_NAMESPACE_BEGIN
#define ATPS_NAMESPACE_BEGIN namespace ATPS {
#endif
#ifndef ATPS_NAMESPACE_END
#define ATPS_NAMESPACE_END }
#endif

ATPS_NAMESPACE_BEGIN

#ifndef SVPOINT3D
#define SVPOINT3D
class SVPoint3d
{
public:
	SVPoint3d()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
		r = 0;
		g = 0;
		b = 0;
	}
	~SVPoint3d(){}
	SVPoint3d(double _x, double _y, double _z) :
		x(_x), y(_y), z(_z) {}
	SVPoint3d(double _x, double _y, double _z, unsigned int _r, unsigned int _g, unsigned int _b) :
		x(_x), y(_y), z(_z), r(_r), g(_g), b(_b) {}
	double X() { return x; }
	double Y() { return y; }
	double Z() { return z; }

public:
	double x;
	double y;
	double z;
	uint8_t r;
	uint8_t g;
	uint8_t b;
};
#endif

#define USE_ATPS_AS_DLL

#if defined(USE_ATPS_AS_DLL) && defined(_WIN32)
#ifdef PC_ATPS_EXPORTS
#define PC_ATPS_API __declspec(dllexport)
#else
#define PC_ATPS_API __declspec(dllimport)
#endif
#else
#define PC_ATPS_API
#endif


/*!
 * \class ATPS_Plane
 * \brief Interface to Class ATPS_Plane
 * \usage 
 *			ATPS_Plane ATPS_plane;
 *
 *			ATPS_plane.SetPoints(); 
 *			ATPS_plane.PlaneSegmentation();
 *			...
 *
 */
class PC_ATPS_API ATPS_Plane
{
public:
	ATPS_Plane();

	ATPS_Plane(int kappa_, double delta_, double tau_, double gamma_, double epsilon_, double theta_);

	ATPS_Plane(const double res);

	~ATPS_Plane();


	int get_kappa();
	double get_delta();
	double get_tau();
	double get_gamma();
	double get_epsilon();
	double get_theta();


	void set_parameters(
		const int kappa_, 
		const double delta_, 
		const double tau_, 
		const double gamma_, 
		const double epsilon_, 
		const double theta_);
	void set_kappa(const int kappa_);
	void set_delta(const double delta_);
	void set_tau(const double tau_);
	void set_gamma(const double gamma_);
	void set_epsilon(const double epsilon_);
	void set_theta(const double theta_);


	bool set_points(const std::string pc_path, std::vector<SVPoint3d>& points, double& res);


	bool ATPS_PlaneSegmentation(
		const std::vector<SVPoint3d> points,
		std::vector<std::vector<SVPoint3d>>& planar_points,
		std::vector<SVPoint3d>& nonplanar_points, 
		std::vector<std::vector<double>>& model_coefficients);


private:
	// parameters

	double r_max;
	double r_min;
	double r_delta;

	// threashold

	int kappa_t;
	double delta_t;
	double tau_t;
	double gamma_t;
	double epsilon_t;
	double theta_t;

	/*
	*---description of parameters---*
	kappa_t: the minimum point number for a valid plane. (1.0/res)
	delta_t: the threshold of curvature for multi-scale supervoxel segmentation. (0.05)
	tau_t: the threshold of distance tolerance value for point-to-plane and plane-to-plane. (0.1)
	gamma_t: the threshold of neighborhood for point-to-plane and plane-to-plane. (res*7.0)
	epsilon_t: the threshold of NFA tolerance value for a-contrario rigorous planar supervoxel generation. (0.0)
	theta_t: the threshold of normal vector angle for hybrid region growing. (10.0)
	*/

	std::string input_path;
};

ATPS_NAMESPACE_END

#endif // __PC_ATPS_HEADER__