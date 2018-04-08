/* License Information
 *
 *  Copyright (C) ONERA, The French Aerospace Lab
 *  Author: Alexandre BOULCH
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this
 *  software and associated documentation files (the "Software"), to deal in the Software
 *  without restriction, including without limitation the rights to use, copy, modify, merge,
 *  publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
 *  to whom the Software is furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all copies or
 *  substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 *  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 *  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 *  OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 *  Note that this library relies on external libraries subject to their own license.
 *  To use this software, you are subject to the dependencies license, these licenses
 *  applies to the dependency ONLY  and NOT this code.
 *  Please refer below to the web sites for license informations:
 *       PCL, BOOST,NANOFLANN, EIGEN
 *
 * When using the software please aknowledge the  corresponding publication:
 * "Deep Learning for Robust Normal Estimation in Unstructured Point Clouds "
 * by Alexandre Boulch and Renaud Marlet
 * Symposium of Geometry Processing 2016, Computer Graphics Forum
 */



#ifndef NORMALS_HEADER
#define NORMALS_HEADER

#include <vector>
#include <iostream>
#include <ctime>
#include <math.h>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <nanoflann.hpp>

#ifdef _OPENMP
#include <omp.h>

#define USE_OPENMP_FOR_NORMEST
#endif

class Eigen_Normal_Estimator{

private:

	const Eigen::MatrixX3d& pts;/*!< Point cloud*/
	Eigen::MatrixX3d& nls;/*!< Normal cloud*/
	std::vector<double> densities; /*!< vector of the densities*/

	////  PARAMETERS  ////
	int n_planes; /*!< Plane number to draw*/
	int n_phi;/*!< Accumulator discretization parameter*/
	int n_rot;/*!< Rotation number*/
	size_t neighborhood_size; /*size of the neighborhood*/
	bool use_density; /*!< use a density estimation of triplets generation*/
	double tol_angle_rad;/*!< Angle parameter for cluster normal selection*/
	size_t k_density; /*!< size of the neighborhood for density estimation*/

	std::function<void(int)> progressCallback;

public:

	//accessor
	const Eigen::MatrixX3d& get_points()const {return pts;}

	Eigen::MatrixX3d& get_normals(){return nls;}
	int& get_T() { return n_planes; }
	int& get_n_phi() { return n_phi; }
	int& get_n_rot() { return n_rot; }
	size_t& get_K() { return neighborhood_size; }
	bool& density_sensitive() { return use_density; }
	double& get_tol_angle_rad() { return tol_angle_rad; }
	size_t& get_K_density() { return k_density; }

	const Eigen::MatrixX3d& get_normals()const {return nls;}
	const int& get_T() const {return n_planes;}
	const int& get_n_phi() const {return n_phi;}
	const int& get_n_rot() const {return n_rot;}
	const size_t& get_K() const { return neighborhood_size; }
	const bool& density_sensitive() const {return use_density;}
	const double& get_tol_angle_rad() const {return tol_angle_rad;}
	const size_t& get_K_density() const { return k_density; }

	////  TYPE DEFINITIONS  ////

	typedef nanoflann::KDTreeEigenMatrixAdaptor< Eigen::MatrixX3d > kd_tree; //a row is a point


	// constructor
	Eigen_Normal_Estimator(const Eigen::MatrixX3d& points, Eigen::MatrixX3d& normals)
		: pts(points)
		, nls(normals)
	{
		n_planes = 700;
		n_rot = 5;
		n_phi = 15;
		tol_angle_rad = 0.79;
		neighborhood_size = 200;
		use_density = false;
		k_density = 5;
	}

	void setProgressCallback(std::function<void(int)> callback)
	{
		progressCallback = callback;
	}

	int maxProgressCounter() const
	{
		return pts.rows() * 2;
	}

	void estimate_normals()
	{

		/*********************************
		 * INIT
		 ********************************/

		//initialize the random number generator
		srand(static_cast<unsigned int>(time(NULL)));

		//creating vector of random int
		std::vector<size_t> vecInt(1000000);
		for (size_t i = 0; i < vecInt.size(); i++)
		{
			vecInt[i] = static_cast<size_t>(rand());
		}

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for (int i = 0; i < n_planes; i++)
		{
			conf_interv[i] = 2.f / std::sqrt(i + 1.f);
		}

		//random permutation of the points (avoid thread difficult block)
		std::vector<int> permutation(pts.rows());
		for (int i = 0; i < pts.rows(); i++)
		{
			permutation[i] = i;
		}
		for (int i = 0; i < pts.rows(); i++)
		{
			int j = rand() % pts.rows();
			std::swap(permutation[i], permutation[j]);
		}

		//creation of the rotation matrices and their inverses
		std::vector<Eigen::Matrix3d> rotMat;
		std::vector<Eigen::Matrix3d> rotMatInv;
		generate_rotation_matrix(rotMat,rotMatInv, n_rot*200);

		//dimensions of the accumulator
		int d1 = 2*n_phi;
		int d2 = n_phi+1;

		//progress
		int progress = 0;


		/*******************************
		 * ESTIMATION
		 ******************************/

		//resizing the normal point cloud
		nls.resize(pts.rows(), 3);

		//kd tree creation
		//build de kd_tree
		kd_tree tree(3, pts, 10 /* max leaf */ );
		tree.index->buildIndex();

		//create the density estimation for each point
		densities.resize(pts.rows());
#if defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for (int per = 0; per < pts.rows(); per++)
		{
			//index of the point
			int n = permutation[per];
			//getting the list of neighbors
			const Eigen::Vector3d& pt_query = pts.row(n);
			std::vector<Eigen::MatrixX3d::Index> pointIdxSearch(k_density + 1);
			std::vector<double> pointSquaredDistance(k_density + 1);
			//knn for k_density+1 because the point is itself include in the search tree
			tree.index->knnSearch(&pt_query[0], k_density + 1, &pointIdxSearch[0], &pointSquaredDistance[0]);
			double d = 0;
			for (size_t i = 0; i < pointSquaredDistance.size(); i++)
			{
				d += std::sqrt(pointSquaredDistance[i]);
			}
			d /= pointSquaredDistance.size() - 1;
			densities[n] = d;

			if (progressCallback)
			{
				progressCallback(++progress);
			}
		}


		int rotations = std::max(n_rot,1);

		//create the list of triplets in KNN case
		Eigen::MatrixX3i trip;
		if (!use_density)
		{
			list_of_triplets(trip, neighborhood_size, rotations*n_planes, vecInt);
		}

#if defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for (int per = 0; per < pts.rows(); per++)
		{
			//index of the point
			int n = permutation[per];

			//getting the list of neighbors
			std::vector<Eigen::MatrixX3d::Index> pointIdxSearch;
			std::vector<double> pointSquaredDistance;

			const Eigen::Vector3d& pt_query = pts.row(n);
			pointIdxSearch.resize(neighborhood_size);
			pointSquaredDistance.resize(neighborhood_size);
			tree.index->knnSearch(&pt_query[0], neighborhood_size, &pointIdxSearch[0], &pointSquaredDistance[0]);

			if (use_density)
				list_of_triplets(trip, rotations*n_planes, pointIdxSearch, vecInt);

			//get the points
			size_t points_size = pointIdxSearch.size();
			Eigen::MatrixX3d points(points_size, 3);
			for (size_t pt = 0; pt<pointIdxSearch.size(); pt++)
			{
				points.row(pt) = pts.row(pointIdxSearch[pt]);
			}

			std::vector<Eigen::Vector3d> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for (int i = 0; i < rotations; i++)
			{
				Eigen::MatrixX3i triplets = trip.block(i*n_planes, 0, n_planes, 3);

				for (size_t pt = 0; pt < points_size; pt++)
				{
					points.row(pt) = rotMat[(n + i) % rotMat.size()] * points.row(pt).transpose();
				}
				normals_conf[i] = normal_at_point(d1, d2, points, n, triplets, conf_interv);

				for (size_t pt = 0; pt < points_size; pt++)
				{
					points.row(pt) = pts.row(pointIdxSearch[pt]);
				}
				normals_vec[i] = rotMatInv[(n + i) % rotMat.size()] * nls.row(n).transpose();

			}

			nls.row(n) = normal_selection(rotations, normals_vec, normals_conf);

			if (progressCallback)
			{
				progressCallback(++progress);
			}
		}

	}

private:

	// PRIVATE METHODS

	/*!
	 * fills a vector of random rotation matrix and their inverse
	 * @param rotMat : table matrices to fill with rotations
	 * @param rotMatInv : table matrices to fill with inverse rotations
	 * @param rotations : number of rotations
	 */
	inline void generate_rotation_matrix(std::vector<Eigen::Matrix3d> &rotMat, std::vector<Eigen::Matrix3d> &rotMatInv, int rotations)
	{
		rotMat.clear();
		rotMatInv.clear();

		if (rotations == 0)
		{
			Eigen::Matrix3d rMat;
			rMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
			rotMat.push_back(rMat);
			rotMatInv.push_back(rMat);
		}
		else
		{
			for (int i = 0; i < rotations; i++)
			{
				double theta = static_cast<double>(rand()) / RAND_MAX * 2 * M_PI;
				double phi =   static_cast<double>(rand()) / RAND_MAX * 2 * M_PI;
				double psi =   static_cast<double>(rand()) / RAND_MAX * 2 * M_PI;
				Eigen::Matrix3d Rt;
				Eigen::Matrix3d Rph;
				Eigen::Matrix3d Rps;
				Rt << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
				Rph << cos(phi), 0, sin(phi), 0, 1, 0, -sin(phi), 0, cos(phi);
				Rps << cos(psi), -sin(psi), 0, sin(psi), cos(psi), 0, 0, 0, 1;
				Eigen::Matrix3d Rtinv;
				Eigen::Matrix3d Rphinv;
				Eigen::Matrix3d Rpsinv;
				Rtinv << 1, 0, 0, 0, cos(theta), sin(theta), 0, -sin(theta), cos(theta);
				Rphinv << cos(phi), 0, -sin(phi), 0, 1, 0, sin(phi), 0, cos(phi);
				Rpsinv << cos(psi), sin(psi), 0, -sin(psi), cos(psi), 0, 0, 0, 1;

				Eigen::Matrix3d rMat = Rt*Rph*Rps;
				Eigen::Matrix3d rMatInv = Rpsinv*Rphinv*Rtinv;
				rotMat.push_back(rMat);
				rotMatInv.push_back(rMatInv);
			}
		}
	}


	/*!
	 * generates a list of triplets
	 * @param triplets : table of 3-vector to fill with the indexes of the points
	 * @param number_of_points : number of points to consider
	 * @param triplet_number : number of triplets to generate
	 * @param vecRandInt : table of random int
	 */
	inline void list_of_triplets(Eigen::MatrixX3i &triplets,
			size_t number_of_points,
			size_t triplet_number,
			const std::vector<size_t> &vecRandInt)
	{
		size_t S = vecRandInt.size();
		triplets.resize(triplet_number, 3);
		size_t pos = vecRandInt[0] % S;
		for (size_t i = 0; i < triplet_number; i++)
		{
			do
			{
				triplets(i, 0) = static_cast<int>(vecRandInt[pos % S] % number_of_points);
				triplets(i, 1) = static_cast<int>(vecRandInt[(pos + vecRandInt[(pos + 1) % S]) % S] % number_of_points);
				triplets(i, 2) = static_cast<int>(vecRandInt[(pos + vecRandInt[(pos + 1 + vecRandInt[(pos + 2) % S]) % S]) % S] % number_of_points);
				pos += vecRandInt[(pos + 3) % S] % S;
			}
			while (triplets(i, 0) == triplets(i, 1) || triplets(i, 1) == triplets(i, 2) || triplets(i, 2) == triplets(i, 0));
		}
	}

	/*!
	 * dichotomic search in sorted vector, find the nearest neighbor
	 * @param elems : sorted vector containing the elements for comparison
	 * @param d : element to search for in elems
	 * @return the index of the nearest neighbor of d in elems
	 */
	//return the index of the nearest element in the vector
	int dichotomic_search_nearest(const std::vector<double> elems, double d){
		size_t i1 = 0;
		size_t i2 = elems.size() - 1;
		size_t i3;
		while(i2 > i1){
			i3 = (i1+i2)/2;
			if(elems[i3] == d){break;}
			if(d < elems[i3]){i2 = i3;}
			if(d > elems[i3]){i1 = i3;}
		}
		return static_cast<int>(i3);
	}

	/*!
	 * generates a list of triplets
	 * @param triplets : table of 3-vector to fill with the indexes of the points
	 * @param triplet_number : number of triplets to generate
	 * @param pointIdxSearch : index of the points used for triplets
	 * @param vecRandInt : table of random int
	 */
	inline void list_of_triplets(Eigen::MatrixX3i &triplets,
		size_t triplet_number,
		const std::vector<Eigen::MatrixX3d::Index>& pointIdxSearch,
		const std::vector<size_t> &vecRandInt)
	{
		std::vector<double> dists;
		double sum = 0;
		for (size_t i = 0; i < pointIdxSearch.size(); i++)
		{
			sum += densities[pointIdxSearch[i]];
			dists.push_back(sum);
		}

		size_t S = vecRandInt.size();
		size_t number_of_points = pointIdxSearch.size();
		triplets.resize(triplet_number, 3);
		size_t pos = vecRandInt[0] % S;;
		for (size_t i = 0; i < triplet_number; i++)
		{
			do
			{
				double d = (vecRandInt[pos % S] + 0.) / RAND_MAX *sum;
				triplets(i, 0) = dichotomic_search_nearest(dists, d);
				d = (vecRandInt[(pos + vecRandInt[(pos + 1) % S]) % S] + 0.) / RAND_MAX;
				triplets(i, 1) = dichotomic_search_nearest(dists, d);
				d = (vecRandInt[(pos + vecRandInt[(pos + 1 + vecRandInt[(pos + 2) % S]) % S]) % S] + 0.) / RAND_MAX;
				triplets(i, 2) = dichotomic_search_nearest(dists, d);
				pos += vecRandInt[(pos + 3) % S] % S;
			}
			while (triplets(i, 0) == triplets(i, 1) || triplets(i, 1) == triplets(i, 2) || triplets(i, 2) == triplets(i, 0));
		}
	}

	/*!
	 * Compute the normal by filling an accumulator for a given neighborhood
	 * @param d1 - First dimension of the accumulator
	 * @param d2 - Second dimension of the accumulator
	 * @param points - table of neighbors
	 * @param n - index of the point where the normal is computed
	 * @param triplets - table of triplets
	 * @param conf_interv - table of confidence intervals
	 */
	float normal_at_point(
			const int d1, const int d2,
			const Eigen::MatrixX3d& points,
			int n,
			Eigen::MatrixX3i &triplets,
			std::vector<float> &conf_interv)
	{
		if (points.size() < 3)
		{
			nls.row(n).setZero();
			return 0;
		}

		//creation and initialization accumulators
		std::vector<double> votes(d1*d2);
		std::vector<Eigen::Vector3d> votesV(d1*d2);
		for (int i = 0; i < d1; i++)
		{
			for (int j = 0; j < d2; j++)
			{
				votes[i + j*d1] = 0;
				votesV[i + j*d1] = Eigen::Vector3d(0, 0, 0);
			}
		}

		float max1 = 0;
		int i1 = 0, i2 = 0;
		int j1 = 0, j2 = 0;

		for (int n_try = 0; n_try < n_planes; n_try++)
		{
			int p0 = triplets(n_try,0);
			int p1 = triplets(n_try,1);
			int p2 = triplets(n_try,2);

			Eigen::Vector3d v1 = points.row(p1).transpose()-points.row(p0).transpose();
			Eigen::Vector3d v2 = points.row(p2).transpose()-points.row(p0).transpose();

			Eigen::Vector3d Pn = v1.cross(v2);
			Pn.normalize();
			if(Pn.dot(points.row(p0).transpose())>0){
				Pn = -Pn;
			}

			double phi = acos(Pn[2]);
			double dphi = M_PI / n_phi;
			int posp = static_cast<int>(floor((phi + dphi / 2) * n_phi / M_PI));
			int post;

			if (posp == 0 || posp == n_phi)
			{
				post = 0;
			}
			else
			{
				double theta = acos(Pn[0] / sqrt(Pn[0] * Pn[0] + Pn[1] * Pn[1]));
				if (Pn[1] < 0)
				{
					theta *= -1;
					theta += 2 * M_PI;
				}
				double dtheta = M_PI / (n_phi*sin(posp*dphi));
				post = static_cast<int>(floor((theta + dtheta / 2) / dtheta)) % (2 * n_phi);
			}

			post = std::max(0, std::min(2 * n_phi - 1, post));
			posp = std::max(0, std::min(n_phi, posp));

			votes[post + posp*d1] += 1.;
			votesV[post + posp*d1] += Pn;

			max1 = votes[i1 + j1*d1] / (n_try + 1);
			float max2 = votes[i2 + j2*d1] / (n_try + 1);
			float votes_val = votes[post + posp*d1] / (n_try + 1);

			if (votes_val > max1)
			{
				max2 = max1;
				i2 = i1;
				j2 = j1;
				max1 = votes_val;
				i1 = post;
				j1 = posp;
			}
			else if (votes_val > max2 && post != i1 && posp != j1)
			{
				max2 = votes_val;
				i2 = post;
				j2 = posp;
			}

			if (max1 - conf_interv[n_try] > max2)
			{
				break;
			}
		}

		votesV[i1 + j1*d1].normalize();
		nls.row(n) = votesV[i1 + j1*d1];

		return max1;
	}


	/*!
	 * Compute the normal depending of the estimation choice (mean, best, cluster)
	 * @param rotations - number of rotations
	 * @param normals_vec - table of estimated normals for the point
	 * @param normals_conf - table of the confidence of normals
	 */
	inline Eigen::Vector3d normal_selection(int rotations,
			std::vector<Eigen::Vector3d> &normals_vec,
			const std::vector<float> &normals_conf){

		std::vector<bool> normals_use(rotations, true);
		//alignement of normals
		for (int i = 1; i < rotations; i++)
		{
			if (normals_vec[0].dot(normals_vec[i]) < 0)
			{
				normals_vec[i] *= -1;
			}
		}

		Eigen::Vector3d normal_final;
		std::vector< std::pair<Eigen::Vector3d, float> > normals_fin;
		int number_to_test = rotations;
		while (number_to_test > 0)
		{
			//getting the max
			float max_conf = 0;
			int idx = 0;
			for (int i = 0; i < rotations; i++)
			{
				if (normals_use[i] && normals_conf[i] > max_conf)
				{
					max_conf = normals_conf[i];
					idx = i;
				}
			}

			normals_fin.push_back(std::pair<Eigen::Vector3d, float>(normals_vec[idx] * normals_conf[idx], normals_conf[idx]));
			normals_use[idx] = false;
			number_to_test--;

			for (int i = 0; i < rotations; i++)
			{
				if (normals_use[i] && acos(normals_vec[idx].dot(normals_vec[i])) < tol_angle_rad)
				{
					normals_use[i] = false;
					number_to_test--;
					normals_fin.back().first += normals_vec[i] * normals_conf[i];
					normals_fin.back().second += normals_conf[i];
				}
			}
		}

		normal_final = normals_fin[0].first;
		float conf_fin = normals_fin[0].second;
		for (size_t i = 1; i < normals_fin.size(); i++)
		{
			if (normals_fin[i].second > conf_fin)
			{
				conf_fin = normals_fin[i].second;
				normal_final = normals_fin[i].first;
			}
		}

		normal_final.normalize();
		return normal_final;
	}
};

#endif
