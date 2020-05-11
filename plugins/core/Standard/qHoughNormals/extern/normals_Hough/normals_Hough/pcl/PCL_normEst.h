/* License Information
 *
 *  Copyright (C) 2013 Boulch Alexandre, Ecole Nationale des Ponts et Chaussees
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
 *  Note that this library relies on external libraries subject to their own license.
 *  To use this software, you are subject to the dependencies license, these licenses 
 *  applies to the dependency ONLY  and NOT this code.
 *  Please refer below to the web sites for license informations.
 *
 *  OPENMP (http://openmp.org/)
 *  POINT CLOUD LIBRARIES (http://pointclouds.org/)
 */


#ifndef NORM_EST_PCL_OMP_H
#define NORM_EST_PCL_OMP_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#include <omp.h>
#endif

#include <iostream>	
#include <time.h>
#include <vector>
#include <set>
#include <map>

/*!
 * \file PCL_normEst.h
 * \brief Normal Estimator for point clouds
 * \author Alexandre Bouclh
 * \version 0.1
 *
 * Normal estimator based on the publication from SGP 2012
 * "Fast and Robust Normal Estimator for Point Clouds"
 */

/*!
 * \class PCL_Normal_Estimator
 * \brief Class grouping different variants of the algorithm
 * Class using a dependency to Point Cloud Library
 */

template< class Point, class Normal>
class PCL_Normal_Estimator{


public:
	enum{MEAN=0, /*!<MEAN value 0, used for normal computation by mean*/
		BEST=1, /*!<BEST value 1, used for normal computation by best confidence*/
		CLUSTER=2, /*!<CLUSTER value 2, used for normal computation by clustering*/
		POINTS=100, /*<POINTS value 100, used for method choice, triplets by random selection in the neighborhood*/
		UNIF=101, /*!<UNIF value 101, used for method choice, triplets by uniform selection in the neighborhood sphere*/
		CUBES=102,/*!<CUBES value 102, used for method choice, triplets by selection with cubes discretization*/
		KNN=200,/*!<KNN value 200, neighborhood selection type, k-nearest neighbors*/
		RADIUS=201 /*!<RADIUS value 201, neighborhood selection type, range search*/
	};

	/*!
	 * \brief Constructor
	 * @param points
	 * @param normals
	 */
	PCL_Normal_Estimator(typename pcl::PointCloud<Point>::Ptr points, typename pcl::PointCloud<Normal>::Ptr normals):
		pts(points),nls(normals){
		set_default_parameters();
	}

	int& number_of_planes(){return n_planes;}
	int number_of_planes() const{return n_planes;}
	int& accum_slices(){return n_phi;}
	int accum_slices() const{return n_phi;}
	int& rotation_number(){return n_rot;}
	int rotation_number() const{return n_rot;}
	int& normal_selection_mode(){return selection_type;}
	int normal_selection_mode() const{return selection_type;}
	float& cluster_angle_rad(){return tol_angle_rad;}
	float cluster_angle_rad() const{return tol_angle_rad;}

	int& minimal_neighbor_number_for_range_search(){return lower_neighbor_bound_neighbors;}
	int minimal_neighbor_number_for_range_search() const{return lower_neighbor_bound_neighbors;}
	float& small_radius_fact(){return small_radius_factor;}
	float small_radius_fact() const{return small_radius_factor;}
	int& number_of_cubes(){return n_cubes;}
	int number_of_cubes() const{return n_cubes;}

	typename pcl::PointCloud<Point>::Ptr& point_cloud(){return pts;}
	typename pcl::PointCloud<Point>::Ptr point_cloud()const{return pts;}
	typename pcl::PointCloud<Normal>::Ptr& normal_cloud(){return nls;}
	typename pcl::PointCloud<Normal>::Ptr normal_cloud() const{return nls;}

	void estimate(int method=POINTS, int neighborhood_type=KNN, float neighborhood_size=200){
		std::cout << "Normal_Estimation ";
		switch(method){
		case POINTS:
			std::cout << "Points ";
			switch(neighborhood_type){
			case KNN:
				std::cout << "Knn=";
				std::cout << (int)neighborhood_size << std::endl;
				points_knn((int)neighborhood_size);
				break;
			case RADIUS:
				std::cout << "radius=";
				std::cout << neighborhood_size << std::endl;
				points_radius(neighborhood_size);
				break;
			default:
				std::cout << "Parameter Error : bad neighborhood type" << std::endl;
				break;
			}
			break;
			case UNIF:
				std::cout << "Unif ";
				switch(neighborhood_type){
				case KNN:
					std::cout << "Knn=";
					std::cout << (int)neighborhood_size << std::endl;
					unif_knn((int)neighborhood_size);
					break;
				case RADIUS:
					std::cout << "radius=";
					std::cout << neighborhood_size << std::endl;
					unif_radius(neighborhood_size);
					break;
				default:
					std::cout << "Parameter Error : bad neighborhood type" << std::endl;
					break;
				}
				break;
				case CUBES:
					std::cout << "Cubes ";
					switch(neighborhood_type){
					case KNN:
						std::cout << "Knn=";
						std::cout << (int)neighborhood_size << std::endl;
						cubes_knn((int)neighborhood_size);
						break;
					case RADIUS:
						std::cout << "radius=";
						std::cout << neighborhood_size << std::endl;
						cubes_radius(neighborhood_size);
						break;
					default:
						std::cout << "Parameter Error : bad neighborhood type" << std::endl;
						break;
					}
					break;
					default:
						std::cout << "Parameter Error : bad method" << std::endl;
						break;
		}
	}

private:


	float PI;/*!<Pi value used in the class*/
	int lower_neighbor_bound_neighbors; /*!<lower_neighbor_bound_neighbors minimal number of neighbors in radius search*/

	int n_planes;/*!< Plane number to draw*/
	int n_phi;/*!< Accumulator discretization parameter*/
	int n_rot;/*!< Rotation number*/
	float tol_angle_rad;/*!< Angle parameter for cluster normal selection*/
	float small_radius_factor;/*!< Factor such that small_radius (for uniform search) = big_radius / small_radius_factor*/
	int n_cubes;/*!< Number of cubes in one dimension*/


	int selection_type;/*!< Type of selection of normals (1: best, 2: cluster, default: mean)*/
	typename pcl::PointCloud<Point>::Ptr pts;/*!< Point cloud*/
	typename pcl::PointCloud<Normal>::Ptr nls;/*!< Normal cloud*/

	/*!
	 * Funtion to set the default paramters (hard coded)
	 */
	void set_default_parameters(){
		PI=3.14159265f;
		n_planes=700;
		n_rot=5;
		n_phi=15;
		tol_angle_rad=0.79f;
		small_radius_factor=4;
		n_cubes=4;
		lower_neighbor_bound_neighbors=10;
		selection_type= CLUSTER;
	}


	/*!
	 * fills a vector of random rotation matrix and their inverse
	 * @param rotMat : table matrices to fill with rotations
	 * @param rotMatInv : table matrices to fill with inverse rotations
	 * @param rotations : number of rotations
	 */
	inline void generate_rotation_matrix(std::vector<Eigen::Matrix3f> &rotMat, std::vector<Eigen::Matrix3f> &rotMatInv, int rotations)
	{
		rotMat.clear();
		rotMatInv.clear();

		if(rotations==0){
			Eigen::Matrix3f rMat;
			rMat << 1,0,0,0,1,0,0,0,1;
			rotMat.push_back(rMat);
			rotMatInv.push_back(rMat);
		}else{
			for(int i=0; i<rotations; i++){
				float theta = (rand()+0.f)/RAND_MAX * 2* 3.14159265f;
				float phi = (rand()+0.f)/RAND_MAX * 2* 3.14159265f;
				float psi = (rand()+0.f)/RAND_MAX * 2* 3.14159265f;
				Eigen::Matrix3f Rt;
				Eigen::Matrix3f Rph;
				Eigen::Matrix3f Rps;
				Rt <<  1, 0, 0,0, cos(theta), -sin(theta),	0, sin(theta), cos(theta);
				Rph << cos(phi),0, sin(phi),0,1,0,-sin(phi),0, cos(phi);
				Rps << cos(psi), -sin(psi), 0,	sin(psi), cos(psi),0,0,0,1;
				Eigen::Matrix3f Rtinv;
				Eigen::Matrix3f Rphinv;
				Eigen::Matrix3f Rpsinv;
				Rtinv <<  1, 0, 0,0, cos(theta) , sin(theta),0, -sin(theta), cos(theta);
				Rphinv << cos(phi) , 0, -sin(phi),0, 1, 0,sin(phi), 0, cos(phi);
				Rpsinv << cos(psi) , sin(psi), 0,	-sin(psi), cos(psi), 0,	0, 0, 1;

				Eigen::Matrix3f rMat = Rt*Rph*Rps;
				Eigen::Matrix3f rMatInv = Rpsinv*Rphinv*Rtinv;
				rotMat.push_back(rMat);
				rotMatInv.push_back(rMatInv);
			}
		}
	}

	/*!
	 * fills a vector of random points in the unit ball
	 * @param points : table of points to fill with random points
	 * @param point_number : number of points
	 */
	inline void generate_random_points_vector(std::vector<Eigen::Vector3f> &points, int point_number){
		points.resize(point_number);
		for(int i=0; i<point_number; i++){
			float x,y,z;
			do{
				x = ((rand()+0.f)/RAND_MAX)*2-1;
				y = ((rand()+0.f)/RAND_MAX)*2-1;
				z = ((rand()+0.f)/RAND_MAX)*2-1;
			}while(x*x + y*y +z*z >1);

			points[i][0] = x;
			points[i][1] = y;
			points[i][2] = z;
		}
	}

	/*!
	 * fills a vector with random int
	 * @param vecInt : table of interger to fill with random int
	 * @param point_number : number of points
	 */
	inline void generate_random_int_vector(std::vector<int> &vecInt, int point_number){
		vecInt.resize(point_number);
		for(int i=0; i<point_number; i++){
			vecInt[i] = rand();
		}
	}

	/*!
	 * generates a list of triplets without double entry, according to the combinatorial number system
	 * @param triplets : table of 3-vector to fill with the indexes of the points
	 * @param number_of_points : number of points to consider
	 * @param plane_number : number of triplets to generate
	 * @param vecInt : table of random int
	 */
	inline void list_of_triplets(std::vector<Eigen::Vector3i> &triplets,
			const int &number_of_points,
			const unsigned int &plane_number,
			std::vector<int> &vecInt){

		/*
		 * Here we take care of not using twice the same plane
		 * For that we use the combinatorial number system
		 */

		//computing the number of permutations
		unsigned long long total_comb = number_of_points;
		total_comb *=(number_of_points-1);
		total_comb *= (number_of_points-2);
		total_comb/=6;


		std::vector<unsigned long long> tab_binome_3(number_of_points+1);
		std::vector<unsigned long long> tab_binome_2(number_of_points+1);

		for(int i=0; i<number_of_points+1; i++){
			if(i>3){
				tab_binome_3[i] = tab_binome_3[i-1] * i /(i-3);
			}else if(i==3){
				tab_binome_3[i] = 1;
			}else{
				tab_binome_3[i] = 0;
			}
			if(i>2){
				tab_binome_2[i] = tab_binome_2[i-1] * i /(i-2);
			}else if(i==2){
				tab_binome_2[i] = 1;
			}else{
				tab_binome_2[i] = 0;
			}
		}



		std::vector<unsigned long long> comb_idx(plane_number);
		for(int i=0; i<plane_number; i++){
			comb_idx[i] = i%total_comb;
		}
		if(total_comb < RAND_MAX){
			std::map<int,int> table_next;
			for(int i=0; i<plane_number; i++){
				int temp_idx = vecInt[i%vecInt.size()]%total_comb;
				if(temp_idx < plane_number){
					int temp = comb_idx[i];
					comb_idx[i] = comb_idx[temp_idx];
					comb_idx[temp_idx] = temp;
				}else{
					std::map<int,int>::iterator itmap = table_next.find(temp_idx);
					if(itmap != table_next.end()){
						int temp = comb_idx[i];
						comb_idx[i] = itmap->second;
						itmap->second = temp;
					}else{
						comb_idx[i] = temp_idx;
						table_next.insert(std::pair<int,int>(temp_idx,i));
					}
				}
			}
		}else{
			unsigned long long ref_RAND_MAX = RAND_MAX;
			int size_test = 0;
			while(ref_RAND_MAX < total_comb){
				size_test++;
				ref_RAND_MAX*=RAND_MAX;
			}
			std::map<unsigned long long ,unsigned long long > table_next;
			int pos=0;
			for(int i=0; i<plane_number; i++){
				//generating a random int
				unsigned long long random_int=vecInt[pos%vecInt.size()];
				pos++;
				for(int j=0; j<size_test; j++){
					random_int += ((unsigned long long)vecInt[pos%vecInt.size()])*RAND_MAX*(j+1);
					pos++;
				}

				random_int = random_int % total_comb;
				if(random_int < plane_number){
					int temp = comb_idx[i];
					comb_idx[i] = comb_idx[random_int];
					comb_idx[random_int] = temp;
				}else{
					std::map<unsigned long long,unsigned long long>::iterator itmap = table_next.find(random_int);
					if(itmap != table_next.end()){
						int temp = comb_idx[i];
						comb_idx[i] = itmap->second;
						itmap->second = temp;
					}else{
						comb_idx[i] = random_int;
						table_next.insert(std::pair<unsigned long long,unsigned long long>(random_int,i));
					}
				}
			}
		}

		//getting the triplets from the numbers
		triplets.resize(plane_number);
		for(int pos = 0; pos< plane_number; pos++){
			int comb[3];
			unsigned long long idx = comb_idx[pos];
			int pos_temp = 0;
			while(tab_binome_3[pos_temp]<=idx){
				pos_temp++;
			}
			pos_temp --;
			comb[0] = pos_temp;
			idx -= tab_binome_3[pos_temp];
			if(idx==0){
				comb[1] = 1;
				comb[2] = 0;
				triplets[pos] = Eigen::Vector3i(comb[0], comb[1], comb[2]);
				continue;
			}

			pos_temp = 0;
			while(tab_binome_2[pos_temp]<=idx){
				pos_temp++;
			}
			pos_temp --;
			comb[1] = pos_temp;
			idx -= tab_binome_2[pos_temp];
			if(idx==0){
				comb[2] = 0;
				triplets[pos] = Eigen::Vector3i(comb[0], comb[1], comb[2]);
				continue;
			}

			pos_temp = 0;
			while(pos_temp!=idx){
				pos_temp++;
			}
			comb[2] = pos_temp;
			triplets[pos] = Eigen::Vector3i(comb[0], comb[1], comb[2]);
		}

	}

	/*!
	 * generates vector n integer (cubes idx) with probability depending of the intersection with the sphere
	 * @param cubes_idx : table of int to fill
	 * @param n : number of integer to be drawn
	 */
	inline void generate_cube_vector(std::vector<int> &cubes_idx,  int n){

		//probabilities of picking a cube (ponderates by an approximation of its volume intersecting the sphere)
		std::vector<float> probas(n_cubes*n_cubes*n_cubes);
		float step = 2.f/n_cubes;
		float xmin = -(n_cubes/2.f)*step;
		float ymin = -(n_cubes/2.f)*step;
		float zmin = -(n_cubes/2.f)*step;
		float sum_prob = 0;
		for(int k=0; k<n_cubes; k++){
			for(int j=0; j<n_cubes; j++){
				for(int i=0; i<n_cubes; i++){
					float prob=0;

					float x1 = xmin + i*step;
					float y1 = ymin + j*step;
					float z1 = zmin + k*step;
					float x2 = x1+step;
					float y2 = y1+step;
					float z2 = z1+step;

					Eigen::Vector3f pt(x1,y1,z1);
					if(pt.squaredNorm()<= 1){
						prob += 0.125;
					}
					pt = Eigen::Vector3f(x2,y1,z1);
					if(pt.squaredNorm()<= 1){
						prob += 0.125;
					}
					pt = Eigen::Vector3f(x1,y2,z1);
					if(pt.squaredNorm()<= 1){
						prob += 0.125;
					}
					pt = Eigen::Vector3f(x2,y2,z1);
					if(pt.squaredNorm()<= 1){
						prob += 0.125;
					}
					pt = Eigen::Vector3f(x1,y1,z2);
					if(pt.squaredNorm()<= 1){
						prob += 0.125;
					}
					pt = Eigen::Vector3f(x2,y1,z2);
					if(pt.squaredNorm()<= 1){
						prob += 0.125;
					}
					pt = Eigen::Vector3f(x1,y2,z2);
					if(pt.squaredNorm()<= 1){
						prob += 0.125;
					}
					pt = Eigen::Vector3f(x2,y2,z2);
					if(pt.squaredNorm()<= 1){
						prob += 0.125;
					}

					probas[i +j*n_cubes +k*n_cubes*n_cubes] = prob;
					sum_prob+= prob;
				}
			}
		}
		//cumulative proba sum
		probas[0] /=sum_prob;
		for(int i=1; i<n_cubes*n_cubes*n_cubes; i++){
			probas[i] /= sum_prob;
			probas[i] += probas[i-1];
		}

		//getting the cubes according to the probas
		cubes_idx.resize(n);
		for(int i=0; i<n; i++) {
			float pos = (rand()+0.f)/RAND_MAX;
			int begin = 0;
			int end = n_cubes*n_cubes*n_cubes-1;
			int temp = (begin+end)/2;
			while(temp != begin){
				if(probas[temp] < pos){
					begin = temp;
				}else{
					end = temp;
				}
				temp  = (begin+end)/2;
			}
			cubes_idx[i] = end;
		}
	}

	/*!
	 * assigns points to the table of small cubes
	 * @param pointIdxRadiusSearch - table of indices of the points in the neighborhood
	 * @param cubes : table of table of indices of points belonging to cubes
	 * @param radius : radius of the neighborhood sphere
	 * @param refPoint : point where the normal is computed
	 * @param points : table of neighborhood points
	 * @param rotMat : table of random rotations
	 */
	inline void assign_points_to_cubes(std::vector<int> &pointIdxRadiusSearch, std::vector<int> cubes[],
			float radius, Point &refPoint, Point points[],
			Eigen::Matrix3f &rotMat
	){

		float step = 2.f/n_cubes * radius;
		for(unsigned int i=0; i<pointIdxRadiusSearch.size(); i++){
			int idx = pointIdxRadiusSearch[i];
			points[i] = pts->points[idx];
			points[i].getVector3fMap() = rotMat*points[i].getVector3fMap();
			Point refPoint2 = refPoint;
			refPoint2.getVector3fMap() = rotMat*refPoint2.getVector3fMap();
			int x = std::max(0,std::min(int((points[i].x - refPoint2.x)/step +(n_cubes/2.f)),n_cubes-1));
			int y = std::max(0,std::min(int((points[i].y - refPoint2.y)/step +(n_cubes/2.f)),n_cubes-1));
			int z = std::max(0,std::min(int((points[i].z - refPoint2.z)/step +(n_cubes/2.f)),n_cubes-1));
			cubes[x+y*n_cubes+z*n_cubes*n_cubes].push_back(i);
		}
	}

	/*!
	 * fills a vector with triplets of int
	 * @param triplets - table of triplets of points to fill
	 * @param cubes - table of table of point indices belonging to the cubes
	 * @param cubes_idx - table of random indices of cubes
	 * @param vecInt - table of random int
	 */
	inline void generate_cubes_triplets(std::vector<Eigen::Vector3i> &triplets,
			std::vector<int> cubes[], std::vector<int> &cubes_idx, std::vector<int> &vecInt){

		triplets.resize(n_planes);

		std::vector<Eigen::Vector3i>::iterator  ittrip = triplets.begin();
		std::vector<int>::iterator  itcube = cubes_idx.begin();
		std::vector<int>::iterator  itint = vecInt.begin();

		int idx = 0;
		while(ittrip != triplets.end() && itcube!= cubes_idx.end()){

			if(cubes[*itcube].size()!=0){
				int new_idx = cubes[*itcube][(*itint)%cubes[*itcube].size()];
				bool is_valid = true;
				for(int i=0; i<idx; i++){
					if(new_idx == (*ittrip)[i]){
						is_valid = false;
					}
				}
				if(is_valid){
					(*ittrip)[idx] = new_idx;
					idx ++;
				}
				if(idx == 3){
					idx =0;
					ittrip ++;
				}

			}
			itcube++;
			itint++;
		}

		while(ittrip != triplets.end()){
			int picked_points = 0;
			while(picked_points < 3){
				int pos = rand()%(n_cubes*n_cubes*n_cubes);
				if(cubes[pos].size()==0){
					continue;
				}
				int idx = rand()%cubes[pos].size();
				bool is_valid = true;
				for(int i=0; i<picked_points; i++){
					if(cubes[pos][idx] == (*ittrip)[i]){
						is_valid = false;
					}
				}
				if(is_valid){
					(*ittrip)[picked_points] = cubes[pos][idx];
					picked_points++;
				}
			}
			ittrip++;
		}
	}

	/*!	compute a random triplet on the sphere of radius
	 * @param tree - kdtree of the neighborhood
	 * @param radius1 - radius of the neighborhood sphere
	 * @param radius2 - small radius for triplet searching
	 * @param triplet - triplet to fill with the selected points indices
	 * @param pt - point where the normal is computed
	 */
	inline void find_a_triplet(pcl::KdTreeFLANN<Point> &tree, float radius1, float radius2, Eigen::Vector3i &triplet,
			Point &pt){

		int picked_points = 0;

		while(picked_points < 3){
			//picking point in the unit balls
			/*
			 *For fast results we pick them in the cube and discard bad points
			 */
			float x,y,z;
			do{
				x = ((rand()+0.f)/RAND_MAX)*2-1;
				y = ((rand()+0.f)/RAND_MAX)*2-1;
				z = ((rand()+0.f)/RAND_MAX)*2-1;
			}while(x*x + y*y +z*z >1);

			x *= radius1;
			y *= radius1;
			z *= radius1;


			x += pt.x;
			y += pt.y;
			z += pt.z;


			Point pt;
			pt.x = x;
			pt.y = y;
			pt.z = z;

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			tree.radiusSearch(pt, radius2, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			if(pointIdxRadiusSearch.size()!=0){
				int new_index = pointIdxRadiusSearch[rand()%pointIdxRadiusSearch.size()];
				bool is_valid = true;
				for(int i=0; i<picked_points; i++){
					if(new_index == triplet[i]){
						is_valid = false;
					}
				}
				if(is_valid){
					triplet[picked_points] = new_index;
					picked_points ++;
				}
			}
		}

	}

	/*!
	 * fills a vector of Point Triplets
	 * @param triplets - table of triplets to fill
	 * @param plane_number - number of triplets
	 * @param radius1 - radius of the neighborhood sphere
	 * @param radius2 - small radius for triplet selection
	 * @param tree - kdtree of the neighborhood
	 * @param pt - point where the normal is computed
	 * @param points - table of random points in the unit ball
	 * @param vecInt - table of random integers
	 */
	inline void generate_list_of_triplets(std::vector<Eigen::Vector3i> &triplets, 
			int plane_number, float radius1,
			float radius2,pcl::KdTreeFLANN<Point> &tree,
			Point & pt,
			std::vector<Eigen::Vector3f> &points,
			std::vector<int> &vecInt){

		triplets.resize(plane_number);

		std::vector<Eigen::Vector3i>::iterator  ittrip = triplets.begin();
		std::vector<Eigen::Vector3f>::iterator  itpoints = points.begin();
		std::vector<int>::iterator  itint = vecInt.begin();

		int idx = 0;
		while(ittrip != triplets.end() && itpoints!= points.end()){

			//getting coordinates
			float x,y,z;
			x = (*itpoints)[0]*radius1 + pt.x;
			y = (*itpoints)[1]*radius1 + pt.y;
			z = (*itpoints)[2]*radius1 + pt.z;

			//searching neighbors of the point
			Point refPoint;
			refPoint.x = x;
			refPoint.y = y;
			refPoint.z = z;
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			tree.radiusSearch(refPoint, radius2, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			//testing the validity
			if(pointIdxRadiusSearch.size()!=0){
				int new_index = pointIdxRadiusSearch[(*itint)%pointIdxRadiusSearch.size()];
				bool is_valid = true;
				for(int i=0; i<idx; i++){
					if(new_index == (*ittrip)[i]){
						is_valid = false;
					}
				}
				if(is_valid){
					(*ittrip)[idx] = new_index;
					idx ++;
				}
				if(idx == 3){
					idx =0;
					ittrip ++;
				}
			}

			itpoints++;
			itint++;
		}

		while(ittrip != triplets.end()){
			find_a_triplet(tree, radius1, radius2, *ittrip,pt);
			ittrip++;
		}

	}

	/*!
	 * Compute the normal by filling an accumulator for a given neighborhood
	 * @param d1 - First dimension of the accumulator
	 * @param d2 - Second dimension of the accumulator
	 * @param points - table of neighbors
	 * @param points_size - size of the neighborhood
	 * @param n - index of the point where the normal is computed
	 * @param triplets - table of triplets
	 * @param conf_interv - table of confidence intervals
	 */
	float normal_at_point(
			const int d1, const int d2,
			Point points[],
			int points_size,
			int n,
			std::vector<Eigen::Vector3i> &triplets,
			std::vector<float> &conf_interv){



		if(points_size < 3){
			nls->points[n].normal_x = 0;
			nls->points[n].normal_y = 0;
			nls->points[n].normal_z = 0;
			return 0;
		}

		//creation and initialization accumulators
		float *votes  = new float[d1*d2];
		Eigen::Vector3f  *votesV = new Eigen::Vector3f[d1*d2];
		for(int i=0; i<d1; i++){
			for(int j=0; j<d2; j++){
				votes[i+j*d1]=0;
				votesV[i+j*d1] = Eigen::Vector3f(0,0,0);
			}
		}


		float max1 = 0, max2=0;
		int i1=0, i2=0;
		int j1=0, j2=0;
		float votes_val;

		//bool cont = true;
		//int icomp = -1;
		//int jcomp = -1;

		for(int n_try=0; n_try< n_planes; n_try++){

			int p0 = triplets[n_try][0];
			int p1 = triplets[n_try][1];
			int p2 = triplets[n_try][2];

			Eigen::Vector3f v1 = points[p1].getVector3fMap()-points[p0].getVector3fMap();
			Eigen::Vector3f v2 = points[p2].getVector3fMap()-points[p0].getVector3fMap();

			Eigen::Vector3f Pn = v1.cross(v2);
			Pn.normalize();
			if(Pn.dot(points[p0].getVector3fMap())>0){
				Pn = -Pn;
			}

			float phi;
			phi = acos((float)Pn[2]);
			float dphi = PI/n_phi;
			int posp, post;
			posp = int(floor( (phi+dphi/2.) *n_phi/  PI));

			if(posp == 0 || posp== n_phi){
				post =0;
			}else{
				float theta = acos((float)Pn[0]/sqrt(float(Pn[0]*Pn[0]+Pn[1]*Pn[1])));
				if(Pn[1]<0){
					theta *= -1;
					theta += 2*PI;
				}
				float dtheta = PI/(n_phi*sin(posp*dphi));
				post = (int)(floor((theta+dtheta/2)/dtheta))%(2*n_phi);
			}

			post = std::max(0,std::min(2*n_phi-1,post));
			posp = std::max(0,std::min(n_phi,posp));


			votes[post+posp*d1] += 1.;
			votesV[post+posp*d1] += Pn;


			max1 = votes[i1+j1*d1]/(n_try+1);
			max2 = votes[i2+j2*d1]/(n_try+1);
			votes_val = votes[post+posp*d1]/(n_try+1);



			if(votes_val > max1){
				max2 = max1;
				i2 = i1;
				j2 = j1;
				max1 = votes_val;
				i1 = post;
				j1 = posp;
			}else if(votes_val>max2 && post!= i1 && posp!=j1){
				max2 = votes_val;
				i2 = post;
				j2 = posp;
			}


			if(max1-conf_interv[n_try] > max2){
				break;
			}

		}
		votesV[i1+j1*d1].normalize();
		nls->points[n].getNormalVector3fMap() = votesV[i1+j1*d1];

		delete[] votes;
		delete[] votesV;

		return max1;
	}


	/*!
	 * Compute the normal depending of the estimation choice (mean, best, cluster)
	 * @param rotations - number of rotations
	 * @param normals_vec - table of estimated normals for the point
	 * @param normals_conf - table of the confidence of normals
	 */
	inline Eigen::Vector3f normal_selection(int &rotations,
			std::vector<Eigen::Vector3f> &normals_vec, std::vector<float> &normals_conf){

		std::vector<bool> normals_use(rotations);
		//alignement of normals
		normals_use[0] = true;
		for(int i=1; i<rotations; i++){
			normals_use[i] = true;
			if(normals_vec[0].dot(normals_vec[i])<0){
				normals_vec[i]*= -1;
			}
		}

		Eigen::Vector3f normal_final;
		switch(selection_type){
		case 1: //best
		{
			float confidence_final=0;
			for(int i=0; i<rotations; i++){
				if(normals_conf[i]>confidence_final){
					confidence_final = normals_conf[i];
					normal_final = normals_vec[i];
				}
			}
		}
		break;
		case 2: //mb
		{
			std::vector<std::pair<Eigen::Vector3f, float> > normals_fin;
			int number_to_test = rotations;
			while(number_to_test>0){
				//getting the max
				float max_conf=0;
				int idx = 0;
				for(int i=0; i<rotations; i++){
					if(normals_use[i] && normals_conf[i]> max_conf){
						max_conf = normals_conf[i];
						idx = i;
					}
				}

				normals_fin.push_back(std::pair<Eigen::Vector3f, float>(normals_vec[idx]*normals_conf[idx], normals_conf[idx]));
				normals_use[idx] = false;
				number_to_test--;

				for(int i=0; i<rotations; i++){
					if(normals_use[i] && acos(normals_vec[idx].dot(normals_vec[i]))< tol_angle_rad){
						normals_use[i] = false;
						number_to_test --;
						normals_fin.back().first += normals_vec[i]*normals_conf[i];
						normals_fin.back().second += normals_conf[i];
					}
				}

			}

			normal_final = normals_fin[0].first;
			float conf_fin = normals_fin[0].second;
			for(unsigned int i=1; i<normals_fin.size(); i++){
				if(normals_fin[i].second> conf_fin){
					conf_fin = normals_fin[i].second;
					normal_final = normals_fin[i].first;
				}
			}
		}
		break;
		default: //mean
		{
			normal_final = normals_conf[0]*normals_vec[0];
			for(int i=1; i<rotations; i++){
				normal_final += normals_conf[i]*normals_vec[i];
			}
		}
		break;
		}



		normal_final.normalize();
		return normal_final;


	}





	/*!
	 * \brief Knn search, Points draw of the planes
	 * Normal Estimation using a k-nearest neighbor search, planes are drawn directly upon the points of the neighborhood
	 * @param neighbor_number : number of neighbors
	 */
	void points_knn(int neighbor_number){

		//resizing the normal point cloud
		nls->resize(pts->size());

		//initialize the random number generator
		srand((unsigned int)time(NULL));

		//kd tree creation
		pcl::KdTreeFLANN<Point> tree;
		tree.setInputCloud(pts);

		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//creation of the rotation matrices and their inverses
		std::vector<Eigen::Matrix3f> rotMat;
		std::vector<Eigen::Matrix3f> rotMatInv;
		generate_rotation_matrix(rotMat,rotMatInv, n_rot*200);

		int rotations;
		if(n_rot==0){
			rotations = 1;
		}else{
			rotations = n_rot;
		}

		//creating the list of triplets
		std::vector<Eigen::Vector3i> trip;
		std::vector<int> vecInt;
		generate_random_int_vector(vecInt, 1000000);
		if(rotations <=1){
			list_of_triplets(trip, neighbor_number,n_planes,vecInt);

		}else{
			list_of_triplets(trip, neighbor_number,rotations*n_planes,vecInt);
		}


		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		//random permutation of the points (avoid thread difficult block)
		std::vector<int> permutation(pts->size());
		for(int i=0; i<pts->size(); i++){
			permutation[i] = i;
		}
		for(int i=0; i<pts->size(); i++){
			int j = rand()%pts->size();
			int temp = permutation[i];
			permutation[i] = permutation[j];
			permutation[j] = temp;
		}

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for(int per=0; per<(int)pts->size(); per++){

			int n = permutation[per];

			//if the point is at the origin do not compute the normal
			//useful for laser point cloud
			if(pts->points[n].getVector3fMap() == Eigen::Vector3f(0,0,0)){
				continue;
			}

			//getting the list of neighbors
			std::vector<int> pointIdxSearch;
			std::vector<float> pointSquaredDistance;
			tree.nearestKSearch (pts->points[n], neighbor_number, pointIdxSearch, pointSquaredDistance);

			unsigned int points_size = (unsigned int) pointIdxSearch.size();
			Point *points = new Point[points_size];
			for(unsigned int pt=0; pt<pointIdxSearch.size(); pt++){
				points[pt] = pts->points[pointIdxSearch[pt]];
			}

			std::vector<Eigen::Vector3f> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for(int i=0; i<rotations; i++){

				std::vector<Eigen::Vector3i>::const_iterator first = trip.begin() + i*n_planes;
				std::vector<Eigen::Vector3i>::const_iterator last = trip.begin() + (i+1)*n_planes;
				std::vector<Eigen::Vector3i> triplets(first, last);

				for(unsigned int pt= 0; pt < points_size; pt++){
					points[pt].getVector3fMap() = rotMat[(n+i)%rotMat.size()]*points[pt].getVector3fMap();
				}
				normals_conf[i] = normal_at_point(d1, d2,points,points_size,  n,  triplets,  conf_interv);

				for(unsigned int pt= 0; pt < points_size; pt++){
					points[pt]=pts->points[pointIdxSearch[pt]];
				}
				normals_vec[i] = rotMatInv[(n+i)%rotMat.size()]*nls->points[n].getNormalVector3fMap();

			}

			nls->points[n].getNormalVector3fMap() = normal_selection(rotations, normals_vec, normals_conf);


			delete[] points;
		}
	}

	/*!
	 * \brief Radius search, Points  draw of the planes
	 * Normal Estimation using a range neighbor search, planes are drwn directly upon the points of the neighborhood
	 * @param radius : range radius for neighborhood search
	 */
	void points_radius(float radius){


		//resizing the normal point cloud
		if(nls != pts){
			nls->resize(pts->size());
		}

		//initialize the random number generator
		srand((unsigned int)time(NULL));

		//kd tree creation
		pcl::KdTreeFLANN<Point> tree;
		tree.setInputCloud(pts);

		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//creation of the rotation matrices and their inverses
		std::vector<Eigen::Matrix3f> rotMat;
		std::vector<Eigen::Matrix3f> rotMatInv;
		generate_rotation_matrix(rotMat,rotMatInv, n_rot*200);

		int rotations;
		if(n_rot==0){
			rotations = 1;
		}else{
			rotations = n_rot;
		}

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		std::vector<int> vecInt;
		generate_random_int_vector(vecInt, 1000000);

		//random permutation of the points (avoid thread difficult block)
		std::vector<int> permutation(pts->size());
		for(int i=0; i<pts->size(); i++){
			permutation[i] = i;
		}
		for(int i=0; i<pts->size(); i++){
			int j = rand()%pts->size();
			int temp = permutation[i];
			permutation[i] = permutation[j];
			permutation[j] = temp;
		}

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for(int per=0; per<(int)pts->size(); per++){

			int n = permutation[per];
			//if the point is at the origin do not compute the normal
			//useful for laser point cloud
			if(pts->points[n].getVector3fMap() == Eigen::Vector3f(0,0,0)){
				continue;
			}

			//getting the list of neighbors
			std::vector<int> pointIdxSearch;
			std::vector<float> pointSquaredDistance;
			tree.radiusSearch (pts->points[n], radius, pointIdxSearch, pointSquaredDistance);

			if(pointIdxSearch.size() < lower_neighbor_bound_neighbors){
				tree.nearestKSearch(pts->points[n], lower_neighbor_bound_neighbors, pointIdxSearch, pointSquaredDistance);
			}

			//creating the list of triplets
			std::vector<Eigen::Vector3i> trip;
			if(rotations <=1){
				list_of_triplets(trip, (int) pointIdxSearch.size(),n_planes,vecInt);
			}else{
				list_of_triplets(trip, (int) pointIdxSearch.size(),rotations*n_planes,vecInt);
			}



			unsigned int points_size = (unsigned int) pointIdxSearch.size();
			Point *points = new Point[points_size];
			for(unsigned int pt=0; pt<pointIdxSearch.size(); pt++){
				points[pt] = pts->points[pointIdxSearch[pt]];
			}



			std::vector<Eigen::Vector3f> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for(int i=0; i<rotations; i++){

				std::vector<Eigen::Vector3i>::const_iterator first = trip.begin() + i*n_planes;
				std::vector<Eigen::Vector3i>::const_iterator last = trip.begin() + (i+1)*n_planes;
				std::vector<Eigen::Vector3i> triplets(first, last);

				for(unsigned int pt= 0; pt < points_size; pt++){
					points[pt].getVector3fMap() = rotMat[(n+i)%rotMat.size()]*points[pt].getVector3fMap();
				}
				normals_conf[i] = normal_at_point( d1, d2,points,points_size,  n, triplets,  conf_interv);

				for(unsigned int pt= 0; pt < points_size; pt++){
					points[pt]=pts->points[pointIdxSearch[pt]];
				}
				normals_vec[i]= rotMatInv[(n+i)%rotMat.size()]*nls->points[n].getNormalVector3fMap();

			}
			nls->points[n].getNormalVector3fMap() = normal_selection( rotations, normals_vec, normals_conf);

			delete[] points;

		}

	}

	/*!
	 * \brief Knn search, Uniform draw of the planes
	 * Normal Estimation using a k-nearest neighbor search, planes are drawn uniformly in the neighborhood sphere
	 * @param neighbor_number : number of neighbors
	 */
	void unif_knn(int neighbor_number){

		//resizing the normal point cloud
		if(nls != pts){
			nls->resize(pts->size());
		}

		//initialize the random number generator
		srand((unsigned int)time(NULL));

		//kd tree creation
		pcl::KdTreeFLANN<Point> tree;
		tree.setInputCloud(pts);

		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		//creation of the rotation matrices and their inverses
		std::vector<Eigen::Matrix3f> rotMat;
		std::vector<Eigen::Matrix3f> rotMatInv;
		generate_rotation_matrix(rotMat,rotMatInv, n_rot*200);

		int rotations;
		if(n_rot==0){
			rotations = 1;
		}else{
			rotations = n_rot;
		}

		//creation of vector of int and points
		std::vector<Eigen::Vector3f> points_rand;
		std::vector<int> vecInt;
		generate_random_int_vector(vecInt, 1000000);
		generate_random_points_vector(points_rand, 1000000);


		//random permutation of the points (avoid thread difficult block)
		std::vector<int> permutation(pts->size());
		for(int i=0; i<pts->size(); i++){
			permutation[i] = i;
		}
		for(int i=0; i<pts->size(); i++){
			int j = rand()%pts->size();
			int temp = permutation[i];
			permutation[i] = permutation[j];
			permutation[j] = temp;
		}

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for(int per=0; per<(int)pts->size(); per++){

			int n = permutation[per];
			//cout << n << endl;
			//if the point is at the origin do not compute the normal
			//useful for laser point cloud
			if(pts->points[n].getVector3fMap() == Eigen::Vector3f(0,0,0)){
				continue;
			}

			//getting the list of neighbors
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			tree.nearestKSearch (pts->points[n], neighbor_number, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			float radius = 0;
			for(unsigned int i=0; i<pointRadiusSquaredDistance.size(); i++){
				if(pointRadiusSquaredDistance[i]>radius){
					radius = pointRadiusSquaredDistance[i];
				}
			}
			radius = sqrt(radius);

			float s_radius = radius / small_radius_factor;


			//point cloud of neighbors and kdtree creation
			typename pcl::PointCloud<Point>::Ptr cloud_neighbors (new pcl::PointCloud<Point>);
			cloud_neighbors->resize(pointIdxRadiusSearch.size());
			for(unsigned int i=0; i<pointIdxRadiusSearch.size(); i++){
				cloud_neighbors->points[i] = pts->points[pointIdxRadiusSearch[i]];
			}
			pcl::KdTreeFLANN<Point> tree_neighbors;
			tree_neighbors.setInputCloud(cloud_neighbors);

			Point *points = new Point[neighbor_number];

			//creating the list of triplets
			std::vector<Eigen::Vector3i> trip;
			generate_list_of_triplets(trip, rotations*n_planes,radius, s_radius,
					tree_neighbors,
					pts->points[n],
					points_rand,
					vecInt);



			std::vector<Eigen::Vector3f> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for(int i=0; i<rotations; i++){

				std::vector<Eigen::Vector3i>::const_iterator first = trip.begin() + i*n_planes;
				std::vector<Eigen::Vector3i>::const_iterator last = trip.begin() + (i+1)*n_planes;
				std::vector<Eigen::Vector3i> triplets(first, last);

				for(unsigned int pt=0; pt <neighbor_number; pt++){
					points[pt]=pts->points[pointIdxRadiusSearch[pt]];
					points[pt].getVector3fMap()=rotMat[(n+i)%rotMat.size()] * points[pt].getVector3fMap();
				}

				normals_conf[i] = normal_at_point( d1, d2,points,neighbor_number,  n,  triplets, conf_interv);
				normals_vec[i] = rotMatInv[(n+i)%rotMat.size()]*nls->points[n].getNormalVector3fMap();
			}

			nls->points[n].getNormalVector3fMap() = normal_selection( rotations,  normals_vec, normals_conf);


			delete[] points;

		}
	}

	/*!
	 * \brief Radius search, Uniform draw of the planes
	 * Normal Estimation using a range neighbor search, planes are drawn uniformly in the neighborhood sphere
	 * @param radius : range radius for neighborhood search
	 */
	void unif_radius(float radius){

		//resizing the normal point cloud
		if(nls != pts){
			nls->resize(pts->size());
		}

		//initialize the random number generator
		srand((unsigned int)time(NULL));

		//kd tree creation
		pcl::KdTreeFLANN<Point> tree;
		tree.setInputCloud(pts);

		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		//creation of the rotation matrices and their inverses
		std::vector<Eigen::Matrix3f> rotMat;
		std::vector<Eigen::Matrix3f> rotMatInv;
		generate_rotation_matrix(rotMat,rotMatInv, n_rot*200);

		int rotations;
		if(n_rot==0){
			rotations = 1;
		}else{
			rotations = n_rot;
		}

		//creation of vector of int and points
		std::vector<Eigen::Vector3f> points_rand;
		std::vector<int> vecInt;
		generate_random_points_vector(points_rand, 1000000);
		generate_random_int_vector(vecInt, 1000000);

		//random permutation of the points (avoid thread difficult block)
		std::vector<int> permutation(pts->size());
		for(int i=0; i<pts->size(); i++){
			permutation[i] = i;
		}
		for(int i=0; i<pts->size(); i++){
			int j = rand()%pts->size();
			int temp = permutation[i];
			permutation[i] = permutation[j];
			permutation[j] = temp;
		}

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for(int per=0; per<(int)pts->size(); per++){

			int n = permutation[per];
			//cout << n << endl;
			//if the point is at the origin do not compute the normal
			//useful for laser point cloud
			if(pts->points[n].getVector3fMap() == Eigen::Vector3f(0,0,0)){
				continue;
			}

			//getting the list of neighbors
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			tree.radiusSearch (pts->points[n], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);



			float radius2=radius;
			if(pointIdxRadiusSearch.size() < lower_neighbor_bound_neighbors){
				radius2=0;
				tree.nearestKSearch(pts->points[n], lower_neighbor_bound_neighbors, pointIdxRadiusSearch, pointRadiusSquaredDistance);
				for(unsigned int i=0; i<pointRadiusSquaredDistance.size(); i++){
					if(pointRadiusSquaredDistance[i] > radius2 ){
						radius2 = pointRadiusSquaredDistance[i];
					}
				}
			}
			float s_radius = radius2 / small_radius_factor;

			//point cloud of neighbors and kdtree creation
			typename pcl::PointCloud<Point>::Ptr cloud_neighbors (new pcl::PointCloud<Point>);
			cloud_neighbors->resize(pointIdxRadiusSearch.size());
			for(unsigned int i=0; i<pointIdxRadiusSearch.size(); i++){
				cloud_neighbors->points[i] = pts->points[pointIdxRadiusSearch[i]];
			}
			pcl::KdTreeFLANN<Point> tree_neighbors;
			tree_neighbors.setInputCloud(cloud_neighbors);

			unsigned int points_size = (unsigned int) pointIdxRadiusSearch.size();
			Point *points = new Point[points_size];



			//creating the list of triplets
			std::vector<Eigen::Vector3i> trip;
			generate_list_of_triplets(trip, rotations*n_planes,radius2, s_radius,
					tree_neighbors,
					pts->points[n],
					points_rand,
					vecInt);



			std::vector<Eigen::Vector3f> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for(int i=0; i<rotations; i++){

				std::vector<Eigen::Vector3i>::const_iterator first = trip.begin() + i*n_planes;
				std::vector<Eigen::Vector3i>::const_iterator last = trip.begin() + (i+1)*n_planes;
				std::vector<Eigen::Vector3i> triplets(first, last);

				for(unsigned int pt=0; pt <points_size; pt++){
					points[pt]=pts->points[pointIdxRadiusSearch[pt]];
					points[pt].getVector3fMap()=rotMat[(n+i)%rotMat.size()] * points[pt].getVector3fMap();
				}

				normals_conf[i] = normal_at_point(d1, d2,points,points_size,  n,  triplets, conf_interv);

				nls->points[n].getNormalVector3fMap() = rotMatInv[(n+i)%rotMat.size()]*nls->points[n].getNormalVector3fMap();

				normals_vec[i] = nls->points[n].getNormalVector3fMap();

			}



			nls->points[n].getNormalVector3fMap() = normal_selection( rotations,  normals_vec, normals_conf);



			delete[] points;

		}
	}

	/*!
	 * \brief Knn search, Cubes draw of the planes
	 * Normal Estimation using a k-nearest neighbor search, planes are drawn using a cubic discretization of the neighborhood sphere
	 * @param neighbor_number : number of neighbors
	 */
	void cubes_knn(int neighbor_number){


		//resizing the normal point cloud
		if(nls != pts){
			nls->resize(pts->size());
		}

		//initialize the random number generator
		srand((unsigned int)time(NULL));


		//kd tree creation
		pcl::KdTreeFLANN<Point> tree;
		tree.setInputCloud(pts);

		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		//creation of the rotation matrices and their inverses
		std::vector<Eigen::Matrix3f> rotMat;
		std::vector<Eigen::Matrix3f> rotMatInv;
		generate_rotation_matrix(rotMat,rotMatInv, n_rot*200);

		int rotations;
		if(n_rot==0){
			rotations = 1;
		}else{
			rotations = n_rot;
		}

		//creation of vector of cubes and int
		std::vector<int> cubes_idx;
		std::vector<int> vecInt;
		generate_cube_vector(cubes_idx, 1000000);
		generate_random_int_vector(vecInt, 1000000);


		//random permutation of the points (avoid thread difficult block)
		std::vector<int> permutation(pts->size());
		for(int i=0; i<pts->size(); i++){
			permutation[i] = i;
		}
		for(int i=0; i<pts->size(); i++){
			int j = rand()%pts->size();
			int temp = permutation[i];
			permutation[i] = permutation[j];
			permutation[j] = temp;
		}

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for(int per=0; per<(int)pts->size(); per++){

			int n = permutation[per];
			//cout << n << endl;
			//if the point is at the origin do not compute the normal
			//useful for laser point cloud
			if(pts->points[n].getVector3fMap() == Eigen::Vector3f(0,0,0)){
				continue;
			}

			//getting the list of neighbors
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			tree.nearestKSearch (pts->points[n], neighbor_number, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			float radius = 0;
			for(unsigned int i=0; i<pointRadiusSquaredDistance.size(); i++){
				if(pointRadiusSquaredDistance[i]>radius){
					radius = pointRadiusSquaredDistance[i];
				}
			}
			radius = sqrt(radius);

			std::vector<int>* cubes = new std::vector<int>[n_cubes*n_cubes*n_cubes];
			Point *points = new Point[neighbor_number];
			std::vector<Eigen::Vector3i> triplets;

			std::vector<Eigen::Vector3f> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for(int i=0; i<rotations; i++){

				assign_points_to_cubes(pointIdxRadiusSearch, cubes,radius,pts->points[n], points,rotMat[(n+i)%rotMat.size()]);

				generate_cubes_triplets(triplets, cubes, cubes_idx, vecInt);

				//cout << "1" << endl;
				normals_conf[i] = normal_at_point(d1, d2,points,neighbor_number,  n,  triplets, conf_interv);
				//cout << "2" << endl;

				for(unsigned int pt= 0; pt < neighbor_number; pt++){
					points[pt]=pts->points[pointIdxRadiusSearch[pt]];
				}
				normals_vec[i] = rotMatInv[(n+i)%rotMat.size()]*nls->points[n].getNormalVector3fMap();
			}

			nls->points[n].getNormalVector3fMap() = normal_selection(rotations, normals_vec, normals_conf);

			delete[] points;
			delete[] cubes;

		}
	}

	/*!
	 * \brief Radius search, Cubes draw of the planes
	 * Normal Estimation using a range neighbor search, planes are drawn using a cubic discretization of the neighborhood sphere
	 * @param radius
	 */
	void cubes_radius(float radius)
	{

		//resizing the normal point cloud
		if(nls != pts){
			nls->resize(pts->size());
		}

		//initialize the random number generator
		srand((unsigned int)time(NULL));


		//kd tree creation
		pcl::KdTreeFLANN<Point> tree;
		tree.setInputCloud(pts);

		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		//creation of the rotation matrices and their inverses
		std::vector<Eigen::Matrix3f> rotMat;
		std::vector<Eigen::Matrix3f> rotMatInv;
		generate_rotation_matrix(rotMat,rotMatInv, n_rot*200);


		int rotations;
		if(n_rot==0){
			rotations = 1;
		}else{
			rotations = n_rot;
		}

		//creation of vector of cubes and int
		std::vector<int> cubes_idx;
		std::vector<int> vecInt;
		generate_cube_vector(cubes_idx, 1000000);
		generate_random_int_vector(vecInt, 1000000);


		//random permutation of the points (avoid thread difficult block)
		std::vector<int> permutation(pts->size());
		for(int i=0; i<pts->size(); i++){
			permutation[i] = i;
		}
		for(int i=0; i<pts->size(); i++){
			int j = rand()%pts->size();
			int temp = permutation[i];
			permutation[i] = permutation[j];
			permutation[j] = temp;
		}

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for(int per=0; per<(int)pts->size(); per++){

			int n = permutation[per];
			//std::cout << n << std::endl;
			//if the point is at the origin do not compute the normal
			//useful for laser point cloud
			if(pts->points[n].getVector3fMap() == Eigen::Vector3f(0,0,0)){
				continue;
			}

			float radius2 = radius;

			//getting the list of neighbors
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			tree.radiusSearch (pts->points[n], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			if(pointIdxRadiusSearch.size() < lower_neighbor_bound_neighbors){
				radius2 =0;
				tree.nearestKSearch(pts->points[n], lower_neighbor_bound_neighbors, pointIdxRadiusSearch, pointRadiusSquaredDistance);
				for(unsigned int i=0; i<pointRadiusSquaredDistance.size(); i++){
					if(pointRadiusSquaredDistance[i] > radius2 ){
						radius2 = pointRadiusSquaredDistance[i];
					}
				}
			}

			std::vector<int>* cubes = new std::vector<int>[n_cubes*n_cubes*n_cubes];
			unsigned int points_size = (unsigned int) pointIdxRadiusSearch.size();
			Point *points = new Point[points_size];


			std::vector<Eigen::Vector3i> triplets;


			std::vector<Eigen::Vector3f> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for(int i=0; i<rotations; i++){



				assign_points_to_cubes( pointIdxRadiusSearch, cubes,radius2,pts->points[n], points,rotMat[(i+n)%rotMat.size()]);
				generate_cubes_triplets(triplets, cubes, cubes_idx, vecInt);

				normals_conf[i] = normal_at_point(d1, d2,points,points_size,  n,  triplets, conf_interv);
				for(unsigned int pt= 0; pt < points_size; pt++){
					points[pt]=pts->points[pointIdxRadiusSearch[pt]];
				}
				normals_vec[i] = rotMatInv[(i+n)%rotMat.size()]*nls->points[n].getNormalVector3fMap();
			}



			nls->points[n].getNormalVector3fMap() = normal_selection(rotations, normals_vec, normals_conf);



			delete[] points;
			delete[] cubes;

		}

	}





};

#endif
