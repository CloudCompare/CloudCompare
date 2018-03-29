/* License Information
 *
 * Copyright (C) 2012 Boulch Alexandre, Ecole Nationale des Ponts et Chaussees -
 * Ecole des Ponts ParisTech
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Note that this library relies on external libraries subject to their own license.
 * To use this software, you are subject to the dependencies license, these licenses applies to the dependency
 * ONLY  and NOT this code.
 * Please refer below to the web sites for license informations.
 *
 * OPENMP (http://openmp.org/)
 * CGAL (http://www.cgal.org/) see CGAL Licence Term
 */

#ifndef NORM_EST_CGAL_OMP_H
#define NORM_EST_CGAL_OMP_H


#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#include <omp.h>
#endif

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_3.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Fuzzy_sphere.h>

#include <iostream>	
#include <time.h>
#include <vector>
#include <map>

/*!
 * \file CGAL_normEst.h
 * \brief Normal Estimator for point clouds
 * \author Alexandre Boulch
 * \version 0.1
 *
 * Normal estimator based on the publication from SGP2012
 * "Fast and Robust Normal Estimator for Point Clouds"
 */


/*!
 * \class CGAL_Normal_Estimator
 * \brief Class grouping different variant of the algorithm
 */
class CGAL_Normal_Estimator{
private:

public:

	/*!
	 * \class My_Triplet
	 * \brief simple class for triplet of points
	 */
	template <typename T>
	class My_Triplet{
	private:
		T data[3];
	public:
		My_Triplet(){};
		My_Triplet(T a, T b, T c){data[0] = a;data[1] = b;data[2] = c;}
		T& operator()(int i){return data[i];}
		T operator()(int i) const {return data[i];}
	};


	typedef CGAL::Simple_cartesian<float> Kernel;
	typedef typename CGAL::Point_3<Kernel> Point3;
	typedef typename CGAL::Vector_3<Kernel> Vector3;
	typedef typename CGAL::Aff_transformation_3<Kernel> Matrix3;
	typedef typename CGAL::Search_traits_3<Kernel> TreeTraits;
	typedef My_Triplet<Point3> Triplet;
	typedef typename CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
	typedef typename CGAL::Kd_tree<TreeTraits> Tree;
	typedef typename CGAL::Fuzzy_sphere<TreeTraits> Fuzzy_sphere;
	typedef typename std::vector<Point3>::iterator vecPt3Iterator;
	typedef typename std::vector<Vector3>::iterator vecVec3Iterator;
	typedef typename std::vector<Triplet>::iterator vecTripIterator;
	typedef typename Neighbor_search::iterator Neighbor_search_iterator;


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
	CGAL_Normal_Estimator(std::vector<Point3> &points, std::vector<Vector3> &normals):
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

	std::vector<Point3>& point_cloud(){return pts;}
	std::vector<Point3> point_cloud()const{return pts;}
	std::vector<Vector3>& normal_cloud(){return nls;}
	std::vector<Vector3> normal_cloud() const{return nls;}

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

	std::vector<Point3>& pts;/*!< Point cloud*/
	std::vector<Vector3>& nls;/*!< Normal cloud*/

	/*!
	 * Function to set the default parameters (hard coded)
	 */
	void set_default_parameters(){
		PI=3.14159265f;
		n_planes=700;
		n_rot=5;
		n_phi=15;
		tol_angle_rad=0.79;
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
	inline void generate_rotation_matrix(std::vector<Matrix3> &rotMat, std::vector<Matrix3> &rotMatInv, int rotations)
	{
		rotMat.clear();
		rotMatInv.clear();

		if(rotations==0){
			Matrix3 rMat(1,0,0,0,1,0,0,0,1);
			rotMat.push_back(rMat);
			rotMatInv.push_back(rMat);
		}else{

			for(int i=0; i<rotations; i++){
				float theta = (rand()+0.f)/RAND_MAX * 2* 3.14159265f;
				float phi = (rand()+0.f)/RAND_MAX * 2* 3.14159265f;
				float psi = (rand()+0.f)/RAND_MAX * 2* 3.14159265f;
				Matrix3 Rt(1, 0, 0,0, cos(theta), -sin(theta),	0, sin(theta), cos(theta));
				Matrix3 Rph(cos(phi),0, sin(phi),0,1,0,-sin(phi),0, cos(phi));
				Matrix3 Rps(cos(psi), -sin(psi), 0,	sin(psi), cos(psi),0,0,0,1);
				Matrix3 Rtinv(1, 0, 0,0, cos(theta) , sin(theta),0, -sin(theta), cos(theta));
				Matrix3 Rphinv(cos(phi) , 0, -sin(phi),0, 1, 0,sin(phi), 0, cos(phi));
				Matrix3 Rpsinv(cos(psi) , sin(psi), 0,	-sin(psi), cos(psi), 0,	0, 0, 1);

				Matrix3 rMat = Rt*Rph*Rps;
				Matrix3 rMatInv = Rpsinv*Rphinv*Rtinv;
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
	inline void generate_random_points_vector(std::vector<Point3> &points, int point_number){
		points.resize(point_number);
		for(int i=0; i<point_number; i++){
			float x,y,z;
			do{
				x = ((rand()+0.f)/RAND_MAX)*2-1;
				y = ((rand()+0.f)/RAND_MAX)*2-1;
				z = ((rand()+0.f)/RAND_MAX)*2-1;
			}while(x*x + y*y +z*z >1);

			points[i]=Point3(x,y,z);
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
	inline void list_of_triplets(std::vector<Vector3> &triplets,
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
				triplets[pos] = Vector3(comb[0], comb[1], comb[2]);
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
				triplets[pos] = Vector3(comb[0], comb[1], comb[2]);
				continue;
			}

			pos_temp = 0;
			while(pos_temp!=idx){
				pos_temp++;
			}
			comb[2] = pos_temp;
			triplets[pos] = Vector3(comb[0], comb[1], comb[2]);
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

					Vector3 pt(x1,y1,z1);
					if(pt*pt<= 1){
						prob += 0.125;
					}
					pt = Vector3(x2,y1,z1);
					if(pt*pt<= 1){
						prob += 0.125;
					}
					pt = Vector3(x1,y2,z1);
					if(pt*pt<= 1){
						prob += 0.125;
					}
					pt = Vector3(x2,y2,z1);
					if(pt*pt<= 1){
						prob += 0.125;
					}
					pt = Vector3(x1,y1,z2);
					if(pt*pt<= 1){
						prob += 0.125;
					}
					pt = Vector3(x2,y1,z2);
					if(pt*pt<= 1){
						prob += 0.125;
					}
					pt = Vector3(x1,y2,z2);
					if(pt*pt<= 1){
						prob += 0.125;
					}
					pt = Vector3(x2,y2,z2);
					if(pt*pt<= 1){
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
	 * @param cubes : table of table of indices of points belonging to cubes
	 * @param radius : radius of the neighborhood sphere
	 * @param refPoint : point where the normal is computed
	 * @param points : table of neighborhood points (transformed)
	 * @param points2 : table of neighborhood points (original)
	 * @param rotMat : table of random rotations
	 */
	inline void assign_points_to_cubes( std::vector<int> cubes[],
			float radius, Point3 &refPoint, std::vector<Point3> &points, std::vector<Point3> &points2,
			Matrix3 &rotMat
	){

		float step = 2.f/n_cubes * radius;
		for(unsigned int i=0; i<points2.size(); i++){
			points[i] = points2[i];
			points[i] = points[i].transform(rotMat);
			Point3 refPoint2 = refPoint;
			refPoint2 = refPoint2.transform(rotMat);
			int x = std::max(0,std::min(int((points[i].x() - refPoint2.x())/step +(n_cubes/2.f)),n_cubes-1));
			int y = std::max(0,std::min(int((points[i].y() - refPoint2.y())/step +(n_cubes/2.f)),n_cubes-1));
			int z = std::max(0,std::min(int((points[i].z() - refPoint2.z())/step +(n_cubes/2.f)),n_cubes-1));
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
	inline void generate_cubes_triplets(std::vector<Vector3> &triplets,
			std::vector<int> cubes[], std::vector<int> &cubes_idx, std::vector<int> &vecInt){

		triplets.resize(n_planes);

		vecVec3Iterator  ittrip = triplets.begin();
		std::vector<int>::iterator  itcube = cubes_idx.begin();
		std::vector<int>::iterator  itint = vecInt.begin();

		int idx = 0;
		int coord[3];
		while(ittrip != triplets.end() && itcube!= cubes_idx.end()){

			if(cubes[*itcube].size()!=0){
				int new_idx = cubes[*itcube][(*itint)%cubes[*itcube].size()];
				bool is_valid = true;
				for(int i=0; i<idx; i++){
					if(new_idx == coord[i]){
						is_valid = false;
					}
				}
				if(is_valid){
					coord[idx] = new_idx;
					idx ++;
				}
				if(idx == 3){
					idx =0;
					*ittrip = Vector3(coord[0], coord[1], coord[2]);
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
					if(cubes[pos][idx] == coord[i]){
						is_valid = false;
					}
				}
				if(is_valid){
					coord[picked_points] = cubes[pos][idx];
					picked_points++;
				}
			}
			(*ittrip) = Vector3(coord[0], coord[1], coord[2]);
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
	inline void find_a_triplet(Tree &tree, float radius1, float radius2, Triplet &triplet,
			Point3 &pt){

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


			x += pt.x();
			y += pt.y();
			z += pt.z();


			Point3 pt(x,y,z);

			Fuzzy_sphere s_query(pt,radius2);
			std::vector<Point3> points_search;
			tree.search(std::back_inserter(points_search), s_query);

			if(points_search.size()!=0){
				Point3 new_Point = points_search[rand()%points_search.size()];
				bool is_valid = true;
				for(int i=0; i<picked_points; i++){
					if(new_Point == triplet(i)){
						is_valid = false;
					}
				}
				if(is_valid){
					triplet(picked_points) = new_Point;
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
	inline void generate_list_of_triplets(std::vector<Triplet> &triplets,
			int plane_number, float radius1,
			float radius2,Tree &tree,
			Point3 & pt,
			std::vector<Point3> &points,
			std::vector<int> &vecInt){

		triplets.resize(plane_number);

		vecTripIterator  ittrip = triplets.begin();
		vecPt3Iterator  itpoints = points.begin();
		std::vector<int>::iterator  itint = vecInt.begin();


		int idx = 0;
		while(ittrip != triplets.end() && itpoints!= points.end()){

			//getting coordinates
			float x,y,z;
			x = (*itpoints).x()*radius1 + pt.x();
			y = (*itpoints).y()*radius1 + pt.y();
			z = (*itpoints).z()*radius1 + pt.z();

			//searching neighbors of the point
			Point3 refPoint(x,y,z);
			Fuzzy_sphere s_query(refPoint,radius2);
			std::vector<Point3> points_search;
			tree.search(std::back_inserter(points_search), s_query);


			//testing the validity
			if(points_search.size()!=0){
				Point3 new_point = points_search[(*itint)%points_search.size()];
				bool is_valid = true;
				for(int i=0; i<idx; i++){
					if(new_point == (*ittrip)(i)){
						is_valid = false;
					}
				}
				if(is_valid){
					(*ittrip)(idx) = new_point;
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
	 * @param n - index of the point where the normal is computed
	 * @param triplets - table of triplets (points indices)
	 * @param conf_interv - table of confidence intervals
	 */
	float normal_at_point(
			const int d1, const int d2,
			std::vector<Point3> &points,
			int n,
			std::vector<Vector3> &triplets,
			std::vector<float> &conf_interv){



		if(points.size() < 3){
			nls[n]=Vector3(0,0,0);
			return 0;
		}

		//creation and initialization accumulators
		float *votes  = new float[d1*d2];
		Vector3  *votesV = new Vector3[d1*d2];
		for(int i=0; i<d1; i++){
			for(int j=0; j<d2; j++){
				votes[i+j*d1]=0;
				votesV[i+j*d1] = Vector3(0,0,0);
			}
		}


		float max1 = 0, max2=0;
		int i1=0, i2=0;
		int j1=0, j2=0;
		float votes_val;

		for(int n_try=0; n_try< n_planes; n_try++){

			int p0 = triplets[n_try][0];
			int p1 = triplets[n_try][1];
			int p2 = triplets[n_try][2];


			Vector3 v1 = points[p1]-points[p0];
			Vector3 v2 = points[p2]-points[p0];

			Vector3 Pn = CGAL::cross_product(v1,v2);
			Pn = Pn / sqrt(Pn*Pn);
			if(Pn*(points[p0]-CGAL::ORIGIN)>0){
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
			votesV[post+posp*d1] = Pn + votesV[post+posp*d1];


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

		nls[n] = votesV[i1+j1*d1] / sqrt(votesV[i1+j1*d1]*votesV[i1+j1*d1]);
		delete[] votes;
		delete[] votesV;

		return max1;
	}

	/*!
	 * Compute the normal by filling an accumulator for a given neighborhood
	 * @param d1 - First dimension of the accumulator
	 * @param d2 - Second dimension of the accumulator
	 * @param points_size - size of the neighborhood
	 * @param n - index of the point where the normal is computed
	 * @param triplets - table of triplets (Points coordinates)
	 * @param conf_interv - table of confidence intervals
	 */
	float normal_at_point(
			const int d1, const int d2,
			int points_size,
			int n,
			std::vector<Triplet> &triplets,
			std::vector<float> &conf_interv){



		if(points_size < 3){
			nls[n]=Vector3(0,0,0);
			return 0;
		}

		//creation and initialization accumulators
		float *votes  = new float[d1*d2];
		Vector3  *votesV = new Vector3[d1*d2];
		for(int i=0; i<d1; i++){
			for(int j=0; j<d2; j++){
				votes[i+j*d1]=0;
				votesV[i+j*d1] = Vector3(0,0,0);
			}
		}


		float max1 = 0, max2=0;
		int i1=0, i2=0;
		int j1=0, j2=0;
		float votes_val;


		for(int n_try=0; n_try< n_planes; n_try++){

			Point3 p0 = triplets[n_try](0);
			Point3 p1 = triplets[n_try](1);
			Point3 p2 = triplets[n_try](2);
			Vector3 v1 = p1-p0;
			Vector3 v2 = p2-p0;

			Vector3 Pn = CGAL::cross_product(v1,v2);
			Pn = Pn / sqrt(Pn*Pn);
			if(Pn*(p0-CGAL::ORIGIN)>0){
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
			votesV[post+posp*d1] = Pn + votesV[post+posp*d1];


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

		nls[n]= votesV[i1+j1*d1]/sqrt(votesV[i1+j1*d1]*votesV[i1+j1*d1]);

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
	inline Vector3 normal_selection(int &rotations,
			std::vector<Vector3> &normals_vec, std::vector<float> &normals_conf){

		std::vector<bool> normals_use(rotations);

		//init normals_use and reorient normals
		normals_use[0] = true;
		for(int i=1; i<rotations; i++){
			normals_use[i] = true;
			if(normals_vec[0] *normals_vec[i]<0){
				normals_vec[i] = -normals_vec[i];
			}
		}

		Vector3 normal_final;
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
			std::vector<std::pair<Vector3, float> > normals_fin;
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

				normals_fin.push_back(std::pair<Vector3, float>(normals_vec[idx]*normals_conf[idx], normals_conf[idx]));
				normals_use[idx] = false;
				number_to_test--;

				for(int i=0; i<rotations; i++){
					if(normals_use[i] && acos(normals_vec[idx]*normals_vec[i])< tol_angle_rad){
						normals_use[i] = false;
						number_to_test --;
						normals_fin.back().first = normals_fin.back().first+normals_vec[i]*normals_conf[i];
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
				normal_final = normal_final + normals_conf[i]*normals_vec[i];
			}
		}
		break;
		}

		return normal_final /sqrt(normal_final*normal_final);
	}





	/*!
	 * \brief Knn search, Points draw of the planes
	 * Normal Estimation using a k-nearest neighbor search, planes are drawn directly upon the points of the neighborhood
	 * @param neighbor_number : number of neighbors
	 */
	void points_knn(int neighbor_number){


		//initialize the random number generator
		srand((unsigned int)time(NULL));

		nls.resize(pts.size());



		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//creation of the rotation matrices and their inverses
		std::vector<Matrix3> rotMat;
		std::vector<Matrix3> rotMatInv;
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


		//creating the list of triplets
		std::vector<Vector3> trip;
		if(rotations <=1){
			list_of_triplets(trip, neighbor_number,n_planes,vecInt);

		}else{
			list_of_triplets(trip, neighbor_number,rotations*n_planes,vecInt);
		}

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel
		{
			Tree tree(pts.begin(), pts.end());
#pragma omp for schedule(guided)
#else
		Tree tree(pts.begin(), pts.end());
#endif
		for(int n=0; n<(int)pts.size(); n++){
			std::vector<Point3>points(neighbor_number);
			std::vector<Point3>points2(neighbor_number);
			int points_size = 0;

			//getting the list of neighbors
			Neighbor_search search(tree, pts[n], neighbor_number);

			for(Neighbor_search_iterator it = search.begin(); it != search.end() && points_size < neighbor_number; ++it){
				points[points_size] = it->first;
				points2[points_size] = it->first;
				points_size++;
			}

			std::vector<Vector3> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);


			for(int i=0; i<rotations; i++){

				vecVec3Iterator first = trip.begin() + i*n_planes;
				vecVec3Iterator last = trip.begin() + (i+1)*n_planes;
				std::vector<Vector3> triplets(first, last);

				for(unsigned int pt= 0; pt < points_size; pt++){
					points[pt]=points2[pt].transform(rotMat[(n+i)%rotMat.size()]);
				}
				normals_conf[i] = normal_at_point(d1, d2,points,  n,  triplets,  conf_interv);

				normals_vec[i] = nls[n].transform(rotMatInv[(n+i)%rotMat.size()]);
			}
			nls[n] = normal_selection( rotations, normals_vec,normals_conf);

		}
#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
	}
#endif
	}

	/*!
	 * \brief Radius search, Points  draw of the planes
	 * Normal Estimation using a range neighbor search, planes are drwn directly upon the points of the neighborhood
	 * @param radius : range radius for neighborhood search
	 */
	void points_radius(float radius){


		//initialize the random number generator
		srand((unsigned int)time(NULL));

		nls.resize(pts.size());


		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//creation of the rotation matrices and their inverses
		std::vector<Matrix3> rotMat;
		std::vector<Matrix3> rotMatInv;
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


#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel
		{
			Tree tree(pts.begin(), pts.end());
#pragma omp for schedule(guided)
#else
		Tree tree(pts.begin(), pts.end());
#endif
		for(int n=0; n<(int)pts.size(); n++){
			//std::cout << n<<std::endl;
			std::vector<Point3>points2;
			Fuzzy_sphere s_query(pts[n],radius);

			//getting the list of neighbors
			tree.search(std::back_inserter(points2), s_query);

			int points_size = (int) points2.size();

			if(points_size <lower_neighbor_bound_neighbors){
				points2.clear();

				//getting the list of neighbors
				Neighbor_search search(tree, pts[n], lower_neighbor_bound_neighbors);

				for(Neighbor_search_iterator it = search.begin(); it != search.end() && points_size < lower_neighbor_bound_neighbors; ++it){
					points2.push_back(it->first);
					points_size++;
				}
				points_size = points2.size();
			}



			unsigned long int max_number_comb = points_size;
			max_number_comb*=points_size-1;
			max_number_comb*=points_size-2;
			if(max_number_comb < n_planes){continue;}
			//creating the list of triplets
			std::vector<Vector3> trip;

			if(rotations <=1){
				list_of_triplets(trip, points_size,n_planes,vecInt);
			}else{
				list_of_triplets(trip, points_size,rotations*n_planes,vecInt);
			}

			std::vector<Vector3> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);
			std::vector<Point3> points(points_size);

			for(int i=0; i<rotations; i++){

				vecVec3Iterator first = trip.begin() + i*n_planes;
				vecVec3Iterator last = trip.begin() + (i+1)*n_planes;
				std::vector<Vector3> triplets(first, last);

				for(unsigned int pt= 0; pt < points_size; pt++){
					points[pt]=points2[pt].transform(rotMat[(n+i)%rotMat.size()]);
				}
				normals_conf[i] = normal_at_point( d1, d2,points,  n,  triplets,  conf_interv);

				normals_vec[i] = nls[n].transform(rotMatInv[(n+i)%rotMat.size()]);
			}

			nls[n] = normal_selection( rotations, normals_vec,normals_conf);

		}
#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
	}
#endif

	}

	/*!
	 * \brief Knn search, Uniform draw of the planes
	 * Normal Estimation using a k-nearest neighbor search, planes are drawn uniformly in the neighborhood sphere
	 * @param neighbor_number : number of neighbors
	 */
	void unif_knn(int neighbor_number)
	{

		//initialize the random number generator
		srand((unsigned int)time(NULL));
		//resizing the normal point cloud
		nls.resize(pts.size());

		Tree tree(pts.begin(), pts.end());

		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		//creation of the rotation matrices and their inverses
		std::vector<Matrix3> rotMat;
		std::vector<Matrix3> rotMatInv;
		generate_rotation_matrix(rotMat,rotMatInv, n_rot*200);

		int rotations;
		if(n_rot==0){
			rotations = 1;
		}else{
			rotations = n_rot;
		}

		//creation of vector of int and points
		std::vector<Point3> points_rand;
		std::vector<int> vecInt;
		generate_random_points_vector(points_rand, 1000000);
		generate_random_int_vector(vecInt, 1000000);

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for(int n=0; n<(int) pts.size(); n++){

			std::vector<Point3>points(neighbor_number);
			int points_size = 0;
			float radius = 0;
#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp critical
#endif
			{
				//getting the list of neighbors
				Neighbor_search search(tree, pts[n], neighbor_number);

				for(Neighbor_search_iterator it = search.begin(); it != search.end() && points_size < neighbor_number; ++it){
					points[points_size] = it->first;
					points_size++;
					if(radius < it->second){
						radius = it->second;
					}
				}
			}

			if(points_size != neighbor_number){continue;}
			radius = sqrt(radius);

			float s_radius = radius / small_radius_factor;


			//point cloud of neighbors and kdtree creation
			Tree tree_neighbors(points.begin(), points.end());

			//creating the list of triplets
			std::vector<Triplet> trip;
			generate_list_of_triplets(trip,rotations*n_planes,radius,s_radius,tree_neighbors,pts[n], points_rand,vecInt);

			std::vector<Vector3> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for(int i=0; i<rotations; i++){

				vecTripIterator first = trip.begin() + i*n_planes;
				vecTripIterator last = trip.begin() + (i+1)*n_planes;
				std::vector<Triplet> triplets(first, last);

				for(unsigned int tr=0; tr <triplets.size(); tr++){
					triplets[tr](0)=triplets[tr](0).transform(rotMat[(n+i)%rotMat.size()]);
					triplets[tr](1)=triplets[tr](1).transform(rotMat[(n+i)%rotMat.size()]);
					triplets[tr](2)=triplets[tr](2).transform(rotMat[(n+i)%rotMat.size()]);
				}

				normals_conf[i] = normal_at_point( d1, d2,points_size,  n,  triplets, conf_interv);

				normals_vec[i] =nls[n].transform(rotMatInv[(n+i)%rotMat.size()]);


			}

			nls[n] = normal_selection( rotations, normals_vec,normals_conf);

		}
	}

	/*!
	 * \brief Radius search, Uniform draw of the planes
	 * Normal Estimation using a range neighbor search, planes are drawn uniformly in the neighborhood sphere
	 * @param radius : range radius for neighborhood search
	 */
	void unif_radius(float radius)
	{

		//initialize the random number generator
		srand((unsigned int)time(NULL));
		//resizing the normal point cloud
		nls.resize(pts.size());

		Tree tree(pts.begin(), pts.end());

		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		//creation of the rotation matrices and their inverses
		std::vector<Matrix3> rotMat;
		std::vector<Matrix3> rotMatInv;
		generate_rotation_matrix(rotMat,rotMatInv, n_rot*200);

		int rotations;
		if(n_rot==0){
			rotations = 1;
		}else{
			rotations = n_rot;
		}

		//creation of vector of int and points
		std::vector<Point3> points_rand;
		std::vector<int> vecInt;
		generate_random_points_vector(points_rand, 1000000);
		generate_random_int_vector(vecInt, 1000000);

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel for schedule(guided)
#endif
		for(int n=0; n<(int) pts.size(); n++){

			std::vector<Point3> points;
			Fuzzy_sphere s_query(pts[n],radius);
#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp critical
#endif
			{
				tree.search(std::back_inserter(points), s_query);
			}

			int points_size = points.size();
			float radius2 = radius;

			if(points_size <lower_neighbor_bound_neighbors){
				radius2 = 0;
				points.clear();
#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp critical
#endif
				{
					//getting the list of neighbors
					Neighbor_search search(tree, pts[n], lower_neighbor_bound_neighbors);

					for(Neighbor_search_iterator it = search.begin(); it != search.end() && points_size < lower_neighbor_bound_neighbors; ++it){
						points.push_back(it->first);
						points_size++;
						if(radius2 < it->second){
							radius2 = it->second;
						}
					}
				}
				points_size = points.size();
			}


			float s_radius = radius2 / small_radius_factor;


			//point cloud of neighbors and kdtree creation
			Tree tree_neighbors(points.begin(), points.end());

			//creating the list of triplets
			std::vector<Triplet> trip;
			generate_list_of_triplets(trip,rotations*n_planes,radius2,s_radius,tree_neighbors,pts[n], points_rand,vecInt);

			std::vector<Vector3> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for(int i=0; i<rotations; i++){

				vecTripIterator first = trip.begin() + i*n_planes;
				vecTripIterator last = trip.begin() + (i+1)*n_planes;
				std::vector<Triplet> triplets(first, last);

				for(unsigned int tr=0; tr <triplets.size(); tr++){
					triplets[tr](0)=triplets[tr](0).transform(rotMat[(n+i)%rotMat.size()]);
					triplets[tr](1)=triplets[tr](1).transform(rotMat[(n+i)%rotMat.size()]);
					triplets[tr](2)=triplets[tr](2).transform(rotMat[(n+i)%rotMat.size()]);
				}

				normals_conf[i] = normal_at_point( d1, d2,points_size,  n,  triplets, conf_interv);

				normals_vec[i] = nls[n].transform(rotMatInv[(n+i)%rotMat.size()]);


			}
			nls[n] = normal_selection( rotations, normals_vec,normals_conf);



		}

	}

	/*!
	 * \brief Knn search, Cubes draw of the planes
	 * Normal Estimation using a k-nearest neighbor search, planes are drawn using a cubic discretization of the neighborhood sphere
	 * @param neighbor_number : number of neighbors
	 */
	void cubes_knn(int neighbor_number)
	{


		//resizing the normal point cloud
		nls.resize(pts.size());

		//initialize the random number generator
		srand((unsigned int)time(NULL));

		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		//creation of the rotation matrices and their inverses
		std::vector<Matrix3> rotMat;
		std::vector<Matrix3> rotMatInv;
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


#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel
		{
			Tree tree(pts.begin(), pts.end());
#pragma omp for schedule(guided)
#else
		Tree tree(pts.begin(), pts.end());
#endif
		for(int n=0; n<(int)pts.size(); n++){

			std::vector<Point3>points2(neighbor_number);
			std::vector<Point3> points(neighbor_number);
			int points_size = 0;
			float radius = 0;

			//getting the list of neighbors
			Neighbor_search search(tree, pts[n], neighbor_number);

			for(Neighbor_search_iterator it = search.begin(); it != search.end() && points_size < neighbor_number; ++it){
				points2[points_size] = it->first;
				points[points_size] = it->first;
				points_size++;
				if(radius < it->second){
					radius = it->second;
				}
			}

			if(points_size != neighbor_number){continue;}
			radius = sqrt(radius);

			std::vector<int>* cubes = new std::vector<int>[n_cubes*n_cubes*n_cubes];
			std::vector<Vector3> triplets;
			std::vector<Vector3> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);

			for(int i=0; i<rotations; i++){

				assign_points_to_cubes(cubes,radius,pts[n], points,points2,rotMat[(n+i)%rotMat.size()]);

				generate_cubes_triplets(triplets,  cubes,  cubes_idx, vecInt);

				//cout << "1" << endl;
				normals_conf[i] = normal_at_point( d1, d2,points,  n,  triplets, conf_interv);
				//cout << "2" << endl;

				for(unsigned int pt= 0; pt < points_size; pt++){
					points[pt]=points2[pt];
				}
				normals_vec[i]= nls[n].transform(rotMatInv[(n+i)%rotMat.size()]);


			}

			nls[n] = normal_selection( rotations, normals_vec,normals_conf);


			delete[] cubes;

		}
#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
	}
#endif
	}


	/*!
	 * \brief Radius search, Cubes draw of the planes
	 * Normal Estimation using a range neighbor search, planes are drawn using a cubic discretization of the neighborhood sphere
	 * @param radius
	 */
	void cubes_radius(float radius)
	{



		//resizing the normal point cloud
		nls.resize(pts.size());

		//initialize the random number generator
		srand((unsigned int)time(NULL));


		//dimensions of the accumulator
		const int d1 = 2*n_phi;
		const int d2 = n_phi+1;

		//confidence intervals (2 intervals length)
		std::vector<float> conf_interv(n_planes);
		for(int i=0; i<n_planes; i++){
			conf_interv[i] = 2.f/std::sqrt(i+1.f);
		}

		//creation of the rotation matrices and their inverses
		std::vector<Matrix3> rotMat;
		std::vector<Matrix3> rotMatInv;
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

#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
#pragma omp parallel
		{
			Tree tree(pts.begin(), pts.end());
#pragma omp for schedule(guided)
#else
		Tree tree(pts.begin(), pts.end());
#endif
		for(int n=0; n<(int)pts.size(); n++){

			std::vector<Point3>points2;
				Fuzzy_sphere s_query(pts[n],radius);
				tree.search(std::back_inserter(points2), s_query);

			int points_size = points2.size();
			float radius2 = radius;
			if(points_size <lower_neighbor_bound_neighbors){
				radius2 = 0;
				points2.clear();
					//getting the list of neighbors
					Neighbor_search search(tree, pts[n], lower_neighbor_bound_neighbors);

					for(Neighbor_search_iterator it = search.begin(); it != search.end() && points_size < lower_neighbor_bound_neighbors; ++it){
						points2.push_back(it->first);
						points_size++;
						if(radius2 < it->second){
							radius2 = it->second;
						}
					}
				points_size = points2.size();
			}

			std::vector<int>* cubes = new std::vector<int>[n_cubes*n_cubes*n_cubes];
			std::vector<Vector3> triplets;
			std::vector<Vector3> normals_vec(rotations);
			std::vector<float> normals_conf(rotations);
			std::vector<Point3> points(points_size);


			for(int i=0; i<rotations; i++){

				assign_points_to_cubes(cubes,radius2,pts[n], points,points2,rotMat[(n+i)%rotMat.size()]);

				generate_cubes_triplets(triplets,  cubes,  cubes_idx, vecInt);

				//cout << "1" << endl;
				normals_conf[i] = normal_at_point( d1, d2,points,  n,  triplets, conf_interv);
				//cout << "2" << endl;

				for(unsigned int pt= 0; pt < points_size; pt++){
					points[pt]=points2[pt];
				}
				normals_vec[i]  = nls[n].transform(rotMatInv[(n+i)%rotMat.size()]);

			}

			nls[n] = normal_selection( rotations, normals_vec,normals_conf);


			delete[] cubes;

		}
#if defined(_OPENMP) && defined(USE_OPENMP_FOR_NORMEST)
	}
#endif
	}






};
#endif
