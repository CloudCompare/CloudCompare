/*
This source code is about a ground filtering algorithm for airborn LiDAR data
based on physical process simulations, specifically cloth simulation.

this code is based on a Cloth Simulation Tutorial at the cg.alexandra.dk blog.
Thanks to Jesper Mosegaard (clothTutorial@jespermosegaard.dk)



When applying the cloth simulation to LIDAR point filtering. A lot of features
have been added to the original source code, including
* configuration file management
* point cloud data read/write
* point-to-point collsion detection
* nearest point search structure from CGAL
* addding a terrain class


*/
//using discrete steps (drop and pull) to approximate the physical process
//test merge 在每个不料点周围找最邻近的N个点，以高程最大值作为所能到达的最低点。

#ifndef _CLOTH_H_
#define _CLOTH_H_

//local
#include "Vec3.h"
#include "Particle.h"
#include "Constraint.h"

//system
#include <vector>
#include <string>

class ccMesh;

struct XY
{
	XY(int x1, int y1)
		: x(x1)
		, y(y1)
	{}
	int x;
	int y;
};

class Cloth
{
private:

	// total number of particles is num_particles_width*num_particles_height
	int constraint_iterations;

	int rigidness;
	double time_step;

	std::vector<Particle> particles; // all particles that are part of this cloth
	std::vector<Constraint> constraints; // alle constraints between particles as part of this cloth


	//初始平面位置
	Vec3 origin_pos1; //左上角顶点  平面均为水平

	//滤波边坡处理参数
	double smoothThreshold;
	double heightThreshold;

	//heightvalues
	std::vector<double> heightvals;

	//movable particle index
	std::vector<int> movableIndex;
	std::vector< std::vector<int> > particle_edges;

	inline Particle& getParticle(int x, int y) { return particles[y*num_particles_width + x]; }
	inline const Particle& getParticle(int x, int y) const { return particles[y*num_particles_width + x]; }
	
	inline void addConstraint(Particle *p1, Particle *p2) { constraints.push_back(Constraint(p1, p2)); }

public:
	
	inline Particle getParticleByIndex(int index) { return particles[index]; }
	inline const Particle getParticleByIndex(int index) const { return particles[index]; }

	int num_particles_width; // number of particles in "width" direction
	int num_particles_height; // number of particles in "height" direction

	inline int getSize() const { return num_particles_width * num_particles_height; }

	inline std::vector<double>& getHeightvals() { return heightvals; }

public:
	
	/* This is a important constructor for the entire system of particles and constraints */
	Cloth(	double width,
			double height,
			int num_particles_width,
			int num_particles_height,
			const Vec3& origin_pos1,
			double smoothThreshold,
			double heightThreshold,
			int rigidness,
			double time_step);

	void setheightvals(const std::vector<double>& heightvals)
	{
		this->heightvals = heightvals;
	}

	/** This is an important methods where the time is progressed one time step for the entire cloth.
		This includes calling satisfyConstraint() for every constraint, and calling timeStep() for all particles
	**/
	double timeStep();

	/* used to add gravity (or any other arbitrary vector) to all particles */
	void addForce(const Vec3& direction);

	//检测布料是否与地形碰撞
	void terrainCollision();

	//对可移动的点进行边坡处理
	void movableFilter();
	//找到每组可移动点，这个连通分量周围的不可移动点。从四周向中间逼近
	void findUnmovablePoint(const std::vector<XY>& connected,
							const std::vector<double>& heightvals,
							std::vector<int>& edgePoints);
	
	//直接对联通分量进行边坡处理
	void handle_slop_connected(	const std::vector<int>& edgePoints,
								const std::vector<XY>& connected,
								const std::vector< std::vector<int> >& neighbors,
								const std::vector<double> &heightvals);

	//将布料点保存到文件
	void saveToFile(std::string path = "");
	//将可移动点保存到文件
	void saveMovableToFile(std::string path = "");

	//! Converts the cloth to a CC mesh structure
	ccMesh* toMesh() const;

};

#endif