#ifndef _PARTICLE_H_
#define _PARTICLE_H_
#include <vector>
#include "Vec3.h"

/* Some physics constants */
#define DAMPING 0.01 // how much to damp the cloth simulation each frame
#define MAX_INF 9999999999 
#define MIN_INF -9999999999

/* The particle class represents a particle that can move around in 3D space*/
class Particle
{
private:
	bool movable; // can the particle move or not ? used to pin parts of the cloth
	//double mass; // the mass of the particle (is always 1 in this example)
	Vec3 acceleration; // a vector representing the current acceleration of the particle
	//Vec3 accumulated_normal; // an accumulated normal (i.e. non normalized), used for OpenGL soft shading
	double time_step2;
public:
	//this two memeber is used in the process of edge smoothing after the cloth simulation step.
	bool isVisited;
	//int neibor_count;
	int pos_x; //position in the cloth grid
	int pos_y;
	int c_pos;//position in the group of movable points
	Vec3 pos; // the current position of the particle in 3D space
	Vec3 old_pos; // the position of the particle in the previous time step, used as part of the verlet numerical integration scheme

	//for constraint computation
	std::vector<Particle *> neighborsList; //record all the neighbors in cloth grid

	//for rasterlization
	std::vector<int> correspondingLidarPointList;//每个布料节点对应的Lidar点的列表  the correspoinding lidar point list
	std::size_t nearestPointIndex;//对应的lidar点最临近点的索引 index  nearest lidar point
	double nearestPointHeight;//该点的y轴值  the height(y) of the nearest lidar point
	double tmpDist;//临时变量，用于计算lidar点再水平面上距离布料点直接的距离  only for inner computation
	
public:

	Particle() {}

	Particle(Vec3 pos, double time_step)
		: movable(true)
		//, mass(1)
		, acceleration(0, 0, 0)
		//, accumulated_normal(0, 0, 0)
		, time_step2(time_step)
		, isVisited(false)
		//, neibor_count(0)
		, pos_x(0)
		, pos_y(0)
		, c_pos(0)
		, pos(pos)
		, old_pos(pos)
		, nearestPointHeight(MIN_INF)
		, tmpDist(MAX_INF)

	{}

	/* This is one of the important methods, where the time is progressed a single step size (TIME_STEPSIZE)
	The method is called by Cloth.time_step()*/
	void timeStep();

	inline bool isMovable() const { return movable; }

	inline void addForce(const Vec3& f) { acceleration += f/*/ mass*/; }

	inline void resetAcceleration() { acceleration = Vec3(0, 0, 0); }

	inline void offsetPos(const Vec3 v) { if (movable) pos += v; }

	inline void makeUnmovable() { movable = false; }

	//do constraint
	void satisfyConstraintSelf(int constraintTimes);

	//inline void addToNormal(Vec3 normal) { accumulated_normal += normal.normalized(); }

	//inline const Vec3& getNormal() const { return accumulated_normal; } // notice, the normal is not unit length

	//inline void resetNormal() { accumulated_normal = Vec3(0, 0, 0); }
};

#endif