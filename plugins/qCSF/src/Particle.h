//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qCSF                              #
//#                                                                                     #
//#        This program is free software; you can redistribute it and/or modify         #
//#        it under the terms of the GNU General Public License as published by         #
//#        the Free Software Foundation; version 2 or later of the License.             #
//#                                                                                     #
//#        This program is distributed in the hope that it will be useful,              #
//#        but WITHOUT ANY WARRANTY; without even the implied warranty of               #
//#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 #
//#        GNU General Public License for more details.                                 #
//#                                                                                     #
//#        Please cite the following paper, If you use this plugin in your work.        #
//#                                                                                     #
//#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
//#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#               RAMM laboratory, School of Geography, Beijing Normal University       #
//#                               (http://ramm.bnu.edu.cn/)                             #
//#                                                                                     #
//#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
//#                                                                                     #
//#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
//#                                                                                     #
//#######################################################################################

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
	std::vector<int> correspondingLidarPointList;//Ã¿¸ö²¼ÁÏ½Úµã¶ÔÓ¦µÄLidarµãµÄÁÐ±í  the correspoinding lidar point list
	std::size_t nearestPointIndex;//¶ÔÓ¦µÄlidarµã×îÁÙ½üµãµÄË÷Òý index  nearest lidar point
	double nearestPointHeight;//¸ÃµãµÄyÖáÖµ  the height(y) of the nearest lidar point
	double tmpDist;//ÁÙÊ±±äÁ¿£¬ÓÃÓÚ¼ÆËãlidarµãÔÙË®Æ½ÃæÉÏ¾àÀë²¼ÁÏµãÖ±½ÓµÄ¾àÀë  only for inner computation
	
public:

	Particle()
		: movable(true)
		//, mass(1)
		, acceleration(0, 0, 0)
		//, accumulated_normal(0, 0, 0)
		, time_step2(0.0)
		, isVisited(false)
		//, neibor_count(0)
		, pos_x(0)
		, pos_y(0)
		, c_pos(0)
		//, pos()
		//, old_pos(pos)
		, nearestPointHeight(MIN_INF)
		, tmpDist(MAX_INF)
	{}
	
	Particle(Vec3 posVec, double time_step)
		: Particle()
	{
		pos = posVec;
		old_pos = posVec;
		time_step2 = time_step;
	}
	
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