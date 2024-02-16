#pragma once

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

#include <vector>
#include <limits>

#include "Vec3.h"

/* The particle class represents a particle that can move around in 3D space*/
class Particle
{
private:
	bool movable; // can the particle move or not ? used to pin parts of the cloth
	//double mass; // the mass of the particle (is always 1 in this example)
	double acceleration; // the current acceleration of the particle (along Y) - DGM: already multiplied by dt^²
	//double time_step2; // square time step

	Vec3 pos; // the current position of the particle in 3D space
	double old_pos_y; // the altitude of the particle at the previous time step, used as part of the verlet numerical integration scheme
public:
	//these members are used in the process of edge smoothing after the cloth simulation step.
	bool isVisited;
	int pos_x; // X position in the cloth grid
	int pos_y; // Y position in the cloth grid
	int c_pos; // position in the group of movable points

	//for constraint computation
	std::vector<Particle*> neighborsList; //record all the neighbors in cloth grid

	//for rasterization
	//std::vector<int> correspondingLidarPointList; // the correspoinding lidar point list (DGM: not used)
	//std::size_t nearestPointIndex; // nearest lidar point (DGM: not used)
	double nearestPointHeight; // the height(y) of the nearest lidar point
	double nearestPointDist; // only for internal computation

public:

	Particle()
		: movable(true)
		//, mass(1.0)
		, acceleration(0)
		//, time_step2(0)
		, isVisited(false)
		, pos_x(0)
		, pos_y(0)
		, c_pos(0)
		, pos(0, 0, 0)
		, old_pos_y(0)
		, nearestPointHeight(std::numeric_limits<double>::lowest())
		, nearestPointDist(std::numeric_limits<double>::max())
	{}
	
	Particle(const Vec3& posVec/*, double squareTimeStep*/)
		: Particle()
	{
		pos = posVec;
		old_pos_y = posVec.y;
		//time_step2 = squareTimeStep;
	}

	inline const Vec3& getPos() const { return pos; }
	inline double getPreviousY() const { return old_pos_y; }

	/* This is one of the important methods, where the time is progressed a single step size (TIME_STEPSIZE)
	The method is called by Cloth.time_step()*/
	void timeStep();

	inline bool isMovable() const { return movable; }

	inline void addForce(double f) { acceleration += f/*/ mass*/; }

	inline void resetAcceleration() { acceleration = 0; }

	inline void offsetPos(double dy)
	{
		if (movable)
		{
			pos.y += dy;
		}
	}

	inline void makeUnmovable() { movable = false; }

	//do constraint
	void satisfyConstraintSelf(int constraintTimes);
};
