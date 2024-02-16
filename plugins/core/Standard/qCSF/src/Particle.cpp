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

#include "Particle.h"

/* Some physics constants */
constexpr double DAMPING = 0.01; // how much to damp the cloth simulation each frame

/* We precompute the overall displacement of a particle accroding to the rigidness */
static const double SingleMove1[15]{ 0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765, 0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322 };
static const double DoubleMove1[15]{ 0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498, 0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5 };

/* This is one of the important methods, where the time is progressed a single step size
	The method is called by Cloth.time_step()
	Given the equation "force = mass * acceleration" the next position is found through verlet integration
*/
void Particle::timeStep()
{
	if (movable)
	{
		double deltaY = pos.y - old_pos_y;
		old_pos_y = pos.y;
		pos.y += deltaY * (1.0 - DAMPING) + acceleration/* * time_step2*/; // DGM: already done in CSF.cpp
	}
}

void Particle::satisfyConstraintSelf(int constraintTimes)
{
	Particle* p1 = this;
	for (Particle* p2 : neighborsList)
	{
		double correctionHeight = p2->pos.y - p1->pos.y;
		if (p1->isMovable() && p2->isMovable())
		{
			double correctionVectorHalf = correctionHeight * (constraintTimes > 14 ? 0.5 : DoubleMove1[constraintTimes]); // Lets make it half that length, so that we can move BOTH p1 and p2.
			p1->offsetPos(correctionVectorHalf);
			p2->offsetPos(-correctionVectorHalf);
		}
		else if (p1->isMovable() && !p2->isMovable())
		{
			double correctionVectorHalf = correctionHeight * (constraintTimes > 14 ? 1 : SingleMove1[constraintTimes]); // Lets make it half that length, so that we can move BOTH p1 and p2.
			p1->offsetPos(correctionVectorHalf);
		}
		else if (!p1->isMovable() && p2->isMovable())
		{
			double correctionVectorHalf = correctionHeight * (constraintTimes > 14 ? 1 : SingleMove1[constraintTimes]); // Lets make it half that length, so that we can move BOTH p1 and p2.
			p2->offsetPos(-correctionVectorHalf);
		}
	}
}
