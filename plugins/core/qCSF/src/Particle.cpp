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
/* This is one of the important methods, where the time is progressed a single step size (TIME_STEPSIZE)
The method is called by Cloth.time_step()
Given the equation "force = mass * acceleration" the next position is found through verlet integration*/
void Particle::timeStep()
{
	if (movable)
	{
		Vec3 temp = pos;
		pos = pos + (pos - old_pos) * (1.0 - DAMPING) + acceleration * time_step2;
		old_pos = temp;
		//acceleration = Vec3(0, 0, 0); // acceleration is reset since it HAS been translated into a change in position (and implicitely into velocity)	
	}
}

//we precompute the overall displacement of a particle accroding to the rigidness
//const double singleMove1[15] = {0, 0.4, 0.64, 0.784, 0.8704, 0.92224, 0.95334, 0.97201, 0.9832, 0.98992, 0.99395, 0.99637, 0.99782, 0.99869, 0.99922 };
const double singleMove1[15] = { 0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765, 0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322 };
//µ±”–¡Ω∂À“∆∂Ø ±
//const double doubleMove1[15] = {0, 0.4, 0.48, 0.496, 0.4992, 0.49984, 0.49997, 0.49999, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
const double doubleMove1[15] = { 0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498, 0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5 };

void Particle::satisfyConstraintSelf(int constraintTimes)
{
	Particle *p1 = this;
	for (int i = 0; i < neighborsList.size(); i++)
	{

		Particle * p2 = neighborsList[i];
		Vec3 correctionVector(0, p2->pos.y - p1->pos.y, 0);
		if (p1->isMovable() && p2->isMovable())
		{
			Vec3 correctionVectorHalf = correctionVector * (constraintTimes>14 ? 0.5 : doubleMove1[constraintTimes]); // Lets make it half that length, so that we can move BOTH p1 and p2.
			p1->offsetPos(correctionVectorHalf);
			p2->offsetPos(-correctionVectorHalf);
		}
		else if (p1->isMovable() && !p2->isMovable())
		{
			Vec3 correctionVectorHalf = correctionVector * (constraintTimes>14 ? 1 : singleMove1[constraintTimes]); // Lets make it half that length, so that we can move BOTH p1 and p2.
			p1->offsetPos(correctionVectorHalf);
		}
		else if (!p1->isMovable() && p2->isMovable())
		{
			Vec3 correctionVectorHalf = correctionVector * (constraintTimes>14 ? 1 : singleMove1[constraintTimes]); // Lets make it half that length, so that we can move BOTH p1 and p2.
			p2->offsetPos(-correctionVectorHalf);
		}
	}

}