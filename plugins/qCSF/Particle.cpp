
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
		acceleration = Vec3(0, 0, 0); // acceleration is reset since it HAS been translated into a change in position (and implicitely into velocity)	
	}
}


