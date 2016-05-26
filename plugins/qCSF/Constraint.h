#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include "Vec3.h"
#include "Particle.h"

class Constraint
{
private:
	double rest_distance; // the length between particle p1 and p2 in rest configuration

public:
	Particle *p1, *p2; // the two particles that are connected through this constraint

	Constraint(Particle *p1, Particle *p2) : p1(p1), p2(p2)
	{
		Vec3 vec = p1->getPos() - p2->getPos();
		rest_distance = vec.length();
	}

	/* This is one of the important methods, where a single constraint between two particles p1 and p2 is solved
	the method is called by Cloth.time_step() many times per frame*/
	void satisfyConstraint();
};




#endif