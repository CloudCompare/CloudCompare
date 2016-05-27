#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include "Vec3.h"
#include "Particle.h"

class Constraint
{
public:

	Constraint(Particle *p1, Particle *p2)
		: p1(p1)
		, p2(p2)
	{}

	/* This is one of the important methods, where a single constraint between two particles p1 and p2 is solved
	the method is called by Cloth.time_step() many times per frame */
	void satisfyConstraint();

	// the two particles that are connected through this constraint
	Particle *p1, *p2;
};

#endif //_CONSTRAINT_H_
