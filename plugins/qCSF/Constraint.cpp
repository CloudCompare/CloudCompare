#include "Constraint.h"

void Constraint::satisfyConstraint()
{
	Vec3 correctionVector(0, p2->pos.y - p1->pos.y, 0);
	
	Vec3 correctionVectorHalf = correctionVector * 0.3; // Lets make it half that length, so that we can move BOTH p1 and p2.
	if (p1->isMovable())
		p1->offsetPos(correctionVectorHalf); // correctionVectorHalf is pointing from p1 to p2, so the length should move p1 half the length needed to satisfy the constraint.
	if (p2->isMovable())
		p2->offsetPos(-correctionVectorHalf); // we must move p2 the negative direction of correctionVectorHalf since it points from p2 to p1, and not p1 to p2.
}