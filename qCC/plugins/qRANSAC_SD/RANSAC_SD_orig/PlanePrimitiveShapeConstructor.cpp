#include "PlanePrimitiveShapeConstructor.h"
#include "PlanePrimitiveShape.h"
#include "ScoreComputer.h"
#include <GfxTL/NullClass.h>

size_t PlanePrimitiveShapeConstructor::Identifier() const
{
	return 0;
}

unsigned int PlanePrimitiveShapeConstructor::RequiredSamples() const
{
	return 3;
}

PrimitiveShape *PlanePrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &points, const MiscLib::Vector< Vec3f > &) const
{
	return new PlanePrimitiveShape(points[0], points[1], points[2]);
}

PrimitiveShape *PlanePrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &samples) const
{
	Plane plane;
	if(!plane.Init(samples))
		return NULL;
	return new PlanePrimitiveShape(plane);
}

PrimitiveShape *PlanePrimitiveShapeConstructor::Deserialize(std::istream *i,
	bool binary) const
{
	Plane plane;
	plane.Init(binary, i);
	PlanePrimitiveShape *shape = new PlanePrimitiveShape(plane);
	return shape;
}

size_t PlanePrimitiveShapeConstructor::SerializedSize() const
{
	return Plane::SerializedSize();
}
