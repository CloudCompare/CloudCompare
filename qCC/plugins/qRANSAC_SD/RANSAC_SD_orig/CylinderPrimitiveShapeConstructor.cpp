#include "CylinderPrimitiveShapeConstructor.h"
#include "ScoreComputer.h"
#include <GfxTL/NullClass.h>

size_t CylinderPrimitiveShapeConstructor::Identifier() const
{
	return 2;
}

unsigned int CylinderPrimitiveShapeConstructor::RequiredSamples() const
{
	return 2;
}

PrimitiveShape *CylinderPrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &points,
	const MiscLib::Vector< Vec3f > &normals) const
{
	Cylinder cy;
	MiscLib::Vector< Vec3f > samples(points);
	std::copy(normals.begin(), normals.end(), std::back_inserter(samples));
	if(!cy.Init(samples))
		return NULL;
	return new CylinderPrimitiveShape(cy);
}

PrimitiveShape *CylinderPrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &samples) const
{
	Cylinder cy;
	if(!cy.Init(samples))
		return NULL;
	return new CylinderPrimitiveShape(cy);
}

PrimitiveShape *CylinderPrimitiveShapeConstructor::Deserialize(
	std::istream *i, bool binary) const
{
	Cylinder cylinder;
	cylinder.Init(binary, i);
	CylinderPrimitiveShape *shape = new CylinderPrimitiveShape(cylinder);
	return shape;
}

size_t CylinderPrimitiveShapeConstructor::SerializedSize() const
{
	return Cylinder::SerializedSize();
}
