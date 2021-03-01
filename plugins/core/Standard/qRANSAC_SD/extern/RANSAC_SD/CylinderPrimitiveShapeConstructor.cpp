#include "CylinderPrimitiveShapeConstructor.h"
#include "ScoreComputer.h"
#include <GfxTL/NullClass.h>

CylinderPrimitiveShapeConstructor::CylinderPrimitiveShapeConstructor(
	float minCylinderRadius, float maxCylinderRadius, float maxCylinderLength)
	: m_minCylinderRadius(minCylinderRadius)
	, m_maxCylinderRadius(maxCylinderRadius)
	, m_maxCylinderLength(maxCylinderLength)

{}

size_t CylinderPrimitiveShapeConstructor::Identifier() const
{
	return 2;
}

unsigned int CylinderPrimitiveShapeConstructor::RequiredSamples() const
{
	return 4; //2; Because the specific 2point/normal 
			  //   constructor is not called in reality 
			  //   we need 4 points
}

PrimitiveShape *CylinderPrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &points,
	const MiscLib::Vector< Vec3f > &normals) const
{
	Cylinder cy;
	MiscLib::Vector< Vec3f > samples(points);
	std::copy(normals.begin(), normals.end(), std::back_inserter(samples));
	if (!cy.Init(samples))
	{
		return NULL;
	}
	if (cy.Radius() > m_minCylinderRadius && cy.Radius() < m_maxCylinderRadius)
	{
		return new CylinderPrimitiveShape(cy, m_minCylinderRadius, m_maxCylinderRadius, m_maxCylinderLength);
	}
	return NULL;
}

PrimitiveShape *CylinderPrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &samples) const
{
	Cylinder cy;
	if (!cy.Init(samples))
	{
		return NULL;
	}
	if (cy.Radius() > m_minCylinderRadius && cy.Radius() < m_maxCylinderRadius)
	{
		return new CylinderPrimitiveShape(cy, m_minCylinderRadius, m_maxCylinderRadius, m_maxCylinderLength);
	}
	return NULL;
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
