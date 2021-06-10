#include "TorusPrimitiveShapeConstructor.h"
#include "TorusPrimitiveShape.h"
#include "ScoreComputer.h"
#include <GfxTL/NullClass.h>

TorusPrimitiveShapeConstructor::TorusPrimitiveShapeConstructor(bool allowAppleShaped
	, float minMinorRadius
	, float minMajorRadius
	, float maxMinorRadius
	, float maxMajorRadius)
	: m_allowAppleShaped(allowAppleShaped)
	, m_minMinorRadius(minMinorRadius)
	, m_minMajorRadius(minMajorRadius)
	, m_maxMinorRadius(maxMinorRadius)
	, m_maxMajorRadius(maxMajorRadius)
{
}

size_t TorusPrimitiveShapeConstructor::Identifier() const
{
	return 4;
}

unsigned int TorusPrimitiveShapeConstructor::RequiredSamples() const
{
	return 4;
}

PrimitiveShape *TorusPrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &points,
	const MiscLib::Vector< Vec3f > &normals) const
{
	MiscLib::Vector< Vec3f > samples;
	for(size_t i = 0; i < points.size(); ++i)
		samples.push_back(points[i]);
	for(size_t i = 0; i < normals.size(); ++i)
		samples.push_back(normals[i]);
	return Construct(samples);
}

PrimitiveShape *TorusPrimitiveShapeConstructor::Construct(
	const MiscLib::Vector< Vec3f > &samples) const
{
	Torus torus;
	if (!torus.Init(samples))
	{
		return NULL;
	}
	
	if ((!m_allowAppleShaped && torus.IsAppleShaped()) ||
		torus.MinorRadius() < m_minMinorRadius ||
		torus.MajorRadius() < m_minMajorRadius ||
		torus.MinorRadius() > m_maxMinorRadius ||
		torus.MajorRadius() > m_maxMajorRadius)
	{
		return NULL;
	}
	return new TorusPrimitiveShape(torus, m_allowAppleShaped, m_maxMinorRadius, m_maxMajorRadius);
}

PrimitiveShape *TorusPrimitiveShapeConstructor::Deserialize(std::istream *i,
	bool binary) const
{
	TorusPrimitiveShape *shape = new TorusPrimitiveShape();
	shape->Deserialize(i, binary);
	return shape;
}

size_t TorusPrimitiveShapeConstructor::SerializedSize() const
{
	return Torus::SerializedSize() +
		TorusPrimitiveShape::ParametrizationType::SerializedSize();
}
