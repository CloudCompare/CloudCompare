#ifndef TORUSPRIMITIVESHAPECONSTRUCTOR_HEADER
#define TORUSPRIMITIVESHAPECONSTRUCTOR_HEADER
#include "PrimitiveShapeConstructor.h"

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE TorusPrimitiveShapeConstructor
: public PrimitiveShapeConstructor
{
public:
	TorusPrimitiveShapeConstructor(bool allowAppleShaped = false
		, float minMinorRadius = -std::numeric_limits<float>::infinity()
		, float minMajorRadius = -std::numeric_limits<float>::infinity()
		, float maxMinorRadius = std::numeric_limits<float>::infinity()
		, float maxMajorRadius = std::numeric_limits<float>::infinity());
	size_t Identifier() const;
	unsigned int RequiredSamples() const;
	PrimitiveShape *Construct(const MiscLib::Vector< Vec3f > &points,
		const MiscLib::Vector< Vec3f > &normals) const;
	PrimitiveShape *Construct(const MiscLib::Vector< Vec3f > &samples) const;
	PrimitiveShape *Deserialize(std::istream *i, bool binary = true) const;
	size_t SerializedSize() const;
private:
	bool m_allowAppleShaped;
	float m_minMinorRadius;
	float m_minMajorRadius;
	float m_maxMinorRadius;
	float m_maxMajorRadius;
};

#endif
