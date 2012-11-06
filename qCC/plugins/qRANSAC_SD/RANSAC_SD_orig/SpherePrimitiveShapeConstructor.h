#ifndef SPHEREPRIMITIVESHAPECONSTRUCTOR_HEADER
#define SPHEREPRIMITIVESHAPECONSTRUCTOR_HEADER
#include "PrimitiveShapeConstructor.h"
#include <limits>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE SpherePrimitiveShapeConstructor
: public PrimitiveShapeConstructor
{
	public:
		SpherePrimitiveShapeConstructor(float maxSphereRadius =
			std::numeric_limits< float >::infinity());
		size_t Identifier() const;
		unsigned int RequiredSamples() const;
        PrimitiveShape *Construct(const MiscLib::Vector< Vec3f > &points,
			const MiscLib::Vector< Vec3f > &normals) const;
		PrimitiveShape *Construct(const MiscLib::Vector< Vec3f > &samples) const;
		PrimitiveShape *Deserialize(std::istream *i, bool binary = true) const;
		size_t SerializedSize() const;

	private:
		float m_maxSphereRadius;
};

#endif
