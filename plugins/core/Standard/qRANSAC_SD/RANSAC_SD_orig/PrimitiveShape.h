#ifndef PRIMITIVESHAPE_HEADER
#define PRIMITIVESHAPE_HEADER
#include "basic.h"
#include <MiscLib/RefCount.h>
#include "PointCloud.h"
#include <GfxTL/VectorXD.h>
#include <utility>
#include <string>
#include <MiscLib/Vector.h>
#include <deque>
#include <iostream>
#include "LevMarFunc.h"
#include <stdio.h>
#include <MiscLib/NoShrinkVector.h>
#include <MiscLib/RefCountPtr.h>

#ifndef DLL_LINKAGE
#define DLL_LINKAGE
#endif

class DLL_LINKAGE PrimitiveShapeVisitor;

/** PrimtiveShape is a shape primitive in conjunction with a parametrization.
 The parametrization is used for connected component computations.
 There are some deprecated serialization functions that do
 not always store the parametrization (at least for some shape types). These
 functions should no longer be used anywhere!
 Note: Computing connected components may change the parametrization! (Some
 shapes align the parametrization so that the memory for the bitmap is
 reduced)
*/
class DLL_LINKAGE PrimitiveShape
: public MiscLib::RefCount
{
public:
	// returns a unique identifier of the shape type
	virtual size_t Identifier() const = 0;
	// returns the number of point samples required to uniquely determine a shape
	virtual unsigned int RequiredSamples() const = 0;
	virtual PrimitiveShape *Clone() const = 0;
	virtual float Distance(const Vec3f &p) const = 0;
	virtual float SignedDistance(const Vec3f &p) const = 0;
	virtual float NormalDeviation(const Vec3f &p,
		const Vec3f &n) const = 0;
	// computes the distance to the shape and the deviation of the normal
	// to the normal at the nearest point on the shape
	virtual void DistanceAndNormalDeviation(
		const Vec3f &p, const Vec3f &n,
		std::pair< float, float > *dn) const = 0;
	// projects a point p onto the nearest point on the shape
	virtual void Project(const Vec3f &p, Vec3f *pp) const = 0;
	// gets the normal of the shape at the projection of p
	virtual void Normal(const Vec3f &p, Vec3f *n) const = 0;
	// finds the largest connected component in the points indexed by indices
	// Upon return the indices of the points that belong to the largest component
	// have been moved to the front of indices and their number is returned.
	// The remaining indices are stored at the end of the indices array
	virtual size_t ConnectedComponent(const PointCloud &pc, float epsilon,
		MiscLib::Vector< size_t > *indices, bool doFiltering = true, float* borderRatio = 0 ) = 0;
	virtual unsigned int ConfidenceTests(unsigned int numTests,
		float epsilon, float normalThresh, float rms, const PointCloud &pc,
		const MiscLib::Vector< size_t > &indices) const = 0;
	// returns descriptive string
	virtual void Description(std::string *s) const = 0;
	// refitting
	virtual bool Fit(const PointCloud &pc, float epsilon,
		float normalThresh,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end) = 0;
	virtual PrimitiveShape *LSFit(const PointCloud &pc, float epsilon,
		float normalThresh, MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end,
		std::pair< size_t, float > *score) const = 0;
	virtual LevMarFunc< float > *SignedDistanceFunc() const = 0;
	typedef MiscLib::Vector< GfxTL::VectorXD< 2, size_t > > BitmapPolygon;
	typedef MiscLib::Vector< BitmapPolygon > ComponentPolygons;
	virtual void TrimmingPolygons(const PointCloud &pc, float epsilon,
		size_t begin, size_t end,
		std::deque< ComponentPolygons > *polys) const = 0;
	// generates points on the shape from the primitive's bitmap i.e. where the bitmap is set
	virtual void GenerateBitmapPoints(const PointCloud &pc, float epsilon,
		size_t begin, size_t end, PointCloud *bmpPc) const = 0;
	/** This is the one and only serialization function
	 It stores all the parameters of the shape as well as the parametrization.
	 The first byte written (in both, binary and ascii mode) is the shape identifier */
	virtual void Serialize(std::ostream *o, bool binary = true) const = 0;
	virtual size_t SerializedSize() const = 0;
	// Serialize parameters to float array
	virtual void Serialize(float* array) const = 0;
	virtual size_t SerializedFloatSize() const = 0;	
	virtual void Transform(float scale, const Vec3f &translate) = 0;
	virtual void Visit(PrimitiveShapeVisitor *visitor) const = 0;
	virtual void SuggestSimplifications(const PointCloud &pc,
		MiscLib::Vector< size_t >::const_iterator begin,
		MiscLib::Vector< size_t >::const_iterator end, float distThresh,
		MiscLib::Vector< MiscLib::RefCountPtr< PrimitiveShape > > *suggestions) const {}
	virtual void OptimizeParametrization(const PointCloud &pc,
		size_t begin, size_t end, float epsilon) {}
	// gets the 2D parametrization coordinates of a point p in 3-space (it is projected first)
	virtual void Parameters(const Vec3f &p,
		std::pair< float, float > *param) const = 0;
	// computes the point and normal in 3-space from coordinates in the parametrization
	virtual bool InSpace(float u, float v, Vec3f *p, Vec3f *n) const = 0;
};

#endif
