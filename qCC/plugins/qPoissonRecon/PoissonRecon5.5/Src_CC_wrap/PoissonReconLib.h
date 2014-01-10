//##########################################################################
//#                                                                        #
//#               CLOUDCOMPARE WRAPPER: PoissonReconLib                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#               COPYRIGHT: Daniel Girardeau-Montaut                      #
//#                                                                        #
//##########################################################################

#ifndef CC_POISSON_RECON_LIB_5_5_WRAPPER
#define CC_POISSON_RECON_LIB_5_5_WRAPPER

//PoissonRecon
#include "../Src/PlyVertex.h"

class LibOctree;

//! Wrapper to use PoissonRecon (Kazdan et. al) as a library
class PoissonReconLib
{
public:

	//! Default constructor
	PoissonReconLib() : m_octree(0) {}

	//! Destructor
	~PoissonReconLib();

	//! Default vertex type
	typedef PlyVertex< float > Vertex;

	//! Default point type
	typedef Point3D< float > Point;

	//! Algorithm parameters
	struct Parameters
	{
		int depth;
		float scale;
		int solverDivide;
		float samplesPerNode;
		bool useConfidence;
		bool nonManifold;
		float pointWeight;
		int minDepth;
		float solverAccuracy;
		int minIters;
		int adaptiveExponent;
		int isoDivide;
		int fixedIters;

		Parameters()
		: depth(8)					// maximum reconstruction depth (running at depth d corresponds to solving on a 2^d x 2^d x 2^d)
		, scale(1.1f)				// specifies the factor of the bounding cube that the input samples should fit into
		, solverDivide(8)			// the depth at which a block Gauss-Seidel solver is used
		, samplesPerNode(1.0f)		// specifies the minimum number of points that should fall within an octree node
		, useConfidence(false)		// if this flag is enabled, the size of a sample's normals is used as a confidence value, affecting the sample's constribution to the reconstruction process
		, nonManifold(false)		// if this flag is enabled, the isosurface extraction does not add a planar polygon's barycenter in order to ensure that the output	mesh is manifold
		, pointWeight(0)			// specifies the weight that point interpolation constraints are given when defining the (screened) Poisson system
		, minDepth(5)				// specifies the minimum depth at which the octree is to be adaptive
		, solverAccuracy(1.0e-4f)	// solver accuracy
		, minIters(24)				// minimum number of solver iterations
		, adaptiveExponent(1)		// specifies the exponent scale for the adaptive weighting
		, isoDivide(8)				// uncomented in PoissonRecon
		, fixedIters(-1)			// uncomented in PoissonRecon
		{}
	};

	//! Initializes reconstruction process
	/** \param[in] count point cloud size
		\param[in] inPoints array of points
		\param[in] inNormals array of normals
		\param[in] params reconstruction parameters
		\param[out] outThreadCount number of thread that will be used (optional)
		\return initialization success
	**/
	bool init(	unsigned count,
				const Point* inPoints,
				const Point* inNormals,
				const Parameters& params,
				int* outThreadCount = 0);

	//! Launches reconstruction process
	/** \param[out] outMesh output mesh (if successful)
		\return reconstruction success
	**/
	bool reconstruct( CoredVectorMeshData< Vertex >& outMesh );

protected:

	//! Current parameters
	Parameters m_params;

	//! Associated octree
	LibOctree* m_octree;
};

#endif // CC_POISSON_RECON_LIB_5_5_WRAPPER
