#ifndef SAITO_SQUARED_DISTANCE_TRANSFORM_HEADER
#define SAITO_SQUARED_DISTANCE_TRANSFORM_HEADER

// Inspired from bil_edt.cxx (VXL) by Ricardo Fabbri (rfabbri), Brown University  (rfabbri@lems.brown.edu)

//Local
#include "CCCoreLib.h"
#include "Grid3D.h"
#include "MathTools.h"

namespace CCLib
{

	class GenericProgressCallback;
	class NormalizedProgress;
	class GenericIndexedMesh;

	//! Class to compute a Squared Distance Field with the Saito algorithm on a 3D grid
	class CC_CORE_LIB_API SaitoSquaredDistanceTransform : public Grid3D<unsigned>, public MathTools
	{
	public:

		//! Default constructor
		SaitoSquaredDistanceTransform() : Grid3D<GridElement>() {}

		//! Initializes the grid
		/** The memory for the grid must be explicitelty reserved prior to any action.
			\return true if the initialization succeeded
		**/
		inline bool initGrid(const Tuple3ui& gridSize)
		{
			return Grid3D<GridElement>::init(gridSize.x, gridSize.y, gridSize.z, 0, 0); //margin = 0 (we need continuous memory)
		}

		//! Initializes the distance transform with a mesh
		inline bool initDT(	GenericIndexedMesh* mesh,
							PointCoordinateType cellLength,
							const CCVector3& gridMinCorner,
							GenericProgressCallback* progressCb = 0)
		{
			return intersecthWith(mesh, cellLength, gridMinCorner, 1, progressCb);
		}

		//! Computes the exact Squared Distance Transform on the whole grid
		/** Propagates the distances on the whole grid.
			
			Base on the implementation by R. Fabbri (which is itslef based on
			two independent implementations by O. Cuisenaire and J. C. Torelli).

			PAPER
			T. Saito and J.I. Toriwaki, "New algorithms for Euclidean distance
			transformations of an n-dimensional digitised picture with applications",
			Pattern Recognition, 27(11), pp. 1551-1565, 1994

			\warning Output distances are squared

			\param progressCb progress callback (optional)
			\return success
		**/
		inline bool propagateDistance(GenericProgressCallback* progressCb = 0) { return SDT_3D(*this, progressCb); }

	protected:

		//! 1D Euclidean Distance Transform
		static bool EDT_1D(GridElement* slice, size_t r, size_t c);
		//! 2D Exact Squared Distance Transform
		static bool SDT_2D(Grid3D<GridElement>& image, size_t sliceIndex, const std::vector<GridElement>& sq);
		//! 3D Exact Squared Distance Transform
		static bool SDT_3D(Grid3D<GridElement>& image, GenericProgressCallback* progressCb = 0);
	};

}

#endif //SIGNED_SAITO_DISTANCE_TRANSFORM_HEADER
