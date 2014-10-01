//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef GEOMETRICAL_ANALYSIS_TOOLS_HEADER
#define GEOMETRICAL_ANALYSIS_TOOLS_HEADER

//Local
#include "CCCoreLib.h"
#include "CCToolbox.h"
#include "Neighbourhood.h"
#include "DgmOctree.h"
#include "Matrix.h"

namespace CCLib
{

class GenericProgressCallback;
class GenericCloud;
class ScalarField;

//! Several algorithms to compute point-clouds geometric characteristics  (curvature, density, etc.)
class CC_CORE_LIB_API GeometricalAnalysisTools : public CCToolbox
{
public:

	//! Computes the local curvature
	/** \warning this method assumes the input scalar field is different from output.
		\param theCloud processed cloud
		\param cType curvature type
		\param kernelRadius neighbouring sphere radius
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
	**/
	static int computeCurvature(GenericIndexedCloudPersist* theCloud,
								Neighbourhood::CC_CURVATURE_TYPE cType,
								PointCoordinateType kernelRadius,
								GenericProgressCallback* progressCb = 0,
								DgmOctree* inputOctree = 0);

	enum Density {	DENSITY_KNN,	/**< The number of points inside the neighborhing sphere **/
					DENSITY_2D,		/**< The number of points divided by the area of the circle that has the same radius as the neighborhing sphere (2D approximation) **/
					DENSITY_3D,		/**< The number of points divided by the neighborhing sphere volume (3D) **/
	};

	//! Computes the local density (approximate)
	/** Old method (based only on the distance to the nearest neighbor).
		\warning As only one neighbor is extracted, the DENSITY_KNN type corresponds in fact to the (inverse) distance to the nearest neighbor.
		\warning This method assumes the input scalar field is different from the output one.
		\param theCloud processed cloud
		\param densityType the 'type' of density to compute
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
	**/
	static int computeLocalDensityApprox(	GenericIndexedCloudPersist* theCloud,
											Density densityType,
											GenericProgressCallback* progressCb = 0,
											DgmOctree* inputOctree = 0);

	//! Computes the local density (at a given scale)
	/** Simply counts the number of points falling inside a sphere around each point
		\warning this method assumes the input scalar field is different from output.
		\param theCloud processed cloud
		\param densityType the 'type' of density to compute
		\param kernelRadius neighbouring sphere radius
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
	**/
	static int computeLocalDensity(	GenericIndexedCloudPersist* theCloud,
									Density densityType,
									PointCoordinateType kernelRadius,
									GenericProgressCallback* progressCb = 0,
									DgmOctree* inputOctree = 0);

	//! Computes the local roughness
	/** Roughness is defined as the distance to the locally (least square) fitted plane.
		LS plane is computed with all neighbour points inside a sphere.
		\warning this method assumes the input scalar field is different from output.
		\param theCloud processed cloud
		\param kernelRadius neighbouring sphere radius
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
	**/
	static int computeRoughness(GenericIndexedCloudPersist* theCloud,
								PointCoordinateType kernelRadius,
								GenericProgressCallback* progressCb = 0,
								DgmOctree* inputOctree = 0);

	//! Computes the gravity center of a point cloud
	/** \warning this method uses the cloud global iterator
		\param theCloud cloud
		\return gravity center
	**/
	static CCVector3 computeGravityCenter(GenericCloud* theCloud);

	//! Computes the cross covariance matrix between two clouds (same size)
	/** Used in the ICP algorithm between the cloud to register and the "Closest Points Set"
		determined from the reference cloud.
		\warning this method uses the clouds global iterators
		\param P the cloud to register
		\param Q the "Closest Point Set"
		\param pGravityCenter the gravity center of P
		\param qGravityCenter the gravity center of Q
		\return cross covariance matrix
	**/
	static SquareMatrixd computeCrossCovarianceMatrix(	GenericCloud* P,
														GenericCloud* Q,
														const CCVector3& pGravityCenter,
														const CCVector3& qGravityCenter);
	
	//! Computes the cross covariance matrix between two clouds (same size) - weighted version
	/** Used in the ICP algorithm between the cloud to register and the "Closest Points Set"
		determined from the reference cloud.
		\warning this method uses the clouds global iterators
		\param P the cloud to register
		\param Q the "Closest Point Set"
		\param pGravityCenter the gravity center of P
		\param qGravityCenter the gravity center of Q
		\param weightsP weights for the points of P (optional)
		\param weightsQ weights for the points of Q (optional)
		\return weighted cross covariance matrix
	**/
	static SquareMatrixd computeWeightedCrossCovarianceMatrix(	GenericCloud* P,
																GenericCloud* Q,
																const CCVector3& pGravityCenter,
																const CCVector3& qGravityCenter,
																ScalarField* weightsP = 0,
																ScalarField* weightsQ = 0);

	//! Computes the covariance matrix of a clouds
	/** \warning this method uses the cloud global iterator
		\param theCloud point cloud
		\param _gravityCenter if available, its gravity center
		\return covariance matrix
	**/
	static CCLib::SquareMatrixd computeCovarianceMatrix(GenericCloud* theCloud,
														const PointCoordinateType* _gravityCenter = 0);

	//! Flag duplicate points
	/** This method only requires an output scalar field. Duplicate points will be
		associated to scalar value 1 (and 0 for the others).
		\param theCloud processed cloud
		\param minDistanceBetweenPoints min distance between (output) points
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
	**/
	static int flagDuplicatePoints(	GenericIndexedCloudPersist* theCloud,
									double minDistanceBetweenPoints = 1.0e-12,
									GenericProgressCallback* progressCb = 0,
									DgmOctree* inputOctree = 0);

protected:

	//! Computes cell curvature inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool computeCellCurvatureAtLevel(const DgmOctree::octreeCell& cell,
											void** additionalParameters,
											NormalizedProgress* nProgress = 0);

	//! Computes approximate point density inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool computeApproxPointsDensityInACellAtLevel(	const DgmOctree::octreeCell& cell,
															void** additionalParameters,
															NormalizedProgress* nProgress = 0);

	//! Computes point density inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool computePointsDensityInACellAtLevel(	const DgmOctree::octreeCell& cell,
													void** additionalParameters,
													NormalizedProgress* nProgress = 0);

	//! Computes point roughness inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool computePointsRoughnessInACellAtLevel(	const DgmOctree::octreeCell& cell,
														void** additionalParameters,
														NormalizedProgress* nProgress = 0);

	//! Flags duplicate points inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool flagDuplicatePointsInACellAtLevel(	const DgmOctree::octreeCell& cell,
													void** additionalParameters,
													NormalizedProgress* nProgress = 0);
};

}

#endif //GEOMETRICAL_ANALYSIS_TOOLS_HEADER
