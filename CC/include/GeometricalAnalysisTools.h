//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 or later of the  #
//#  License.                                                              #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef GEOMETRICAL_ANALYSIS_TOOLS_HEADER
#define GEOMETRICAL_ANALYSIS_TOOLS_HEADER

//Local
#include "DgmOctree.h"
#include "Neighbourhood.h"

namespace CCLib
{

class GenericProgressCallback;
class GenericCloud;
class ScalarField;

//! Several algorithms to compute point-clouds geometric characteristics  (curvature, density, etc.)
class CC_CORE_LIB_API GeometricalAnalysisTools : public CCToolbox
{
public:

	enum GeomCharacteristic	{	Feature,			/**< See Neighbourhood::GeomFeature **/
								Curvature,			/**< See Neighbourhood::CurvatureType **/
								LocalDensity,		/**< Accurate local density (see GeometricalAnalysisTools::Density) **/
								ApproxLocalDensity,	/**< Approximate local density (see GeometricalAnalysisTools::Density) **/
								Roughness,			/**< Roughness **/
								MomentOrder1		/**< 1st order moment **/
	};

	//! Density measurement
	enum Density {	DENSITY_KNN = 1,	/**< The number of points inside the neighborhing sphere **/
					DENSITY_2D,			/**< The number of points divided by the area of the circle that has the same radius as the neighborhing sphere (2D approximation) **/
					DENSITY_3D,			/**< The number of points divided by the neighborhing sphere volume (3D) **/
	};

	enum ErrorCode {	NoError = 0,
						InvalidInput = -1,
						NotEnoughPoints = -2,
						OctreeComputationFailed = -3,
						ProcessFailed = -4,
						UnhandledCharacteristic = -5,
						NotEnoughMemory = -6,
						ProcessCancelledByUser = -7
	};
	//! Unified way to compute a geometric characteristic
	/** Once the main geometric characterstic is chosen, the subOption parameter is used to specify
		the actual feature / curvature type / local density computation algorithm if necessary.
		\param c geometric characterstic
		\param subOption feature / curvature type / local density computation algorithm or nothing (0)
		\param cloud cloud to compute the characteristic on
		\param kernelRadius neighbouring sphere radius
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if not set as input, octree will be automatically computed.
		\return succes
	**/
	static ErrorCode ComputeCharactersitic(	GeomCharacteristic c,
											int subOption,
											GenericIndexedCloudPersist* cloud,
											PointCoordinateType kernelRadius,
											GenericProgressCallback* progressCb = nullptr,
											DgmOctree* inputOctree = nullptr);

	//! Computes the local density (approximate)
	/** Old method (based only on the distance to the nearest neighbor).
		\warning As only one neighbor is extracted, the DENSITY_KNN type corresponds in fact to the (inverse) distance to the nearest neighbor.
		\warning This method assumes the input scalar field is different from the output one.
		\param cloud processed cloud
		\param densityType the 'type' of density to compute
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
	**/
	static ErrorCode ComputeLocalDensityApprox(	GenericIndexedCloudPersist* cloud,
												Density densityType,
												GenericProgressCallback* progressCb = nullptr,
												DgmOctree* inputOctree = nullptr);

	//! Computes the gravity center of a point cloud
	/** \warning this method uses the cloud global iterator
		\param theCloud cloud
		\return gravity center
	**/
	static CCVector3 ComputeGravityCenter(GenericCloud* theCloud);

	//! Computes the weighted gravity center of a point cloud
	/** \warning this method uses the cloud global iterator
		\param theCloud cloud
		\param weights per point weights (only absolute values are considered)
		\return gravity center
	**/
	static CCVector3 ComputeWeightedGravityCenter(GenericCloud* theCloud, ScalarField* weights);

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
	static SquareMatrixd ComputeCrossCovarianceMatrix(	GenericCloud* P,
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
		\param coupleWeights weights for each (Pi,Qi) couple (optional)
		\return weighted cross covariance matrix
	**/
	static SquareMatrixd ComputeWeightedCrossCovarianceMatrix(	GenericCloud* P,
																GenericCloud* Q,
																const CCVector3& pGravityCenter,
																const CCVector3& qGravityCenter,
																ScalarField* coupleWeights = nullptr);

	//! Computes the covariance matrix of a clouds
	/** \warning this method uses the cloud global iterator
		\param theCloud point cloud
		\param _gravityCenter if available, its gravity center
		\return covariance matrix
	**/
	static CCLib::SquareMatrixd ComputeCovarianceMatrix(GenericCloud* theCloud,
														const PointCoordinateType* _gravityCenter = nullptr);

	//! Flag duplicate points
	/** This method only requires an output scalar field. Duplicate points will be
		associated to scalar value 1 (and 0 for the others).
		\param theCloud processed cloud
		\param minDistanceBetweenPoints min distance between (output) points
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param inputOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
	**/
	static ErrorCode FlagDuplicatePoints(	GenericIndexedCloudPersist* theCloud,
											double minDistanceBetweenPoints = std::numeric_limits<double>::epsilon(),
											GenericProgressCallback* progressCb = nullptr,
											DgmOctree* inputOctree = nullptr);

	//! Tries to detect a sphere in a point cloud
	/** Inspired from "Parameter Estimation Techniques: A Tutorial with Application
		to Conic Fitting" by Zhengyou Zhang (Inria Technical Report 2676).
		More specifically the section 9.5 about Least Median of Squares.
		\param[in]  cloud input cloud
		\param[in]  outliersRatio proportion of outliers (between 0 and 1)
		\param[out] center center of the detected sphere
		\param[out] radius radius of the detected sphere
		\param[out] rms residuals RMS for the detected sphere
		\param[in] progressCb for progress notification (optional)
		\param[in] confidence probability that the detected sphere is the right one (strictly below 1)
		\param[in] seed if different than 0, this seed will be used for random numbers generation (instead of a random one)
		\result success
	**/
	static ErrorCode DetectSphereRobust(	GenericIndexedCloudPersist* cloud,
											double outliersRatio,
											CCVector3& center,
											PointCoordinateType& radius,
											double& rms,
											GenericProgressCallback* progressCb = nullptr,
											double confidence = 0.99,
											unsigned seed = 0);

	//! Computes the center and radius of a sphere passing through 4 points
	/** \param[in] A first point
		\param[in] B second point
		\param[in] C third point
		\param[in] D fourth point
		\param[out] center center of the sphere
		\param[out] radius radius of the sphere
		\return success
	**/
	static ErrorCode ComputeSphereFrom4(	const CCVector3& A,
											const CCVector3& B,
											const CCVector3& C,
											const CCVector3& D,
											CCVector3& center,
											PointCoordinateType& radius );

protected:

	//! Computes geom characteristic inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool ComputeGeomCharacteristicAtLevel(	const DgmOctree::octreeCell& cell,
													void** additionalParameters,
													NormalizedProgress* nProgress = nullptr);
	//! Computes approximate point density inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool ComputeApproxPointsDensityInACellAtLevel(	const DgmOctree::octreeCell& cell,
															void** additionalParameters,
															NormalizedProgress* nProgress = nullptr);

	//! Flags duplicate points inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool FlagDuplicatePointsInACellAtLevel(	const DgmOctree::octreeCell& cell,
													void** additionalParameters,
													NormalizedProgress* nProgress = nullptr);

	//! Refines the estimation of a sphere by (iterative) least-squares
	static bool RefineSphereLS(	GenericIndexedCloudPersist* cloud,
								CCVector3& center,
								PointCoordinateType& radius,
								double minReltaiveCenterShift = 1.0e-3);

};

} //namespace CCLib

#endif //GEOMETRICAL_ANALYSIS_TOOLS_HEADER
