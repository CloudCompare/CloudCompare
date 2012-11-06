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
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#ifndef GEOMETRICAL_ANALYSIS_TOOLS
#define GEOMETRICAL_ANALYSIS_TOOLS

#include "CCToolbox.h"
#include "Neighbourhood.h"
#include "DgmOctree.h"
#include "Matrix.h"

namespace CCLib
{

class GenericProgressCallback;
class GenericCloud;
class ScalarField;

//In case we face overflow issues (warning: may slow down computation)
//#define CC_OVERFLOW_SAFEGAURD

//! Several algorithms to compute point-clouds geometric characteristics  (curvature, density, etc.)
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
class CC_DLL_API GeometricalAnalysisTools : public CCToolbox
#else
class GeometricalAnalysisTools : public CCToolbox
#endif
{
public:

	//! Computes the local curvature
    /** Warning: this method assumes the input scalar field is different from output.
        \param theCloud processed cloud
        \param cType curvature type
        \param kernelRadius neighbouring sphere radius
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param _theOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
    **/
	static int computeCurvature(GenericIndexedCloudPersist* theCloud, Neighbourhood::CC_CURVATURE_TYPE cType, float kernelRadius, GenericProgressCallback* progressCb=0, DgmOctree* _theOctree=0);

	//! Computes the local density
    /** Warning: this method assumes the input scalar field is different from output.
        \param theCloud processed cloud
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param _theOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
    **/
	static int computeLocalDensity(GenericIndexedCloudPersist* theCloud, GenericProgressCallback* progressCb=0, DgmOctree* _theOctree=0);

	//! Computes the local roughness
	/** Roughness is defined as the distance to the locally (least square) fitted plane.
        LS plane is computed with all neighbour points inside a sphere.
        Warning: this method assumes the input scalar field is different from output.
        \param theCloud processed cloud
        \param kernelRadius neighbouring sphere radius
		\param progressCb client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param _theOctree if not set as input, octree will be automatically computed.
		\return success (0) or error code (<0)
    **/
	static int computeRoughness(GenericIndexedCloudPersist* theCloud, float kernelRadius, GenericProgressCallback* progressCb=0, DgmOctree* _theOctree=0);

	//! Computes the gravity center of a point cloud
	/** WARNING: this method uses the cloud global iterator
		\param theCloud cloud
		\return gravity center
	**/
	static CCVector3 computeGravityCenter(GenericCloud* theCloud);

	//! Computes the cross covariance matrix between two clouds (same size)
	/** Used in the ICP algorithm between the cloud to register and the "Closest Points Set"
		determined from the reference cloud.
		WARNING: this method uses the clouds global iterators
		\param P the cloud to register
		\param Q the "Closest Point Set"
		\param pGravityCenter if available, the gravity center of P
		\param qGravityCenter if available, the gravity center of Q
		\return cross covariance matrix
	**/
	static SquareMatrixd computeCrossCovarianceMatrix(GenericCloud* P,
																GenericCloud* Q,
																const PointCoordinateType* pGravityCenter=0,
																const PointCoordinateType* qGravityCenter=0);
	
	//! Computes the cross covariance matrix between two clouds (same size) - weighted version
	/** Used in the ICP algorithm between the cloud to register and the "Closest Points Set"
		determined from the reference cloud.
		WARNING: this method uses the clouds global iterators
		\param P the cloud to register
		\param Q the "Closest Point Set"
		\param pGravityCenter if available, the gravity center of P
		\param qGravityCenter if available, the gravity center of Q
		\param weightsP weights for the points of P (optional)
		\param weightsQ weights for the points of Q (optional)
		\param 
		\return weighted cross covariance matrix
	**/
	static SquareMatrixd computeWeightedCrossCovarianceMatrix(GenericCloud* P,
																		GenericCloud* Q,
																		const PointCoordinateType* pGravityCenter=0,
																		const PointCoordinateType* qGravityCenter=0,
																		ScalarField* weightsP=0,
																		ScalarField* weightsQ=0);

	//! Computes the covariance matrix of a clouds
	/** WARNING: this method uses the cloud global iterator
		\param theCloud point cloud
		\param _gravityCenter if available, its gravity center
		\return covariance matrix
	**/
	static CCLib::SquareMatrixd computeCovarianceMatrix(GenericCloud* theCloud, const PointCoordinateType* _gravityCenter=0);

protected:

	//! Computes cell curvature inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
	**/
	static bool computeCellCurvatureAtLevel(const DgmOctree::octreeCell& cell, void** additionalParameters);

	//! Computes point density inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
	**/
	static bool computePointsDensityInACellAtLevel(const DgmOctree::octreeCell& cell, void** additionalParameters);

	//! Computes point roughness inside a cell
	/**	\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
	**/
	static bool computePointsRoughnessInACellAtLevel(const DgmOctree::octreeCell& cell, void** additionalParameters);
};

}

#endif
