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

#ifndef SCALAR_FIELD_TOOLS_HEADER
#define SCALAR_FIELD_TOOLS_HEADER

//Local
#include "CCToolbox.h"
#include "DgmOctree.h"


namespace CCLib
{

class GenericCloud;
class GenericIndexedCloud;
class GenericIndexedCloudPersist;
class GenericProgressCallback;

//! A K-mean class position and boundaries
struct KMeanClass
{
	//! K-mean class mean value
	ScalarType mean;
	//! K-mean class minimum value
	ScalarType minValue;
	//! K-mean class maximum value
	ScalarType maxValue;
};

//! Several scalar field treatment algorithms (gradient, classification, etc.)
/** This toolbox provides several algorithms to apply
	treatments and handle scalar fields
**/
class CC_CORE_LIB_API ScalarFieldTools : public CCToolbox
{
public:

	//! Computes the mean value of a scalar field associated to a cloud
	/** Returns the mean of SF values associated to each point of a cloud
		Warning: be sure to activate an OUTPUT scalar field on the cloud
		\param theCloud the point cloud
		\return the associated scalar field mean value
	**/
	static ScalarType computeMeanScalarValue(GenericCloud* theCloud);

	//! Computes the mean square value of a scalar field associated to a cloud
	/** Returns the mean of squared SF values associated to each point of a cloud
		Warning: be sure to activate an OUTPUT scalar field on the cloud
		\param theCloud the point cloud
		\return the associated scalar field mean of squares value
	**/
	static ScalarType computeMeanSquareScalarValue(GenericCloud* theCloud);

	//! Computes the geometrical gradient of a scalar field associated to a point cloud
	/** See Daniel Girardeau-Montaut's PhD manuscript (Chapter 3, section 3.3.2) for more
		information. As explained in this section, if the scalar field corresponds to
		(euclidean) distances between the points and another entity, then it is possible
		to filter out aberrant values.
		\param theCloud a point cloud (associated to scalar values)
		\param radius spherical neighborhood size (or 0 for automatic size)
		\param euclideanDistances indicates if the scalar values are euclidean distances
		\param sameInAndOutScalarField specifies that the 'in' and 'out' scalar field of the input point cloud are the same structure
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param theOctree the octree, if it has already been computed
		\return error code (0 if ok)
	**/
	static int computeScalarFieldGradient(	GenericIndexedCloudPersist* theCloud, 
											PointCoordinateType radius,
											bool euclideanDistances,
											bool sameInAndOutScalarField = false,
											GenericProgressCallback* progressCb = nullptr, 
											DgmOctree* theOctree = nullptr);

	//! Computes a spatial gaussian filter on a scalar field associated to a point cloud
	/** The "amplitutde" of the gaussian filter must be precised (sigma).
		As 99% of the gaussian distribution is between -3*sigma and +3*sigma
		around the mean value, this filter will only look for neighbouring
		points (around each point) in a sphere of radius 3*sigma.
		It also permits to use the filter as a bilateral filter. Where the wights are computed also considering the
		distance of the neighbor's scalar value from the current point scalar value. (weighted with gaussian as distances are)
		Warning: this method assumes the input scalar field is different from output.
		\param sigma filter variance
		\param theCloud a point cloud (associated to scalar values)
		\param sigmaSF the sigma for the bilateral filter. when different than -1 turns the gaussian filter into a bilateral filter
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
		\param theOctree the octree, if it has already been computed
		\return success
	**/
	static bool applyScalarFieldGaussianFilter(	PointCoordinateType sigma, 
												GenericIndexedCloudPersist* theCloud, 
												PointCoordinateType sigmaSF,
												GenericProgressCallback* progressCb = nullptr, 
												DgmOctree* theOctree = nullptr);

	//! Multiplies two scalar fields of the same size
	/** The first scalar field is updated (S1 = S1*S2).
		\param firstCloud the first point cloud (associated to scalar values)
		\param secondCloud the second point cloud (associated to scalar values)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	**/
	static void multiplyScalarFields(	GenericIndexedCloud* firstCloud, 
										GenericIndexedCloud* secondCloud, 
										GenericProgressCallback* progressCb = nullptr);

	//! Computes an histogram of the scalar field with a given number of classes
	/** The scalar values are projected in a given number of classes,
		regularly spaced between the lowest and the highest value of
		the scalar field.
		\param theCloud a point cloud (associated to scalar values)
		\param numberOfClasses number of histogram classes
		\param histo number of elements per histogram class
	**/
	static void computeScalarFieldHistogram(const GenericCloud* theCloud, 
											unsigned numberOfClasses, 
											std::vector<int>& histo);

	//! Compute the extreme values of a scalar field
	/** \param theCloud a point cloud, with a scalar field activated
		\param minV a field to store the minimum value
		\param maxV a field to store the maximum value
	**/
	static void computeScalarFieldExtremas(	const GenericCloud* theCloud, 
											ScalarType& minV, 
											ScalarType& maxV);

	//! Count the number of valid values in a scalar field
	/** \param theCloud a point cloud, with a scalar field activated
	**/
	static unsigned countScalarFieldValidValues(const GenericCloud* theCloud);

	//! Classifies automaticaly a scalar field in K classes with the K-means algorithm
	/** The initial K classes positions are regularly spaced between the
		lowest and the highest values of the scalar field. Eventually the
		algorithm will converge and produce K classes.
		\param theCloud a point cloud (associated to scalar values)
		\param K the number of classes
		\param kmcc an array of size K which will be filled with the computed classes limits (see ScalarFieldTools::KmeanClass)
		\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	**/
	static bool computeKmeans(	const GenericCloud* theCloud, 
								unsigned char K, 
								KMeanClass kmcc[], 
								GenericProgressCallback* progressCb = nullptr);

	//! Sets the distance value associated to a point
	/** Generic function that can be used with the GenericCloud::foreach() method.
		\param P a 3D point
		\param scalarValue its associated scalar value
	**/
	static void SetScalarValueToNaN(const CCVector3& P, ScalarType& scalarValue);

	//! Sets the distance value associated to a point to zero
	/** Generic function that can be used with the GenericCloud::foreach() method.
		\param P a 3D point
		\param scalarValue its associated scalar value
	**/
	static void SetScalarValueToZero(const CCVector3 &P, ScalarType& scalarValue);

	static void SetScalarValueInverted(const CCVector3 &P, ScalarType& scalarValue);
protected:

	//! "Cellular" function to compute the gradient norms of points inside an octree cell
	/** This function is meant to be applied to all cells of the octree
		(it is of the form DgmOctree::localFunctionPtr).
		See ScalarFieldTools::computeScalarFieldGradient.
		Method parameters (defined in "additionalParameters") are :
		- (GenericCloud*) the point cloud
		- (bool*) specifies if the scalar values are euclidean distances
		- (PointCoordinateType*) radius
		- (std::vector<ScalarType>*) the gradient norms container
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool computeMeanGradientOnPatch(	const DgmOctree::octreeCell& cell,
											void** additionalParameters,
											NormalizedProgress* nProgress = nullptr);

	//! "Cellular" function to apply a gaussian filter on the scalar values of points inside an octree cell
	/** This function is meant to be applied to all cells of the octree
		The method also permits to use a bilateral behaviour for the filter. This is automatically switched on
		if its sigmaSF parameter in additionalParameters is different than -1
		(it is of the form DgmOctree::localFunctionPtr).
		See ScalarFieldTools::applyScalarFieldGaussianFilter.
		Method parameters (defined in "additionalParameters") are :
		- (PointCoordinateType*) sigma
		- (std::vector<ScalarType>*) the smoothed values
		\param cell structure describing the cell on which processing is applied
		\param additionalParameters see method description
		\param nProgress optional (normalized) progress notification (per-point)
	**/
	static bool computeCellGaussianFilter(	const DgmOctree::octreeCell& cell,
											void** additionalParameters,
											NormalizedProgress* nProgress = nullptr);

};

}

#endif //SCALAR_FIELD_TOOLS_HEADER
