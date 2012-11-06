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

#ifndef NORMAL_DISTRIBUTION_HEADER
#define NORMAL_DISTRIBUTION_HEADER

#include "GenericDistribution.h"

namespace CCLib
{

//! Scalar values container
typedef std::vector<DistanceType> distancesContainer;

//! The Normal/Gaussian statistical distribution
/** Implements the GenericDistribution interface.
**/

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API NormalDistribution : public GenericDistribution
#else
class NormalDistribution : public GenericDistribution
#endif
{
public:

	//! NormalDistribution constructor
	NormalDistribution();

	//! NormalDistribution constructor
	/** Distrubtion parameters can be directly set during object
		construction.
		\param _mu the normal distribution mean
		\param _sigma2 the normal distribution variance
	**/
	NormalDistribution(DistanceType _mu, DistanceType _sigma2);

	//inherited methods (see GenericDistribution)
	virtual bool computeParameters(const GenericCloud* Yk, bool includeNegValues);
	virtual double computeP(DistanceType x) const;
	virtual double computePfromZero(DistanceType x) const;
	virtual double computeP(DistanceType x1, DistanceType x2) const;
	virtual double computeChi2Dist(const GenericCloud* Yk, unsigned numberOfClasses, bool includeNegValues, int* histo=0);
	virtual void getTextualDescription(char* buffer) const;
	virtual bool isValid() const {return parametersDefined;};

	//! Returns the distribution parameters
	/** \param _mu a field to transmit the distribution mean
		\param _sigma2 a field to transmit the distribution variance
		return the parameters validity
	**/
	bool getParameters(DistanceType &_mu, DistanceType &_sigma2) const;

	//! Sets the distribution parameters
	/** \param _mu the distribution mean
		\param _sigma2 the distribution variance
		return the parameters validity
	**/
	bool setParameters(DistanceType _mu, DistanceType _sigma2);

	//! Returns the distribution mean
	inline DistanceType getMu() const {return mu;};

	//! Returns the distribution variance
	inline DistanceType getSigma2() const {return sigma2;};

	//! Computes the distribution parameters from an array of scalar values
	/** Specific method to compute the parameters directly from an array
		(vector) of scalar values, without associated points.
		\param values the scalar values
		\param includeNegValues specifies whether negative values should be included in computation
		\return the validity of the computed parameters
	**/
	bool computeParameters(const distancesContainer& values, bool includeNegValues);

	//! Computes robust parameters for the distribution from an array of scalar values
	/** Specific method to compute the parameters directly from an array
		(vector) of scalar values, without associated points. After a first pass,
		only the values close enough to the mean (in terms of nSigma times the initial
		variance) are kept to make a second and more robust evaluation of the parameters.
		\param values the scalar values
		\param nSigma the values filtering interval size ([mu -nSigma * stddev : mu + nSigma * stddev])
		\param includeNegValues specifies whether negative values should be included in computation
		\return the validity of the computed parameters
	**/
	bool computeRobustParameters(const distancesContainer& values, double nSigma, bool includeNegValues);

protected:

	//! Compute each Chi2 class limits
	/** This method is used (internally) to accelerate the Chi2 distance computation.
		\param numberOfClasses the number of classes that will be used for Chi2 distance computation
		\return success
	**/
	virtual bool setChi2ClassesPositions(unsigned numberOfClasses);

	//! Parameters validity
	bool parametersDefined;

	//! Mean
	DistanceType mu;
	//! Variance
	DistanceType sigma2;
	//! exponential quotient
	DistanceType qFactor;
	//! Normalization factor
	double normFactor;

	//! Structure used during the Chi2 distance computation
	std::vector<DistanceType> Pi;
};

}

#endif
