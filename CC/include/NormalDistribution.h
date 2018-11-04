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

#ifndef NORMAL_DISTRIBUTION_HEADER
#define NORMAL_DISTRIBUTION_HEADER

//Local
#include "GenericDistribution.h"

namespace CCLib
{

//! The Normal/Gaussian statistical distribution
/** Implements the GenericDistribution interface.
**/
class CC_CORE_LIB_API NormalDistribution : public GenericDistribution
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
	NormalDistribution(ScalarType _mu, ScalarType _sigma2);

	//inherited methods (see GenericDistribution)
	bool computeParameters(const ScalarContainer& values) override;
	double computeP(ScalarType x) const override;
	double computePfromZero(ScalarType x) const override;
	double computeP(ScalarType x1, ScalarType x2) const override;
	double computeChi2Dist(const GenericCloud* Yk, unsigned numberOfClasses, int* histo = nullptr) override;
	const char* getName() const override { return "Gauss"; }

	//! Returns the distribution parameters
	/** \param _mu a field to transmit the distribution mean
		\param _sigma2 a field to transmit the distribution variance
		return the parameters validity
	**/
	bool getParameters(ScalarType &_mu, ScalarType &_sigma2) const;

	//! Sets the distribution parameters
	/** \param _mu the distribution mean
		\param _sigma2 the distribution variance
		return the parameters validity
	**/
	bool setParameters(ScalarType _mu, ScalarType _sigma2);

	//! Returns the distribution mean
	inline ScalarType getMu() const { return m_mu; }

	//! Returns the distribution variance
	inline ScalarType getSigma2() const { return m_sigma2; }

	//! Computes the distribution parameters from a point cloud (with scalar values)
	bool computeParameters(const GenericCloud* cloud);

	//! Computes robust parameters for the distribution from an array of scalar values
	/** Specific method to compute the parameters directly from an array
		(vector) of scalar values, without associated points. After a first pass,
		only the values close enough to the mean (in terms of nSigma times the initial
		variance) are kept to make a second and more robust evaluation of the parameters.
		\param values the scalar values
		\param nSigma the values filtering interval size ([mu -nSigma * stddev : mu + nSigma * stddev])
		\return the validity of the computed parameters
	**/
	bool computeRobustParameters(const ScalarContainer& values, double nSigma);

protected:

	//! Compute each Chi2 class limits
	/** This method is used (internally) to accelerate the Chi2 distance computation.
		\param numberOfClasses the number of classes that will be used for Chi2 distance computation
		\return success
	**/
	virtual bool setChi2ClassesPositions(unsigned numberOfClasses);

	//! Mean
	ScalarType m_mu;
	//! Variance
	ScalarType m_sigma2;
	//! Exponential quotient
	double m_qFactor;
	//! Normalization factor
	double m_normFactor;

	//! Chi2 classes limits
	/** Used internally. Stores both limits for each class in a vector
		(min_class_1, max_class_1, min_class_2, max_class_2, etc.).
	**/
	std::vector<ScalarType> m_chi2ClassesPositions;

	//! Structure used during the Chi2 distance computation
	std::vector<ScalarType> m_Pi;
};

}

#endif //NORMAL_DISTRIBUTION_HEADER
