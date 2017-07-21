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

#ifndef WEIBULL_DISTRIBUTION_HEADER
#define WEIBULL_DISTRIBUTION_HEADER

//Local
#include "GenericDistribution.h"

namespace CCLib
{

//! The Weibull statistical parametric distribution
/** Implementats the GenericDistribution interface.
**/
class CC_CORE_LIB_API WeibullDistribution : public GenericDistribution
{
public:

	//! WeibullDistribution constructor
	WeibullDistribution();

	//! WeibullDistribution constructor
	/** Distrubtion parameters can be directly set during object
		construction.
		\param _a the Weibull a parameter
		\param _b the Weibull b parameter
		\param _valueShift a value shift
	**/
	WeibullDistribution(ScalarType _a, ScalarType _b, ScalarType _valueShift = 0);

	//! Returns the distribution parameters
	/** \param _a a field to transmit the Weibull a parameter
		\param _b a field to transmit the Weibull b parameter
		return the parameters validity
	**/
	bool getParameters(ScalarType &_a, ScalarType &_b) const;

	//! Returns the normal distribution equivalent parameters
	/** \param _mu a field to transmit the equivalent mean
		\param _sigma2 a field to transmit the equivalent variance
		return the parameters validity
	**/
	bool getOtherParameters(ScalarType &_mu, ScalarType &_sigma2) const;

	//! Sets the distribution parameters
	/** \param _a the Weibull a parameter
		\param _b the Weibull b parameter
		\param _valueShift a value shift
		return the parameters validity
	**/
	bool setParameters(ScalarType _a, ScalarType _b, ScalarType _valueShift = 0);

	//! Sets the distribution value shift
	/** \param vs value shift
	**/
	void setValueShift(ScalarType vs);

	//! Returns the distribution value shift
	inline ScalarType getValueShift() const { return valueShift; }

	//inherited methods (see GenericDistribution)
	virtual bool computeParameters(const GenericCloud* Yk);
	virtual double computeP(ScalarType x) const;
	virtual double computePfromZero(ScalarType x) const;
	virtual double computeP(ScalarType x1, ScalarType x2) const;
	virtual double computeChi2Dist(const GenericCloud* cloud, unsigned numberOfClasses, int* histo=0);
	virtual const char* getName() const { return "Weibull"; }

protected:

	//! Compute each Chi2 class limits
	/** This method is used (internally) to accelerate the Chi2 distance computation.
		\param numberOfClasses the number of classes that will be used for Chi2 distance computation
		\return success
	**/
	virtual bool setChi2ClassesPositions(unsigned numberOfClasses);

	//! Chi2 classes limits
	/** Used internally. Stores both limits for each class in a vector
		(min_class_1, max_class_1, min_class_2, max_class_2, etc.).
	**/
	std::vector<ScalarType> chi2ClassesPositions;

	//! Parameters validity
	bool parametersDefined;
	//! Weibull distribution parameter a
	ScalarType a;
	//! Weibull distribution parameter b
	ScalarType b;
	//! Weibull distribution parameter 'value shift'
	ScalarType valueShift;

	//! Normal distribution equivalent parameter: mean
	ScalarType mu;
	//! Normal distribution equivalent parameter: variance
	ScalarType sigma2;

	//! internal function for parameters evaluation from sample points
	/** inverseVmax can be optionally specified for overflow-safe version
	**/
	ScalarType computeG(const GenericCloud* Yk, ScalarType a, ScalarType* inverseVmax = 0) const;
	//! internal function for parameters evaluation from sample points
	ScalarType findGRoot(const GenericCloud* Yk, ScalarType inverseMaxValue) const;
};

}

#endif //WEIBULL_DISTRIBUTION_HEADER
