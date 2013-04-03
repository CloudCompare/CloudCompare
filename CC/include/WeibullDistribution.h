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

#ifndef WEIBULL_DISTRIBUTION_HEADER
#define WEIBULL_DISTRIBUTION_HEADER

#include "GenericDistribution.h"

namespace CCLib
{

//! The Weibull statistical parametric distribution
/** Implementats the GenericDistribution interface.
**/

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API WeibullDistribution : public GenericDistribution
#else
class WeibullDistribution : public GenericDistribution
#endif
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
	WeibullDistribution(ScalarType _a, ScalarType _b, ScalarType _valueShift=0.0);

	//! Returns the distribution parameters
	/** \param _a a field to transmit the Weibull a paramter
		\param _b a field to transmit the Weibull b paramter
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
	bool setParameters(ScalarType _a, ScalarType _b, ScalarType _valueShift=0.0);

	//! Sets the distribution value shift
	/** \param vs value shift
	**/
	void setValueShift(ScalarType vs) {if (vs != valueShift) parametersDefined = false; valueShift=vs;}

	//! Returns the distribution value shift
	ScalarType getValueShift() const {return valueShift;};

	//inherited methods (see GenericDistribution)
	virtual bool computeParameters(const GenericCloud* Yk, bool includeNegValues);
	virtual double computeP(ScalarType x) const;
	virtual double computePfromZero(ScalarType x) const;
	virtual double computeP(ScalarType x1, ScalarType x2) const;
	virtual double computeChi2Dist(const GenericCloud* Yk, unsigned numberOfClasses, bool includeNegValues, int* histo=0);
	virtual void getTextualDescription(char* buffer) const;
	virtual bool isValid() const {return parametersDefined;};


protected:

	//! Compute each Chi2 class limits
	/** This method is used (internally) to accelerate the Chi2 distance computation.
		\param numberOfClasses the number of classes that will be used for Chi2 distance computation
		\return success
	**/
	virtual bool setChi2ClassesPositions(unsigned numberOfClasses);

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
	ScalarType computeG(const GenericCloud* Yk, ScalarType a) const;
	//! internal function for parameters evaluation from sample points (overflow-safe version)
	ScalarType computeG(const GenericCloud* Yk, ScalarType a, ScalarType inverseVmax) const;
	//! internal function for parameters evaluation from sample points
	ScalarType findGRoot(const GenericCloud* Yk, ScalarType inverseMaxValue) const;
};

}

#endif //WEIBULL_DISTRIBUTION_HEADER
