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
	WeibullDistribution(DistanceType _a, DistanceType _b, DistanceType _valueShift=0.0);

	//! Returns the distribution parameters
	/** \param _a a field to transmit the Weibull a paramter
		\param _b a field to transmit the Weibull b paramter
		return the parameters validity
	**/
	bool getParameters(DistanceType &_a, DistanceType &_b) const;

	//! Returns the normal distribution equivalent parameters
	/** \param _mu a field to transmit the equivalent mean
		\param _sigma2 a field to transmit the equivalent variance
		return the parameters validity
	**/
	bool getOtherParameters(DistanceType &_mu, DistanceType &_sigma2) const;

	//! Sets the distribution parameters
	/** \param _a the Weibull a parameter
		\param _b the Weibull b parameter
		\param _valueShift a value shift
		return the parameters validity
	**/
	bool setParameters(DistanceType _a, DistanceType _b, DistanceType _valueShift=0.0);

	//! Sets the distribution value shift
	/** \param vs value shift
	**/
	void setValueShift(DistanceType vs) {if (vs != valueShift) parametersDefined = false; valueShift=vs;}

	//! Returns the distribution value shift
	DistanceType getValueShift() const {return valueShift;};

	//inherited methods (see GenericDistribution)
	virtual bool computeParameters(const GenericCloud* Yk, bool includeNegValues);
	virtual double computeP(DistanceType x) const;
	virtual double computePfromZero(DistanceType x) const;
	virtual double computeP(DistanceType x1, DistanceType x2) const;
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
	DistanceType a;
	//! Weibull distribution parameter b
	DistanceType b;
	//! Weibull distribution parameter 'value shift'
	DistanceType valueShift;

	//! Normal distribution equivalent parameter: mean
	DistanceType mu;
	//! Normal distribution equivalent parameter: variance
	DistanceType sigma2;

	//! internal function for parameters evaluation from sample points
	DistanceType computeG(const GenericCloud* Yk, DistanceType a) const;
	//! internal function for parameters evaluation from sample points (overflow-safe version)
	DistanceType computeG(const GenericCloud* Yk, DistanceType a, DistanceType inverseVmax) const;
	//! internal function for parameters evaluation from sample points
	DistanceType findGRoot(const GenericCloud* Yk, DistanceType inverseMaxValue) const;
};

}

#endif
