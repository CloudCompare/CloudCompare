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

#ifdef _MSC_VER
//To get rid of the really annoying warnings about unsafe methods
#pragma warning( disable: 4996 )
#endif

#include <WeibullDistribution.h>

//local
#include <GenericCloud.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>

using namespace CCLib;

//FONCION GAMMA
static double gamma_cc(double x)
{
	static double g[] =
	{
		1.0,
		0.5772156649015329,
		-0.6558780715202538,
		-0.420026350340952e-1,
		0.1665386113822915,
		-0.421977345555443e-1,
		-0.9621971527877e-2,
		0.7218943246663e-2,
		-0.11651675918591e-2,
		-0.2152416741149e-3,
		0.1280502823882e-3,
		-0.201348547807e-4,
		-0.12504934821e-5,
		0.1133027232e-5,
		-0.2056338417e-6,
		0.6116095e-8,
		0.50020075e-8,
		-0.11812746e-8,
		0.1043427e-9,
		0.77823e-11,
		-0.36968e-11,
		0.51e-12,
		-0.206e-13,
		-0.54e-14,
		0.14e-14
	};

	if (x > 171.0)
		return 1e308;    // This value is an overflow flag.

	if (x == static_cast<int>(x))
	{
		if (x > 0.0) // use factorial
		{
			double ga = 1.0;
			for (int i=2; i<x; i++)
				ga *= i;
			return ga;
		}
		else
		{
			return 1e308;
		}
	}

	double z = 0.0, r = 0.0;
	if (fabs(x) > 1.0)
	{
		z = fabs(x);
		int m = static_cast<int>(z);
		r = 1.0;
		for (int k=1; k<=m; k++)
			r *= (z-k);
		z -= m;
	}
	else
	{
		z = x;
	}

	double gr = g[24];
	for (int k=23; k>=0; k--)
		gr = gr*z+g[k];
	double ga = 1.0/(gr*z);
	if (fabs(x) > 1.0)
	{
		ga *= r;
		if (x < 0.0)
		{
			ga = -M_PI/(x*ga*sin(M_PI*x));
		}
	}
	return ga;
}

WeibullDistribution::WeibullDistribution()
{
	setParameters(0.0,0.0,0.0);
}

WeibullDistribution::WeibullDistribution(ScalarType _a, ScalarType _b, ScalarType _valueShift)
{
	setParameters(_a,_b,_valueShift);
}

bool WeibullDistribution::getParameters(ScalarType &_a, ScalarType &_b) const
{
	_a = a;
	_b = b;

	return isValid();
}

bool WeibullDistribution::getOtherParameters(ScalarType &_mu, ScalarType &_sigma2) const
{
	_mu = mu;
	_sigma2 = sigma2;

	return isValid();
}

bool WeibullDistribution::setParameters(ScalarType _a, ScalarType _b, ScalarType _valueShift)
{
	valueShift = _valueShift;
	a = _a;
	b = _b;

	//pour le test du Chi2
	chi2ClassesPositions.clear();

	if (a > 0.0 && b >= 0.0)
	{
		//moyenne et ecart type
		mu = static_cast<ScalarType>(gamma_cc(1.0+1.0/a) * b);
		sigma2 = static_cast<ScalarType>(gamma_cc(1.0+2.0/a) * (b*b) - (mu*mu));

		setValid(true);
	}
	else
	{
		mu = sigma2 = 0.0;
		setValid(false);
	}

	return isValid();
};

bool WeibullDistribution::computeParameters(const GenericCloud* cloud)
{
	setValid(false);

	int n = cloud->size();
	if (n == 0)
		return false;

	//we look for the maximum value of the SF so as to avoid overflow
	ScalarType maxValue = 0;
	ScalarFieldTools::computeScalarFieldExtremas(cloud, valueShift, maxValue);

	if (!ScalarField::ValidValue(valueShift))
	{
		//sf is only composed of NAN values?!
		return false;
	}

	valueShift -= static_cast<ScalarType>(ZERO_TOLERANCE);

	if (maxValue <= valueShift)
		return false;

	ScalarType inverseMaxValue = static_cast<ScalarType>(1.0)/(maxValue-valueShift);

	a = findGRoot(cloud,inverseMaxValue);

	if (a < 0)
		return false;

	//we can compute b
	b = 0;
	unsigned counter = 0;
	for (int i=0; i<n; ++i)
	{
		ScalarType v = cloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(v)) //we ignore NaN values
		{
			v -= valueShift;
			if (v >= 0)
			{
				b += pow(v*inverseMaxValue,a);
				++counter;
			}
		}
	}

	if (counter == 0)
		return false;

	b = (maxValue-valueShift) * pow(b/counter,static_cast<ScalarType>(1.0)/a);

	return setParameters(a,b,valueShift);
}

double WeibullDistribution::computeP(ScalarType _x) const
{
	double x = static_cast<double>(_x-valueShift)/b;
	if (x < 0)
		return 0;

	double xp = pow(x,a-1.0);
	return (static_cast<double>(a)/b) * xp * exp(-xp*x);
}

double WeibullDistribution::computePfromZero(ScalarType x) const
{
	return (x <= valueShift ? 0.0 : 1.0-exp(-pow(static_cast<double>(x-valueShift)/b,static_cast<double>(a))));
}

double WeibullDistribution::computeP(ScalarType x1, ScalarType x2) const
{
	if (x1 < valueShift)
		x1 = valueShift;
	if (x2 < valueShift)
		return 0;
	//pi = computeP(minV+(ScalarType(k)+0.5)*step)*step;
	//...instead we take the sampling into account and then integrate
	return exp(-pow(static_cast<double>(x1-valueShift)/b,static_cast<double>(a))) - exp(-pow(static_cast<double>(x2-valueShift)/b,static_cast<double>(a)));
}

ScalarType WeibullDistribution::computeG(const GenericCloud* cloud, ScalarType r, ScalarType* inverseVmax/*=0*/) const
{
	unsigned n = cloud->size();

	//a & n should be strictly positive!
	if (r <= 0 || n == 0)
		return static_cast<ScalarType>(1.0); //a positive value means that computeG failed

	double p = 0, q = 0, s = 0;
	unsigned counter = 0, zeroValues = 0;

	for (unsigned i = 0; i < n; ++i)
	{
		ScalarType v = cloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(v)) //we ignore NaN values
		{
			v -= valueShift;
			if (v > ZERO_TOLERANCE)
			{
				double ln_v = log(v);
				if (inverseVmax)
					v *= (*inverseVmax);
				double v_a = pow(v,r);

				s += ln_v;
				q += v_a;
				p += v_a*ln_v;

				++counter;
			}
			else
			{
				++zeroValues;
			}
		}
	}

	if (zeroValues)
	{
		double ln_v = log(ZERO_TOLERANCE) * zeroValues;
		double epsilon = ZERO_TOLERANCE;
		if (inverseVmax)
			epsilon *= (*inverseVmax);
		double v_a = pow(epsilon,static_cast<double>(r));
		s += ln_v;
		q += v_a * zeroValues;
		p += ln_v * v_a;
		counter += zeroValues;
	}

	if (counter == 0)
		return static_cast<ScalarType>(1.0); //a positive value will make computeG fail

	return static_cast<ScalarType>((p/q - s/counter)*r - 1.0);
}

ScalarType WeibullDistribution::findGRoot(const GenericCloud* cloud, ScalarType inverseMaxValue) const
{
	ScalarType r = -static_cast<ScalarType>(1.0);
	ScalarType aMin,aMax;
	aMin = aMax = 1.0;
	ScalarType v,vMin,vMax;
	vMin = vMax = v = computeG(cloud,aMin,&inverseMaxValue);

	//find min value for binary search so that computeG(aMin) < 0
	while (vMin > 0 && aMin > ZERO_TOLERANCE)
	{
		aMin /= 10;
		vMin = computeG(cloud,aMin,&inverseMaxValue);
	}

	if (fabs(vMin) < ZERO_TOLERANCE)
		return aMin;
	else if (vMin > 0)
		return r; //r = -1 (i.e. problem)

	//find max value for binary search so that computeG(aMax) > 0
	while (vMax < 0 && aMax < 1.0e3)
	{
		aMax *= 2; //tends to become huge quickly as we compute x^a!!!!
		vMax = computeG(cloud,aMax,&inverseMaxValue);
	}

	if (fabs(vMax) < ZERO_TOLERANCE)
		return aMax;
	else if (vMax < 0)
		return r; //r = -1 (i.e. problem)

	//binary search to find r so that fabs(computeG(r)) < ZERO_TOLERANCE
	while (fabs(v)*100 > ZERO_TOLERANCE) //DGM: *100 ?! (can't remember why ;)
	{
		r = (aMin+aMax)/2;
		ScalarType old_v = v;
		v = computeG(cloud,r,&inverseMaxValue);

		if (fabs(old_v-v) < ZERO_TOLERANCE)
			return r;

		if (v < 0)
			aMin = r;
		else
			aMax = r;
	}

	assert(false);
	return r; //shouldn't happen!
}

double WeibullDistribution::computeChi2Dist(const GenericCloud* cloud, unsigned numberOfClasses, int* inputHisto)
{
	assert(cloud);

	//we must refine the real number of elements
	unsigned numberOfElements = ScalarFieldTools::countScalarFieldValidValues(cloud);

	if (numberOfElements == 0)
		return -1.0;

	if (numberOfClasses < 1 || numberOfClasses*numberOfClasses > numberOfElements)
		return -1.0;
	else if (numberOfClasses == 1)
		return 0.0;

	if (!setChi2ClassesPositions(numberOfClasses))
		return -1.0;

	assert(chi2ClassesPositions.size()+1 == numberOfClasses);

	int* histo = inputHisto;
	if (!histo)
	{
		histo = new int[numberOfClasses];
		if (!histo)
			return -1.0; //not enough memory
	}
	memset(histo, 0, numberOfClasses * sizeof(int));

	//compute the histogram
	unsigned n = cloud->size();
	for (unsigned i = 0; i < n; ++i)
	{
		ScalarType V = cloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
			unsigned j = 0;
			for (; j < numberOfClasses - 1; ++j)
				if (V < chi2ClassesPositions[j])
					break;

			++histo[j];
		}
	}

	//Chi2 distance
	double dk = 0;
	{
		double nPi = static_cast<double>(numberOfElements)/numberOfClasses;
		for (unsigned i=0; i<numberOfClasses; ++i)
		{
			double tempValue = static_cast<double>(histo[i]) - nPi;
			dk += tempValue*tempValue;
		}
		dk /= nPi;
	}

	if (histo && !inputHisto)
		delete[] histo;
	histo = nullptr;

	return dk;
}

bool WeibullDistribution::setChi2ClassesPositions(unsigned numberOfClasses)
{
	chi2ClassesPositions.clear();

	if (!isValid() || numberOfClasses < 2)
		return false;

	try
	{
		chi2ClassesPositions.resize(numberOfClasses-1);
	}
	catch (const std::bad_alloc&)
	{
		//not engouh memory
		return false;
	}

	//we create "numberOfClasses" equiprobable classes (for all of themn nPi>=sqrt(n) if n>=4)
	double areaPerClass = 1.0/numberOfClasses;
	double currentArea = areaPerClass;
	double invA = 1.0/a;

	for (unsigned i=1; i<numberOfClasses; ++i)
	{
		chi2ClassesPositions[i-1] = b * static_cast<ScalarType>(pow(-log(1.0-currentArea),invA));
		currentArea += areaPerClass;
	}

	return true;
}

void WeibullDistribution::setValueShift(ScalarType vs)
{
	if (vs != valueShift)
		setValid(false);
	
	valueShift = vs;
}
