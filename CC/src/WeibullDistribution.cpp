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

//GAMMA function
static double Gamma_cc(double x)
{
	static const double g[25] =
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
	{
		return std::numeric_limits<double>::max();
	}

	if (x == static_cast<int>(x))
	{
		if (x > 0.0) // use factorial
		{
			double ga = 1.0;
			for (int i = 2; i < x; i++)
				ga *= i;
			return ga;
		}
		else
		{
			return std::numeric_limits<double>::max();
		}
	}

	double z = 0.0;
	double r = 0.0;
	if (std::abs(x) > 1.0)
	{
		z = std::abs(x);
		int m = static_cast<int>(z);
		r = 1.0;
		for (int k = 1; k <= m; k++)
			r *= (z - k);
		z -= m;
	}
	else
	{
		z = x;
	}

	double gr = g[24];
	for (int k = 23; k >= 0; k--)
		gr = gr*z + g[k];
	double ga = 1.0 / (gr*z);
	if (std::abs(x) > 1.0)
	{
		ga *= r;
		if (x < 0.0)
		{
			ga = -M_PI / (x*ga*sin(M_PI*x));
		}
	}
	return ga;
}

WeibullDistribution::WeibullDistribution()
{
	setParameters(0, 0, 0);
}

WeibullDistribution::WeibullDistribution(ScalarType a, ScalarType b, ScalarType valueShift)
{
	setParameters(a, b, valueShift);
}

bool WeibullDistribution::getParameters(ScalarType &a, ScalarType &b) const
{
	a = m_a;
	b = m_b;

	return isValid();
}

bool WeibullDistribution::getOtherParameters(ScalarType &mu, ScalarType &sigma2) const
{
	mu = m_mu;
	sigma2 = m_sigma2;

	return isValid();
}

bool WeibullDistribution::setParameters(ScalarType a, ScalarType b, ScalarType valueShift)
{
	m_valueShift = valueShift;
	m_a = a;
	m_b = b;

	//for the Chi2 test
	chi2ClassesPositions.resize(0);

	if (m_a > 0.0 && m_b >= 0.0)
	{
		//mean and standard deviation
		m_mu = static_cast<ScalarType>(Gamma_cc(1.0 + 1.0 / m_a) * m_b);
		m_sigma2 = static_cast<ScalarType>(Gamma_cc(1.0 + 2.0 / m_a) * (m_b*m_b) - (m_mu * m_mu));

		setValid(true);
	}
	else
	{
		m_mu = m_sigma2 = 0.0;
		setValid(false);
	}

	return isValid();
};

bool WeibullDistribution::computeParameters(const ScalarContainer& values)
{
	setValid(false);

	size_t n = values.size();
	if (n == 0)
		return false;

	//we look for the maximum value of the SF so as to avoid overflow
	ScalarType minValue;
	ScalarType maxValue;
	bool firstValue = true;
	for (ScalarType s : values)
	{
		if (!ScalarField::ValidValue(s))
			continue;

		if (firstValue)
		{
			minValue = maxValue = s;
			firstValue = false;
		}
		else
		{
			if (s < minValue)
				minValue = s;
			else if (s > maxValue)
				maxValue = s;
		}
	}

	if (firstValue)
	{
		//sf is only composed of NAN values?!
		return false;
	}

	double valueRange = maxValue - minValue;
	if (valueRange < std::numeric_limits<ScalarType>::epsilon())
	{
		return false;
	}

	double a = FindGRoot(values, minValue, valueRange);
	if (a < 0.0)
		return false;

	//we can compute b
	double b = 0;
	unsigned counter = 0;
	for (size_t i = 0; i < n; ++i)
	{
		ScalarType v = values[i];
		if (ScalarField::ValidValue(v)) //we ignore NaN values
		{
			if (v >= minValue)
			{
				b += pow((static_cast<double>(v) - minValue) / valueRange, a);
				++counter;
			}
		}
	}
	if (counter == 0)
		return false;

	return setParameters(	static_cast<ScalarType>(a),
							static_cast<ScalarType>(valueRange * pow(b / counter, 1.0 / a)),
							minValue );
}

double WeibullDistribution::computeP(ScalarType _x) const
{
	double x = static_cast<double>(_x - m_valueShift) / m_b;
	if (x < 0)
		return 0;

	double xp = pow(x, m_a - 1.0);
	return (static_cast<double>(m_a) / m_b) * xp * exp(-xp*x);
}

double WeibullDistribution::computePfromZero(ScalarType x) const
{
	return (x <= m_valueShift ? 0.0 : 1.0 - exp(-pow(static_cast<double>(x - m_valueShift) / m_b, static_cast<double>(m_a))));
}

double WeibullDistribution::computeP(ScalarType x1, ScalarType x2) const
{
	if (x1 < m_valueShift)
		x1 = m_valueShift;
	if (x2 < m_valueShift)
		return 0;
	//pi = computeP(minV+(ScalarType(k)+0.5)*step)*step;
	//...instead we take the sampling into account and then integrate
	return exp(-pow(static_cast<double>(x1 - m_valueShift) / m_b, static_cast<double>(m_a))) - exp(-pow(static_cast<double>(x2 - m_valueShift) / m_b, static_cast<double>(m_a)));
}

double WeibullDistribution::ComputeG(const ScalarContainer& values, double r, ScalarType valueShift, double valueRange)
{
	size_t n = values.size();

	//a & n should be strictly positive!
	if (r <= 0.0 || n == 0)
		return 1.0; //a positive value means that ComputeG failed

	double p = 0;
	double q = 0;
	double s = 0;
	unsigned counter = 0;
	unsigned zeroValues = 0;

	for (unsigned i = 0; i < n; ++i)
	{
		ScalarType v = values[i];
		if (ScalarField::ValidValue(v)) //we ignore NaN values
		{
			double v0 = static_cast<double>(v) - valueShift;
			if (v0 > ZERO_TOLERANCE)
			{
				double ln_v = log(v0);
				double v_a = pow(v0 / valueRange, r);

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
		double v_a = pow(ZERO_TOLERANCE / valueRange, static_cast<double>(r));
		s += ln_v;
		q += v_a * zeroValues;
		p += ln_v * v_a;
		counter += zeroValues;
	}

	if (counter == 0)
	{
		return 1.0; //a positive value will make ComputeG fail
	}

	return (p / q - s / counter) * r - 1.0;
}

double WeibullDistribution::FindGRoot(const ScalarContainer& values, ScalarType valueShift, double valueRange)
{
	double r = -1.0;
	double aMin = 1.0;
	double aMax = 1.0;
	double v = ComputeG(values, aMin, valueShift, valueRange);
	double vMin = v;
	double vMax = v;

	//find min value for binary search so that ComputeG(aMin) < 0
	while (vMin > 0 && aMin > ZERO_TOLERANCE)
	{
		aMin /= 10;
		vMin = ComputeG(values, aMin, valueShift, valueRange);
	}

	if (std::abs(vMin) < ZERO_TOLERANCE)
		return aMin;
	else if (vMin > 0)
		return r; //r = -1 (i.e. problem)

	//find max value for binary search so that ComputeG(aMax) > 0
	while (vMax < 0 && aMax < 1.0e3)
	{
		aMax *= 2; //tends to become huge quickly as we compute x^a!!!!
		vMax = ComputeG(values, aMax, valueShift, valueRange);
	}

	if (std::abs(vMax) < ZERO_TOLERANCE)
		return aMax;
	else if (vMax < 0)
		return r; //r = -1 (i.e. problem)

	//binary search to find r so that std::abs(ComputeG(r)) < ZERO_TOLERANCE
	while (std::abs(v) * 100 > ZERO_TOLERANCE) //DGM: *100 ?! (can't remember why ;)
	{
		r = (aMin + aMax) / 2;
		double old_v = v;
		v = ComputeG(values, r, valueShift, valueRange);

		if (std::abs(old_v - v) < ZERO_TOLERANCE)
			return r;

		if (v < 0)
			aMin = r;
		else
			aMax = r;
	}

	return r;
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

	assert(chi2ClassesPositions.size() + 1 == numberOfClasses);

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
		double nPi = static_cast<double>(numberOfElements) / numberOfClasses;
		for (unsigned i = 0; i < numberOfClasses; ++i)
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
	chi2ClassesPositions.resize(0);

	if (!isValid() || numberOfClasses < 2)
		return false;

	try
	{
		chi2ClassesPositions.resize(numberOfClasses - 1);
	}
	catch (const std::bad_alloc&)
	{
		//not engouh memory
		return false;
	}

	//we create "numberOfClasses" equiprobable classes (for all of themn nPi>=sqrt(n) if n>=4)
	double areaPerClass = 1.0 / numberOfClasses;
	double currentArea = areaPerClass;
	double invA = 1.0 / m_a;

	for (unsigned i = 1; i < numberOfClasses; ++i)
	{
		chi2ClassesPositions[i - 1] = m_b * static_cast<ScalarType>(pow(-log(1.0 - currentArea), invA));
		currentArea += areaPerClass;
	}

	return true;
}

void WeibullDistribution::setValueShift(ScalarType vs)
{
	if (vs != m_valueShift)
		setValid(false);

	m_valueShift = vs;
}

double WeibullDistribution::computeMode() const
{
	double mode = m_valueShift;
	if (m_a > 1.0)
	{
		mode += m_b * pow((m_a - 1.0) / m_a, 1.0 / m_a);
	}
	return mode;
}

double WeibullDistribution::computeSkewness() const
{
	if (	!isValid()
		||	std::abs(m_a) < std::numeric_limits<double>::epsilon()
		||	m_sigma2 < std::numeric_limits<double>::epsilon()
		)
	{
		return std::numeric_limits<double>::quiet_NaN();
	}
	return (Gamma_cc(1.0 + 3.0 / m_a) * (m_b * m_b * m_b) - 3.0 * m_mu * m_sigma2 - (m_mu * m_mu * m_mu)) / (m_sigma2 * sqrt(m_sigma2));
}
