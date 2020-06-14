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

#include <NormalDistribution.h>

//local
#include <DistanceComputationTools.h>
#include <ErrorFunction.h>
#include <GenericCloud.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>


using namespace CCLib;

NormalDistribution::NormalDistribution()
	: GenericDistribution()
	, m_mu(0)
	, m_sigma2(0)
	, m_qFactor(0)
	, m_normFactor(0)
{
}

NormalDistribution::NormalDistribution(ScalarType mu, ScalarType sigma2)
{
	setParameters(mu, sigma2);
}

bool NormalDistribution::getParameters(ScalarType &mu, ScalarType &sigma2) const
{
	mu = m_mu;
	sigma2 = m_sigma2;

	return isValid();
}

bool NormalDistribution::setParameters(ScalarType mu, ScalarType sigma2)
{
	m_mu = mu;
	m_sigma2 = sigma2;

	//update Chi2 data
	m_chi2ClassesPositions.resize(0);
	m_Pi.resize(0);

	if (m_sigma2 >= 0)
	{
		setValid(true);
		m_qFactor = 1.0 / (2.0 * m_sigma2);
		m_normFactor = 1.0 / sqrt(2.0 * M_PI * m_sigma2);
	}
	else
	{
		setValid(false);
		m_qFactor = 1.0;
		m_normFactor = 1.0;
	}

	return isValid();
}

double NormalDistribution::computeP(ScalarType x) const
{
	double p = static_cast<double>(x - m_mu);
	return exp(-p*p*m_qFactor)*m_normFactor;
}

double NormalDistribution::computeP(ScalarType x1, ScalarType x2) const
{
	return 0.5 * (ErrorFunction::erf(static_cast<double>(x2 - m_mu) / sqrt(2 * m_sigma2))
				- ErrorFunction::erf(static_cast<double>(x1 - m_mu) / sqrt(2 * m_sigma2)));
}

double NormalDistribution::computePfromZero(ScalarType x) const
{
	return 0.5 * (ErrorFunction::erf(static_cast<double>(x - m_mu) / sqrt(2 * m_sigma2)) + 1.0);
}

bool NormalDistribution::computeParameters(const GenericCloud* cloud)
{
	setValid(false);

	double mean = 0.0;
	double stddev2 = 0.0;
	unsigned counter = 0;

	unsigned n = cloud->size();
	for (unsigned i = 0; i < n; ++i)
	{
		ScalarType v = cloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(v))
		{
			mean += v;
			stddev2 += static_cast<double>(v) * v;
			++counter;
		}
	}

	if (counter == 0)
	{
		return false;
	}

	mean /= counter;
	stddev2 = std::abs(stddev2 / counter - mean*mean);

	return setParameters(static_cast<ScalarType>(mean), static_cast<ScalarType>(stddev2));
}

bool NormalDistribution::computeParameters(const ScalarContainer& values)
{
	setValid(false);

	//compute mean and std. dev.
	double mean = 0.0;
	double stddev2 = 0.0;
	unsigned counter = 0;

	for (ScalarType v : values)
	{
		if (ScalarField::ValidValue(v))
		{
			mean += v;
			stddev2 += static_cast<double>(v) * v;
			++counter;
		}
	}

	if (counter == 0)
	{
		return false;
	}

	mean /= counter;
	stddev2 = std::abs(stddev2 / counter - mean*mean);

	return setParameters(static_cast<ScalarType>(mean), static_cast<ScalarType>(stddev2));
}

bool NormalDistribution::computeRobustParameters(const ScalarContainer& values, double nSigma)
{
	if (!computeParameters(values))
		return false;

	//max std. deviation
	const double maxStddev = sqrt(static_cast<double>(m_sigma2))*nSigma;

	unsigned counter = 0;
	double mean = 0.0;
	double stddev2 = 0.0;

	for (ScalarType v : values)
	{
		if (static_cast<double>(std::abs(v - m_mu)) < maxStddev)
		{
			mean += v;
			stddev2 += static_cast<double>(v) * v;
			++counter;
		}
	}

	if (counter == 0)
	{
		return false;
	}

	mean /= counter;
	stddev2 = std::abs(stddev2 / counter - mean*mean);

	return setParameters(static_cast<ScalarType>(mean), static_cast<ScalarType>(stddev2));
}

double NormalDistribution::computeChi2Dist(const GenericCloud* cloud, unsigned numberOfClasses, int* histo)
{
	assert(cloud);

	unsigned n = cloud->size();

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

	assert(m_Pi.size() == numberOfClasses);

	int* _histo = histo;
	if (!_histo)
		_histo = new int[numberOfClasses];
	if (!_histo)
		return -1.0;

	memset(_histo, 0, numberOfClasses*sizeof(int));

	//histogram computation
	for (unsigned i = 0; i < n; ++i)
	{
		ScalarType V = cloud->getPointScalarValue(i);
		if (ScalarField::ValidValue(V))
		{
			unsigned j = 0;
			for (; j < numberOfClasses - 1; ++j)
				if (V < m_chi2ClassesPositions[j])
					break;

			++_histo[j];
		}
	}

	//calcul de la distance du Chi2
	double dk = 0.0;
	{
		for (unsigned i = 0; i < numberOfClasses; ++i)
		{
			double nPi = static_cast<double>(m_Pi[i]) * numberOfElements;
			double tempValue = static_cast<double>(_histo[i]) - nPi;
			dk += tempValue*tempValue / nPi;
		}
	}

	if (_histo && !histo)
		delete[] _histo;
	_histo = nullptr;

	return dk;
}

bool NormalDistribution::setChi2ClassesPositions(unsigned numberOfClasses)
{
	m_chi2ClassesPositions.resize(0);
	m_Pi.resize(0);

	if (!isValid() || numberOfClasses < 2)
		return false;

	try
	{
		m_Pi.reserve(numberOfClasses);
		m_chi2ClassesPositions.reserve(numberOfClasses - 1);
	}
	catch (const std::bad_alloc&)
	{
		//not engouh memory
		return false;
	}

	//simplest case
	if (numberOfClasses == 2)
	{
		m_Pi.push_back(0.5);
		m_chi2ClassesPositions.push_back(m_mu);
		m_Pi.push_back(0.5);
	}
	else //general case: numberOfClasses>2
	{
		ScalarType sigma = sqrt(m_sigma2);
		//1st class between -inf and mu-2.sigma
		ScalarType x = m_mu - 2 * sigma;
		ScalarType y = static_cast<ScalarType>(computePfromZero(x));
		m_Pi.push_back(y);
		m_chi2ClassesPositions.push_back(x);

		//numberOfClasses-2 classes between mu-2.sigma and mu+2.sigma
		ScalarType pas = 4 * sigma / (numberOfClasses - 2);
		for (unsigned i = 0; i < numberOfClasses - 2; ++i)
		{
			x = x + pas;
			ScalarType oldy = y;
			y = static_cast<ScalarType>(computePfromZero(x));
			m_Pi.push_back(y - oldy);
			m_chi2ClassesPositions.push_back(x);
		}

		//last class between mu+2.sigma and +inf
		//x = m_mu + 2 * sigma;
		y = 1 - y;
		m_Pi.push_back(y);
	}

	return true;
}
