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

#ifdef _MSC_VER
//To get rid of the really annoying warnings about unsafe methods
#pragma warning( disable: 4996 )
#endif

#include "WeibullDistribution.h"
#include "GenericCloud.h"
#include "CCConst.h"
#include "ScalarFieldTools.h"

#include <math.h>
#include <string.h>
#include <assert.h>

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

	if (x == (int)x)
	{
		if (x > 0.0) // use factorial
		{
			double ga = 1.0;
			for (int i=2;i<x;i++)
				ga *= i;
			return ga;
		}
		else
		{
			return 1e308;
		}
	}

	double z,r=0.0;
	if (fabs(x) > 1.0)
	{
		z = fabs(x);
		int m = (int)z;
		r = 1.0;
		for (int k=1;k<=m;k++)
			r *= (z-k);
		z -= m;
	}
	else
	{
		z = x;
	}

	double gr = g[24];
	for (int k=23;k>=0;k--)
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

WeibullDistribution::WeibullDistribution(DistanceType _a, DistanceType _b, DistanceType _valueShift)
{
	setParameters(_a,_b,_valueShift);
}

bool WeibullDistribution::getParameters(DistanceType &_a, DistanceType &_b) const
{
	_a = a;
	_b = b;

	return parametersDefined;
}

bool WeibullDistribution::getOtherParameters(DistanceType &_mu, DistanceType &_sigma2) const
{
	_mu = mu;
	_sigma2 = sigma2;

	return parametersDefined;
}

bool WeibullDistribution::setParameters(DistanceType _a, DistanceType _b, DistanceType _valueShift)
{
	valueShift = _valueShift;
	a = _a;
	b = _b;

	//pour le test du Chi2
	chi2ClassesPositions.clear();

	if (a>0.0 && b>=0.0)
	{
		//moyenne et écart type
		mu = (DistanceType)((double)b * gamma_cc(1.0+1.0/a));
		sigma2 =(DistanceType)((double)(b*b) * gamma_cc(1.0+2.0/a) - (double)(mu*mu));

		parametersDefined=true;
	}
	else
	{
		mu=sigma2=0.0;
		parametersDefined=false;
	}

	return parametersDefined;
};

bool WeibullDistribution::computeParameters(const GenericCloud* Yk, bool includeNegValues)
{
	parametersDefined = false;

	int n = Yk->size();
	if (n == 0)
		return false;

	//on cherche la valeur maximale du champ scalaire pour éviter les overflows
	DistanceType maxValue=0.0;
	ScalarFieldTools::computeScalarFieldExtremas(Yk, valueShift, maxValue, includeNegValues);

	/*if (!includeNegValues)
	valueShift = 0.0;
	else
	//*/
	valueShift -= (DistanceType)ZERO_TOLERANCE;

	if (maxValue<=valueShift)
		return false;

	//printf("maxValue = %f\n",maxValue);
	DistanceType inverseMaxValue = 1.0f/(maxValue-valueShift);

	a = findGRoot(Yk,inverseMaxValue);

	if (a<0.0)
		return false;

	//on peut calculer b
	b=0.0;
	DistanceType v;
	int i,counter = 0;
	for (i=0;i<n;++i)
	{
		v = Yk->getPointScalarValue(i)-valueShift;
		if (v>=0.0)
		{
			b += pow(v*inverseMaxValue,a);
			++counter;
		}
	}

	if (counter==0)
		return false;

	b = (maxValue-valueShift)*pow(b/DistanceType(counter),DistanceType(1.0/a));

	return setParameters(a,b,valueShift);
}

double WeibullDistribution::computeP(DistanceType _x) const
{
	double x = (double)((_x-valueShift)/b);
	if (x<0.0)
		return 0.0;

	double xp = pow(x,(double)(a-1.0));
	return (double)(a/b)*xp*exp(-xp*x);
}

double WeibullDistribution::computePfromZero(DistanceType x) const
{
	return (x<=valueShift ? 0.0 : 1.0-exp(-pow((double)((x-valueShift)/b),(double)a)));
}

double WeibullDistribution::computeP(DistanceType x1, DistanceType x2) const
{
	if (x1 < valueShift)
		x1 = valueShift;
	if (x2 < valueShift)
		return 0.0;
	//pi = computeP(minV+(DistanceType(k)+0.5)*step)*step;
	//... on va plutôt prendre en compte l'échantillonnage et intégrer :
	return exp(-pow((double)((x1-valueShift)/b),(double)a))-exp(-pow((double)((x2-valueShift)/b),(double)a));
}

void WeibullDistribution::getTextualDescription(char* buffer) const
{
	if (!parametersDefined)
		sprintf(buffer,"Undefined Weibull distribution");
	else
	{
		if (valueShift != 0.0)
			sprintf(buffer,"WeibullDistribution [a=%2.4f,b=%5.4f,shfit=%5.4f]",a,b,valueShift);
		else
			sprintf(buffer,"WeibullDistribution [a=%2.4f,b=%5.4f]",a,b);
	}
}

DistanceType WeibullDistribution::computeG(const GenericCloud* Yk, DistanceType r) const
{
	int n = Yk->size();

	//a & n sould be > 0.0 !
	if (r<=0.0 || n==0)
		return 1.0; //une valeur positive va faire échouer la fonction "computeG"

	double p=0.0,q=0.0,s=0.0,v,ln_v,v_a;
	int i,counter=0,zeroValues=0;

	for (i=0;i<n;++i)
	{
		v = Yk->getPointScalarValue(i)-valueShift;
		if (v >= 0) //ici il ne faut pas prendre en compte les valeurs négatives (= points cachés/filtrés)
		{
			if (v > ZERO_TOLERANCE)
			{
				ln_v = log(v);
				v_a = pow(v,(double)r);

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
		ln_v = (DistanceType)zeroValues * log(ZERO_TOLERANCE);
		v_a = pow((DistanceType)ZERO_TOLERANCE,r);
		s += ln_v;
		q += v_a * (DistanceType)zeroValues;
		p += ln_v * v_a;
		counter += zeroValues;
	}

	if (counter==0)
		return 1.0; //une valeur positive va faire échouer la fonction "computeG"

	return (DistanceType)((double)r * (p/q - s/(double)counter) - 1.0);
}

DistanceType WeibullDistribution::computeG(const GenericCloud* Yk, DistanceType r, DistanceType inverseVmax) const
{
	int n = Yk->size();

	//r & n sould be > 0.0 !
	if (r<=0.0 || n==0)
		return 1.0; //une valeur positive va faire échouer la fonction "computeG"

	double p=0.0,q=0.0,s=0.0,v,ln_v,v_a;
	int i,counter=0,zeroValues=0;

	for (i=0;i<n;++i)
	{
		v = Yk->getPointScalarValue(i)-valueShift;
		if (v>=0.0)
		{
			if (v > ZERO_TOLERANCE)
			{
				ln_v = log(v);
				v *= inverseVmax;
				v_a = pow(v,(double)r);

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
		ln_v = (double)zeroValues * log(ZERO_TOLERANCE);
		v_a = pow((double)inverseVmax * ZERO_TOLERANCE,(double)r);
		s += ln_v;
		q += v_a * (DistanceType)zeroValues;
		p += ln_v * v_a;
		counter += zeroValues;
	}

	//printf("p=%f q=%f s=%f\n",p,q,s);
	if (counter==0)
		return 1.0; //une valeur positive va faire échouer la fonction "computeG"

	return (DistanceType)((double)r * (p/q - s/(double)counter) - 1.0);
}

DistanceType WeibullDistribution::findGRoot(const GenericCloud* Yk, DistanceType inverseMaxValue) const
{
	DistanceType r=-1.0;
	DistanceType v,vMin,vMax,aMin,aMax;
	aMin = aMax = 1.0;
	vMin = vMax = v = computeG(Yk,aMin,inverseMaxValue);

	//on cherche une borne minimale pour la dichotomie telle que computeG(aMin)<0.0
	while (vMin>0.0 && aMin>1e-7)
	{
		aMin *= 0.1f;
		vMin = computeG(Yk,aMin,inverseMaxValue);
		//printf("*** aMin = %f / vMin = %f\n",aMin,vMin);
	}
	//printf("aMin = %f / vMin = %f\n",aMin,vMin);

	if (fabs(vMin)<1e-7)
		return aMin;
	else if (vMin>0.0)
		return -1.0; //problème

	//on cherche une borne maximale pour la dichotomie telle que computeG(aMax)>0.0
	while (vMax<0.0 && aMax<1000.0)
	{
		aMax *= 2.0; //puisqu'on calcule des x^a, ça devient vite énorme !!!!
		vMax = computeG(Yk,aMax,inverseMaxValue);
		//printf("*** aMax = %f / vMax = %f\n",aMax,vMax);
	}
	//printf("aMax = %f / vMax = %f\n",aMax,vMax);

	if (fabs(vMax)<1e-7)
		return aMax;
	else if (vMax<0.0)
		return -1.0; //problème

	//system("PAUSE");
	//printf("aMin = %f - aMax = %f\n vMin = %f - vMax = %f\n",aMin,aMax,vMin,vMax);

	//dichotomie pour trouver r tq computeG(r)<1e-7
	DistanceType old_v;
	while (fabs(v)>1e-5)
	{
		r = (aMin+aMax)*0.5f;
		//printf("r=%f\n",r);
		old_v=v;
		v = computeG(Yk,r,inverseMaxValue);

		if (fabs(old_v-v)<1e-7)
			return r;

		//printf("v=%f [%f:%f]\n",v,aMin,aMax);

		if (v<0.0)
			aMin = r;
		else
			aMax = r;
	}

	return r; //shouldn't be here !
}

double WeibullDistribution::computeChi2Dist(const GenericCloud* Yk, unsigned numberOfClasses, bool includeNegValues, int* histo)
{
	assert(Yk);

	unsigned i,n = Yk->size();

	//we must refine the real number of elements
	unsigned numberOfElements = n;
	if (!includeNegValues)
		numberOfElements = ScalarFieldTools::countScalarFieldPositiveValues(Yk);

	if (numberOfElements==0)
		return -1.0;

	if (numberOfClasses < 1 || numberOfClasses*numberOfClasses>numberOfElements)
		return -1.0;
	else if (numberOfClasses == 1)
		return 0.0;

	if (!setChi2ClassesPositions(numberOfClasses))
		return -1.0;

	assert(chi2ClassesPositions.size()+1 == numberOfClasses);

	int* _histo = histo;
	if (!_histo)
		_histo = new int[numberOfClasses];
	if (!_histo)
		return -1.0; //problème d'allocation
	memset(_histo,0,numberOfClasses*sizeof(int));

	//calcul de l'histogramme
	unsigned j;
	DistanceType V;
	for (i=0;i<n;++i)
	{
		V = Yk->getPointScalarValue(i);
		if (includeNegValues || V>=0.0)
		{
			for (j=0;j<numberOfClasses-1;++j)
				if (V<chi2ClassesPositions[j])
					break;

			++_histo[j];
		}
	}

	//calcul de la distance du Chi2
	DistanceType nPi = (DistanceType)numberOfElements/(DistanceType)numberOfClasses;
	DistanceType tempValue,dk = 0.0;
	for (i=0;i<numberOfClasses;++i)
	{
		tempValue = (DistanceType)_histo[i] - nPi;
		dk += tempValue*tempValue;
	}
	dk /= nPi;

	if (_histo && !histo)
		delete[] _histo;
	_histo=0;

	return dk;
}

bool WeibullDistribution::setChi2ClassesPositions(unsigned numberOfClasses)
{
	chi2ClassesPositions.clear();

	if (!parametersDefined || numberOfClasses<2)
		return false;

	try
	{
		chi2ClassesPositions.resize(numberOfClasses-1);
	}
	catch (.../*const std::bad_alloc&*/) //out of memory
	{
		return false;
	}

	//on créé "numberOfClasses" classes equiprobables (elles auront des nPi>=sqrt(n) quoi qu'il arrive (avec n>=4, c bien)
	double areaPerClass = 1.0/(double)numberOfClasses;
	double currentArea = areaPerClass;
	double invA = 1.0f/(double)a;

	for (unsigned i=1;i<numberOfClasses;++i)
	{
		chi2ClassesPositions[i-1] = b * (DistanceType)pow(-log(1.0-currentArea),invA);
		//printf("Classe %i/%i : %f\n",i-1,numberOfClasses,chi2ClassesPositions.back());
		currentArea += areaPerClass;
	}

	return true;
}
