#ifndef LEVMARFITTING_HEADER
#define LEVMARFITTING_HEADER
#include <algorithm>
#include <iostream>
#include "LevMarFunc.h"
#ifdef DOPARALLEL
#include <omp.h>
#endif

template< class ScalarT >
bool Cholesky(ScalarT *a, size_t n, ScalarT p[])
/*Given a positive-definite symmetric matrix a[1..n][1..n], this routine constructs its Cholesky
decomposition, A = L · LT . On input, only the upper triangle of a need be given; it is not
modified. The Cholesky factor L is returned in the lower triangle of a, except for its diagonal
elements which are returned in p[1..n].*/
{
	size_t i, j, k;
	ScalarT sum;
	for(i = 0; i < n; ++i)
	{
		for(j = i; j < n; ++j)
		{
			for(sum = a[i * n + j], k = i - 1; k != -1; --k)
				sum -= a[i * n + k] * a[j * n + k];
			if(i == j)
			{
				if(sum <= ScalarT(0))
					// a, with rounding errors, is not positive definite.
					return false;
				p[i] = std::sqrt(sum);
			}
			else
				a[j * n + i]= sum / p[i];
		}
	}
	return true;
}

template< class ScalarT, unsigned int N >
bool Cholesky(ScalarT *a, ScalarT p[])
/*Given a positive-definite symmetric matrix a[1..n][1..n], this routine constructs its Cholesky
decomposition, A = L · LT . On input, only the upper triangle of a need be given; it is not
modified. The Cholesky factor L is returned in the lower triangle of a, except for its diagonal
elements which are returned in p[1..n].*/
{
	size_t i, j, k;
	ScalarT sum;
	for(i = 0; i < N; ++i)
	{
		for(j = i; j < N; ++j)
		{
			for(sum = a[i * N + j], k = i - 1; k != -1; --k)
				sum -= a[i * N + k] * a[j * N + k];
			if(i == j)
			{
				if(sum <= ScalarT(0))
					// a, with rounding errors, is not positive definite.
					return false;
				p[i] = std::sqrt(sum);
			}
			else
				a[j * N + i]= sum / p[i];
		}
	}
	return true;
}

template< class ScalarT >
void CholeskySolve(ScalarT *a, size_t n, ScalarT p[], ScalarT b[], ScalarT x[])
/*Solves the set of n linear equations A · x = b, where a is a positive-definite symmetric matrix.
a[1..n][1..n] and p[1..n] are input as the output of the routine choldc. Only the lower
subdiagonal portion of a is accessed. b[1..n] is input as the right-hand side vector. The
solution vector is returned in x[1..n]. a, n, and p are not modified and can be left in place
for successive calls with different right-hand sides b. b is not modified unless you identify b and
x in the calling sequence, which is allowed.*/
{
	size_t i, k;
	ScalarT sum;
	for(i = 0; i < n; i++)
	{ // Solve L · y = b, storing y in x.
		for(sum = b[i], k = i-1; k != -1; --k)
			sum -= a[i * n + k] * x[k];
		x[i] = sum / p[i];
	}
	for(i = n - 1; i != -1; --i)
	{ // Solve LT · x = y.
		for(sum = x[i], k = i + 1; k < n; ++k)
			sum -= a[k * n + i] * x[k];
		x[i]= sum / p[i];
	}
}

template< class ScalarT, unsigned int N >
void CholeskySolve(ScalarT *a, ScalarT p[], ScalarT b[], ScalarT x[])
/*Solves the set of n linear equations A · x = b, where a is a positive-definite symmetric matrix.
a[1..n][1..n] and p[1..n] are input as the output of the routine choldc. Only the lower
subdiagonal portion of a is accessed. b[1..n] is input as the right-hand side vector. The
solution vector is returned in x[1..n]. a, n, and p are not modified and can be left in place
for successive calls with different right-hand sides b. b is not modified unless you identify b and
x in the calling sequence, which is allowed.*/
{
	size_t i, k;
	ScalarT sum;
	for(i = 0; i < N; i++)
	{ // Solve L · y = b, storing y in x.
		for(sum = b[i], k = i-1; k != -1; --k)
			sum -= a[i * N + k] * x[k];
		x[i] = sum / p[i];
	}
	for(i = N - 1; i != -1; --i)
	{ // Solve LT · x = y.
		for(sum = x[i], k = i + 1; k < N; ++k)
			sum -= a[k * N + i] * x[k];
		x[i]= sum / p[i];
	}
}

template< class IteratorT, class FuncT >
bool LevMar(IteratorT begin, IteratorT end, FuncT &func,
	typename FuncT::ScalarType *param)
{
	typedef typename FuncT::ScalarType ScalarType;
	enum { paramDim = FuncT::NumParams };
	bool retVal = true;
	size_t totalSize = end - begin;
	if (!totalSize)
		return false;
	ScalarType lambda = ScalarType(0.0001);
	ScalarType *F0 = new ScalarType[totalSize * paramDim];
	ScalarType *U = new ScalarType[paramDim * paramDim];
	ScalarType *H = new ScalarType[paramDim * paramDim];
	ScalarType *v = new ScalarType[paramDim];
	ScalarType *d = new ScalarType[totalSize];
	ScalarType *temp = new ScalarType[totalSize];
	ScalarType *x = new ScalarType[paramDim];
	ScalarType *p = new ScalarType[paramDim];
	ScalarType *paramNew = new ScalarType[paramDim];
	size_t nu = 2;
	func.Normalize(param);
	ScalarType paramNorm = 0;

	// do fitting in different steps
	unsigned int subsets = std::max(int(std::floor(std::log((float)totalSize)/std::log(2.f)))-8, 1);
#ifdef PRECISIONLEVMAR
	subsets = 1;
#endif
	MiscLib::Vector< unsigned int > subsetSizes(subsets);
	for(size_t i = subsetSizes.size(); i;)
	{
		--i;
		subsetSizes[i] = static_cast<unsigned>(totalSize);
		if(i)
			subsetSizes[i] = subsetSizes[i] >> 1;
		totalSize -= subsetSizes[i];
	}
	unsigned int curSubset = 0;
	unsigned int size = 0;

	// get current error
	ScalarType chi = 0, newChi = 0;
	ScalarType rho = 1;
	size_t outerIter = 0,
#ifndef PRECISIONLEVMAR
		maxOuterIter = 200 / subsetSizes.size(),
#else
		maxOuterIter = 500,
#endif
		usefulIter = 0, totalIter = 0;;
	do
	{
		// get current error
		size += subsetSizes[curSubset];
		newChi = func.Chi(param, begin, begin + size, d, temp);
		for(unsigned int i = 0; i < paramDim; ++i)
			paramNew[i] = param[i];
		outerIter = 0;
		if(rho < 0)
			rho = 1;
		do
		{
			++outerIter;
			++totalIter;
			if(rho > 0)
			{
				nu = 2;
				chi = newChi;
				for(size_t i = 0; i < paramDim; ++i)
					param[i] = paramNew[i];
#ifndef PRECISIONLEVMAR
				if(std::sqrt(chi / size) < ScalarType(1e-5)) // chi very small? -> will be hard to improve
				{
					//std::cout << "LevMar converged because of small chi" << std::endl;
					break;
				}
#endif
				paramNorm = 0;
				for(size_t i = 0; i < paramDim; ++i)
					paramNorm += param[i] * param[i];
				paramNorm = std::sqrt(paramNorm);
				// construct the needed matrices
				// F0 is the matrix constructed from param
				// F0 has gradient_i(param) as its ith row
				func.Derivatives(param, begin, begin + size, d, temp, F0);
				// U = F0_t * F0
				// v = F0_t * d(param) (d(param) = [d_i(param)])
#ifdef DOPARALLEL
				#pragma omp parallel for
#endif
				for(int i = 0; i < paramDim; ++i)
				{
					for(size_t j = i; j < paramDim; ++j) // j = i since only upper triangle is needed
					{
						U[i * paramDim + j] = 0;
						for(size_t k = 0; k < size; ++k)
						{
							U[i * paramDim + j] += F0[k * paramDim + i] *
								F0[k * paramDim + j];
						}
					}
				}
				ScalarType vmag = 0; // magnitude of v
#ifdef DOPARALLEL
				#pragma omp parallel for
#endif
				for(int i = 0; i < paramDim; ++i)
				{
					v[i] = 0;
					for(size_t k = 0; k < size; ++k)
						v[i] += F0[k * paramDim + i] * d[k];
					v[i] *= -1;
#ifndef DOPARALLEL
					vmag = std::max((ScalarType)fabs(v[i]), vmag);
#endif
				}
#ifdef DOPARALLEL
				for(unsigned int i = 0; i < paramDim; ++i)
					vmag = std::max(fabs(v[i]), vmag);
#endif
				// and check for convergence with magnitude of v
#ifndef PRECISIONLEVMAR
				if(vmag < ScalarType(1.0e-6))
#else
				if(vmag < ScalarType(1e-8))
#endif
				{
					//std::cout << "LevMar converged with small gradient" << std::endl;
					//retVal = chi < initialChi;
					//goto cleanup;
					break;
				}
				if(outerIter == 1)
				{
					// compute magnitue of F0
					ScalarType fmag = fabs(F0[0]);
					for(size_t i = 1; i < paramDim * size; ++i)
						if(fmag < fabs(F0[i]))
							fmag = fabs(F0[i]);
					lambda = 1e-3f * fmag;
				}
				else
					lambda *= std::max( ScalarType(0.3), 1 - ScalarType(std::pow(2 * rho - 1, 3)) );
			}

			memcpy(H, U, sizeof(ScalarType) * paramDim * paramDim);
			for(size_t i = 0; i < paramDim; ++i)
				H[i * paramDim + i] += lambda; // * (ScalarType(1) + H[i * paramDim + i]);
			// now H is positive definite and symmetric
			// solve Hx = -v with Cholesky
			ScalarType xNorm = 0, L = 0;
			if(!Cholesky< ScalarType, paramDim >(H, p))
				goto increment;
			CholeskySolve< ScalarType, paramDim >(H, p, v, x);

			// magnitude of x small? If yes we are done
			for(size_t i = 0; i < paramDim; ++i)
				xNorm += x[i] * x[i];
			xNorm = std::sqrt(xNorm);
#ifndef PRECISIONLEVMAR
			if(xNorm <= ScalarType(1.0e-6) * (paramNorm + ScalarType(1.0e-6)))
#else
			if(xNorm <= ScalarType(1e-8) * (paramNorm + ScalarType(1e-8)))
#endif
			{
				//std::cout << "LevMar converged with small step" << std::endl;
				//goto cleanup;
				break;
			}

			for(size_t i = 0; i < paramDim; ++i)
				paramNew[i] = param[i] + x[i];
			func.Normalize(paramNew);

			// get new error
			newChi = func.Chi(paramNew, begin, begin + size, d, temp);

			// the following test is taken from
			// "Methods for non-linear least squares problems"
			// by Madsen, Nielsen, Tingleff
			L = 0;
			for(size_t i = 0; i < paramDim; ++i)
				L += .5f * x[i] * (lambda * x[i] + v[i]);
			rho = (chi - newChi) / L;
			if(rho > 0)
			{
				++usefulIter;
#ifndef PRECISIONLEVMAR
				if((chi - newChi) < 1e-4 * chi)
				{
					//std::cout << "LevMar converged with small chi difference" << std::endl;
					chi = newChi;
					for(size_t i = 0; i < paramDim; ++i)
						param[i] = paramNew[i];
					break;
				}
#endif
				continue;
			}

	increment:
			rho = -1;
			// increment lambda
			lambda = nu * lambda;
			size_t nu2 = nu << 1;
			if(nu2 < nu)
				nu2 = 2;
			nu = nu2;
		}
		while(outerIter < maxOuterIter);
		++curSubset;
	}
	while(curSubset < subsetSizes.size());
	retVal = usefulIter > 0;
	delete[] F0;
	delete[] U;
	delete[] H ;
	delete[] v;
	delete[] d;
	delete[] temp;
	delete[] x;
	delete[] p;
	delete[] paramNew;
	return retVal;
}

template< class ScalarT >
ScalarT LevMar(unsigned int paramDim, unsigned int imgDim,
	const LevMarFunc< ScalarT > **funcs, ScalarT *param)
{
	ScalarT retVal = -1;
	size_t size = imgDim;
	float lambda = ScalarT(0.0001);
	ScalarT *F0 = new ScalarT[size * paramDim];
	ScalarT *U = new ScalarT[paramDim * paramDim];
	ScalarT *H = new ScalarT[paramDim * paramDim];
	ScalarT *v = new ScalarT[paramDim];
	ScalarT *d = new ScalarT[size];
	ScalarT *dNew = new ScalarT[size];
	ScalarT *x = new ScalarT[paramDim];
	ScalarT *p = new ScalarT[paramDim];
	ScalarT *paramNew = new ScalarT[paramDim];
	size_t outerIter = 0, maxOuterIter = 10;
	// get current error
	ScalarT chi = 0, newChi;
	for(size_t i = 0; i < size; ++i)
	{
		d[i] = (*(funcs[i]))(param);
		chi += d[i] * d[i];
	}
	do
	{
		++outerIter;
		lambda *= ScalarT(0.04);
		// construct the needed matrices
		// F0 is the matrix constructed from param
		// F0 has gradient_i(param) as its ith row
		for(size_t i = 0; i < size; ++i)
			(*(funcs[i]))(param, &F0[i * paramDim]);
		// U = F0_t * F0
		for(size_t i = 0; i < paramDim; ++i)
			for(size_t j = i; j < paramDim; ++j) // j = i since only upper triangle is needed
			{
				U[i * paramDim + j] = 0;
				for(size_t k = 0; k < size; ++k)
					U[i * paramDim + j] += F0[k * paramDim + i] *
						F0[k * paramDim + j];
			}
		// v = F_t * d(param) (d(param) = [d_i(param)])
		for(size_t i = 0; i < paramDim; ++i)
		{
			v[i] = 0;
			for(size_t k = 0; k < size; ++k)
				v[i] += F0[k * paramDim + i] * d[k];
			v[i] *= -1;
		}
		size_t iter = 0, maxIter = 10;
		do
		{
			++iter;
			// increment lambda
			lambda = 10 * lambda;
			memcpy(H, U, sizeof(ScalarT) * paramDim * paramDim);
			for(size_t i = 0; i < paramDim; ++i)
				H[i * paramDim + i] += lambda * (ScalarT(1) + H[i * paramDim + i]);
			// now H is positive definite and symmetric
			// solve Hx = -v with Cholesky
			if(!Cholesky(H, paramDim, p))
				goto cleanup;
			CholeskySolve(H, paramDim, p, v, x);
			for(size_t i = 0; i < paramDim; ++i)
				paramNew[i] = param[i] + x[i];
			// get new error
			newChi = 0;
			for(size_t i = 0; i < size; ++i)
			{
				dNew[i] = (*(funcs[i]))(paramNew);
				newChi += dNew[i] * dNew[i];
			}
			// check for convergence
			/*float cvgTest = 0;
			for(size_t i = 0; i < paramDim; ++i)
			{
				//float c = param[i] - paramNew[i];
				cvgTest += x[i] * x[i];
			}
			if(std::sqrt(cvgTest) < 1.0e-6)
			{
				for(size_t i = 0; i < paramDim; ++i)
					param[i] = paramNew[i];
				goto cleanup;
			}*/
			if(/*newChi <= chi &&*/ fabs(chi - newChi)
				/ chi < ScalarT(1e-4))
			{
				for(size_t i = 0; i < paramDim; ++i)
					param[i] = paramNew[i];
				retVal = newChi;
				goto cleanup;
			}
		}
		while(newChi > chi && iter < maxIter);
		if(newChi < chi)
		{
			chi = newChi;
			for(size_t i = 0; i < paramDim; ++i)
				param[i] = paramNew[i];
			std::swap(d, dNew);
		}
	}
	while(outerIter < maxOuterIter);
cleanup:
	delete[] F0;
	delete[] U;
	delete[] H ;
	delete[] v;
	delete[] d;
	delete[] dNew;
	delete[] x;
	delete[] p;
	delete[] paramNew;
	return retVal;
}

#endif
