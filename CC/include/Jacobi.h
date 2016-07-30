//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                    COPYRIGHT: CloudCompare project                     #
//#                                                                        #
//##########################################################################

#ifndef JACOBI_EIGEN_HEADER
#define JACOBI_EIGEN_HEADER

//Local
#include "SquareMatrix.h"


//! Jacobi eigen vectors/values decomposition
template <typename Scalar> class Jacobi
{
public:

	typedef CCLib::SquareMatrixTpl<Scalar> SquareMatrix;
	typedef std::vector<Scalar> EigenValues;

	//! Computes the eigenvalues and eigenvectors of a given square matrix
	/** It uses Rutishauser's modfications of the classical Jacobi rotation method with threshold pivoting. 

		Note: this code is inspired from John Burkardt's 'jacobi_eigenvalue' code
		See https://people.sc.fsu.edu/~jburkardt/cpp_src/jacobi_eigenvalue/jacobi_eigenvalue.cpp

		\param[in] matrix input square matrix
		\param[out] eigenVectors eigenvectors (as a square matrix)
		\param[out] eigenValues eigenvalues
		\param[in] maxIterationCount max number of iteration (optional)
		\return success
	**/
	static bool ComputeEigenValuesAndVectors(const SquareMatrix& matrix, SquareMatrix& eigenVectors, EigenValues& eigenValues, unsigned maxIterationCount = 50)
	{
		if (!matrix.isValid())
		{
			return false;
		}

		unsigned n = matrix.size();

		//duplicate the current matrix (as it may be modified afterwards!)
		SquareMatrix a(matrix);

		//output eigen vectors matrix
		eigenVectors = SquareMatrix(n);
		if (!eigenVectors.isValid())
		{
			return false;
		}
		eigenVectors.toIdentity();

		std::vector<Scalar> bw,zw;
		try
		{
			eigenValues.resize(n);
			zw.resize(n,0);
			bw.resize(n);

			//initialize bw and d to the diagonal of this matrix
			for (unsigned i=0; i<n; i++)
			{
				bw[i] = eigenValues[i] = a.m_values[i][i];
			}
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return false;
		}

		for (unsigned it=0; it<maxIterationCount; ++it)
		{
			//The convergence threshold is based on the size of the elements in
			//the strict upper triangle of the matrix.
			double sum = 0.0;
			for (unsigned j=1; j<n; ++j)
			{
				for (unsigned i=0; i<j; ++i)
				{
					double val = a.m_values[i][j];
					sum += val * val;
				}
			}

			Scalar thresh = static_cast<Scalar>(sqrt(sum) / (4*n));
			if (thresh == 0)
			{
				break;
			}

			for (unsigned p=0; p<n; ++p)
			{
				for (unsigned q=p+1; q<n; ++q)
				{
					Scalar gapq = fabs(a.m_values[p][q]) * static_cast<Scalar>(10);
					Scalar termp = gapq + fabs(eigenValues[p]);
					Scalar termq = gapq + fabs(eigenValues[q]);

					//Annihilate tiny offdiagonal elements
					if (   it < 4
						&& termp == fabs(eigenValues[p])
						&& termq == fabs(eigenValues[q]) )
					{
						a.m_values[p][q] = 0;
					}
					//Otherwise, apply a rotation
					else if (thresh <= fabs(a.m_values[p][q]))
					{
						Scalar h = eigenValues[q] - eigenValues[p];
						Scalar term = fabs(h) + gapq;

						Scalar t = 0;
						if (term == fabs(h))
						{
							t = a.m_values[p][q] / h;
						}
						else
						{
							Scalar theta = h / (2 * a.m_values[p][q]);
							t = 1 / (fabs(theta) + sqrt(1 + theta*theta));
							if ( theta < 0 )
							{
								t = - t;
							}
						}
						Scalar c = 1 / sqrt(1 + t*t);
						Scalar s = t * c;
						Scalar tau = s / (1 + c);
						h = t * a.m_values[q][p];

						//Accumulate corrections to diagonal elements.
						zw[p] -= h;                 
						zw[q] += h;
						eigenValues[p]  -= h;
						eigenValues[q]  += h;

						a.m_values[p][q] = 0;

						// Rotate, using information from the upper triangle of A only.
						unsigned j;
						for (j=0; j<p; ++j)
						{
							Scalar g = a.m_values[j][p];
							Scalar h = a.m_values[j][q];
							a.m_values[j][p] = g - s * (h + g * tau);
							a.m_values[j][q] = h + s * (g - h * tau);
						}

						for (j=p+1; j<q; ++j)
						{
							Scalar g = a.m_values[p][j];
							Scalar h = a.m_values[j][q];
							a.m_values[p][j] = g - s * ( h + g * tau );
							a.m_values[j][q] = h + s * ( g - h * tau );
						}

						for (j=q+1; j<n; ++j)
						{
							Scalar g = a.m_values[p][j];
							Scalar h = a.m_values[q][j];
							a.m_values[p][j] = g - s * ( h + g * tau );
							a.m_values[q][j] = h + s * ( g - h * tau );
						}

						//Accumulate information in the eigenvector matrix.
						for (j=0; j<n; ++j)
						{
							Scalar g = eigenVectors.m_values[j][p];
							Scalar h = eigenVectors.m_values[j][q];
							eigenVectors.m_values[j][p] = g - s * ( h + g * tau );
							eigenVectors.m_values[j][q] = h + s * ( g - h * tau );
						}
					}
				}
			}

			for (unsigned i=0; i<n; ++i)
			{
				bw[i] += zw[i];
				eigenValues[i] = bw[i];
				zw[i] = 0.0;
			}
		}

		return true;
	}

	//! Sorts the eigenvectors in the decreasing order of their associated eigenvalues
	/** \param eigenVectors eigenvectors (as a square matrix)
		\param eigenValues eigenvalues
		\return success
	**/
	static bool SortEigenValuesAndVectors(SquareMatrix& eigenVectors, EigenValues& eigenValues)
	{
		if (!eigenVectors.isValid()
			|| eigenVectors.size() < 2
			|| eigenVectors.size() != eigenValues.size())
		{
			assert(false);
			return false;
		}

		unsigned n = eigenVectors.size();
		for (unsigned i=0; i<n-1; i++)
		{
			unsigned maxValIndex = i;
			for (unsigned j=i+1; j<n; j++)
				if (eigenValues[j] > eigenValues[maxValIndex])
					maxValIndex = j;

			if (maxValIndex != i)
			{
				std::swap(eigenValues[i], eigenValues[maxValIndex]);
				for (unsigned j=0; j<n; ++j)
					std::swap(eigenVectors.m_values[j][i], eigenVectors.m_values[j][maxValIndex]);
			}
		}

		return true;
	}

	//! Returns the given eigenvector
	/** \param eigenVectors eigenvectors (as a square matrix)
		\param index requested eigenvector index (< eigenvectors matrix size)
		\param eigenVector output vector (size = matrix size)
		\return success
	**/
	static bool GetEigenVector(const SquareMatrix& eigenVectors, unsigned index, Scalar eigenVector[])
	{
		if (eigenVector && index < eigenVectors.size())
		{
			for (unsigned i=0; i<eigenVectors.size(); ++i)
			{
				eigenVector[i] = eigenVectors.m_values[i][index];
			}
			return true;
		}
		else
		{
			assert(false);
			return false;
		}
	}
	
	//! Returns the biggest eigenvalue and its associated eigenvector
	/** \param eigenVectors eigenvectors (as a square matrix)
		\param eigenValues eigenvalues
		\param maxEigenValue biggest eigenvalue
		\param maxEigenVector eigenvector vector corresponding to the biggest eigenvalue
		\return success
	**/
	static bool GetMaxEigenValueAndVector(const SquareMatrix& eigenVectors, const EigenValues& eigenValues, Scalar& maxEigenValue, Scalar maxEigenVector[])
	{
		if (!eigenVectors.isValid()
			|| eigenVectors.size() < 2
			|| eigenVectors.size() != eigenValues.size())
		{
			assert(false);
			return false;
		}

		unsigned maxIndex = 0;
		for (unsigned i=1; i<eigenVectors.size(); ++i)
			if (eigenValues[i] > eigenValues[maxIndex])
				maxIndex = i;

		maxEigenValue = eigenValues[maxIndex];
		return GetEigenVector(eigenVectors, maxIndex, maxEigenVector);
	}

	//! Returns the smallest eigenvalue and its associated eigenvector
	/** \param eigenVectors eigenvectors (as a square matrix)
		\param eigenValues eigenvalues
		\param minEigenValue smallest eigenvalue
		\param minEigenVector eigenvector vector corresponding to the smallest eigenvalue
		\return success
	**/
	static bool GetMinEigenValueAndVector(const SquareMatrix& eigenVectors, const EigenValues& eigenValues, Scalar& minEigenValue, Scalar minEigenVector[])
	{
		if (!eigenVectors.isValid()
			|| eigenVectors.size() < 2
			|| eigenVectors.size() != eigenValues.size())
		{
			assert(false);
			return false;
		}

		unsigned minIndex = 0;
		for (unsigned i=1; i<eigenVectors.size(); ++i)
			if (eigenValues[i] < eigenValues[minIndex])
				minIndex = i;

		minEigenValue = eigenValues[minIndex];
		return GetEigenVector(eigenVectors, minIndex, minEigenVector);
	}
};

#endif //JACOBI_EIGEN_HEADER
