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

#ifndef CONJUGATE_GRADIENT_HEADER
#define CONJUGATE_GRADIENT_HEADER

//Local
#include "MathTools.h"
#include "SquareMatrix.h"


namespace CCLib
{

//! A class to perform a conjugate gradient optimization
/**	Inspired from the "Numerical Recipes".
	Template parameter 'N' is the dimension of the linear system.
	Lets "A*Xn=b" be the system to optimize (at iteration n).
	First the user must init the A matrix (N*N) and b vector (N*1).
	Then the solver is initialized with X0 (see initConjugateGradient).
	And the conjugate gradient is iterated with iterConjugateGradient.
**/
template <int N, class Scalar> class ConjugateGradient : MathTools
{
public:

	//! Default constructor
	ConjugateGradient()
		: cg_A(N)
	{
		memset(cg_Gn, 0, sizeof(Scalar)*N);
		memset(cg_Hn, 0, sizeof(Scalar)*N);
		memset(cg_u,  0, sizeof(Scalar)*N);
		memset(cg_b,  0, sizeof(Scalar)*N);
	}

	//! Default destructor
	virtual ~ConjugateGradient() = default;

	//! Returns A matrix
	inline CCLib::SquareMatrixTpl<Scalar>& A() { return cg_A; }

	//! Returns b vector
	inline Scalar* b() { return cg_b; }

	//! Initializes the conjugate gradient
	/** \param X0 the initial state (size N)
	**/
	void initConjugateGradient(const Scalar* X0)
	{
		//we init the Gn (residuals) and Hn vectors
		//H0 = G0 = A.X0-b
		cg_A.apply(X0,cg_Gn);
		for (unsigned k=0; k<N; ++k)
			cg_Hn[k] = (cg_Gn[k] -= cg_b[k]);
	}

	//! Iterates the conjugate gradient
	/** Xn will be automatically updated to Xn+1 on output.
		\param Xn the current estimation of Xn (size N)
		\return mean square error
	**/
	Scalar iterConjugateGradient(Scalar* Xn)
	{
		//we compute Xn+1
		cg_A.apply(cg_Hn,cg_u);		//u = A.Hn

		unsigned k;
		Scalar d = 0, e = 0, f = 0;
		for (k=0; k<N; ++k)
		{
			d += cg_Hn[k]*cg_Gn[k];	    // t^Hn.Gn
			e += cg_Hn[k]*cg_u[k];	    // t^Hn.A.Hn
			f += cg_Gn[k]*cg_Gn[k];	    // t^Gn.Gn (needed for Hn+1 - see below)
		}

		//Xn+1 = Xn - Hn*(t^Hn.Gn)/(t^Hn.A.Hn)
		d /= e;
		for (k=0; k<N; ++k)
			Xn[k] -= cg_Hn[k]*d;

		//we compute Gn+1
		cg_A.apply(Xn,cg_u);           // u = A.Xn+1
		for (k=0; k<N; ++k)
			cg_Gn[k] = cg_u[k]-cg_b[k];	//Gn+1 = A.Xn+1-b

		//sum of square errors
		e=cg_Gn[0]*cg_Gn[0];
		for (k=1;k<N;++k)
			e += cg_Gn[k]*cg_Gn[k];	    //t^Gn+1.Gn+1

		//we update Hn+1 for next iteration
		d = e/f;
		for (k=0; k<N; ++k)
			cg_Hn[k] = cg_Gn[k] + cg_Hn[k]*d;	//Hn+1 = Gn+1 + Hn.(t^Gn+1.Gn+1)/(t^Gn.Gn)

		return e/N;
	}

protected:

	//! Residuals vector
	Scalar cg_Gn[N];
	
	//! 'Hn' vector
	/** Intermediary computation result
	**/
	Scalar cg_Hn[N];
	
	//! 'u' vector
	/** Intermediary computation result
	**/
	Scalar cg_u[N];

	//! 'b' vector
	/** Equation solved: "A.X=b"
	**/
	Scalar cg_b[N];

	//! 'A' matrix
	/** Equation solved: "A.X=b"
	**/
	CCLib::SquareMatrixTpl<Scalar> cg_A;
};

}

#endif //CONJUGATE_GRADIENT_HEADER
