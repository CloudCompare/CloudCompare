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

#ifndef ERROR_FUNCTION_HEADER
#define ERROR_FUNCTION_HEADER

#include "MathTools.h"

namespace CCLib
{

//! Relative error for Error Function computation
/** You can adjust it to trade off between accuracy and speed
	but don't ask for more than 15 figures (assuming usual 52 bit mantissa
	in a double). Example: 1E-12 <--> calculate 12 significant figures
**/
static const double erfRelativeError= 1E-12;

//! A class to compute the Error function (erf)
/** See for example http://mathworld.wolfram.com/Erf.html.
	Most of the code comes from "erf.cpp" by Steve Strand
	(29-Jan-04).
**/

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API ErrorFunction : MathTools
#else
class ErrorFunction : MathTools
#endif
{
public:


	//! Computes erfc(x)
	/** erfc(x) = 2/sqrt(pi)*integral(exp(-t^2),t,x,inf)
				= exp(-x^2)/sqrt(pi) * [1/x+ (1/2)/x+ (2/2)/x+ (3/2)/x+ (4/2)/x+ ...]
				= 1-erf(x)
		(expression inside [] is a continued fraction so '+' means add to denominator
		only).
		\param x a real variable
		\return erfc(x)
	**/
	static double erfc(double x);

	//! Computes erf(x)
	/** erf(x) = 2/sqrt(pi)*integral(exp(-t^2),t,0,x)
			   = 2/sqrt(pi)*[x - x^3/3 + x^5/5*2! - x^7/7*3! + ...]
		       = 1-erfc(x)
		\param x a real variable
		\return erf(x)
	**/
	static double erf(double x);
};

}

#endif
