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

#ifndef CC_MISC_TOOLS_HEADER
#define CC_MISC_TOOLS_HEADER

#include "CCToolbox.h"

#include <math.h>

#if !defined(_WIN32) && !defined(WIN32) && defined(__GNUC__)
#include <stdint.h>
#define __int64 int64_t
#endif

namespace CCLib
{

/*** USEFUL MACROS ***/

#ifndef ccMax
#define ccMax(a,b) (a<b ? b : a)
#endif

#ifndef ccMin
#define ccMin(a,b) (a<b ? a : b)
#endif

//! Space char. ASCII code
#define SPACE_ASCII_CODE 32
//! Tab char. ASCII code
#define TAB_ASCII_CODE 9
//! Return char. ASCII code
#define ENTER_ASCII_CODE 10

//! Miscellaneous useful functions (mainly for string & geometrical elements handling)
#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"
class CC_DLL_API CCMiscTools : public CCToolbox
#else
class CCMiscTools : public CCToolbox
#endif
{
public:

	//! Shortcut to the 64 bits "fseek" function for handling files greater than 2 Gb
	/** If not available (Visual 2003 and before), it points to the fseek function.
	In this case, CloudCompare won't be able to open ASCII files greater than 2 Gb.
	**/
	static int fseek64(FILE *f, __int64 depl, int pos);

	//! Shortcut to the 64 bits "ftell" function for handling files greater than 2 Gb
	/** Same as CCMiscTools::fseek64.
	**/
	static __int64 ftell64(FILE *f);

	/*** strings (char*) handling ***/

	//! Counts the number of lines in an ascii file
	/** \param filename the ascii file name
		\return the number of lines
	**/
	static unsigned fileLinesCount(const char* filename);

	//! Counts the number of spaces in a string
	/** \param string a string (as a character array)
		\param stringSize the array size
		\return the number of spaces
	**/
	static int countSpaces(const char* string, int stringSize);

	//! Counts the number of occurences of a specific character in a string
	/** \param theChar a character
		\param string a string (as a character array)
		\param stringSize the array size
		\return the number of occurences
	**/
	static int countChar(const char theChar, const char* string, int stringSize);

	//! Finds the last position of a given character in a string
	/** \param theChar a character
		\param string a string (as a null-terminated character array)
		\return the position (-1 if character hasn't been found)
	**/
	static int findCharLastOccurence(const char theChar, const char* string);

	//! Returns the string length
	/** \param string a string (as a null-terminated character array)
		\return the string length
	**/
	static int length(const char* string);

	//! Sets all characters (a-to-z) as uppercase
	/** \param string a string (as a null-terminated character array)
	**/
	static void upperCase(char* string);

	/*** Geometry ***/

	//! Proportionally enlarges a 3D box
	/** \param dimMin the upper-left corner of the box
		\param dimMax the lower-right corner of the box
		\param coef the enlargement coefficient (1.1 <-> +10%)
	**/
	static void enlargeBox(CCVector3& dimMin, CCVector3& dimMax, double coef);

	//! Transforms a 3D box into a 3D cube
	/** The cube dimensions will be equal to the largest box dimension.
		\param dimMin the upper-left corner of the rectangle
		\param dimMax the lower-right corner of the rectangle
		\param enlargeFactor the resulting box can be automatically enlarged if this parameter is greater than 0
	**/
	static void makeMinAndMaxCubical(CCVector3& dimMin, CCVector3& dimMax, double enlargeFactor=0.01);

	//! Computes base vectors for a given 3D plane
	/** Determines at least two orthogonal vectors (inside the plane) and can also
		computes the last one, orthogonal to the two others (and therefore to the
		plane).
		\param aPlane the plane eaquations (an array of 4 coefficients : ax+by+cz+d=0)
		\param u the first vector (a 3 coordinates array to be updated by the algorithm)
		\param v the second vector (a 3 coordinates array to be updated by the algorithm)
		\param n the last vector, orthogonal to the plane (optionnal - a 3 coordinates array to be updated by the algorithm)
	**/
	static void computeBaseVectors(const PointCoordinateType *aPlane, PointCoordinateType* u, PointCoordinateType* v, PointCoordinateType* n=0);

	//! Ovelap test between a 3D cubical box and a triangle
	/** \param boxcenter the box center (as a 3 coordinates array)
		\param boxhalfsize the box half size
		\param triverts the 3 summits (as 3 arrays of 3 coordinates)
		\return true if cube and triangle overlap, false otherwise
	**/
	static bool triBoxOverlap(float* boxcenter, float boxhalfsize, const CCVector3* triverts[3]);

	//! Sample points on the unit sphere
	/** As points are sampled on the unit sphere, they can be also considered
		as directions.
		WARNING: returned array is on the user responsibilty!
		\param N number of desired sampled directions
		\return an array of 3*N floats (3 floats by point)
	**/
	static float* sampleSphere(unsigned N);

};

}

#endif
