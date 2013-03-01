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

#ifndef CC_MISC_TOOLS_HEADER
#define CC_MISC_TOOLS_HEADER

//local
#include "CCToolbox.h"
#include "CCGeom.h"

namespace CCLib
{

	//! Miscellaneous useful functions (geometrical elements handling)
	#ifdef CC_USE_AS_DLL
	#include "CloudCompareDll.h"
	class CC_DLL_API CCMiscTools : public CCToolbox
	#else
	class CCMiscTools : public CCToolbox
	#endif
	{
	public:

		//! Proportionally enlarges a 3D box
		/** \param dimMin the upper-left corner of the box
			\param dimMax the lower-right corner of the box
			\param coef the enlargement coefficient (1.1 <-> +10%)
		**/
		static void EnlargeBox(CCVector3& dimMin, CCVector3& dimMax, double coef);

		//! Transforms a 3D box into a 3D cube
		/** The cube dimensions will be equal to the largest box dimension.
			\param dimMin the upper-left corner of the rectangle
			\param dimMax the lower-right corner of the rectangle
			\param enlargeFactor the resulting box can be automatically enlarged if this parameter is greater than 0
		**/
		static void MakeMinAndMaxCubical(CCVector3& dimMin, CCVector3& dimMax, double enlargeFactor=0.01);

		//! Computes base vectors for a given 3D plane
		/** Determines at least two orthogonal vectors (inside the plane) and can also
			computes the last one, orthogonal to the two others (and therefore to the
			plane).
			\param aPlane the plane eaquations (an array of 4 coefficients : ax+by+cz+d=0)
			\param u the first vector (a 3 coordinates array to be updated by the algorithm)
			\param v the second vector (a 3 coordinates array to be updated by the algorithm)
			\param n the last vector, orthogonal to the plane (optionnal - a 3 coordinates array to be updated by the algorithm)
		**/
		static void ComputeBaseVectors(const PointCoordinateType *aPlane, PointCoordinateType* u, PointCoordinateType* v, PointCoordinateType* n=0);

		//! Ovelap test between a 3D cubical box and a triangle
		/** \param boxcenter the box center (as a 3 coordinates array)
			\param boxhalfsize the box half size
			\param triverts the 3 summits (as 3 arrays of 3 coordinates)
			\return true if cube and triangle overlap, false otherwise
		**/
		static bool TriBoxOverlap(float* boxcenter, float boxhalfsize, const CCVector3* triverts[3]);

		//! Sample points on the unit sphere
		/** As points are sampled on the unit sphere, they can be also considered
			as directions.
			WARNING: returned array is on the user responsibilty!
			\param N number of desired sampled directions
			\return an array of 3*N floats (3 floats by point)
		**/
		static float* SampleSphere(unsigned N);

	};

}

#endif //CC_MISC_TOOLS_HEADER
