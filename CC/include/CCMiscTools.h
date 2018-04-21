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

#ifndef CC_MISC_TOOLS_HEADER
#define CC_MISC_TOOLS_HEADER

//Local
#include "CCGeom.h"
#include "CCToolbox.h"

namespace CCLib
{
	//! Miscellaneous useful functions (geometrical elements handling)
	class CC_CORE_LIB_API CCMiscTools : public CCToolbox
	{
	public:

		//! Proportionally enlarges a 3D box
		/** \param dimMin the upper-left corner of the box
			\param dimMax the lower-right corner of the box
			\param coef the enlargement coefficient (1.1 <-> +10%)
		**/
		static void EnlargeBox(	CCVector3& dimMin,
								CCVector3& dimMax,
								double coef);

		//! Transforms a 3D box into a 3D cube
		/** The cube dimensions will be equal to the largest box dimension.
			\param dimMin the upper-left corner of the rectangle
			\param dimMax the lower-right corner of the rectangle
			\param enlargeFactor the resulting box can be automatically enlarged if this parameter is greater than 0
		**/
		static void MakeMinAndMaxCubical(	CCVector3& dimMin,
											CCVector3& dimMax,
											double enlargeFactor = 0.01);

		//! Computes base vectors for a given 3D plane
		/** Determines at least two orthogonal vectors perpendicular to a third one
			\param[in] N a non null vector
			\param[out] X the first vector (a 3 coordinates array to be updated by the algorithm)
			\param[out] Y the second vector (a 3 coordinates array to be updated by the algorithm)
		**/
		static void ComputeBaseVectors(	const CCVector3 &N,
										CCVector3& X,
										CCVector3& Y);
		//! Computes base vectors for a given 3D plane - double version
		/** Determines at least two orthogonal vectors perpendicular to a third one
			\param[in] N a non null vector
			\param[out] X the first vector (a 3 coordinates array to be updated by the algorithm)
			\param[out] Y the second vector (a 3 coordinates array to be updated by the algorithm)
		**/
		static void ComputeBaseVectors(	const CCVector3d &N,
										CCVector3d& X,
										CCVector3d& Y);

		//! Ovelap test between a 3D box and a triangle
		/** \param boxcenter the box center
		\param boxhalfsize the box half dimensions
		\param triverts the 3 triangle vertices
		\return true if the input box and triangle overlap, false otherwise
		**/
		static bool TriBoxOverlap(const CCVector3& boxcenter,
			const CCVector3& boxhalfsize,
			const CCVector3* triverts[3]);

		//! Ovelap test between a 3D box and a triangle (double version)
		/** \param boxcenter the box center
			\param boxhalfsize the box half dimensions
			\param triverts the 3 triangle vertices
			\return true if the input box and triangle overlap, false otherwise
		**/
		static bool TriBoxOverlapd(const CCVector3d& boxcenter,
			const CCVector3d& boxhalfsize,
			const CCVector3d triverts[3]);

	};

}

#endif //CC_MISC_TOOLS_HEADER
