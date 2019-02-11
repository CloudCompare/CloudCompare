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

#ifndef CC_BOUNDING_BOX_HEADER
#define CC_BOUNDING_BOX_HEADER

//Local
#include "SquareMatrix.h"

namespace CCLib
{
	//! Bounding box structure
	class CC_CORE_LIB_API BoundingBox
	{
	public:

		//! Default constructor
		BoundingBox();
		//! Constructor from two vectors (lower min. and upper max. corners)
		BoundingBox(const CCVector3& minCorner, const CCVector3& maxCorner);

		//! Returns the 'sum' of this bounding-box and another one
		BoundingBox operator + (const BoundingBox& aBBox) const;
		//! In place 'sum' of this bounding-box with another one
		const BoundingBox& operator += (const BoundingBox& aBBox);
		//! Shifts the bounding box with a vector
		const BoundingBox& operator += (const CCVector3& aVector);
		//! Shifts the bounding box with a vector
		const BoundingBox& operator -= (const CCVector3& aVector);
		//! Scales the bounding box
		const BoundingBox& operator *= (PointCoordinateType scaleFactor);
		//! Rotates the bounding box
		const BoundingBox& operator *= (const SquareMatrix& aMatrix);

		//! Resets the bounding box
		/** (0,0,0) --> (0,0,0)
		**/
		void clear();

		//! 'Enlarges' the bounding box with a point
		void add(const CCVector3& aPoint);

		//! Returns min corner (const)
		inline const CCVector3& minCorner() const { return m_bbMin; }
		//! Returns max corner (const)
		inline const CCVector3& maxCorner() const { return m_bbMax; }

		//! Returns min corner
		inline CCVector3& minCorner() { return m_bbMin; }
		//! Returns max corner
		inline CCVector3& maxCorner() { return m_bbMax; }

		//! Returns center
		CCVector3 getCenter() const;
		//! Returns diagonal vector
		CCVector3 getDiagVec() const;
		//! Returns diagonal length
		inline PointCoordinateType getDiagNorm() const { return getDiagVec().norm(); }
		//! Returns diagonal length (double precision)
		double getDiagNormd() const { return getDiagVec().normd(); }
		//! Returns minimal box dimension
		PointCoordinateType getMinBoxDim() const;
		//! Returns maximal box dimension
		PointCoordinateType getMaxBoxDim() const;
		//! Returns the bounding-box volume
		double computeVolume() const;

		//! Sets bonding box validity
		inline void setValidity(bool state) { m_valid = state; }

		//! Returns whether bounding box is valid or not
		inline bool isValid() const { return m_valid; }

		//! Computes min gap (absolute distance) between this bounding-box and another one
		/** \return min gap (>=0) or -1 if at least one of the box is not valid
		**/
		PointCoordinateType minDistTo(const BoundingBox& box) const;

		//! Returns whether a points is inside the box or not
		/** Warning: box should be valid!
		**/
		inline bool contains(const CCVector3& P) const
		{
			return (P.x >= m_bbMin.x && P.x <= m_bbMax.x &&
					P.y >= m_bbMin.y && P.y <= m_bbMax.y &&
					P.z >= m_bbMin.z && P.z <= m_bbMax.z);
		}

	protected:

		//! Lower min. corner
		CCVector3 m_bbMin;
		//! Upper max. corner
		CCVector3 m_bbMax;
		//! Validity
		bool m_valid;
	};

} //namespace

#endif //CC_BOUNDING_BOX_HEADER
