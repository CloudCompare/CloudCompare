//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
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
//$Author:: dgm                                                            $
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

#ifndef CC_BBOX_HEADER
#define CC_BBOX_HEADER

//CCLib
#include <CCGeom.h>
#include <Matrix.h>

#include "ccGLMatrix.h"
#include "ccBasicTypes.h"

//! Bounding box structure
/** Supports several operators such addition (to a matrix or a vector) and
    multiplication (by a matrix or a scalar).
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccBBox
#else
class ccBBox
#endif
{
public:

    //! Default constructor
    ccBBox();
    //! Copy constructor
    ccBBox(const ccBBox& aBBox);
    //! Constructor from two vectors (lower min. and upper max. corners)
    ccBBox(const CCVector3 &bbMinCorner, const CCVector3 &bbMaxCorner);

    //! Returns the 'sum' of this bounding-box and another one
    ccBBox operator + (const ccBBox& aBBox) const;
    //! In place 'sum' of this bounding-box with another one
    const ccBBox& operator += (const ccBBox& aBBox);
    //! Shifts the bounding box with a vector
    const ccBBox& operator += (const CCVector3& aVector);
    //! Shifts the bounding box with a vector
    const ccBBox& operator -= (const CCVector3& aVector);
    //! Scales the bounding box
    const ccBBox& operator *= (const PointCoordinateType& scaleFactor);
    //! Rotates the bounding box
    const ccBBox& operator *= (const CCLib::SquareMatrix& aMatrix);
    //! Applies transformation to the bounding box
    const ccBBox& operator *= (const ccGLMatrix& mat);

    //! Resets the bounding box
    /** (0,0,0) --> (0,0,0)
    **/
    void clear();

    //! 'Enlarges' the bounding box with a vector
    /** equivalent to operator +=(CCVector3)
    **/
    void add(const CCVector3& aVector);

    //! Returns min corner (const)
    const CCVector3& minCorner() const;
    //! Returns max corner (const)
    const CCVector3& maxCorner() const;

    //! Returns min corner
    CCVector3& minCorner();
    //! Returns max corner
    CCVector3& maxCorner();

    //! Returns center
    CCVector3 getCenter() const;
    //! Returns diagonal vector
    CCVector3 getDiagVec() const;
    //! Returns diagonal length
    PointCoordinateType getDiagNorm() const;

    //! Draws bounding box (OpenGL)
    /** \param col (R,G,B) color
    **/
    void draw(const colorType col[]) const;

    //! Sets bonding box validity
    void setValidity(bool state);

    //! Returns whether bounding box is valid or not
    bool isValid() const;

protected:

    //! Lower min. corner
    CCVector3 bbMin;
    //! Upper max. corner
    CCVector3 bbMax;
    //! Validity
    bool valid;
};

#endif
