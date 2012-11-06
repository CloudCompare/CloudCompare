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

#include "ccBBox.h"
#include "ccIncludeGL.h"

#include <CCMiscTools.h>

ccBBox::ccBBox()
{
    clear();
}

ccBBox::ccBBox(const CCVector3 &bbMinCorner, const CCVector3 &bbMaxCorner)
	: bbMin(bbMinCorner)
	, bbMax(bbMaxCorner)
	, valid(true)
{
}

ccBBox::ccBBox(const ccBBox& aBox)
	: bbMin(aBox.bbMin)
	, bbMax(aBox.bbMax)
	, valid(aBox.valid)
{
}

void ccBBox::clear()
{
    bbMin.x = bbMin.y = bbMin.z = 0.0;
    bbMax.x = bbMax.y = bbMax.z = 0.0;
    valid = false;
}

CCVector3 ccBBox::getCenter() const
{
	return (bbMax+bbMin)*0.5;
}

CCVector3 ccBBox::getDiagVec() const
{
    return bbMax-bbMin;
}

PointCoordinateType ccBBox::getDiagNorm() const
{
	return getDiagVec().norm();
}

void ccBBox::setValidity(bool state)
{
    valid = state;
}

bool ccBBox::isValid() const
{
    return valid;
}

const CCVector3& ccBBox::minCorner() const
{
    return bbMin;
}

const CCVector3& ccBBox::maxCorner() const
{
    return bbMax;
}

CCVector3& ccBBox::minCorner()
{
    return bbMin;
}

CCVector3& ccBBox::maxCorner()
{
    return bbMax;
}

void ccBBox::draw(const colorType col[]) const
{
    if (!valid)
        return;

    glColor3ubv(col);

    glBegin(GL_LINE_LOOP);
    glVertex3fv(bbMin.u);
    glVertex3f(bbMax.x,bbMin.y,bbMin.z);
    glVertex3f(bbMax.x,bbMax.y,bbMin.z);
    glVertex3f(bbMin.x,bbMax.y,bbMin.z);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3f(bbMin.x,bbMin.y,bbMax.z);
    glVertex3f(bbMax.x,bbMin.y,bbMax.z);
    glVertex3fv(bbMax.u);
    glVertex3f(bbMin.x,bbMax.y,bbMax.z);
    glEnd();

    glBegin(GL_LINES);
    glVertex3fv(bbMin.u);
    glVertex3f(bbMin.x,bbMin.y,bbMax.z);
    glVertex3f(bbMax.x,bbMin.y,bbMin.z);
    glVertex3f(bbMax.x,bbMin.y,bbMax.z);
    glVertex3f(bbMax.x,bbMax.y,bbMin.z);
    glVertex3fv(bbMax.u);
    glVertex3f(bbMin.x,bbMax.y,bbMin.z);
    glVertex3f(bbMin.x,bbMax.y,bbMax.z);
    glEnd();
}

ccBBox ccBBox::operator + (const ccBBox& aBBox) const
{
    if (!valid)
        return ccBBox(aBBox);

    ccBBox tempBox;

    tempBox.bbMin.x = ccMin(bbMin.x, aBBox.bbMin.x);
    tempBox.bbMin.y = ccMin(bbMin.y, aBBox.bbMin.y);
    tempBox.bbMin.z = ccMin(bbMin.z, aBBox.bbMin.z);
    tempBox.bbMax.x = ccMax(bbMax.x, aBBox.bbMax.x);
    tempBox.bbMax.y = ccMax(bbMax.y, aBBox.bbMax.y);
    tempBox.bbMax.z = ccMax(bbMax.z, aBBox.bbMax.z);

    return tempBox;
}

const ccBBox& ccBBox::operator += (const ccBBox& aBBox)
{
    if (!valid)
    {
        *this = aBBox;
    }
    else if (aBBox.isValid())
    {
        bbMin.x = ccMin(bbMin.x, aBBox.bbMin.x);
        bbMin.y = ccMin(bbMin.y, aBBox.bbMin.y);
        bbMin.z = ccMin(bbMin.z, aBBox.bbMin.z);
        bbMax.x = ccMax(bbMax.x, aBBox.bbMax.x);
        bbMax.y = ccMax(bbMax.y, aBBox.bbMax.y);
        bbMax.z = ccMax(bbMax.z, aBBox.bbMax.z);
    }

    return *this;
}

const ccBBox& ccBBox::operator += (const CCVector3& aVector)
{
    if (valid)
    {
        bbMin += aVector;
        bbMax += aVector;
    }

    return *this;
}

const ccBBox& ccBBox::operator -= (const CCVector3& aVector)
{
    if (valid)
    {
        bbMin -= aVector;
        bbMax -= aVector;
    }

    return *this;
}

const ccBBox& ccBBox::operator *= (const PointCoordinateType& scaleFactor)
{
    if (valid)
    {
        bbMin *= scaleFactor;
        bbMax *= scaleFactor;
    }

    return *this;
}

void ccBBox::add(const CCVector3& aVector)
{
    if (valid)
    {
        if (aVector.x<bbMin.x)
            bbMin.x = aVector.x;
        else if (aVector.x>bbMax.x)
            bbMax.x = aVector.x;

        if (aVector.y<bbMin.y)
            bbMin.y = aVector.y;
        else if (aVector.y>bbMax.y)
            bbMax.y = aVector.y;

        if (aVector.z<bbMin.z)
            bbMin.z = aVector.z;
        else if (aVector.z>bbMax.z)
            bbMax.z = aVector.z;
    }
    else
    {
        bbMax = bbMin = aVector;
        valid = true;
    }
}

const ccBBox& ccBBox::operator *= (const CCLib::SquareMatrix& mat)
{
    if (valid)
    {
        CCVector3 boxCorners[8];

        boxCorners[0] = bbMin;
        boxCorners[1] = CCVector3(bbMin.x,bbMin.y,bbMax.z);
        boxCorners[2] = CCVector3(bbMin.x,bbMax.y,bbMin.z);
        boxCorners[3] = CCVector3(bbMax.x,bbMin.y,bbMin.z);
        boxCorners[4] = bbMax;
        boxCorners[5] = CCVector3(bbMin.x,bbMax.y,bbMax.z);
        boxCorners[6] = CCVector3(bbMax.x,bbMax.y,bbMin.z);
        boxCorners[7] = CCVector3(bbMax.x,bbMin.y,bbMax.z);

        clear();

        for (int i=0;i<8;++i)
            add(mat*boxCorners[i]);
    }

    return *this;
}

const ccBBox& ccBBox::operator *= (const ccGLMatrix& mat)
{
    if (valid)
    {
        CCVector3 boxCorners[8];

        boxCorners[0] = bbMin;
        boxCorners[1] = CCVector3(bbMin.x,bbMin.y,bbMax.z);
        boxCorners[2] = CCVector3(bbMin.x,bbMax.y,bbMin.z);
        boxCorners[3] = CCVector3(bbMax.x,bbMin.y,bbMin.z);
        boxCorners[4] = bbMax;
        boxCorners[5] = CCVector3(bbMin.x,bbMax.y,bbMax.z);
        boxCorners[6] = CCVector3(bbMax.x,bbMax.y,bbMin.z);
        boxCorners[7] = CCVector3(bbMax.x,bbMin.y,bbMax.z);

        clear();

        for (int i=0;i<8;++i)
            add(mat*boxCorners[i]);
    }

    return *this;
}
