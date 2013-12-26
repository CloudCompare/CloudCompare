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

//Always first
#include "ccIncludeGL.h"

#include "ccBBox.h"

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

PointCoordinateType ccBBox::getMinBoxDim() const
{
	CCVector3 V = getDiagVec();

	return std::min(V.x,std::min(V.y,V.z));
}

PointCoordinateType ccBBox::getMaxBoxDim() const
{
	CCVector3 V = getDiagVec();

	return std::max(V.x,std::max(V.y,V.z));
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
    ccGL::Vertex3v(bbMin.u);
    ccGL::Vertex3(bbMax.x,bbMin.y,bbMin.z);
    ccGL::Vertex3(bbMax.x,bbMax.y,bbMin.z);
    ccGL::Vertex3(bbMin.x,bbMax.y,bbMin.z);
    glEnd();

    glBegin(GL_LINE_LOOP);
    ccGL::Vertex3(bbMin.x,bbMin.y,bbMax.z);
    ccGL::Vertex3(bbMax.x,bbMin.y,bbMax.z);
    ccGL::Vertex3v(bbMax.u);
    ccGL::Vertex3(bbMin.x,bbMax.y,bbMax.z);
    glEnd();

    glBegin(GL_LINES);
    ccGL::Vertex3v(bbMin.u);
    ccGL::Vertex3(bbMin.x,bbMin.y,bbMax.z);
    ccGL::Vertex3(bbMax.x,bbMin.y,bbMin.z);
    ccGL::Vertex3(bbMax.x,bbMin.y,bbMax.z);
    ccGL::Vertex3(bbMax.x,bbMax.y,bbMin.z);
    ccGL::Vertex3v(bbMax.u);
    ccGL::Vertex3(bbMin.x,bbMax.y,bbMin.z);
    ccGL::Vertex3(bbMin.x,bbMax.y,bbMax.z);
    glEnd();
}

ccBBox ccBBox::operator + (const ccBBox& aBBox) const
{
    if (!valid)
        return ccBBox(aBBox);

    ccBBox tempBox;

    tempBox.bbMin.x = std::min(bbMin.x, aBBox.bbMin.x);
    tempBox.bbMin.y = std::min(bbMin.y, aBBox.bbMin.y);
    tempBox.bbMin.z = std::min(bbMin.z, aBBox.bbMin.z);
    tempBox.bbMax.x = std::max(bbMax.x, aBBox.bbMax.x);
    tempBox.bbMax.y = std::max(bbMax.y, aBBox.bbMax.y);
    tempBox.bbMax.z = std::max(bbMax.z, aBBox.bbMax.z);

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
        bbMin.x = std::min(bbMin.x, aBBox.bbMin.x);
        bbMin.y = std::min(bbMin.y, aBBox.bbMin.y);
        bbMin.z = std::min(bbMin.z, aBBox.bbMin.z);
        bbMax.x = std::max(bbMax.x, aBBox.bbMax.x);
        bbMax.y = std::max(bbMax.y, aBBox.bbMax.y);
        bbMax.z = std::max(bbMax.z, aBBox.bbMax.z);
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

PointCoordinateType ccBBox::minDistTo(const ccBBox& box) const
{
    if (valid && box.isValid())
    {
		CCVector3 d(0,0,0);

		for (unsigned char dim=0; dim<3; ++dim)
		{
			//if the boxes overlap in one dimension, the distance is zero (in this dimension)
			if (box.bbMin.u[dim] > bbMax.u[dim])
				d.u[dim] = box.bbMin.u[dim] - bbMax.u[dim];
			else if (box.bbMax.u[dim] < bbMin.u[dim])
				d.x = bbMin.u[dim] - box.bbMax.u[dim];
		}

		return d.norm();
    }
    else
    {
		return (PointCoordinateType)-1.0;
    }
}
