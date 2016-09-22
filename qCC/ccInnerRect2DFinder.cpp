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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccInnerRect2DFinder.h"

//qCC_db
#include <ccLog.h>

//system
#include <assert.h>

ccInnerRect2DFinder::ccInnerRect2DFinder()
	: m_maxArea(0)
	, m_cloud(0)
	, m_X(0)
	, m_Y(1)
{}

ccBox* ccInnerRect2DFinder::process( ccGenericPointCloud* cloud, unsigned char zDim/*=2*/ )
{
	if (!init(cloud,zDim))
		return 0;

	//Find the 'biggest' rectangle
	m_maxRect = Rect();
	m_maxArea = 0;
	findBiggestRect(m_boundingRect,0);

	ccBox* resultBox = 0;
	if (m_maxArea > 0)
	{
		ccBBox bbox = cloud->getOwnBB();
		assert(bbox.isValid());

		//box size
		CCVector3 boxDim = bbox.getDiagVec();
		boxDim.u[m_X] = static_cast<PointCoordinateType>(m_maxRect.width());
		boxDim.u[m_Y] = static_cast<PointCoordinateType>(m_maxRect.height());

		CCVector3 boxCenter = bbox.getCenter();
		boxCenter.u[m_X] = static_cast<PointCoordinateType>((m_maxRect.x0 + m_maxRect.x1)/2);
		boxCenter.u[m_Y] = static_cast<PointCoordinateType>((m_maxRect.y0 + m_maxRect.y1)/2);

		ccGLMatrix shiftMat;
		shiftMat.setTranslation(boxCenter);
		resultBox = new ccBox(boxDim,&shiftMat,"Biggest rect");
	}

	return resultBox;
}

bool ccInnerRect2DFinder::init(ccGenericPointCloud* cloud, unsigned char zDim)
{
	if (!cloud || cloud->size() == 0)
	{
		ccLog::Error("[ccInnerRect2DFinder] Invalid input cloud");
		return false;
	}

	ccBBox bbox = cloud->getOwnBB();
	if (!bbox.isValid())
	{
		ccLog::Error("[ccInnerRect2DFinder] Invalid input cloud");
		return false;
	}

	if (zDim > 2)
	{
		ccLog::Error("[ccInnerRect2DFinder] Invalid input parameter (zDim)");
		return false;
	}

	unsigned char Z = zDim;
	unsigned char X = ((Z+1) % 3);
	unsigned char Y = ((X+1) % 3);

	m_cloud = cloud;
	m_X = X;
	m_Y = Y;

	//init bounding rectangle
	{
		const CCVector3* P0 = m_cloud->getPoint(0);
		m_boundingRect = Rect(P0->u[m_X],P0->u[m_Y],P0->u[m_X],P0->u[m_Y]);

		unsigned pointCloud = m_cloud->size();
		for (unsigned i=1; i<pointCloud; ++i)
		{
			const CCVector3* P = m_cloud->getPoint(i);
			if (P->u[m_X] < m_boundingRect.x0)
				m_boundingRect.x0 = P->u[m_X];
			else if (P->u[m_X] > m_boundingRect.x1)
				m_boundingRect.x1 = P->u[m_X];

			if (P->u[m_Y] < m_boundingRect.y0)
				m_boundingRect.y0 = P->u[m_Y];
			else if (P->u[m_Y] > m_boundingRect.y1)
				m_boundingRect.y1 = P->u[m_Y];
		}
	}

	return true;
}

void ccInnerRect2DFinder::findBiggestRect(const Rect& rect, unsigned startIndex)
{
	assert(m_cloud);

	//test if at least one point falls inside the input rectangle
	const CCVector3* Pinside = 0;
	{
		unsigned pointCount = m_cloud->size();
		double minSquareDistToCenter = -1.0;
		double xc = (rect.x0+rect.x1)/2;
		double yc = (rect.y0+rect.y1)/2;
		for (unsigned i=startIndex; i<pointCount; ++i)
		{
			const CCVector3* P = m_cloud->getPoint(i);
			if (	P->u[m_X] > rect.x0 && P->u[m_X] < rect.x1	//strict inequalities!
				&&	P->u[m_Y] > rect.y0 && P->u[m_Y] < rect.y1 )
			{
				double dist2 = (xc - P->u[m_X])*(xc - P->u[m_X]) + (yc - P->u[m_Y])*(yc - P->u[m_Y]);
				if (minSquareDistToCenter < 0)
				{
					Pinside = P;
					minSquareDistToCenter = dist2;
					startIndex = i;
				}
				else if (dist2 < minSquareDistToCenter)
				{
					Pinside = P;
					minSquareDistToCenter = dist2;
				}
				//break;
			}
		}
	}

	//do we have an empty rectangle?
	if (!Pinside)
	{
		//we remember it only if its size is bigger
		double surf = rect.area();
		if (surf > m_maxArea)
		{
			m_maxArea = surf;
			m_maxRect = rect;
		}
	}
	else //otherwise we test the 4 sub-rectangles
	{
		//left sub-rectangle
		Rect r(rect.x0,rect.y0,Pinside->u[m_X],rect.y1);
		if (r.area() > m_maxArea)
			findBiggestRect(r,startIndex);
		//right sub-rectangle
		r = Rect(Pinside->u[m_X],rect.y0,rect.x1,rect.y1);
		if (r.area() > m_maxArea)
			findBiggestRect(r,startIndex);
		//upper sub-rectangle
		r = Rect(rect.x0,rect.y0,rect.x1,Pinside->u[m_Y]);
		if (r.area() > m_maxArea)
			findBiggestRect(r,startIndex);
		//lower sub-rectangle
		r = Rect(rect.x0,Pinside->u[m_Y],rect.x1,rect.y1);
		if (r.area() > m_maxArea)
			findBiggestRect(r,startIndex);
	}
}