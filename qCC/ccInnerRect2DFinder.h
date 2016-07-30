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

#ifndef CC_INNER_RECT_2D_FINDER_HEADER
#define CC_INNER_RECT_2D_FINDER_HEADER

//qCC_db
#include <ccBox.h>
#include <ccGenericPointCloud.h>

//! Finds a the biggets enclosed rectangle in a point cloud (2D)
class ccInnerRect2DFinder
{

public:

	//! Default constructor
	ccInnerRect2DFinder();

	//! Finds the biggest enclosed rectangle
	ccBox* process(	ccGenericPointCloud* cloud, unsigned char zDim = 2 );

protected:

	//! Initializes internal structures
	bool init(ccGenericPointCloud* cloud, unsigned char zDim);

	//! 2D rectangle
	struct Rect
	{
		Rect() : x0(0),y0(0),x1(0),y1(0) {}
		Rect(double _x0, double _y0, double _x1, double _y1) : x0(_x0),y0(_y0),x1(_x1),y1(_y1) {}

		double x0, y0, x1, y1;

		inline double width() const { return x1 - x0; }
		inline double height() const { return y1 - y0; }
		inline double area() const { return width() * height(); }
	};

	//! Internal processs
	void findBiggestRect(const Rect& rect, unsigned startIndex);

	//! Global rectangle
	Rect m_boundingRect;

	//! Inner rectangle
	Rect m_maxRect;
	//! Inner rectangle max area
	double m_maxArea;

	//! Associated cloud
	ccGenericPointCloud* m_cloud;

	//! X dimension
	unsigned char m_X;
	//! Y dimension
	unsigned char m_Y;
};

#endif //CC_INNER_RECT_2D_FINDER_HEADER
