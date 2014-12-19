#ifndef ISOLINES_HEADER
#define ISOLINES_HEADER

/**
*  Transcription of FindIsolines.java for C++
*
*  Fast implementation of marching squares
*
*  AUTHOR:   Murphy Stein, Greg Borenstein
*            New York University
*  CREATED:  Jan-Sept 2012 
*  MODIFIED: Dec 2014 (DGM)
*
*  LICENSE:  BSD
*
*  Copyright (c) 2012 New York University.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms are permitted
*  provided that the above copyright notice and this paragraph are
*  duplicated in all such forms and that any documentation,
*  advertising materials, and other materials related to such
*  distribution and use acknowledge that the software was developed
*  by New York Univserity.  The name of the
*  University may not be used to endorse or promote products derived
*  from this software without specific prior written permission.
*  THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
*  IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* 
*/

/**
* This is a fast implementation of the marching squares algorithm for finding isolines (lines of equal color) in an image.
* 
*/

//qCC_db
#include <ccLog.h>

//system
#include <assert.h>
#include <vector>
#include <math.h>

template< typename T > class Isolines
{
protected:
	std::vector<double> m_minx;
	std::vector<double> m_miny;
	std::vector<double> m_maxx;
	std::vector<double> m_maxy;

	std::vector<int>    m_cd;
	std::vector<double> m_cx;
	std::vector<double> m_cy;
	std::vector<int>    m_cl;
	std::vector<int>    m_co;

	int m_w;
	int m_h;
	int m_numContours;

	T m_threshold;

public:

	//! Default constructor
	Isolines(int w, int h)
		: m_w(w)
		, m_h(h)
		, m_threshold(0)
		, m_numContours(0)
	{
		//DGM: as this is done in the constructor,
		//we don't catch the exception (so that the
		//caller can properly handle the error!)
		//try
		{
			m_cd.resize(w*h,0);
		}
		//catch(std::bad_alloc)
		//{
		//	//not enough memory!
		//}
	}

	//! Sets isoline value to trace
	inline void setThreshold(T t) { m_threshold = t; }

	//! Find isolines
	int find(const T* in)
	{
		//createOnePixelBorder(in, m_threshold + 1); //DGM: modifies 'in', the user will have to do it himself :(
		preCodeImage(in);
		return findIsolines(in);
	}

	//! Returns the number of found contours
	inline int getNumContours() const { return m_numContours; }

	//! Returns the length of a given contour
	inline int getContourLength(int contour) const  { return m_cl[contour]; }

	//! Returns the given point (x,y) of a given contour
	void getContourPoint(int contour, size_t index, double& x, double& y)
	{
		assert(static_casti<int>(index) < getContourLength(contour));
		x = getContourX(contour, index);
		y = getContourY(contour, index);
	}

	//! Measures the area delineated by a given contour
	inline double measureArea(int contour) const { return measureArea(contour, 0, getContourLength(contour)); }

	//! Measures the perimeter of a given contour
	inline double measurePerimeter(int contour) const { return measurePerimeter(contour, 0, getContourLength(contour)); }

	//! Creates a single pixel, 0-valued border around the grid
	void createOnePixelBorder(T* in, T borderval) const
	{
		for (int i = 0, j = m_w*m_h-1; i < m_w; i++, j--)
		{
			in[i] = in[j] = borderval;
		}
		for (int i = 0, j = m_w-1; i < m_h; i++, j+=m_w)
		{
			in[i] = in[j] = borderval;
		}
	}

protected:

	//! Computes a code for each group of 4x4 cells
	/** The code depends only on whether each of four corners is
		above or below threshold.
	**/
	void preCodeImage(const T* in)
	{
		for (int x = 0; x < m_w-1; x++)
		{
			for (int y = 0; y < m_h-1; y++)
			{
				int b0(in[ixy(x + 0, y + 0)] < m_threshold ? 0x00001000 : 0x00000000;
				int b1(in[ixy(x + 1, y + 0)] < m_threshold ? 0x00000100 : 0x00000000;
				int b2(in[ixy(x + 1, y + 1)] < m_threshold ? 0x00000010 : 0x00000000;
				int b3(in[ixy(x + 0, y + 1)] < m_threshold ? 0x00000001 : 0x00000000;
				m_cd[ixy(x, y)] = b0 | b1 | b2 | b3;
			}
		} 
	}

	static const int CASE0	=	0x00000000; 
	static const int CASE1	=	0x00000001;
	static const int CASE2	=	0x00000010;
	static const int CASE3	=	0x00000011;
	static const int CASE4	=	0x00000100;
	static const int CASE5	=	0x00000101;
	static const int CASE6	=	0x00000110;
	static const int CASE7	=	0x00000111;
	static const int CASE8	=	0x00001000;
	static const int CASE9	=	0x00001001;
	static const int CASE10	=	0x00001010;
	static const int CASE11	=	0x00001011;
	static const int CASE12	=	0x00001100;
	static const int CASE13	=	0x00001101;
	static const int CASE14	=	0x00001110;
	static const int CASE15	=	0x00001111;

	////////////////////////////////////////////////////
	// SEARCH image for contours from topleft corner
	// to bottomright corner.
	////////////////////////////////////////////////////    
	int findIsolines(const T* in)
	{
		m_cx.clear();
		m_cy.clear();
		m_cl.clear();
		m_co.clear();

		try
		{
			int cellIndex = 0;
			int toedge = -1;
			int nextCellIndex = -1;
			int length = 0;

			while (cellIndex < m_w * m_h)
			{
				int fromedge = toedge;
				if (nextCellIndex < 0)
					nextCellIndex = ++cellIndex;
				int x = nextCellIndex % m_w;
				int y = nextCellIndex / m_w;
			
				if (x+1 >= m_w || y+1 >= m_h)
				{
					nextCellIndex = -1;
					continue;
				}

				int code = m_cd[nextCellIndex];
				switch (code)
				{
				case CASE0:									// CASE 0
					toedge = -1;
					break;

				case CASE1:									// CASE 1
					m_cd[nextCellIndex] = 0;
					toedge = 2;
					break;

				case CASE2:									// CASE 2
					m_cd[nextCellIndex] = 0;
					toedge = 1;
					break;

				case CASE3:									// CASE 3
					m_cd[nextCellIndex] = 0;
					toedge = 1;
					break;

				case CASE4:									// CASE 4
					m_cd[nextCellIndex] = 0;
					toedge = 0;
					break;

				case CASE5:									// CASE 5, saddle
					{
						double avg = 0.25 * (	in[ixy(x + 0, y + 0)]
											+	in[ixy(x + 1, y + 0)]
											+	in[ixy(x + 0, y + 1)]
											+	in[ixy(x + 1, y + 1)] );
					
						if (avg > m_threshold)
						{
							if (fromedge == 3)				// treat as case 1, then switch code to case 4
							{
								toedge = 2;
								m_cd[nextCellIndex] = CASE4;
							}
							else							// treat as case 4, then switch code to case 1
							{
								toedge = 0;
								m_cd[nextCellIndex] = CASE1;
							}
						}
						else
						{
							if (fromedge == 3)				// treat as case 7, then switch code to case 13
							{
								toedge = 0;
								m_cd[nextCellIndex] = CASE13;
							}
							else							// treat as case 13, then switch code to case 7
							{
								toedge = 2;
								m_cd[nextCellIndex] = CASE7;
							}
						}
					}
					break;

				case CASE6:									// CASE 6
					m_cd[nextCellIndex] = 0;
					toedge = 0;
					break;

				case CASE7:									// CASE 7
					m_cd[nextCellIndex] = 0;
					toedge = 0;
					break;

				case CASE8:									// CASE 8
					m_cd[nextCellIndex] = 0;
					toedge = 3;
					break;

				case CASE9:									// CASE 9
					m_cd[nextCellIndex] = 0;
					toedge = 2;
					break;

				case CASE10:								// CASE 10, saddle
					{
						double avg = 0.25 * (	in[ixy(x + 0, y + 0)]
											+	in[ixy(x + 1, y + 0)]
											+	in[ixy(x + 0, y + 1)]
											+	in[ixy(x + 1, y + 1)] );
					
						if (avg > m_threshold)
						{
							if (fromedge == 0)				// treat as case 8, then switch code to case 2
							{
								toedge = 3;
								m_cd[nextCellIndex] = CASE2;
							}
							else							// treat as case 2, then switch code to case 8
							{
								toedge = 1;
								m_cd[nextCellIndex] = CASE8;
							}
						}
						else
						{
							if (fromedge == 2)				// treat as case 14, then switch code to case 11
							{
								toedge = 3;
								m_cd[nextCellIndex] = CASE11;
							}
							else							// treat as case 11, then switch code to case 14
							{
								toedge = 1;
								m_cd[nextCellIndex] = CASE14;
							}
						}
					}
					break;

				case CASE11:								// CASE 11
					m_cd[nextCellIndex] = 0;
					toedge = 1;
					break;

				case CASE12:								// CASE 12
					m_cd[nextCellIndex] = 0;
					toedge = 3;
					break;

				case CASE13:								// CASE 13
					m_cd[nextCellIndex] = 0;
					toedge = 2;
					break;

				case CASE14:								// CASE 14
					m_cd[nextCellIndex] = 0;
					toedge = 3;
					break;

				case CASE15:								// CASE 15
					toedge = -1;
					break;

				default:
					assert(false);
					break;
				}

				if (fromedge == -1 && toedge > -1)
				{
					// starting a new contour
					m_cl.push_back(0);
					m_co.push_back(static_cast<int>(m_cx.size()));
					//ccLog::Print(QString("New contour: #%1 - origin = %2 - (x=%3, y=%4)").arg(m_cl.size()).arg(m_co.back()).arg(x).arg(y));
				}

				double x2 = 0.0, y2 = 0.0;
				switch (toedge)
				{
				case 0: 
					x2 = x + LERP(in[ixy(x + 0, y + 0)], in[ixy(x + 1, y + 0)]);
					y2 = y;
					nextCellIndex = ixy(x + 0, y - 1);
					break;
				case 1:
					x2 = x + 1;
					y2 = y + LERP(in[ixy(x + 1, y + 0)], in[ixy(x + 1, y + 1)]);
					nextCellIndex = ixy(x + 1, y + 0);
					break;
				case 2:
					x2 = x + LERP(in[ixy(x + 0, y + 1)], in[ixy(x + 1, y + 1)]);
					y2 = y + 1;
					nextCellIndex = ixy(x, y + 1);
					break;
				case 3:
					x2 = x;
					y2 = y + LERP(in[ixy(x + 0, y + 0)], in[ixy(x + 0, y + 1)]);
					nextCellIndex = ixy(x - 1, y + 0);
					break;
				default:
					nextCellIndex = -1;
					length = 0;
					continue;
				}

				if (length > 1)
				{
					size_t vertCount = m_cx.size();
					const double& x0 = m_cx[vertCount-2];
					const double& y0 = m_cy[vertCount-2];
					double& x1 = m_cx.back();
					double& y1 = m_cy.back();
					double ux = x1 - x0;
					double uy = y1 - y0;
					double vx = x2 - x0;
					double vy = y2 - y0;
					//test colinearity so as to merge both segments if possible
					double dotprod = (ux*vx + uy*vy);
					if (fabsl(dotprod - sqrt((vx*vx+vy*vy) * (ux*ux+uy*uy))) < 1.0e-6)
					{
						//merge: we replace the last vertex by this one
						x1 = x2;
						y1 = y2;
					}
					else
					{
						//new vertex
						m_cx.push_back(x2);
						m_cy.push_back(y2);
						m_cl.back() = ++length;
					}
				}
				else
				{
					//new vertex
					m_cx.push_back(x2);
					m_cy.push_back(y2);
					m_cl.back() = ++length;
				}
			}
		}
		catch(std::bad_alloc)
		{
			//not enough memory
			m_cx.clear();
			m_cy.clear();
			m_cl.clear();
			return -1;
		}

		m_numContours = static_cast<int>(m_cl.size());

		computeBoundingBoxes();

		return m_numContours;
	}

	////////////////////////////////////////////////////
	// LERP between to values
	////////////////////////////////////////////////////    
	inline double LERP(T A, T B) const
	{
		T AB = A-B;
		return AB == 0 ? 0 : static_cast<double>(A - m_threshold) / AB;
	}

	inline int getLastIndex() const
	{
		int nc = getNumContours();
		return nc > 0 ? m_co[nc - 1] + m_cl[nc - 1] : 0;
	}

	inline void setContourX(int contour, int v, double x)
	{
		int o = m_co[contour];
		m_cx[wrap(o + v, o, o + m_cl[contour])] = x;
	}

	inline void setContourY(int contour, int v, double y)
	{
		int o = m_co[contour];
		m_cy[wrap(o + v, o, o + m_cl[contour])] = y;
	}

	inline int getValidIndex(int contour, int v) const
	{
		int o = m_co[contour];
		return wrap(o + v, o, o + m_cl[contour]);
	}

	double measureArea(int contour, int first, int last) const
	{
		double area = 0;
		if (getValidIndex(contour, first) == getValidIndex(contour, last))
			last = last - 1;
		int n = last - first + 1;
		
		double w = 0, h = 0;
		for (int i = first; i < last ; i++)
		{
			w = getContourX(contour, i + 1) - getContourX(contour, i);
			h = (getContourY(contour, i + 1) + getContourY(contour, i)) / 2.0;
			area += w * h;
		}
		w = getContourX(contour, first) - getContourX(contour, last);
		h = (getContourY(contour, first) + getContourY(contour, last)) / 2.0;
		area += w * h;
		
		return area;
	}

	double measureMeanX(int contour) const
	{
		double mean = 0.0;
		int l = getContourLength(contour);
		for (int i = 0; i < l; i++)
			mean += getContourX(contour, i);
		return (l == 0 ? 0 : mean / l);
	}

	double measureMeanY(int contour) const
	{
		double mean = 0.0;
		int l = getContourLength(contour);
		for (int i = 0; i < l; i++)
			mean += getContourY(contour, i);
		return (l == 0 ? 0 : mean / l);
	}

	double measurePerimeter(int contour, int first, int last)  const
	{
		if (getValidIndex(contour, first) == getValidIndex(contour, last))
			last = last - 1;
		
		double perim = 0;
		for (int i = first; i < last ; i++)
			perim += measureLength(contour, i);
		perim += measureDistance(contour,first,last);
		
		return perim;
	}

	double measureNormalX(int contour, int i) const
	{
		double ret = getContourY(contour, i) - getContourY(contour, i + 1);
		ret = ret / measureLength(contour, i);
		return ret;
	}

	double measureNormalY(int contour, int i) const
	{
		double ret = getContourX(contour, i + 1) - getContourX(contour, i);
		ret = ret / measureLength(contour, i);
		return ret;
	}

	double measureNormalY(int contour, int first, int last) const
	{
		double ret = 0;
		for (int i = first; i < last ; i++)
			ret += measureNormalY(contour, i);
		return ret;
	}

	double measureNormalX(int contour, int first, int last) const
	{
		double ret = 0;
		for (int i = first; i < last ; i++)
			ret += measureNormalX(contour, i);
		return ret;
	}

	double measureAngleChange(int contour, int first, int last) const
	{
		double sum = 0;
		for (int i = first; i <= last; i++)
			sum += measureAngle(contour, i);
		return sum;
	}

	static int wrap(int i, int lo, int hi)
	{
		int l = hi - lo;
		int d = i - lo;
		int w = 0;
		if (d < 0)
			w = hi - ((-d) % l);
		else
			w = lo + (d % l);
		
		if (w == hi)
			w = lo;
		
		if (w < lo)
		{
			assert(false);
			printf("went below lo\n");
		}
		else if (w >= hi)
		{
			assert(false);
			printf("went above hi\n");
		}
		
		return w;
	}

	inline int ixy(int x, int y) const { return x + y * m_w; }

	inline double measureDistance(int contour, int first, int second) const
	{
		double dx = getContourX(contour, first) - getContourX(contour, second);
		double dy = getContourY(contour, first) - getContourY(contour, second);
		return sqrt(dx * dx + dy * dy);
	}

	// return length from i to i + 1
	double measureLength(int contour, int i) const
	{
		int lo = m_co[contour];
		int n  = m_cl[contour];
		int hi = lo + n;

		int v1 = wrap(lo + i+0, lo, hi);
		int v2 = wrap(lo + i+1, lo, hi);

		double aftx = m_cx[v2] - m_cx[v1];
		double afty = m_cy[v2] - m_cy[v1];

		return sqrt(aftx * aftx + afty * afty);
	}

	// return the relative angle change in radians
	// about the point i (assuming ccw is positive)
	double measureAngle(int contour, int i) const
	{
		double befx = getContourX(contour, i + 0) - getContourX(contour, i - 1);
		double befy = getContourY(contour, i + 0) - getContourY(contour, i - 1);
		double aftx = getContourX(contour, i + 1) - getContourX(contour, i + 0);
		double afty = getContourY(contour, i + 1) - getContourY(contour, i + 0);

		double befl = sqrt(befx * befx + befy * befy); 
		befx /= befl;
		befy /= befl;
		double aftl = sqrt(aftx * aftx + afty * afty);
		aftx /= aftl;
		afty /= aftl;		

		double dot = befx * aftx + befy * afty;
		if (dot > 1.0)
			dot = 1.0;
		else if (dot < 0)
			dot = 0;
		double rads = acos(dot);
		assert(rads == rads); //otherwise it means that rads is NaN!!!

		if (aftx * befy - afty * befx < 0)
			rads = -rads;

		return rads;
	}

public:

	inline double getContourX(int contour, int v) const
	{
		int o = m_co[contour];
		return m_cx[wrap(o + v, o, o + m_cl[contour])];
	}

	inline double getContourY(int contour, int v) const
	{
		int o = m_co[contour];
		return m_cy[wrap(o + v, o, o + m_cl[contour])];
	}

	inline double measureCurvature(int contour, int i) const
	{
		return measureAngle(contour, i) / measureLength(contour, i);
	}

	void findAreas(int window, std::vector<double>& tips)
	{
		tips.resize(m_w * m_h);

		for (int k = 0; k < m_numContours; k++)
		{
			int l = getContourLength(k);
			for (int i = 0; i < l; i++)
			{
				int lo = i - window;
				int hi = i + window;
				tips[getValidIndex(k, i)] = measureArea(k, lo, hi);
			}
		}
	}

	void findRoundedCorners(int window, std::vector<double>& tips)
	{
		tips.resize(m_w * m_h);

		for (int k = 0; k < m_numContours; k++)
		{
			int l = getContourLength(k);
			for (int i = 0; i < l; i++)
			{
				int lo = i - window;
				int hi = i + window;
				tips[getValidIndex(k, i)] = measureArea(k, lo, hi) / measurePerimeter(k, lo, hi);
			}
		}
	}

	int getMaxContour() const
	{
		int maxlength = 0;
		int idx = 0;
		for (int k = 0; k < m_numContours; k++)
		{
			int l = getContourLength(k);
			if (l > maxlength)
			{
				maxlength = l;
				idx = k;
			}
		}
		return idx;
	}

	// POLYGON HIT TESTING ROUTINES

	bool computeBoundingBoxes()
	{
		int numContours = getNumContours();
		if (numContours == 0)
		{
			m_minx.clear();
			m_miny.clear();
			m_maxx.clear();
			m_maxy.clear();
			return true;
		}

		try
		{
			m_minx.resize(numContours);
			m_miny.resize(numContours);
			m_maxx.resize(numContours);
			m_maxy.resize(numContours);
		}
		catch(std::bad_alloc)
		{
			//not enough memory!
			return false;
		}

		for (int k = 0; k < numContours; k++)
		{
			int o = m_co[k];
			m_minx[k] = m_cx[o];
			m_miny[k] = m_cy[o];
			m_maxx[k] = m_cx[o];
			m_maxy[k] = m_cy[o];
			for (int i = 1; i < getContourLength(k); i++)
			{
				int j = o + i;
				if (m_cx[j] < m_minx[k]) 
					m_minx[k] = m_cx[j];
				else if (m_cx[j] > m_maxx[k]) 
					m_maxx[k] = m_cx[j];
				if (m_cy[j] < m_miny[k]) 
					m_miny[k] = m_cy[j];
				else if (m_cy[j] > m_maxy[k]) 
					m_maxy[k] = m_cy[j];
			}
		}

		return true;
	}

	inline double getBBMinX(int contour) { return m_minx[contour]; }
	inline double getBBMaxX(int contour) { return m_maxx[contour]; }
	inline double getBBMinY(int contour) { return m_miny[contour]; }
	inline double getBBMaxY(int contour) { return m_maxy[contour]; }

	bool contains(int k, double x, double y) const
	{
		bool inside = false;
		int l = getContourLength(k);
		for (int i = 0, j = -1; i < l; j = i++)
		{
			double yi = getContourY(k, i);
			double yj = getContourY(k, j);
			if ((yj > y) != (yi > y))
			{
				double xi = getContourX(k, i);
				double xj = getContourX(k, j);
				if (yi == yj)
				{
					if (x < xi)
						inside = !inside;
				}
				else if (x < xi + (y - yi) * (xi - xj) / (yi - yj))
				{
					inside = !inside;
				}
			}
		}
		return inside;
	}

	bool containsContour(int k1, int k2)
	{
		if (!bbIntersect(k1, k2))
			return false;

		double minx = getBBMinX(k2);
		double maxx = getBBMaxX(k2);
		double miny = getBBMinY(k2);
		double maxy = getBBMaxY(k2);

		return (	contains(k1, minx, miny)
			&&	contains(k1, maxx, miny)
			&&	contains(k1, maxx, maxy)
			&&	contains(k1, minx, maxy) );
	}

	inline bool containsBoundingBox(int k, double minx, double miny, double maxx, double maxy) const
	{
		return (	contains(k, minx, miny)
				&&	contains(k, maxx, miny)
				&&	contains(k, maxx, maxy)
				&&	contains(k, minx, maxy) );
	}

	bool contains(const std::vector<double>& polyx, const std::vector<double>& polyy, double x, double y) const
	{
		bool inside = false;
		size_t l = polyx.size();
		if (l < 1)
			return false;
		for (size_t i = 0, j = l - 1; i < l; j = i++)
		{
			double yi = polyy[i];
			double yj = polyy[j];
			if ((yj > y) != (yi > y))
			{
				double xi = polyx[i];
				double xj = polyx[j];
				if (yi == yj)
				{
					if (x < xi)
						inside = !inside;
				}
				else if (x < xi + (y - yi) * (xi - xj) / (yi - yj))
				{
					inside = !inside;
				}
			}
		}
		return inside;
	}

	// intersects contour k1 with contour k2
	bool bbIntersect(int k1, int k2) const
	{
		double minx1 = getBBMinX(k1);
		double maxx1 = getBBMaxX(k1);
		double miny1 = getBBMinY(k1);
		double maxy1 = getBBMaxY(k1);
		double minx2 = getBBMinX(k2);
		double maxx2 = getBBMaxX(k2);
		double miny2 = getBBMinY(k2);
		double maxy2 = getBBMaxY(k2);

		double lt = minx1 > minx2 ? minx1 : minx2;
		double rt = maxx1 < maxx2 ? maxx1 : maxx2;
		double tp = miny1 > miny2 ? miny1 : miny2;
		double bt = maxy1 < maxy2 ? maxy1 : maxy2;

		return (lt < rt && tp < bt);
	}

	bool bbContainsBB(int k1, int k2) const
	{
		double minx1 = getBBMinX(k1);
		double maxx1 = getBBMaxX(k1);
		double miny1 = getBBMinY(k1);
		double maxy1 = getBBMaxY(k1);
		double minx2 = getBBMinX(k2);
		double maxx2 = getBBMaxX(k2);
		double miny2 = getBBMinY(k2);
		double maxy2 = getBBMaxY(k2);

		return (minx1 <= minx2 && maxx1 >= maxx2 && miny1 <= miny2 && maxy1 >= maxy2);
	}

	inline double bbArea(int k) const
	{
		double w = getBBMaxX(k) - getBBMinX(k);
		double h = getBBMaxY(k) - getBBMinY(k);
		return w * h;
	}

	inline double getBBCenterX(int k) const { return (getBBMinX(k) + getBBMaxX(k)) / 2; }
	inline double getBBCenterY(int k) const { return (getBBMinY(k) + getBBMaxY(k)) / 2; }
};

#endif //ISOLINES_HEADER
