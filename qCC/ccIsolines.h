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
#include <cmath>
#include <vector>

template< typename T > class Isolines
{
protected:
	std::vector<double> m_minx;
	std::vector<double> m_miny;
	std::vector<double> m_maxx;
	std::vector<double> m_maxy;

	std::vector<int>    m_cd;
	std::vector<double> m_contourX;
	std::vector<double> m_contourY;
	std::vector<int>    m_contourLength;
	std::vector<int>    m_contourOrigin;
	std::vector<int>    m_contourIndexes;
	std::vector<bool>   m_contourClosed;

	int m_w;
	int m_h;
	int m_numContours;

	T m_threshold;

public:

	//! Default constructor
	Isolines(int w, int h)
		: m_w(w)
		, m_h(h)
		, m_numContours(0)
		, m_threshold(0)
	{
		//DGM: as this is done in the constructor,
		//we don't catch the exception (so that the
		//caller can properly handle the error!)
		//try
		{
			m_cd.resize(w*h,0);
		}
		//catch (const std::bad_alloc&)
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
	inline int getContourLength(int contour) const { return m_contourLength[contour]; }

	//! Returns whether a given contour is closed or not
	inline bool isContourClosed(int contour) const { return m_contourClosed[contour]; }

	//! Returns the given point (x,y) of a given contour
	void getContourPoint(int contour, size_t index, double& x, double& y) const
	{
		assert(static_cast<int>(index) < getContourLength(contour));
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
		//rows
		{
			int shift = (m_h-1) * m_w;
			for (int i=0; i<m_w; i++)
			{
				in[i] = in[i+shift] = borderval;
			}
		}
		//columns
		{
			for (int j=0; j<m_h; j++)
			{
				in[j*m_w] = in[(j+1)*m_w-1] = borderval;
			}
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
				int b0(in[ixy(x + 0, y + 1)] < m_threshold ? 1 : 0); //bottom-left is below threshold
				int b1(in[ixy(x + 1, y + 1)] < m_threshold ? 2 : 0); //bottom-right is below threshold
				int b2(in[ixy(x + 1, y + 0)] < m_threshold ? 4 : 0); //top-right is below threshold
				int b3(in[ixy(x + 0, y + 0)] < m_threshold ? 8 : 0); //top-left is below threshold
				m_cd[ixy(x, y)] = b0 | b1 | b2 | b3;
			}
		} 
	}

	//! 2x2 cell configuration codes
	enum ConfigurationCodes
	{
		CASE0   =  0, 
		CASE1   =  1,
		CASE2   =  2,
		CASE3   =  3,
		CASE4   =  4,
		CASE5   =  5,
		CASE6   =  6,
		CASE7   =  7,
		CASE8   =  8,
		CASE9   =  9,
		CASE10  = 10,
		CASE11  = 11,
		CASE12  = 12,
		CASE13  = 13,
		CASE14  = 14,
		CASE15  = 15,
		VISITED = 16
	};

	//! Entry/exit edges
	enum Edges
	{
		NONE   = -1,
		TOP    =  0,
		RIGHT  =  1,
		BOTTOM =  2,
		LEFT   =  3
	};

	void endContour(bool closed, bool alternatePath)
	{
		if (alternatePath)
		{
			//we have to merge this path with the previous one!
			//try //will be taken care of by 'findIsolines'
			//{
				size_t length = m_contourLength.back();
				size_t firstIndex = m_contourOrigin.back();
				m_contourLength.pop_back();
				m_contourOrigin.pop_back();
				m_contourClosed.pop_back();

				//backup the alternate part of the contour
				std::vector<double> subContourX(length), subContourY(length);
				std::vector<int> subContourIndexes(length);
				{
					for (size_t i=0; i<length; ++i)
					{
						subContourX[i] = m_contourX[firstIndex+i];
						subContourY[i] = m_contourY[firstIndex+i];
						subContourIndexes[i] = m_contourIndexes[firstIndex+i];
					}
				}

				assert(!m_contourLength.empty() && !m_contourOrigin.empty());
				size_t length0 = m_contourLength.back();
				size_t firstIndex0 = m_contourOrigin.back();

				//shift the first part values
				{
					for (int i=static_cast<int>(length0); i>=0; --i) //we start by end so as to not overwrite values!
					{
						m_contourX[firstIndex0+length+i] = m_contourX[firstIndex0+i];
						m_contourX[firstIndex0+length+i] = m_contourY[firstIndex0+i];
						m_contourIndexes[firstIndex0+length+i] = m_contourIndexes[firstIndex0+i];
					}
				}

				//now copy the second part values
				{
					for (size_t i=0; i<length; ++i)
					{
						m_contourX[firstIndex0+i] = subContourX[i];
						m_contourY[firstIndex0+i] = subContourY[i];
						m_contourIndexes[firstIndex0+i] = subContourIndexes[i];
					}
				}

				//even though we are merging contours, if we are here it
				//means that the contour was not a proper loop!
				closed = false;
				assert(!m_contourClosed.empty());
				m_contourClosed.back() = false;
			//}
			//catch (const std::bad_alloc&)
			//{
			//	return false;
			//}
		}

		//simply update the closed state (just to be sure)
		assert(!m_contourClosed.empty());
		assert(!closed || m_contourLength.back() > 2);
		m_contourClosed.back() = closed;

		//return true;
	}

	//! Searches image for contours from topleft to bottomright corners
	int findIsolines(const T* in)
	{
		//traversal case
		static const int TRAVERSAL[4]           = { /*TOP=*/BOTTOM,  /*RIGHT=*/LEFT,   /*BOTTOM=*/TOP,   /*LEFT=*/RIGHT   };
		//disambiguation cases (CASES 5 and 10)
		static const int DISAMBIGUATION_UP  [4] = { /*TOP=*/LEFT,    /*RIGHT=*/BOTTOM, /*BOTTOM=*/RIGHT, /*LEFT=*/TOP     };
		static const int DISAMBIGUATION_DOWN[4] = { /*TOP=*/RIGHT,   /*RIGHT=*/TOP,    /*BOTTOM=*/LEFT,  /*LEFT=*/BOTTOM  };

		m_contourX.clear();
		m_contourY.clear();
		m_contourLength.clear();
		m_contourOrigin.clear();
		m_contourIndexes.clear();
		m_contourClosed.clear();

		try
		{
			int toEdge = NONE;
			int cellIndex = -1;
			int x = 0, y = 0;

			//mechanism for merging two parts of a non-closed contour
			int altToEdge = NONE;
			int altStartIndex = 0;
			bool alternatePath = false;

			const int maxCellIndex = m_w * (m_h-1); //DGM: last line is only 0!
			while (cellIndex < maxCellIndex)
			{
				//entry edge
				int fromEdge = (toEdge == NONE ? NONE : TRAVERSAL[toEdge]);
				//exit edge
				toEdge = NONE;
				//alternative exit edge (when starting a new contour)
				if (fromEdge == NONE)
					altToEdge = NONE;

				int currentCellIndex = -1;

				//last isoline is 'finished'
				if (fromEdge == NONE)
				{
					//do we have an alternate path?
					if (altToEdge != NONE && !alternatePath)
					{
						//we know that we are coming from the TOP (case 2 or 13)
						fromEdge = TOP;
						currentCellIndex = altStartIndex + m_w; //same coumn, next row
						alternatePath = true;
						//we start a new (temporary) contour
						m_contourLength.push_back(0);
						m_contourClosed.push_back(false);
						m_contourOrigin.push_back(static_cast<int>(m_contourX.size())+1);
					}
					else
					{
						// we have to look for a new starting point
						alternatePath = false;
						altToEdge = NONE;
						//skip empty cells
						while (	++cellIndex < maxCellIndex
							&&	(m_cd[cellIndex] == CASE0 || m_cd[cellIndex] == CASE15) )
						{
						}
						if (cellIndex == maxCellIndex)
							break;
						currentCellIndex = cellIndex;
					}
					x = currentCellIndex % m_w;
					y = currentCellIndex / m_w;
				}

				//we have reached a border (bottom or right)
				if (x < 0 || x >= m_w-1 || y < 0 || y >= m_h-1)
				{
					//if an isoline was underway, it won't be closed!
					if (fromEdge != NONE)
					{
						endContour(false,alternatePath);
					}
					//toEdge = NONE;
					continue;
				}
				else if (fromEdge != NONE)
				{
					//isoline underway: we have to re-evaluate the
					//current position in the grid
					currentCellIndex = ixy(x,y);
				}

				assert(currentCellIndex >= 0);
				int& currentCode = m_cd[currentCellIndex];
				switch (currentCode)
				{
				case CASE0:									// CASE 0
					//we have reached a border!
					//if an isoline was underway, it won't be closed!
					if (fromEdge != NONE)
					{
						endContour(false,alternatePath);
					}
					//toEdge = NONE;
					continue;

				case CASE1:									// CASE 1
				case CASE14:								// CASE 14
					toEdge = (fromEdge == NONE || fromEdge == LEFT ? BOTTOM : LEFT);
					currentCode |= VISITED;
					break;

				case CASE2:									// CASE 2
				case CASE13:								// CASE 13
					//if it's a new contour, we have 2 options (RIGHT and BOTTOM)
					if (fromEdge == NONE)
					{
						if (x >= m_w-2) //can't go on the right!
						{
							if (y >= m_h-2) //can't go lower
							{
								//toEdge = NONE;
								continue;
							}
							else
							{
								toEdge = BOTTOM;
							}
						}
						else
						{
							//go right by default
							toEdge = RIGHT;
							if (y < m_h-2)
							{
								//if we can go lower, rembemr this as an alternate rout
								altToEdge = BOTTOM;
								altStartIndex = currentCellIndex;
							}
						}
					}
					else
					{
						toEdge = (fromEdge == BOTTOM ? RIGHT : BOTTOM);
					}
					currentCode |= VISITED;
					break;

				case CASE3:									// CASE 3
				case CASE12:								// CASE 12
					toEdge = (fromEdge == NONE || fromEdge == LEFT ? RIGHT : LEFT);
					currentCode |= VISITED;
					break;

				case CASE11:								// CASE 11
					//assert(fromEdge != NONE);
				case CASE4:									// CASE 4
					toEdge = (fromEdge == NONE || fromEdge == TOP ? RIGHT : TOP);
					currentCode |= VISITED;
					break;

				case CASE5:									// CASE 5, saddle
				case CASE10:								// CASE 10, saddle
					{
						if (fromEdge != NONE)
						{
							//check if we are not looping as the saddle points can't be flagged as VISITED!
							assert(!m_contourOrigin.empty() && !m_contourIndexes.empty());
							if (m_contourIndexes[m_contourOrigin.back()] == currentCellIndex)
							{
								//isoline loop is closed!
								assert(!alternatePath);
								endContour(true,false);
								//no need to look at the alternate path!
								altToEdge = NONE;
								//toEdge = NONE;
								continue;
							}
						}

						double avg = (	in[ixy(x + 0, y + 0)]
									+	in[ixy(x + 1, y + 0)]
									+	in[ixy(x + 0, y + 1)]
									+	in[ixy(x + 1, y + 1)] ) / 4.0;

						//see http://en.wikipedia.org/wiki/Marching_squares for the disambiguation cases
						bool down = (	(currentCode == CASE5  && avg <  m_threshold)
									||	(currentCode == CASE10 && avg >= m_threshold) );
						if (down)
						{
							if (fromEdge == NONE)
							{
								//if we start here, we know that some routes have already been taken (the ones coming from UP or LEFT)
								//we can ignore the cell
								continue;
							}
							else
							{
								toEdge = DISAMBIGUATION_DOWN[fromEdge];
							}
						}
						else
						{
							if (fromEdge == NONE)
							{
								//if we start here, we know that some routes have already been taken (the ones coming from UP or LEFT)
								//it can only be on the right
								if (x < m_w-2)
								{
									if (m_cd[currentCellIndex+1] < VISITED)
										toEdge = RIGHT;
									else
										//we can ignore the cell
										continue;
								}
								else
								{
									//we can ignore the cell
									continue;
								}
							}
							else
							{
								toEdge = DISAMBIGUATION_UP[fromEdge];
							}
						}
					}
					break;

				case CASE6:									// CASE 6
					//assert(fromEdge != NONE);
				case CASE9:									// CASE 9
					toEdge = (fromEdge == NONE || fromEdge == TOP ? BOTTOM : TOP);
					currentCode |= VISITED;
					break;

				case CASE7:									// CASE 7
					//assert(fromEdge != NONE);
				case CASE8:									// CASE 8
					toEdge = (fromEdge == NONE || fromEdge == LEFT ? TOP : LEFT);
					currentCode |= VISITED;
					break;

				case CASE15:								// CASE 15
					//assert(fromEdge == NONE); //apart at the very beginning!
					//toEdge = NONE;
					continue;

				default:
					assert(currentCode > 0 && ((currentCode & VISITED) == VISITED));
					if (fromEdge != NONE)
					{
						//check that we are indeed coming back to the start
						assert(!m_contourOrigin.empty() && !m_contourIndexes.empty());
						if (m_contourIndexes[m_contourOrigin.back()] == currentCellIndex)
						{
							//isoline loop is closed!
							assert(!alternatePath);
							endContour(true,false);
							//no need to look at the alternate path!
							altToEdge = NONE;
						}
						else
						{
							//have we reached a kind of tri-point? (DGM: not sure it's possible)
							endContour(false,alternatePath);
						}
					}
					//toEdge = NONE;
					continue;
				}

				assert(toEdge != NONE);
				
				if (fromEdge == NONE)
				{
					// starting a new contour
					m_contourLength.push_back(0);
					m_contourClosed.push_back(false);
					m_contourOrigin.push_back(static_cast<int>(m_contourX.size()));
					//ccLog::Print(QString("New contour: #%1 - origin = %2 - (x=%3, y=%4)").arg(m_contourLength.size()).arg(m_contourOrigin.back()).arg(x).arg(y));
				}

				double x2 = 0.0, y2 = 0.0;
				switch (toEdge)
				{
				case TOP: 
					x2 = x + LERP(in[ixy(x + 0, y + 0)], in[ixy(x + 1, y + 0)]);
					y2 = y;
					y--;
					break;
				case RIGHT:
					x2 = x + 1;
					y2 = y + LERP(in[ixy(x + 1, y + 0)], in[ixy(x + 1, y + 1)]);
					x++;
					break;
				case BOTTOM:
					x2 = x + LERP(in[ixy(x + 0, y + 1)], in[ixy(x + 1, y + 1)]);
					y2 = y + 1;
					y++;
					break;
				case LEFT:
					x2 = x;
					y2 = y + LERP(in[ixy(x + 0, y + 0)], in[ixy(x + 0, y + 1)]);
					x--;
					break;
				default:
					assert(false);
					continue;
				}

				//if (m_contourLength.back() > 1)
				//{
				//	size_t vertCount = m_contourX.size();
				//	const double& x0 = m_contourX[vertCount - 2];
				//	const double& y0 = m_contourY[vertCount - 2];
				//	double& x1 = m_contourX.back();
				//	double& y1 = m_contourY.back();
				//	double ux = x1 - x0;
				//	double uy = y1 - y0;
				//	double vx = x2 - x0;
				//	double vy = y2 - y0;
				//	//test colinearity so as to merge both segments if possible
				//	double dotprod = (ux*vx + uy*vy) / sqrt((vx*vx + vy*vy) * (ux*ux + uy*uy));
				//	if (fabsl(dotprod - 1.0) < 1.0e-6)
				//	{
				//		//merge: we replace the last vertex by this one
				//		x1 = x2;
				//		y1 = y2;
				//		m_contourIndexes.back() = currentCellIndex;
				//	}
				//	else
				//	{
				//		//new vertex
				//		m_contourX.push_back(x2);
				//		m_contourY.push_back(y2);
				//		m_contourIndexes.push_back(currentCellIndex);
				//		m_contourLength.back()++;
				//	}
				//}
				//else
				{
					//new vertex
					m_contourX.push_back(x2);
					m_contourY.push_back(y2);
					m_contourIndexes.push_back(currentCellIndex);
					m_contourLength.back()++;
				}
			}
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			m_contourX.clear();
			m_contourY.clear();
			m_contourLength.clear();
			m_contourIndexes.clear();
			return -1;
		}

		m_numContours = static_cast<int>(m_contourLength.size());

		computeBoundingBoxes();

		return m_numContours;
	}

	//! LERP between two values
	inline double LERP(T A, T B) const
	{
		T AB = A-B;
		return AB == 0 ? 0 : static_cast<double>(A - m_threshold) / AB;
	}

	inline int getLastIndex() const
	{
		int nc = getNumContours();
		return nc > 0 ? m_contourOrigin[nc - 1] + m_contourLength[nc - 1] : 0;
	}

	inline void setContourX(int contour, int v, double x)
	{
		int o = m_contourOrigin[contour];
		m_contourX[wrap(o + v, o, o + m_contourLength[contour])] = x;
	}

	inline void setContourY(int contour, int v, double y)
	{
		int o = m_contourOrigin[contour];
		m_contourY[wrap(o + v, o, o + m_contourLength[contour])] = y;
	}

	inline int getValidIndex(int contour, int v) const
	{
		int o = m_contourOrigin[contour];
		return wrap(o + v, o, o + m_contourLength[contour]);
	}

	double measureArea(int contour, int first, int last) const
	{
		double area = 0;
		if (getValidIndex(contour, first) == getValidIndex(contour, last))
			last = last - 1;
		
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
		return std::sqrt(dx * dx + dy * dy);
	}

	// return length from i to i + 1
	double measureLength(int contour, int i) const
	{
		int lo = m_contourOrigin[contour];
		int n  = m_contourLength[contour];
		int hi = lo + n;

		int v1 = wrap(lo + i+0, lo, hi);
		int v2 = wrap(lo + i+1, lo, hi);

		double aftx = m_contourX[v2] - m_contourX[v1];
		double afty = m_contourY[v2] - m_contourY[v1];

		return std::sqrt(aftx * aftx + afty * afty);
	}

	// return the relative angle change in radians
	// about the point i (assuming ccw is positive)
	double measureAngle(int contour, int i) const
	{
		double befx = getContourX(contour, i + 0) - getContourX(contour, i - 1);
		double befy = getContourY(contour, i + 0) - getContourY(contour, i - 1);
		double aftx = getContourX(contour, i + 1) - getContourX(contour, i + 0);
		double afty = getContourY(contour, i + 1) - getContourY(contour, i + 0);

		double befl = std::sqrt(befx * befx + befy * befy); 
		befx /= befl;
		befy /= befl;
		double aftl = std::sqrt(aftx * aftx + afty * afty);
		aftx /= aftl;
		afty /= aftl;		

		double dot = befx * aftx + befy * afty;
		if (dot > 1.0)
			dot = 1.0;
		else if (dot < 0)
			dot = 0;
		double rads = std::acos(dot);
		assert(rads == rads); //otherwise it means that rads is NaN!!!

		if (aftx * befy - afty * befx < 0)
			rads = -rads;

		return rads;
	}

public:

	inline double getContourX(int contour, int v) const
	{
		int o = m_contourOrigin[contour];
		return m_contourX[wrap(o + v, o, o + m_contourLength[contour])];
	}

	inline double getContourY(int contour, int v) const
	{
		int o = m_contourOrigin[contour];
		return m_contourY[wrap(o + v, o, o + m_contourLength[contour])];
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
		catch (const std::bad_alloc&)
		{
			//not enough memory!
			return false;
		}

		for (int k = 0; k < numContours; k++)
		{
			int o = m_contourOrigin[k];
			m_minx[k] = m_contourX[o];
			m_miny[k] = m_contourY[o];
			m_maxx[k] = m_contourX[o];
			m_maxy[k] = m_contourY[o];
			for (int i = 1; i < getContourLength(k); i++)
			{
				int j = o + i;
				if (m_contourX[j] < m_minx[k]) 
					m_minx[k] = m_contourX[j];
				else if (m_contourX[j] > m_maxx[k]) 
					m_maxx[k] = m_contourX[j];
				if (m_contourY[j] < m_miny[k]) 
					m_miny[k] = m_contourY[j];
				else if (m_contourY[j] > m_maxy[k]) 
					m_maxy[k] = m_contourY[j];
			}
		}

		return true;
	}

	inline double getBBMinX(int contour) const { return m_minx[contour]; }
	inline double getBBMaxX(int contour) const { return m_maxx[contour]; }
	inline double getBBMinY(int contour) const { return m_miny[contour]; }
	inline double getBBMaxY(int contour) const { return m_maxy[contour]; }

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

	inline double getBBCenterX(int k) const { return (getBBMinX(k) + getBBMaxX(k)) / 2.0; }
	inline double getBBCenterY(int k) const { return (getBBMinY(k) + getBBMaxY(k)) / 2.0; }
};

#endif //ISOLINES_HEADER
