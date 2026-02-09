//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: FFDPlugin                       #
//#           Free Form Deformation - Non-rigid Transformation             #
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
//##########################################################################

#include "FFDLattice.h"
#include <cmath>
#include <algorithm>

FFDLattice::FFDLattice( const std::array<unsigned int, 3> &latticeSize, const ccBBox &boundingBox )
	: m_latticeSize( latticeSize )
	, m_boundingBox( boundingBox )
{
	// Calculate the size of each cell
	CCVector3 bbSize = boundingBox.maxCorner() - boundingBox.minCorner();
	m_cellSize = CCVector3d(
		bbSize.x / (latticeSize[0] - 1.0),
		bbSize.y / (latticeSize[1] - 1.0),
		bbSize.z / (latticeSize[2] - 1.0)
	);

	// Initialize control points in a regular grid
	size_t totalPoints = latticeSize[0] * latticeSize[1] * latticeSize[2];
	m_controlPoints.reserve( totalPoints );
	m_originalControlPoints.reserve( totalPoints );

	CCVector3 minCorner = boundingBox.minCorner();

	for ( unsigned int z = 0; z < latticeSize[2]; ++z )
	{
		for ( unsigned int y = 0; y < latticeSize[1]; ++y )
		{
			for ( unsigned int x = 0; x < latticeSize[0]; ++x )
			{
				CCVector3d controlPoint(
					minCorner.x + x * m_cellSize.x,
					minCorner.y + y * m_cellSize.y,
					minCorner.z + z * m_cellSize.z
				);
				m_controlPoints.push_back( controlPoint );
				m_originalControlPoints.push_back( controlPoint );
			}
		}
	}
}

void FFDLattice::moveControlPoint( unsigned int indexX, unsigned int indexY, unsigned int indexZ, const CCVector3d &displacement )
{
	if ( indexX >= m_latticeSize[0] || indexY >= m_latticeSize[1] || indexZ >= m_latticeSize[2] )
	{
		return; // Index out of bounds
	}

	size_t index = indexZ * m_latticeSize[1] * m_latticeSize[0] + indexY * m_latticeSize[0] + indexX;
	m_controlPoints[index] += displacement;
}

CCVector3d FFDLattice::getControlPoint( unsigned int indexX, unsigned int indexY, unsigned int indexZ ) const
{
	if ( indexX >= m_latticeSize[0] || indexY >= m_latticeSize[1] || indexZ >= m_latticeSize[2] )
	{
		return CCVector3d( 0.0, 0.0, 0.0 );
	}

	size_t index = indexZ * m_latticeSize[1] * m_latticeSize[0] + indexY * m_latticeSize[0] + indexX;
	return m_controlPoints[index];
}

void FFDLattice::reset()
{
	m_controlPoints = m_originalControlPoints;
}

void FFDLattice::setAllControlPoints(const std::vector<CCVector3d> &controlPoints)
{
	if (controlPoints.size() == m_controlPoints.size())
	{
		m_controlPoints = controlPoints;
	}
}

double FFDLattice::cubicBSplineWeight( double t )
{
	// Normalize t to [0, 1]
	t = std::clamp( t, 0.0, 1.0 );

	// Cubic B-spline basis function for smooth interpolation
	// This provides C^2 continuity
	if ( t < 0.5 )
	{
		double t2 = t * t;
		return 0.5 * (2.0 - 4.0 * t2 + 4.0 * t * t2);
	}
	else
	{
		double mt = 1.0 - t;
		double mt2 = mt * mt;
		return 0.5 * mt2 * mt;
	}
}

double FFDLattice::computeControlPointWeight( const CCVector3d &point,
											   unsigned int cpIndexX,
											   unsigned int cpIndexY,
											   unsigned int cpIndexZ ) const
{
	// Compute normalized coordinates within the bounding box
	CCVector3 bbMin = m_boundingBox.minCorner();
	CCVector3d normalizedPoint = point - CCVector3d( bbMin.x, bbMin.y, bbMin.z );

	// Compute the distance from the control point in normalized coordinates
	double dx = std::abs( normalizedPoint.x / m_cellSize.x - cpIndexX );
	double dy = std::abs( normalizedPoint.y / m_cellSize.y - cpIndexY );
	double dz = std::abs( normalizedPoint.z / m_cellSize.z - cpIndexZ );

	// Apply cubic B-spline basis function
	// Only consider control points within 2 cells distance
	if ( dx >= 2.0 || dy >= 2.0 || dz >= 2.0 )
	{
		return 0.0;
	}

	return cubicBSplineWeight( dx ) * cubicBSplineWeight( dy ) * cubicBSplineWeight( dz );
}

CCVector3d FFDLattice::deformPoint( const CCVector3d &originalPoint ) const
{
	// Compute normalized coordinates within the bounding box
	CCVector3 bbMin = m_boundingBox.minCorner();
	CCVector3 bbMax = m_boundingBox.maxCorner();
	
	CCVector3d pointRelative = originalPoint - CCVector3d(bbMin.x, bbMin.y, bbMin.z);
	CCVector3d bbSize = CCVector3d(bbMax.x - bbMin.x, bbMax.y - bbMin.y, bbMax.z - bbMin.z);

	// Clamp point to [0, 1] in normalized space
	double u = std::clamp(pointRelative.x / bbSize.x, 0.0, 1.0);
	double v = std::clamp(pointRelative.y / bbSize.y, 0.0, 1.0);
	double w = std::clamp(pointRelative.z / bbSize.z, 0.0, 1.0);

	// Convert to lattice grid coordinates
	double gridX = u * (m_latticeSize[0] - 1);
	double gridY = v * (m_latticeSize[1] - 1);
	double gridZ = w * (m_latticeSize[2] - 1);

	// Get integer and fractional parts
	int xi = static_cast<int>(std::floor(gridX));
	int yi = static_cast<int>(std::floor(gridY));
	int zi = static_cast<int>(std::floor(gridZ));

	double fu = gridX - xi;
	double fv = gridY - yi;
	double fw = gridZ - zi;

	// Clamp grid indices to valid range
	xi = std::clamp(xi, 0, static_cast<int>(m_latticeSize[0]) - 2);
	yi = std::clamp(yi, 0, static_cast<int>(m_latticeSize[1]) - 2);
	zi = std::clamp(zi, 0, static_cast<int>(m_latticeSize[2]) - 2);

	// Get the 8 surrounding control points
	auto getCP = [this](int x, int y, int z) -> CCVector3d {
		if (x < 0 || x >= static_cast<int>(m_latticeSize[0]) ||
		    y < 0 || y >= static_cast<int>(m_latticeSize[1]) ||
		    z < 0 || z >= static_cast<int>(m_latticeSize[2]))
			return CCVector3d(0, 0, 0);
		size_t idx = z * m_latticeSize[1] * m_latticeSize[0] + y * m_latticeSize[0] + x;
		return m_controlPoints[idx];
	};

	// Trilinear interpolation
	CCVector3d c000 = getCP(xi,     yi,     zi);
	CCVector3d c100 = getCP(xi + 1, yi,     zi);
	CCVector3d c010 = getCP(xi,     yi + 1, zi);
	CCVector3d c110 = getCP(xi + 1, yi + 1, zi);
	CCVector3d c001 = getCP(xi,     yi,     zi + 1);
	CCVector3d c101 = getCP(xi + 1, yi,     zi + 1);
	CCVector3d c011 = getCP(xi,     yi + 1, zi + 1);
	CCVector3d c111 = getCP(xi + 1, yi + 1, zi + 1);

	// Interpolate in z direction first
	CCVector3d c00 = c000 * (1.0 - fw) + c001 * fw;
	CCVector3d c10 = c100 * (1.0 - fw) + c101 * fw;
	CCVector3d c01 = c010 * (1.0 - fw) + c011 * fw;
	CCVector3d c11 = c110 * (1.0 - fw) + c111 * fw;

	// Interpolate in y direction
	CCVector3d c0 = c00 * (1.0 - fv) + c01 * fv;
	CCVector3d c1 = c10 * (1.0 - fv) + c11 * fv;

	// Interpolate in x direction
	CCVector3d deformedPoint = c0 * (1.0 - fu) + c1 * fu;

	return deformedPoint;
}
