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

void FFDLattice::cubicBSplineBasis( double t, double basis[4] )
{
	// Cubic uniform B-spline basis functions evaluated at t in [0, 1].
	// These four functions sum to 1 and provide C2 continuity.
	//   B0(t) = (1-t)^3 / 6
	//   B1(t) = (3t^3 - 6t^2 + 4) / 6
	//   B2(t) = (-3t^3 + 3t^2 + 3t + 1) / 6
	//   B3(t) = t^3 / 6
	t = std::clamp( t, 0.0, 1.0 );

	double t2 = t * t;
	double t3 = t2 * t;
	double omt = 1.0 - t;
	double omt3 = omt * omt * omt;

	basis[0] = omt3 / 6.0;
	basis[1] = ( 3.0 * t3 - 6.0 * t2 + 4.0 ) / 6.0;
	basis[2] = ( -3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0 ) / 6.0;
	basis[3] = t3 / 6.0;
}

CCVector3d FFDLattice::deformPoint( const CCVector3d &originalPoint ) const
{
	switch (m_deformationType)
	{
	case DeformationType::Linear:
		return deformPointLinear(originalPoint);
	case DeformationType::BSpline:
	default:
		return deformPointBSpline(originalPoint);
	}
}

CCVector3d FFDLattice::deformPointLinear( const CCVector3d &originalPoint ) const
{
	// Compute normalized coordinates within the bounding box [0, 1]
	CCVector3 bbMin = m_boundingBox.minCorner();
	CCVector3 bbMax = m_boundingBox.maxCorner();

	CCVector3d pointRelative = originalPoint - CCVector3d(bbMin.x, bbMin.y, bbMin.z);
	CCVector3d bbSize = CCVector3d(bbMax.x - bbMin.x, bbMax.y - bbMin.y, bbMax.z - bbMin.z);

	double u = std::clamp(pointRelative.x / bbSize.x, 0.0, 1.0);
	double v = std::clamp(pointRelative.y / bbSize.y, 0.0, 1.0);
	double w = std::clamp(pointRelative.z / bbSize.z, 0.0, 1.0);

	// Convert to lattice grid coordinates
	double gridX = u * (m_latticeSize[0] - 1);
	double gridY = v * (m_latticeSize[1] - 1);
	double gridZ = w * (m_latticeSize[2] - 1);

	// Get integer cell index and fractional part
	int xi = static_cast<int>(std::floor(gridX));
	int yi = static_cast<int>(std::floor(gridY));
	int zi = static_cast<int>(std::floor(gridZ));

	// Clamp cell indices to valid range
	xi = std::clamp(xi, 0, static_cast<int>(m_latticeSize[0]) - 2);
	yi = std::clamp(yi, 0, static_cast<int>(m_latticeSize[1]) - 2);
	zi = std::clamp(zi, 0, static_cast<int>(m_latticeSize[2]) - 2);

	double fu = gridX - xi;
	double fv = gridY - yi;
	double fw = gridZ - zi;

	// Helper to get displacement at a control point
	auto getDisp = [this](int x, int y, int z) -> CCVector3d {
		size_t idx = static_cast<size_t>(z) * m_latticeSize[1] * m_latticeSize[0]
		           + static_cast<size_t>(y) * m_latticeSize[0]
		           + static_cast<size_t>(x);
		return m_controlPoints[idx] - m_originalControlPoints[idx];
	};

	// Trilinear interpolation of displacement over the 2x2x2 cell
	CCVector3d d000 = getDisp(xi,     yi,     zi);
	CCVector3d d100 = getDisp(xi + 1, yi,     zi);
	CCVector3d d010 = getDisp(xi,     yi + 1, zi);
	CCVector3d d110 = getDisp(xi + 1, yi + 1, zi);
	CCVector3d d001 = getDisp(xi,     yi,     zi + 1);
	CCVector3d d101 = getDisp(xi + 1, yi,     zi + 1);
	CCVector3d d011 = getDisp(xi,     yi + 1, zi + 1);
	CCVector3d d111 = getDisp(xi + 1, yi + 1, zi + 1);

	// Interpolate in z
	CCVector3d d00 = d000 * (1.0 - fw) + d001 * fw;
	CCVector3d d10 = d100 * (1.0 - fw) + d101 * fw;
	CCVector3d d01 = d010 * (1.0 - fw) + d011 * fw;
	CCVector3d d11 = d110 * (1.0 - fw) + d111 * fw;

	// Interpolate in y
	CCVector3d d0 = d00 * (1.0 - fv) + d01 * fv;
	CCVector3d d1 = d10 * (1.0 - fv) + d11 * fv;

	// Interpolate in x
	CCVector3d displacement = d0 * (1.0 - fu) + d1 * fu;

	return originalPoint + displacement;
}

CCVector3d FFDLattice::deformPointBSpline( const CCVector3d &originalPoint ) const
{
	// Compute normalized coordinates within the bounding box [0, 1]
	CCVector3 bbMin = m_boundingBox.minCorner();
	CCVector3 bbMax = m_boundingBox.maxCorner();
	
	CCVector3d pointRelative = originalPoint - CCVector3d(bbMin.x, bbMin.y, bbMin.z);
	CCVector3d bbSize = CCVector3d(bbMax.x - bbMin.x, bbMax.y - bbMin.y, bbMax.z - bbMin.z);

	double u = std::clamp(pointRelative.x / bbSize.x, 0.0, 1.0);
	double v = std::clamp(pointRelative.y / bbSize.y, 0.0, 1.0);
	double w = std::clamp(pointRelative.z / bbSize.z, 0.0, 1.0);

	// Convert to lattice grid coordinates
	double gridX = u * (m_latticeSize[0] - 1);
	double gridY = v * (m_latticeSize[1] - 1);
	double gridZ = w * (m_latticeSize[2] - 1);

	// Get integer cell index and fractional part
	int xi = static_cast<int>(std::floor(gridX));
	int yi = static_cast<int>(std::floor(gridY));
	int zi = static_cast<int>(std::floor(gridZ));

	// Clamp cell indices to valid range
	xi = std::clamp(xi, 0, static_cast<int>(m_latticeSize[0]) - 2);
	yi = std::clamp(yi, 0, static_cast<int>(m_latticeSize[1]) - 2);
	zi = std::clamp(zi, 0, static_cast<int>(m_latticeSize[2]) - 2);

	double fu = gridX - xi;
	double fv = gridY - yi;
	double fw = gridZ - zi;

	// Evaluate cubic B-spline basis functions for each axis
	double basisU[4], basisV[4], basisW[4];
	cubicBSplineBasis(fu, basisU);
	cubicBSplineBasis(fv, basisV);
	cubicBSplineBasis(fw, basisW);

	// Accumulate B-spline interpolated displacement over the 4x4x4 neighbourhood.
	// We interpolate the *displacement* (current CP - original CP) rather than
	// absolute positions, which guarantees an identity mapping when no control
	// points have been moved.
	int maxX = static_cast<int>(m_latticeSize[0]) - 1;
	int maxY = static_cast<int>(m_latticeSize[1]) - 1;
	int maxZ = static_cast<int>(m_latticeSize[2]) - 1;

	CCVector3d displacement(0, 0, 0);

	for (int dz = -1; dz <= 2; ++dz)
	{
		int cz = std::clamp(zi + dz, 0, maxZ);
		double wW = basisW[dz + 1];

		for (int dy = -1; dy <= 2; ++dy)
		{
			int cy = std::clamp(yi + dy, 0, maxY);
			double wV = basisV[dy + 1];

			for (int dx = -1; dx <= 2; ++dx)
			{
				int cx = std::clamp(xi + dx, 0, maxX);
				double weight = basisU[dx + 1] * wV * wW;

				size_t idx = static_cast<size_t>(cz) * m_latticeSize[1] * m_latticeSize[0]
				           + static_cast<size_t>(cy) * m_latticeSize[0]
				           + static_cast<size_t>(cx);

				displacement += (m_controlPoints[idx] - m_originalControlPoints[idx]) * weight;
			}
		}
	}

	return originalPoint + displacement;
}
