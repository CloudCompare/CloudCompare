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

#ifndef FFD_LATTICE_HEADER
#define FFD_LATTICE_HEADER

#include <ccPointCloud.h>
#include <CCGeom.h>
#include <vector>
#include <array>

//! Deformation interpolation type
enum class DeformationType
{
	Linear = 0,  //!< Trilinear interpolation (C0 continuous)
	BSpline = 1  //!< Cubic B-spline interpolation (C2 continuous)
};

//! Free Form Deformation Lattice Structure
/*!
 * Represents a control lattice grid for FFD transformation.
 * The lattice is composed of control points (nodes) that can be moved.
 * Points in the cloud are transformed based on cubic B-spline interpolation
 * over the lattice, ensuring C2-continuous smooth deformation.
 */
class FFDLattice
{
public:
	//! Constructor
	/*!
	 * \param latticeSize number of control points in each dimension (X, Y, Z)
	 * \param boundingBox the bounding box of the point cloud
	 */
	FFDLattice( const std::array<unsigned int, 3> &latticeSize, const ccBBox &boundingBox );

	//! Destructor
	virtual ~FFDLattice() = default;

	//! Move a control point
	/*!
	 * \param indexX X coordinate of the control point
	 * \param indexY Y coordinate of the control point
	 * \param indexZ Z coordinate of the control point
	 * \param displacement the displacement vector to apply
	 */
	void moveControlPoint( unsigned int indexX, unsigned int indexY, unsigned int indexZ, const CCVector3d &displacement );

	//! Get a control point position
	CCVector3d getControlPoint( unsigned int indexX, unsigned int indexY, unsigned int indexZ ) const;

	//! Apply deformation to a single point
	/*!
	 * Uses either trilinear or cubic B-spline interpolation depending
	 * on the current deformation type.
	 * \param originalPoint the original point coordinates
	 * \return the deformed point coordinates
	 */
	CCVector3d deformPoint( const CCVector3d &originalPoint ) const;

	//! Set deformation interpolation type
	void setDeformationType( DeformationType type ) { m_deformationType = type; }

	//! Get deformation interpolation type
	DeformationType getDeformationType() const { return m_deformationType; }

	//! Get lattice size
	const std::array<unsigned int, 3> &getLatticeSize() const { return m_latticeSize; }

	//! Get bounding box
	const ccBBox &getBoundingBox() const { return m_boundingBox; }

	//! Reset lattice to original position (undo all deformations)
	void reset();

	//! Get the number of control points
	size_t getControlPointCount() const { return m_controlPoints.size(); }

	//! Get all control points
	const std::vector<CCVector3d> &getAllControlPoints() const { return m_controlPoints; }

	//! Set all control points (for undo/redo)
	void setAllControlPoints(const std::vector<CCVector3d> &controlPoints);

private:
	//! Cubic uniform B-spline basis functions
	/*!
	 * Evaluates the four cubic B-spline basis functions B0..B3
	 * at parameter t in [0, 1]. These provide C2 continuity.
	 * \param t normalized parameter [0, 1]
	 * \param basis output array of 4 basis function values
	 */
	static void cubicBSplineBasis( double t, double basis[4] );

	//! Trilinear deformation (linear interpolation over 2x2x2 cell)
	CCVector3d deformPointLinear( const CCVector3d &originalPoint ) const;

	//! Cubic B-spline deformation (interpolation over 4x4x4 neighbourhood)
	CCVector3d deformPointBSpline( const CCVector3d &originalPoint ) const;

	//! Recompute the displacement cache from current and original control points
	void updateDisplacementCache();

	DeformationType m_deformationType = DeformationType::BSpline; //!< Current interpolation type
	std::array<unsigned int, 3> m_latticeSize;      //!< Number of control points in each dimension
	ccBBox m_boundingBox;                            //!< Bounding box of the affected region
	std::vector<CCVector3d> m_controlPoints;         //!< Current positions of control points
	std::vector<CCVector3d> m_originalControlPoints; //!< Original positions (for reset)
	std::vector<CCVector3d> m_displacements;         //!< Cached displacements (current - original)
	bool m_dirty = false;                            //!< True if any control point has been moved
	CCVector3d m_cellSize;                           //!< Size of each lattice cell
};

#endif // FFD_LATTICE_HEADER
