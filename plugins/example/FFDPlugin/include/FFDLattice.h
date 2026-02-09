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

//! Free Form Deformation Lattice Structure
/*!
 * Represents a control lattice grid for FFD transformation.
 * The lattice is composed of control points (nodes) that can be moved.
 * Points in the cloud are transformed based on their trilinear interpolation
 * within the lattice cells, ensuring smooth, continuous deformation.
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
	 * Uses trilinear interpolation to compute the deformed position.
	 * This ensures smooth, continuous transformation without discontinuities.
	 * \param originalPoint the original point coordinates
	 * \return the deformed point coordinates
	 */
	CCVector3d deformPoint( const CCVector3d &originalPoint ) const;

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
	//! Trilinear interpolation weight function
	/*!
	 * Computes the smooth interpolation weight for continuous deformation.
	 * \param t normalized parameter [0, 1]
	 * \return the interpolation weight
	 */
	static double cubicBSplineWeight( double t );

	//! Compute the weight for a control point in the deformation
	double computeControlPointWeight( const CCVector3d &point, 
									   unsigned int cpIndexX, 
									   unsigned int cpIndexY, 
									   unsigned int cpIndexZ ) const;

	std::array<unsigned int, 3> m_latticeSize;      //!< Number of control points in each dimension
	ccBBox m_boundingBox;                            //!< Bounding box of the affected region
	std::vector<CCVector3d> m_controlPoints;         //!< Current positions of control points
	std::vector<CCVector3d> m_originalControlPoints; //!< Original positions (for reset)
	CCVector3d m_cellSize;                           //!< Size of each lattice cell
};

#endif // FFD_LATTICE_HEADER
