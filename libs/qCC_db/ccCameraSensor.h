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

#ifndef CC_CAMERA_SENSOR_HEADER
#define CC_CAMERA_SENSOR_HEADER

//local
#include "qCC_db.h"
#include "ccSensor.h"
#include "ccOctree.h"

//CCLib
#include <ReferenceCloud.h>
#include <DgmOctree.h>

//system
#include <set>
#include <assert.h>

//! Camera (projective) sensor
class QCC_DB_LIB_API ccCameraSensor : public ccSensor
{
public:

	//! Intrinsic parameters of the camera sensor
	struct IntrinsicParameters
	{
		float focalLength;		/**< focal length **/
		float pixelSize[2];		/**< real dimension of one pixel (in meters) **/
		float skew;				/**< skew **/
		float vFieldOfView;		/**< vertical field of view (in Radians) **/
		float zBoundary[2];		/**< zBoundary[0]=zNear ; zBoundary[1]=zFar **/
		int	  imageSize[2];		/**< imageSize[0]=width ; imageSize[1]=height **/
	};

	//! Parameters ion order to correct lens distortion.
	/**	To know how to use K & P parameters, please read:
		"Decentering Distortion of Lenses", Duane C. Brown 
		To know how to use the linearDisparityParams parameter (kinect attribute), please read:
		"Accuracy and Resolution of Kinect Depth Data for Indoor Mapping Applications", K. Khoshelham and S.O. Elberink
	**/
	struct UncertaintyParameters
	{
		float principalPointOffset[2];		/**< offset of the principal point (in meters) **/
		float linearDisparityParams[2];		/**< contains A and B where : 1/Z = A*d' + B (with Z=depth and d'=normalized disparity) **/
		float K_BrownParams[3];				/**< radial parameters Brown's distortion model **/
		float P_BrownParams[2];				/**< tangential parameters Brown's distortion model **/
	};

	//! Frustum information structure
	/** Used to draw the frustrum associated to a camera sensor.
	**/
	struct FrustumInformation
	{
		bool isComputed;
		bool drawFrustum;
		bool drawSidePlanes;
		CCVector3 frustumCorners[8];
		CCVector3 center;					/**< center of the circumscribed sphere **/
		
		FrustumInformation() 
			: isComputed(false)
			, drawFrustum(false) 
			, drawSidePlanes(false)
		{}
	};

	//! Default constructor
	ccCameraSensor();

	//! Destructor
	virtual ~ccCameraSensor();

	//! Returns whether the frustum should be displayed or not
	inline bool frustrumIsDrawn() const { return m_frustrumInfos.drawFrustum; }

	//! Sets whether the frustum should be displayed or not
	inline void drawFrustrum(bool state) { m_frustrumInfos.drawFrustum = state; }

	//! Returns whether the frustum planes should be displayed or not
	inline bool frustrumPlanesAreDrawn() const { return m_frustrumInfos.drawSidePlanes; }

	//! Sets whether the frustum planes should be displayed or not
	inline void drawFrustrumPlanes(bool state) { m_frustrumInfos.drawSidePlanes = state; }

	//! Returns the camera projection matrix
	inline const ccGLMatrix& getProjectionMatrix() const {return m_projecMatrix; }
	
	//! Computes the coordinates of a 3D point in the global coordinate system knowing its coordinates in the sensor coordinate system.
	/** \param localCoord local coordinates of the 3D point (input)
		\param globalCoord corresponding global coordinates of the 3D point (output)
	**/
	bool fromLocalCoordToGlobalCoord(const CCVector3& localCoord, CCVector3& globalCoord) const;

	//! Computes the coordinates of a 3D point in the sensor coordinate system knowing its coordinates in the global coordinate system.
	/** \param globalCoord global coordinates of the 3D point (input)
		\param localCoord corresponding local coordinates of the 3D point (output)
	**/
	bool fromGlobalCoordToLocalCoord(const CCVector3& globalCoord, CCVector3& localCoord) const;
	
	//! Computes the coordinates of a 3D point in the global coordinate system knowing its coordinates in the sensor coordinate system.
	/** \param localCoord local coordinates of the 3D point (input)
		\param imageCoord image coordinates of the projected point on the image (output) --> !! Note that the first index is (0,0) and the last (width-1,height-1) !!
		//TODO: withLensError if we want to simulate what the projection would be with an imperfect lens
		\return if operation has succeded (typically, errors occur when the projection of the initial 3D points is not into the image boundaries, or when the 3D point is behind the camera)
	**/
	bool fromLocalCoordToImageCoord(const CCVector3& localCoord, CCVector2i& imageCoord/*, bool withLensError*/) const;

	//! Computes the coordinates of a 3D point in the sensor coordinate system knowing its coordinates in the global coordinate system.
	/** \param imageCoord image coordinates of the pixel (input) --> !! Note that the first index is (0,0) and the last (width-1,height-1) !!
		\param localCoord local coordinates of the corresponding 3D point (output)
		\param withLensCorrection if we want to correct the initial pixel coordinates with the lens correction formula
		\param depth if known, depth of the input pixel in meters, in order to recover third coordinates (must be positive) ; if depth is 0.0, then the reprojection is made in the focal plane 
		\return if operation has succeded (typically, errors occur when the initial pixel coordinates are not into the image boundaries)
	**/
	bool fromImageCoordToLocalCoord(const CCVector2i& imageCoord, CCVector3& localCoord, bool withLensCorrection, float depth = 0) const;

	//! Computes the coordinates of a 3D point in the image knowing its coordinates in the global coordinate system.
	/** \param globalCoord global coordinates of the 3D point
		\param localCoord to get back the local coordinates of the 3D point
		\param imageCoord to get back the image coordinates of the projected 3D point --> !! Note that the first index is (0,0) and the last (width-1,height-1) !!
		//TODO withLensError if we want to simulate what the projection would be with an imperfect lens
		\return if operation has succeded (typically, errors occur when the projection of the initial 3D points is not into the image boundaries, or when the 3D point is behind the camera)
	**/ 
	bool fromGlobalCoordToImageCoord(const CCVector3& globalCoord, CCVector3& localCoord, CCVector2i& imageCoord/*, bool withLensError*/) const;
	
	//! Computes the global coordinates of a 3D points from its 3D coordinates (pixel position in the image)
	/** \param imageCoord image coordinates of the pixel (input) --> !! Note that the first index is (0,0) and the last (width-1,height-1) !!
		\param localCoord local coordinates of the corresponding 3D point (output)
		\param globalCoord global coordinates of the corresponding 3D point (output)
		\param withLensCorrection if we want to correct the initial pixel coordinates with the lens correction formula
		\param depth if known, depth of the input pixel, in order to recover third coordinates (must be positive) ; if depth is 0.0, then the reprojection is made in the focal plane
		\return if operation has succeded (typically, errors occur when the initial pixel coordinates are not into the image boundaries)
	**/
	bool fromImageCoordToGlobalCoord(const CCVector2i& imageCoord, CCVector3& localCoord, CCVector3& globalCoord, bool withLensCorrection, float depth = 0) const;

	//! Apply the Brown's lens correction to the real projection (through a lens) of a 3D point in the image
	/**	\param real real 2D coordinates of a pixel (asumming that this pixel coordinate is obtained after projection through a lens) (input) !! Note that the first index is (0,0) and the last (width-1,height-1) !!
		\param ideal after applying lens correction (output) --> !! Note that the first index is (0,0) and the last (width-1,height-1) !!
	**/
	bool fromRealImCoordToIdealImCoord(const CCVector2i& real, CCVector2i& ideal) const;

	//! Knowing the ideal projection of a 3D point, computes what would be the real projection (through a lens)
	/** \warning The first pixel is (0,0) and the last (width-1,height-1)
		\param[in] ideal 2D coordinates of the ideal projection
		\param[out] real what would be the real 2D coordinates of the projection trough a lens
	**/
	//TODO
	//bool fromIdealImCoordToRealImCoord(const CCVector2i& ideal, CCVector2i& real) const;

	//! Computes the uncertainty of a point knowing its depth (from the sensor view point) and pixel projection coordinates
	/**	\param pixel coordinates of the pixel where the 3D points is projected --> !! Note that the first index is (0,0) and the last (width-1,height-1) !!
		\param depth depth from sensor center to 3D point (must be positive)
		\param sigma uncertainty vector (along X, Y and Z) 
		\return operation has succeded (typically, errors occur when the initial pixel coordinates are not into the image boundaries, or when the depth of the 3D point is negative)
	**/
	bool computeUncertainty(const CCVector2i& pixel, const float depth, Vector3Tpl<ScalarType>& sigma) const;
	
	//! Computes the coordinates of a 3D point in the sensor coordinate system knowing its coordinates in the global coordinate system.
	/** \param points the points we want to compute the uncertainty
		\param accuracy to get back the uncertainty
		//TODO lensDistortion if we want to take the lens distortion into consideration
		\return success
	**/ 
	bool computeUncertainty(CCLib::ReferenceCloud* points, std::vector< Vector3Tpl<ScalarType> >& accuracy/*, bool lensDistortion*/) const;
	
	//! Tests if a 3D point is in the field of view of the camera.
	/** \param globalCoord global coordinates of the 3D point
		//TODO withLensCorrection if we want to take the lens distortion into consideration
		\return if operation has succeded
	**/ 
	bool isGlobalCoordInFrustrum(const CCVector3& globalCoord/*, bool withLensCorrection*/) const;

	//! Filters an octree : all the box visible in the frustum will be drawn in red.
	/** \param octree Octree
		\param inCameraFrustrum indices of points in the frustrum
	**/
	void filterOctree(ccOctree* octree, std::vector<unsigned>& inCameraFrustrum);
	
	//inherited from ccHObject
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::CAMERA_SENSOR; }
	virtual bool isSerializable() const { return true; }
	virtual ccBBox getMyOwnBB();
	//virtual ccBBox getDisplayBB();

	//! Compute the coefficients of the 6 planes frustrum in the global coordinates system (normal vector are headed the frustrum inside), the edges direction vectors and the frustrum center
	/** \param planeCoefficients coefficients of the six planes
		\param edges direction vectors of the frustrum edges (there are 12 edges but some of them are colinear)
		\param ptsFrustrum the 8 frustrum corners in the global coordinates system
		\param center center of the the frustrum circumscribed sphere 
	**/
	void computeGlobalPlaneCoefficients(float planeCoefficients[6][4], CCVector3 ptsFrustrum[8], CCVector3 edges[6], CCVector3& center);
	
protected:

	//! Compute the projection matrix (from intrinsic parameters)
	void computeProjectionMatrix();

	//! Computes the eight corners of the frustrum
	void computeFrustumCorners();

	//Inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

protected:

	//! Camera intrinsic parameters
	IntrinsicParameters m_intrinsicParams;

	//! Lens distortion parameters 
	UncertaintyParameters m_uncertaintyParams;

	//! Frustrum information structure
	/** Used to draw it properly.
	**/
	FrustumInformation m_frustrumInfos;

	//! Intrinsic parameters matrix
	ccGLMatrix m_projecMatrix;	
};

class ccOctreeFrustrumIntersector
{
public:
	//! Definition of the state of a cell compared to a frustrum
	/** OUTSIDE : the celle is completely outside the frustrum (no intersection, no inclusion)
		INSIDE : the cell is completely inside the frustrum
		INTERSECT : other cases --> the frustrum is completely inside the cell OR the frustrum and the cell have an intersection
	**/
	enum OctreeCellVisibility
	{
		CELL_OUTSIDE_FRUSTRUM	= 0,
		CELL_INSIDE_FRUSTRUM	= 1,
		CELL_INTERSECT_FRUSTRUM	= 2,
	};

	//! Default constructor
	ccOctreeFrustrumIntersector()
		: m_associatedOctree(0)
	{
	}

	//! Prepares structure for frustrum filtering
	bool build(CCLib::DgmOctree* octree);

	//! Returns the cell visibility
	OctreeCellVisibility positionFromFrustum(CCLib::DgmOctree::OctreeCellCodeType truncatedCode, uchar level) const
	{
		assert(m_associatedOctree);

		std::set<CCLib::DgmOctree::OctreeCellCodeType>::const_iterator got = m_cellsInFrustum[level].find(truncatedCode);
		if (got != m_cellsInFrustum[level].end())
			return CELL_INSIDE_FRUSTRUM;
		got = m_cellsIntersectFrustum[level].find(truncatedCode);
		if (got != m_cellsIntersectFrustum[level].end())
			return CELL_INTERSECT_FRUSTRUM;
		return CELL_OUTSIDE_FRUSTRUM;
	}

	//! Compute intersection betwen the octree and a frustrum and send back the indices of 3D points inside the frustrum or in cells interescting it. 
	/** Every cells of each level of the octree will be classified as INSIDE, OUTSIDE or INTERSECTING the frustrum. 
		Their truncated code are then stored in m_cellsInFrustum (for cells INSIDE) or m_cellsIntersectFrustum (for 
		cells INTERSECTING).
		\param pointsToTest contains the indice and 3D position (global coordinates system) of every 3D points stored in an INTERSECTING cell
		\param inCameraFrustrum contains the indice of every 3D points stored in an INSIDE cell
		\param planesCoefficients coefficients (a, b, c and d) of the six frustrum planes (0:right, 1:bottom, 2:left, 3:top, 4:near, 5:far)
		\param ptsFrustrum 3D coordinates of the eight corners of the frustrum (global coordinates sytem)
		\param edges 3D coordinates (global coordinates sytem) of the six director vector of the frustrum edges
		\param center 3D coordinates of the frustrum center (global coordinates sytem) ; this is the center of the circumscribed sphere
	**/
	void computeFrustumIntersectionWithOctree(	std::vector< std::pair<unsigned, CCVector3> >& pointsToTest,
												std::vector<unsigned>& inCameraFrustrum,
												const float planesCoefficients[6][4],
												const CCVector3 ptsFrustrum[8],
												const CCVector3 edges[6],
												const CCVector3& center);
	
	//! Compute intersection betwen the octree and the height children cells of a parent cell. 
	/** \param level current level
		\param parentTruncatedCode truncated code of the parent cell (at level-1)
		\param parentResult contains in which class the parent cell has been classified (OUTSIDE, INTERSECTING, INSIDE)
		\param planesCoefficients coefficients (a, b, c and d) of the six frustrum planes (0:right, 1:bottom, 2:left, 3:top, 4:near, 5:far)
		\param ptsFrustrum 3D coordinates of the eight corners of the frustrum (global coordinates sytem)
		\param edges 3D coordinates (global coordinates sytem) of the six director vector of the frustrum edges
		\param center 3D coordinates of the frustrum center (global coordinates sytem) ; this is the center of the circumscribed sphere
	**/
	void computeFrustumIntersectionByLevel(	unsigned char level,
											CCLib::DgmOctree::OctreeCellCodeType parentTruncatedCode,
											OctreeCellVisibility parentResult,
											const float planesCoefficients[6][4],
											const CCVector3 ptsFrustrum[8],
											const CCVector3 edges[6],
											const CCVector3& center);
	
	//! Separating Axis Test
	/** See "Detecting intersection of a rectangular solid and a convex polyhedron" of Ned Greene 
		See	"OBBTree: A Hierarchical Structure for Rapid Interference Detection" of S. Gottschalk, M. C. Lin and D. Manocha
		\param bbMin minimum coordinates of the cell
		\param bbMax maximum coordinates of the cell
		\param planesCoefficients coefficients (a, b, c and d) of the six frustrum planes (0:right, 1:bottom, 2:left, 3:top, 4:near, 5:far)
		\param frustrumCorners 3D coordinates of the eight corners of the frustrum (global coordinates sytem)
		\param frustrumEdges 3D coordinates (global coordinates sytem) of the six director vector of the frustrum edges
		\param frustrumCenter 3D coordinates of the frustrum center (global coordinates sytem) ; this is the center of the circumscribed sphere
	**/
	OctreeCellVisibility separatingAxisTest(const CCVector3& bbMin,
											const CCVector3& bbMax,
											const float planesCoefficients[6][4],
											const CCVector3 frustrumCorners[8],
											const CCVector3 frustrumEdges[6],
											const CCVector3& frustrumCenter);

protected:

	CCLib::DgmOctree* m_associatedOctree;

	// contains the truncated code of the cells built in the octree
	std::set<CCLib::DgmOctree::OctreeCellCodeType> m_cellsBuilt[CCLib::DgmOctree::MAX_OCTREE_LEVEL+1];
	// contains the truncated code of the cells INSIDE the frustrum
	std::set<CCLib::DgmOctree::OctreeCellCodeType> m_cellsInFrustum[CCLib::DgmOctree::MAX_OCTREE_LEVEL+1];
	// contains the truncated code of the cells INTERSECTING the frustrum
	std::set<CCLib::DgmOctree::OctreeCellCodeType> m_cellsIntersectFrustum[CCLib::DgmOctree::MAX_OCTREE_LEVEL+1];
};


#endif //CC_CAMERA_SENSOR_HEADER
