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
#include "ccSensor.h"
#include "ccOctree.h"

//CCLib
#include <ReferenceCloud.h>

//! Camera (projective) sensor
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccCameraSensor : public ccSensor
#else
class ccCameraSensor :	public ccSensor
#endif
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
	typedef struct FrustumInformation
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
		\param globalCoord image coordinates of the projected point on the image (output) --> !! Note that the first index is (0,0) and the last (width-1,height-1) !!
		\param withLensError if we want to simulate what the projection would be with an imperfect lens
		\return if operation has succeded (typically, errors occur when the projection of the initial 3D points is not into the image boundaries, or when the 3D point is behind the camera)
	**/
	bool fromLocalCoordToImageCoord(const CCVector3& localCoord, CCVector2i& imageCoord, const bool withLensError) const;

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
		\param withLensError if we want to simulate what the projection would be with an imperfect lens
		\return if operation has succeded (typically, errors occur when the projection of the initial 3D points is not into the image boundaries, or when the 3D point is behind the camera)
	**/ 
	bool fromGlobalCoordToImageCoord(const CCVector3& globalCoord, CCVector3& localCoord, CCVector2i& imageCoord, const bool withLensError) const;
	
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
	/**	\param ideal 2D coordinates of the ideal projection (input) --> !! Note that the first index is (0,0) and the last (width-1,height-1) !!
		\param real what would be the real 2D coordinates of the projection trough a lens (output) --> !! Note that the first index is (0,0) and the last (width-1,height-1) !!
	**/
	bool fromIdealImCoordToRealImCoord(const CCVector2i& ideal, CCVector2i& real) const;

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
		\param lensDistortion if we want to take the lens distortion into consideration
		\return success
	**/ 
	bool computeUncertainty(CCLib::ReferenceCloud* points, std::vector<Vector3Tpl<ScalarType>>& accuracy, bool lensDistortion) const;
	
	//! Tests if a 3D point is in the field of view of the camera.
	/** \param globalCoord global coordinates of the 3D point
		\param withLensCorrection if we want to take the lens distortion into consideration
		\return if operation has succeded
	**/ 
	bool isGlobalCoordInFrustrum(const CCVector3& globalCoord, bool withLensCorrection) const;

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
	
protected:

	//! Compute the projection matrix (from intrinsic parameters)
	void computeProjectionMatrix();

	//! Computes the eight corners of the frustrum
	void computeFrustumCorners();

	//! Compute the coefficients of the 6 planes frustrum in the global coordinates system (normal vector are headed the frustrum inside), the edges direction vectors and the frustrum center
	/** \param planeCoefficients coefficients of the six planes
		\param edges direction vectors of the frustrum edges (there are 12 edges but some of them are colinear)
		\param ptsFrustrum the 8 frustrum corners in the global coordinates system
		\param center center of the the frustrum circumscribed sphere 
	**/
	void computeGlobalPlaneCoefficients(float planeCoefficients[6][4], CCVector3 ptsFrustrum[8], CCVector3 edges[6], CCVector3& center);
	
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


#endif //CC_CAMERA_SENSOR_HEADER
