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

#ifndef CC_GROUND_LIDAR_SENSOR_HEADER
#define CC_GROUND_LIDAR_SENSOR_HEADER

#include "ccSensor.h"

//Local
#include "ccGLMatrix.h"

//CCLib
#include <GenericCloud.h>
#include <SimpleCloud.h>
#include <CCGeom.h>

//! Ground based LiDAR sensor model
/** An implementation of the ccSensor interface that can be used
	to project a point cloud from the point of view of a ground
	based laser scanner (generation of a depth map). The scanner
	should use rotating mirrors/body around two perpendicular axes,
	such as the Mensi Soisic and GS sensors (but also Riegl's, etc.).
	The depth map structure can then be used to determine a
	3D point "visiblity" relatively to the laser scanner point of
	view. This can be useful for filtering out points that
	shouldn't be compared while computing the distances between two point
	clouds. See Daniel Girardeau-Montaut's PhD manuscript for more
	information on this particular topic (Chapter 2, section 2.3.3).
**/
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API ccGBLSensor : public ccSensor
#else
class ccGBLSensor : public ccSensor
#endif
{
public:

	//! The order of inner-rotations of a ground based lidar sensor
	/** Either along Theta (long.) then Phi (lat.) as Riegl sensors or Mensi/Trimble GS.
		Or the opposite (as the very old Mensi Soisic).
	**/
	enum ROTATION_ORDER {	THETA_PHI = 0,
							PHI_THETA = 1};

	//! Default constructor
	ccGBLSensor(ROTATION_ORDER rotOrder = THETA_PHI);

	//! Destructor
	virtual ~ccGBLSensor();

    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const { return CC_GBL_SENSOR; };

	//! Sets the lateral angular scanning limits
	/** \param minV min latitude
		\param maxV max latitude
	**/
	void setPhi(float minV, float maxV);

	//! Sets the lateral angular scanning step
	/** \param dPhi latitudinal step
	**/
	void setDeltaPhi(float dPhi);

	//! Returns the lateral minimal angular scanning limit
	float getPhiMin() const;

	//! Returns the lateral maximal angular scanning limit
	float getPhiMax() const;

	//! Returns the lateral angular scanning step
	float getDeltaPhi() const;

	//! Sets the vertical angular scanning limits
	/** \param minV min longitude
		\param maxV max longitude
	**/
	void setTheta(float minV, float maxV);

	//! Sets the vertical angular scanning step
	/** \param dTheta longitudinal step
	**/
	void setDeltaTheta(float dTheta);

	//! Returns the vertical minimal angular scanning limit
	float getThetaMin() const;

	//! Returns the vertical maximal angular scanning limit
	float getThetaMax() const;

	//! Returns the vertical angular scanning step
	float getDeltaTheta() const;

	//! Returns the sensor base (distance between emitter and reciever)
	float getSensorBase() const;

	//! Sets the sensor base (distance between emitter and reciever)
	/** \param base the sensor base
	**/
	void setSensorBase(PointCoordinateType base);

	//! Returns the sensor max. range
	ScalarType getSensorRange() const;

	//! Sets the sensor max. range
	/** \param range max. range of the sensor
	**/
	void setSensorRange(ScalarType range);

	//! Returns the sensor optical center
	CCVector3 getSensorCenter() const;

	//! Sets the sensor optical center
	/** \param C the sensor optical center
	**/
	void setSensorCenter(const CCVector3& C);

	//! Returns the Z-buffer uncertainty on depth values
	ScalarType getUncertainty() const;

	//! Sets the Z-buffer uncertainty on depth values
	/** The uncertainty is used to handle numerical inaccuracies
		\param u the Z-buffer uncertainty
	**/
	void setUncertainty(ScalarType u);

	//! Returns the sensor attitude
	/** Only rotation part is valid
	**/
	const ccGLMatrix& getOrientationMatrix() const;

	//! Sets the sensor attitude (as a rotation matrix)
	/** \param mat transformation matrix (only rotation)
	**/
	void setOrientationMatrix(const ccGLMatrix& mat);

	//! Returns the sensor rotations order
	ROTATION_ORDER getRotationOrder() const;

	//! Sets the sensor rotations order
	/** \param rotOrder the sensor rotations order
	**/
	void setRotationOrder(ROTATION_ORDER rotOrder);

	//! Projects a point cloud along the sensor point of view defined by this instance
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param errorCode error code in case the returned cloud is 0
		\param autoParameters try to deduce most trivial parameters (min and max angles, max range and uncertainty) from input cloud
		\return a point cloud with the projected 2D points (Theta, Phi) + distances to sensor as a scalar field [should be deleted by the user if not used]
	**/
	CCLib::SimpleCloud* project(CCLib::GenericCloud* cloud, int& errorCode, bool autoParameters=false);

	/** Apply a mean filter to fill the small holes (lack of information) of the depth map.
		The depth buffer must have been created before (see GroundBasedLidarSensor::project).
		\return a negative value if an error occurs, 0 otherwise
	**/
	int fillZBufferHoles();

	//! Projects a set of point cloud normals in the sensor world
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param norms the normals vectors (should have the same size and order as the point cloud)
		\return a bidimensional array of 3D vectors (same size as the depth buffer)
	**/
	PointCoordinateType* projectNormals(CCLib::GenericCloud* cloud, GenericChunkedArray<3,PointCoordinateType>& norms) const;

	//! Projects a set of point cloud colors in the sensor frame defined by this instance
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param rgbColors the RGB colors (should have the same size and order as the point cloud)
		\return a bidimensional array of RGB colors (same size as the depth buffer)
	**/
	uchar* projectColors(CCLib::GenericCloud* cloud, GenericChunkedArray<3,uchar>& rgbColors) const;

	//! Determines a point "visibility"
	/** \param P the point to test
	**/
	virtual uchar checkVisibility(const CCVector3& P) const;

	//! Sensor "depth map"
	/** Contains an array of depth values (along each scanned direction) and its dimensions.
		This array corresponds roughly to what have been "seen" by the sensor during
		acquisition (the 3D points are simply projected in the sensor frame).
	**/
	struct DepthBuffer
	{
		//! Z-Buffer grid
		ScalarType* zBuff;
		//! Buffer width
		int width;
		//! Buffer height
		int height;

		//! Default constructor
		DepthBuffer()
			: zBuff(0)
			, width(0)
			, height(0)
		{
		}

		//! Destructor
		~DepthBuffer()
		{
			if (zBuff)
				delete[] zBuff;
		}
	};

	//! Returns the corresponding depth buffer
	/** If the point cloud hasen't been "projected" yet (see GroundBasedLidarSensor::project),
		dB.zBuff will be 0. Otherwise dB will contain the projection result expressed
		as a depth buffer.
	**/
	const DepthBuffer& getDepthBuffer() const;

	//! Updates graphic representation to reflect current sensor parameters
	void updateGraphicRepresentation();

    //! Sets the sensor graphic representation scale
	void setGraphicScale(double scale);

    //! Returns the sensor graphic representation scale
	double getGraphicScale() const;

    //Inherited from ccHObject
    //virtual ccBBox getMyOwnBB();
    virtual ccBBox getDisplayBB();

protected:

    //Inherited from ccHObject
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

	//! Projects a point in the sensor world
	/** \param[in] sourcePoint 3D point to project
		\param[out] destPoint projected point in polar coordinates: (theta,phi) or (phi,theta)
		\param[out] depth distance from the sensor optical center to the source point
	**/
	void projectPoint(const CCVector3& sourcePoint, CCVector2& destPoint, ScalarType &depth) const;

	//! Base distance (distance form emitter to receptor)
	PointCoordinateType base;
	//! Center (origin)
	CCVector3 sensorCenter;
	//! Orientation
	ccGLMatrix m_orientation;

	//! lateral minimal angular scanning limit
	float phiMin;
	//! lateral maximal angular scanning limit
	float phiMax;
	//! lateral angular scanning step
	float deltaPhi;

	//! Vertical minimal angular scanning limit
	float thetaMin;
	//! Vertical maximal angular scanning limit
	float thetaMax;
	//! Vertical angular scanning step
	float deltaTheta;

	//! Mirrors rotation order
	ROTATION_ORDER rotationOrder;

	//! Sensor max range
	ScalarType sensorRange;
	//! Associated Z-buffer
	DepthBuffer m_depthBuffer;
	//! Z-buffer uncertainty
	ScalarType uncertainty;
	
	//! Sensor graphic representation scale
    double scale;

    //! Bounding-box (body)
    ccBBox bBox;
};

#endif //CC_GROUND_LIDAR_SENSOR_HEADER
