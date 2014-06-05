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

//Local
#include "qCC_db.h"
#include "ccSensor.h"
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
class QCC_DB_LIB_API ccGBLSensor : public ccSensor
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

	//! Copy constructor
	ccGBLSensor(const ccGBLSensor & sensor);

	//! Destructor
	virtual ~ccGBLSensor();

	//inherited from ccHObject
	virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::GBL_SENSOR; };
	virtual bool isSerializable() const { return true; }

	//! Sets the lateral angular scanning limits
	/** \param minV min latitude
		\param maxV max latitude
	**/
	void setPhi(PointCoordinateType minV, PointCoordinateType maxV) { m_phiMin = minV; m_phiMax = maxV; }

	//! Sets the lateral angular scanning step
	/** \param dPhi latitudinal step
	**/
	void setDeltaPhi(PointCoordinateType dPhi)  { m_deltaPhi = dPhi; }

	//! Returns the lateral minimal angular scanning limit
	PointCoordinateType getPhiMin() const { return m_phiMin; }

	//! Returns the lateral maximal angular scanning limit
	PointCoordinateType getPhiMax() const { return m_phiMax; }

	//! Returns the lateral angular scanning step
	PointCoordinateType getDeltaPhi() const { return m_deltaPhi; }

	//! Sets the vertical angular scanning limits
	/** \param minV min longitude
		\param maxV max longitude
	**/
	void setTheta(PointCoordinateType minV, PointCoordinateType maxV)  { m_thetaMin = minV; m_thetaMax = maxV; }

	//! Sets the vertical angular scanning step
	/** \param dTheta longitudinal step
	**/
	void setDeltaTheta(PointCoordinateType dTheta) { m_deltaTheta = dTheta; }

	//! Returns the vertical minimal angular scanning limit
	PointCoordinateType getThetaMin() const { return m_thetaMin; }

	//! Returns the vertical maximal angular scanning limit
	PointCoordinateType getThetaMax() const { return m_thetaMax; }

	//! Returns the vertical angular scanning step
	PointCoordinateType getDeltaTheta() const { return m_deltaTheta; }

	//! Returns the sensor max. range
	ScalarType getSensorRange() const { return m_sensorRange; }

	//! Sets the sensor max. range
	/** \param range max. range of the sensor
	**/
	void setSensorRange(ScalarType range) { m_sensorRange = range; }

	//! Returns the Z-buffer uncertainty on depth values
	ScalarType getUncertainty() const { return m_uncertainty; }

	//! Sets the Z-buffer uncertainty on depth values
	/** The uncertainty is used to handle numerical inaccuracies
		\param u the Z-buffer uncertainty
	**/
	void setUncertainty(ScalarType u) { m_uncertainty = u; }

	//! Returns the sensor rotations order
	ROTATION_ORDER getRotationOrder() const { return m_rotationOrder; }

	//! Sets the sensor rotations order
	/** \param rotOrder the sensor rotations order
	**/
	void setRotationOrder(ROTATION_ORDER rotOrder) { m_rotationOrder = rotOrder; }

	//! Projects a point cloud along the sensor point of view defined by this instance
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param errorCode error code in case the returned cloud is 0
		\param autoParameters try to deduce most trivial parameters (min and max angles, max range and uncertainty) from input cloud
		\return a point cloud with the projected 2D points (Theta, Phi) + distances to sensor as a scalar field [should be deleted by the user if not used]
	**/
	CCLib::SimpleCloud* project(CCLib::GenericCloud* cloud, int& errorCode, bool autoParameters = false);

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
	colorType* projectColors(CCLib::GenericCloud* cloud, GenericChunkedArray<3,colorType>& rgbColors) const;

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
		unsigned width;
		//! Buffer height
		unsigned height;

		//! Default constructor
		DepthBuffer();
		//! Destructor
		~DepthBuffer();

		//! Applies a mean filter to fill the small holes (lack of information) of the depth map.
		/**	The depth buffer must have been created before (see GroundBasedLidarSensor::project).
			\return a negative value if an error occurs, 0 otherwise
		**/
		int fillHoles();

	};

	//! Returns the corresponding depth buffer
	/** If the point cloud hasen't been "projected" yet (see GroundBasedLidarSensor::project),
		dB.zBuff will be 0. Otherwise dB will contain the projection result expressed
		as a depth buffer.
	**/
	const DepthBuffer& getDepthBuffer() const { return m_depthBuffer; }

	//Inherited from ccHObject
	virtual ccBBox getMyOwnBB();
	virtual ccBBox getDisplayBB();

protected:

	//Inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);

	//! Projects a point in the sensor world
	/** \param[in] sourcePoint 3D point to project
		\param[out] destPoint projected point in polar coordinates: (theta,phi) or (phi,theta)
		\param[out] depth distance from the sensor optical center to the source point
		\param[in] posIndex (optional) sensor position index (see ccIndexedTransformationBuffer)
	**/
	void projectPoint(const CCVector3& sourcePoint, CCVector2& destPoint, ScalarType &depth, double posIndex = 0) const;

	//! lateral minimal angular scanning limit
	PointCoordinateType m_phiMin;
	//! lateral maximal angular scanning limit
	PointCoordinateType m_phiMax;
	//! lateral angular scanning step
	PointCoordinateType m_deltaPhi;

	//! Vertical minimal angular scanning limit
	PointCoordinateType m_thetaMin;
	//! Vertical maximal angular scanning limit
	PointCoordinateType m_thetaMax;
	//! Vertical angular scanning step
	PointCoordinateType m_deltaTheta;

	//! Mirrors rotation order
	ROTATION_ORDER m_rotationOrder;

	//! Sensor max range
	ScalarType m_sensorRange;
	//! Z-buffer uncertainty
	ScalarType m_uncertainty;

	//! Associated Z-buffer
	DepthBuffer m_depthBuffer;
};

#endif //CC_GROUND_LIDAR_SENSOR_HEADER
