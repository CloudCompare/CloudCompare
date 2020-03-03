//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_GROUND_LIDAR_SENSOR_HEADER
#define CC_GROUND_LIDAR_SENSOR_HEADER

//Local
#include "ccSensor.h"
#include "ccDepthBuffer.h"

//CCLib
#include <GenericCloud.h>

class ccPointCloud;

//! Ground-based Laser sensor
/** An implementation of the ccSensor interface that can be used to represent a depth sensor
	relying on 2 rotations relatively to two perpendicular axes, such as ground based laser
	scanners typically.
**/
class QCC_DB_LIB_API ccGBLSensor : public ccSensor
{
public:

	//! The order of inner-rotations of the sensor (body/mirrors)
	/** Either the first rotation is made around the Z axis (yaw) then around the lateral
		axis (pitch) as most scanners do today (Leica, Riegl, Faro, etc.). Othewise the
		opposite order is used (as the very old Mensi Soisic).
	**/
	enum ROTATION_ORDER {	YAW_THEN_PITCH = 0,
							PITCH_THEN_YAW = 1 };

	//! Default constructor
	/** \param rotOrder inner rotations order
	**/
	explicit ccGBLSensor(ROTATION_ORDER rotOrder = YAW_THEN_PITCH);

	//! Copy constructor
	/** \warning The depth buffer is not copied!
		\param sensor sensor structure to copy
	**/
	ccGBLSensor(const ccGBLSensor& sensor);

	//! Destructor
	~ccGBLSensor() override = default;

	//inherited from ccHObject
	CC_CLASS_ENUM getClassID() const override { return CC_TYPES::GBL_SENSOR; }
	bool isSerializable() const override { return true; }
	ccBBox getOwnBB(bool withGLFeatures = false) override;
	ccBBox getOwnFitBB(ccGLMatrix& trans) override;

	//inherited from ccSensor
	bool applyViewport(ccGenericGLDisplay* win = nullptr) override;

	//! Determines a 3D point "visibility" relatively to the sensor field of view
	/** Relies on the sensor associated depth map (see ccGBLSensor::computeDepthBuffer).
		The depth map is used to determine the "visiblity" of a 3D point relatively to
		the laser scanner field of view. This can be useful for filtering out points
		that shouldn't be compared while computing the distances between two point
		clouds for instance (for more information on this	particular topic, refer to
		Daniel Girardeau-Montaut's PhD manuscript - Chapter 2, section 2.3.3).
		\param P the point to test
		\return the point's visibility (POINT_VISIBLE, POINT_HIDDEN, POINT_OUT_OF_RANGE or POINT_OUT_OF_FOV)
	**/
	unsigned char checkVisibility(const CCVector3& P) const override;

	//! Computes angular parameters automatically (all but the angular steps!)
	/** WARNING: this method uses the cloud global iterator.
	**/
	bool computeAutoParameters(CCLib::GenericCloud* theCloud);

	//! Returns the error string corresponding to an error code
	/** Errors codes are returned by ccGBLSensor::computeDepthBuffer or ccDepthBuffer::fillHoles for instance.
	**/
	static QString GetErrorString(int errorCode);

public: //setters and getters

	//! Sets the pitch scanning limits
	/** \param minPhi min pitch angle (in radians)
		\param maxPhi max pitch angle (in radians)
	**/
	void setPitchRange(PointCoordinateType minPhi, PointCoordinateType maxPhi);

	//! Returns the minimal pitch limit (in radians)
	inline PointCoordinateType getMinPitch() const { return m_phiMin; }

	//! Returns the maximal pitch limit (in radians)
	inline PointCoordinateType getMaxPitch() const { return m_phiMax; }

	//! Sets the pitch step
	/** \param dPhi pitch step (in radians)
	**/
	void setPitchStep(PointCoordinateType dPhi);

	//! Returns the lateral pitch step (in radians)
	inline PointCoordinateType getPitchStep() const { return m_deltaPhi; }

	//! Returns whether the pitch angles are shifted (i.e. between [0 ; 2pi] instead of [-pi ; pi])
	bool picthIsShifted() const { return m_pitchAnglesAreShifted; }

	//! Sets the yaw scanning limits
	/** \param minTheta min yaw angle (in radians)
		\param maxTheta max yaw angle (in radians)
	**/
	void setYawRange(PointCoordinateType minTheta, PointCoordinateType maxTheta);

	//! Returns the minimal yaw limit (in radians)
	inline PointCoordinateType getMinYaw() const { return m_thetaMin; }

	//! Returns the maximal yaw limit (in radians)
	inline PointCoordinateType getMaxYaw() const { return m_thetaMax; }

	//! Sets the yaw step
	/** \param dTheta yaw step (in radians)
	**/
	void setYawStep(PointCoordinateType dTheta);

	//! Returns the yaw step (in radians)
	inline PointCoordinateType getYawStep() const { return m_deltaTheta; }

	//! Returns whether the yaw angles are shifted (i.e. between [0 ; 2pi] instead of [-pi ; pi])
	bool yawIsShifted() const { return m_yawAnglesAreShifted; }

	//! Returns the sensor max. range
	inline PointCoordinateType getSensorRange() const { return m_sensorRange; }

	//! Sets the sensor max. range
	/** \param range max. range of the sensor
	**/
	inline void setSensorRange(PointCoordinateType range) { m_sensorRange = range; }

	//! Returns the Z-buffer uncertainty on depth values
	inline PointCoordinateType getUncertainty() const { return m_uncertainty; }

	//! Sets the Z-buffer uncertainty on depth values
	/** The uncertainty is used to handle numerical inaccuracies
		\param u the Z-buffer uncertainty
	**/
	inline void setUncertainty(PointCoordinateType u) { m_uncertainty = u; }

	//! Returns the sensor internal rotations order
	ROTATION_ORDER getRotationOrder() const { return m_rotationOrder; }

	//! Sets the sensor internal rotations order
	/** \param rotOrder internal rotations order
	**/
	inline void setRotationOrder(ROTATION_ORDER rotOrder) { m_rotationOrder = rotOrder; }

public: //projection tools

	//! Projects a point in the sensor world
	/** \param[in] sourcePoint 3D point to project
		\param[out] destPoint projected point in polar coordinates: (theta,phi) = (yaw,pitch) (angles between [-pi,+pi] or [0 ; 2pi] if the corresponding angle is 'shifted')
		\param[out] depth distance between the sensor optical center and the 3D point
		\param[in] posIndex (optional) sensor position index (see ccIndexedTransformationBuffer)
	**/
	void projectPoint(	const CCVector3& sourcePoint,
						CCVector2& destPoint,
						PointCoordinateType &depth,
						double posIndex = 0 ) const;

	//! 2D grid of normals
	using NormalGrid = std::vector<CCVector3>;

	//! Projects a set of point cloud normals in the sensor world
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param norms the normals vectors (should have the same size and order as the point cloud)
		\param posIndex (optional) sensor position index (see ccIndexedTransformationBuffer)
		\return a bidimensional array of 3D vectors (same size as the depth buffer)
	**/
	NormalGrid* projectNormals(	CCLib::GenericCloud* cloud,
								const NormalGrid& norms,
								double posIndex = 0 ) const;

	//! 2D grid of colors
	using ColorGrid = std::vector<ccColor::Rgb>;

	//! Projects a set of point cloud colors in the sensor frame defined by this instance
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param rgbColors the RGB colors (should have the same size and order as the point cloud)
		\return a set of RGB colors organized as a bidimensional grid (same size as the depth buffer)
	**/
	ColorGrid* projectColors(	CCLib::GenericCloud* cloud,
								const ColorGrid& rgbColors ) const;

public: //depth buffer management

	//! Projects a point cloud along the sensor point of view defined by this instance
	/** WARNING: this method uses the cloud global iterator
		\param cloud a point cloud
		\param errorCode error code in case the returned cloud is 0
		\param projectedCloud optional (empty) cloud to store the projected points
		\return whether the depth buffer was successfully created or not
	**/
	bool computeDepthBuffer(CCLib::GenericCloud* cloud, int& errorCode, ccPointCloud* projectedCloud = nullptr);

	//! Returns the associated depth buffer
	/** Call ccGBLSensor::computeDepthBuffer first otherwise the returned buffer will be 0.
	**/
	inline const ccDepthBuffer& getDepthBuffer() const { return m_depthBuffer; }

	//! Removes the associated depth buffer
	void clearDepthBuffer();

protected:

	//Inherited from ccHObject
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//! Converts 2D angular coordinates (yaw,pitch) in integer depth buffer coordinates
	bool convertToDepthMapCoords(PointCoordinateType yaw, PointCoordinateType pitch, unsigned& i, unsigned& j) const;

	//! Minimal pitch limit (in radians)
	/** Phi = 0 corresponds to the scanner vertical direction (upward) **/
	PointCoordinateType m_phiMin;
	//! Maximal pitch limit (in radians)
	/** Phi = 0 corresponds to the scanner vertical direction (upward) **/
	PointCoordinateType m_phiMax;
	//! Pitch step (in radians)
	PointCoordinateType m_deltaPhi;
	//! Whether the pitch angular range is shifted (i.e in [0 ; 2pi] instead of [-pi ; pi])
	bool m_pitchAnglesAreShifted;

	//! Minimal yaw limit (in radians)
	/** Theta = 0 corresponds to the scanner X direction **/
	PointCoordinateType m_thetaMin;
	//! Maximal yaw limit (in radians)
	/** Theta = 0 corresponds to the scanner X direction **/
	PointCoordinateType m_thetaMax;
	//! Yaw step (in radians)
	PointCoordinateType m_deltaTheta;
	//! Whether the yaw range is shifted (i.e in [0 ; 2pi] instead of [-pi ; pi]))
	bool m_yawAnglesAreShifted;

	//! Mirrors rotation order
	ROTATION_ORDER m_rotationOrder;

	//! Sensor max range
	PointCoordinateType m_sensorRange;
	//! Z-buffer uncertainty
	PointCoordinateType m_uncertainty;

	//! Associated Z-buffer
	ccDepthBuffer m_depthBuffer;
};

#endif //CC_GROUND_LIDAR_SENSOR_HEADER
