//##########################################################################
//#                                                                        #
//#                               CCLIB                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU Library General Public License as       #
//#  published by the Free Software Foundation; version 2 of the License.  #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************
//

#ifndef GROUND_LIDAR_HEADER
#define GROUND_LIDAR_HEADER

#include "GenericSensor.h"
#include "GenericChunkedArray.h"
#include "Matrix.h"

namespace CCLib
{

class GenericCloud;
class GenericIndexedCloud;
class SimpleCloud;

//! The rotations order of a ground based lidar sensor
/** Either along Theta (long.) then Phi (lat.) as Riegl sensors or Mensi GS.
    Or the opposite as Mensi Soisic.
**/
enum CC_SENSOR_ROTATION_ORDER {GBL_THETA_PHI /*GS,RIEGL*/, GBL_PHI_THETA /*SOISIC*/};

//! Ground based LiDAR sensor mirror and body rotation order
const char CC_SENSOR_ROTATION_ORDER_NAMES[2][16] = {
													"THETA_PHI",		//Rotation: body then mirror (Mensi GS-like)
													"PHI_THETA"			//Rotation: mirror then body (Mensi Soisic-like)
};

//! Ground based LiDAR sensor model
/** An implementation of the GenericSensor class that can be used
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

#ifdef CC_USE_AS_DLL
#include "CloudCompareDll.h"

class CC_DLL_API GroundBasedLidarSensor : public GenericSensor
#else
class GroundBasedLidarSensor : public GenericSensor
#endif
{

public:

	//! the GroundBasedLidarSensor constructor
	/** \param rotOrder the sensor rotations order
	**/
	GroundBasedLidarSensor(CC_SENSOR_ROTATION_ORDER rotOrder);

	//! the GroundBasedLidarSensor destructor
	virtual ~GroundBasedLidarSensor();

    //inherited from GenericSensor
	virtual CC_SENSOR_TYPE getType() {return GROUND_BASED_LIDAR;}

	//! Sets the lateral angular scanning limits
	/** \param minV min latitude
		\param maxV max latitude
	**/
	void setPhi(float minV, float maxV);

	//! Sets the lateral minimal angular scanning limit
	/** \param minV min latitude
	**/
	void setPhiMin(float minV);

	//! Sets the lateral maximal angular scanning limit
	/** \param maxV max latitude
	**/
	void setPhiMax(float maxV);

	//! Sets the lateral angular scanning step
	/** \param dPhi latitudinal step
	**/
	void setDeltaPhi(float dPhi);

	//! Returns the lateral minimal angular scanning limit
	/** \return min latitude
	**/
	float getPhiMin();

	//! Returns the lateral maximal angular scanning limit
	/** \return max latitude
	**/
	float getPhiMax();

	//! Returns the lateral angular scanning step
	/** \return latitudinal step
	**/
	float getDeltaPhi();

	//! Sets the vertical angular scanning limits
	/** \param minV min longitude
		\param maxV max longitude
	**/
	void setTheta(float minV, float maxV);

	//! Sets the vertical minimal angular scanning limit
	/** \param minV min longitude
	**/
	void setThetaMin(float minV);

	//! Sets the vertical maximal angular scanning limit
	/** \param maxV max longitude
	**/
	void setThetaMax(float maxV);

	//! Sets the vertical angular scanning step
	/** \param dTheta longitudinal step
	**/
	void setDeltaTheta(float dTheta);

	//! Returns the vertical minimal angular scanning limit
	/** \return min longitude
	**/
	float getThetaMin();

	//! Returns the vertical maximal angular scanning limit
	/** \return max longitude
	**/
	float getThetaMax();

	//! Returns the vertical angular scanning step
	/** \return longitudinal step
	**/
	float getDeltaTheta();

	//! Returns the sensor base (distance between emitter and reciever)
	/** \return the sensor base
	**/
	float getSensorBase();

	//! Sets the sensor base (distance between emitter and reciever)
	/** \param _base the sensor base
	**/
	void setSensorBase(PointCoordinateType _base);

	//! Returns the sensor max. range
	/** \return the sensor max. range
	**/
	DistanceType getSensorRange();

	//! Sets the sensor max. range
	/** \param range max. range of the sensor
	**/
	void setSensorRange(DistanceType range);

	//! Returns the sensor optical center
	/** \return the sensor optical center (as a 3-size array)
	**/
	CCVector3 getSensorCenter();

	//! Sets the sensor optical center
	/** \param C the sensor optical center (as a 3-size array)
	**/
	void setSensorCenter(const CCVector3& C);

	//! Returns the Z-buffer uncertainty distance
	/** The uncertainty is used to handle numerical inaccuracies
		\return the Z-buffer uncertainty distance
	**/
	DistanceType getUncertainty();

	//! Sets the Z-buffer uncertainty distance
	/** The uncertainty is used to handle numerical inaccuracies
		\param u the Z-buffer uncertainty distance
	**/
	void setUncertainty(DistanceType u);


	//! Returns the sensor attitude (as a rotation matrix)
	/** \return a rotation matrix
	**/
	CCLib::SquareMatrix* getRotationMatrix();

	//! Sets the sensor attitude (as a rotation matrix)
	/** \param m a rotation matrix
	**/
	void setRotationMatrix(CCLib::SquareMatrix* m);

	//! Returns the sensor attitude (as 3 axes)
	/** Roughly the inverse of the rotation matrix (see getRotationMatrix).
        \return 3 vectors as a matrix
	**/
	CCLib::SquareMatrix* getAxisMatrix();

	//! Sets the sensor attitude (as 3 axes)
	/** Roughly the inverse of the rotation matrix (see setRotationMatrix).
        \param m 3 vectors as a matrix
	**/
	void setAxisMatrix(CCLib::SquareMatrix* m);

	//! Returns the sensor rotations order
	/** \return the sensor rotations order
	**/
	CC_SENSOR_ROTATION_ORDER getRotationOrder();

	//! Sets the sensor rotations order
	/** \param r the sensor rotations order
	**/
	void setRotationOrder(CC_SENSOR_ROTATION_ORDER r);

	//! Projects a point cloud along the sensor point of view defined by this instance
	/** WARNING: this method uses the cloud global iterator
		\param aCloud a point cloud
		\param errorCode error code in case the returned cloud is 0
		\param getParameters if true, some trivial parameters should be determined directly from the data (min and max angles, max range and uncertainty)
		\return a point cloud with the projected points with coordinates (Theta, Phi, Distance to sensor) [can be deleted by the user if not used]
	**/
	SimpleCloud* project(GenericCloud* aCloud, int& errorCode, bool getParameters=false);

	/** Apply a mean filter to fill the small holes (lack of information) of the depth map.
		The depth buffer must have been created before (see GroundBasedLidarSensor::project).
		\return a negative value if an error occurs, 0 otherwise
	**/
	int fillZBufferHoles();

	//! Projects a set of point cloud normals in the sensor frame defined by this instance
	/** WARNING: this method uses the cloud global iterator
		\param aCloud a point cloud
		\param theNorms the normals vectors (should have the same size and order as the point cloud)
		\return a bidimensional array of 3D vectors (same size as the depth buffer)
	**/
	PointCoordinateType* projectNormals(GenericCloud* aCloud, GenericChunkedArray<3,PointCoordinateType>& theNorms);

	//! Projects a set of point cloud colors in the sensor frame defined by this instance
	/** WARNING: this method uses the cloud global iterator
		\param aCloud a point cloud
		\param theColors the RGB colors (should have the same size and order as the point cloud)
		\return a bidimensional array of RGB colors (same size as the depth buffer)
	**/
	uchar* projectColors(GenericCloud* aCloud, GenericChunkedArray<3,uchar>& theColors);

	//! Determines a point "visibility"
	/** \param aPoint the point to test
	**/
	virtual CC_VISIBILITY_TYPE checkVisibility(const CCVector3& aPoint);

	//! Sensor "depth map"
	/** Contains an array of depth values (along each scanned direction) and its dimensions.
		This array corresponds roughly to what have been "seen" by the sensor during
		acquisition (the 3D points are simply projected in the sensor frame).
	**/
	struct DepthBuffer
	{
	    //! Z-Buffer
		DistanceType* zBuff;
		//! Buffer width
		int l_buff;
		//! Buffer height
		int h_buff;
	};

	//! Returns the corresponding depth buffer
	/** If the point cloud hasen't been "projected" yet (see GroundBasedLidarSensor::project),
		dB.zBuff will be 0. Otherwise dB will contain the projection result expressed
		as a depth buffer.
		\return a depth buffer description (see GroundBasedLidarSensor::DepthBuffer)
	**/
	DepthBuffer getDepthBuffer();


protected :

	//! Projects a point cloud along the scanner point of view defined by this instance
	/** \param sourcePoint the point to project
		\param destPoint the projected point - (theta,phi) or (phi,theta) - will be stored here
		\param dist the distance between the sensor optical center and the source point will be stored here
	**/
	void projectPoint(const CCVector3& sourcePoint, CCVector2& destPoint, DistanceType &dist);

	//! Base distance (distance form emitter to receptor)
	PointCoordinateType base;
	//! Center (origin)
	CCVector3 sensorCenter;
	//! Axis (inverse of rotation)
	CCLib::SquareMatrix *axis; //TODO: inverse is the just the transpose! We could supress this attribute
	//! Rotation
	CCLib::SquareMatrix *rotation;

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
	CC_SENSOR_ROTATION_ORDER rotationOrder;

	//! Sensor max range
	DistanceType sensorRange;
	//! Associated Z-buffer
	DepthBuffer dB;
	//! Z-buffer uncertainty
	DistanceType uncertainty;
};

}

#endif
