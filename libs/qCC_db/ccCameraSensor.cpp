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

#include <cmath>

#include "ccCameraSensor.h"

//local
#include "ccGenericGLDisplay.h"
#include "ccImage.h"
#include "ccMesh.h"
#include "ccPointCloud.h"

//CCLib
#include <ConjugateGradient.h>

//Qt
#include <QDir>
#include <QTextStream>

ccCameraSensor::IntrinsicParameters::IntrinsicParameters()
	: vertFocal_pix(1.0f)
	, skew(0)
	, vFOV_rad(0)
	, zNear_mm(0.001f)
	, zFar_mm(1000.0f)
	, arrayWidth(0)
	, arrayHeight(0)
{
	pixelSize_mm[0] = 1.0f;
	pixelSize_mm[1] = 1.0f;

	principal_point[0] = arrayWidth / 2.0f;
	principal_point[1] = arrayHeight / 2.0f;
}

void ccCameraSensor::IntrinsicParameters::GetKinectDefaults(IntrinsicParameters& params)
{
	//default Kinect parameters from:
	// "Accuracy and Resolution of Kinect Depth Data for Indoor Mapping Applications"
	// Kourosh Khoshelham and Sander Oude Elberink
	constexpr float focal_mm		= static_cast<float>(5.45 * 1.0e-3);	// focal length (real distance in meter)
	constexpr float pixelSize_mm	= static_cast<float>(9.3 * 1.0e-6);		// pixel size (real distance in meter)
	
	params.vertFocal_pix      = ConvertFocalMMToPix(focal_mm, pixelSize_mm);
	params.pixelSize_mm[0]    = pixelSize_mm;
	params.pixelSize_mm[1]    = pixelSize_mm;
	params.skew               = static_cast<float>(0.0);							// skew in image
	params.vFOV_rad           = static_cast<float>(43.0 * M_PI / 180.0);			// vertical field of view (in rad)
	params.zNear_mm           = static_cast<float>(0.5);							// distance to the closest recordable depth
	params.zFar_mm            = static_cast<float>(5.0);							// distance to the furthest recordable depth
	params.arrayWidth         = 640;												// image width
	params.arrayHeight        = 480;												// image height
	params.principal_point[0] = params.arrayWidth / 2.0f;
	params.principal_point[1] = params.arrayHeight / 2.0f;
}

ccCameraSensor::BrownDistortionParameters::BrownDistortionParameters()
{
	principalPointOffset[0]  = 0;
	principalPointOffset[1]  = 0;
	linearDisparityParams[0] = 0;
	linearDisparityParams[1] = 0;
	K_BrownParams[0]         = 0;
	K_BrownParams[1]         = 0;
	K_BrownParams[2]         = 0;
	P_BrownParams[0]         = 0;
	P_BrownParams[1]         = 0;
}

void ccCameraSensor::BrownDistortionParameters::GetKinectDefaults(BrownDistortionParameters& params)
{
	//default Kinect parameters from:
	// "Accuracy and Resolution of Kinect Depth Data for Indoor Mapping Applications"
	// Kourosh Khoshelham and Sander Oude Elberink
	params.principalPointOffset[0]  = static_cast<float>(-0.063 * 1.0e-3);
	params.principalPointOffset[1]  = static_cast<float>(-0.039 * 1.0e-3);
	params.linearDisparityParams[0] = static_cast<float>(-2.85 * 1.0e-3);
	params.linearDisparityParams[1] = static_cast<float>(3.0);
	params.K_BrownParams[0]         = static_cast<float>(2.42 * 1.0e-3);
	params.K_BrownParams[1]         = static_cast<float>(-1.7 * 1.0e-4);
	params.K_BrownParams[2]         = static_cast<float>(0.0);
	params.P_BrownParams[0]         = static_cast<float>(-3.3 * 1.0e-4);
	params.P_BrownParams[1]         = static_cast<float>(5.25 * 1.0e-4);
}

ccCameraSensor::FrustumInformation::FrustumInformation()
	: isComputed(false)
	, drawFrustum(false)
	, drawSidePlanes(false)
	, frustumCorners(nullptr)
	, frustumHull(nullptr)
{}

ccCameraSensor::FrustumInformation::~FrustumInformation()
{
	//always delete the hull before the corners, are it depends on them!
	if (frustumHull)
	{
		delete frustumHull;
		frustumHull = nullptr;
	}
	if (frustumCorners)
	{
		delete frustumCorners;
		frustumCorners = nullptr;
	}
}

bool ccCameraSensor::FrustumInformation::initFrustumCorners()
{
	if (!frustumCorners)
	{
		frustumCorners = new ccPointCloud("Frustum corners");
	}
	else
	{
		frustumCorners->clear();
	}

	if (!frustumCorners->reserve(8))
	{
		//not enough memory to load frustum corners!
		delete frustumCorners;
		frustumCorners = nullptr;
		return false;
	}
	return true;
}

bool ccCameraSensor::FrustumInformation::initFrustumHull()
{
	//we only need to do this once!
	if (frustumHull)
		return true;

	if (!frustumCorners || frustumCorners->size() < 8)
	{
		ccLog::Warning("[ccCameraSensor::FrustumInformation::initFrustumHull] Corners are not initialized!");
		return false;
	}

	frustumHull = new ccMesh(frustumCorners);
	if (!frustumHull->reserve(6 * 2))
	{
		ccLog::Warning("[ccCameraSensor::FrustumInformation::initFrustumHull] Not enough memory!");
		delete frustumHull;
		frustumHull = nullptr;
		return false;
	}

	frustumHull->addTriangle(0, 2, 3);
	frustumHull->addTriangle(0, 3, 1);

	frustumHull->addTriangle(2, 4, 5);
	frustumHull->addTriangle(2, 5, 3);

	frustumHull->addTriangle(4, 6, 7);
	frustumHull->addTriangle(4, 7, 5);

	frustumHull->addTriangle(6, 0, 1);
	frustumHull->addTriangle(6, 1, 7);

	frustumHull->addTriangle(6, 4, 2);
	frustumHull->addTriangle(6, 2, 0);

	frustumHull->addTriangle(1, 3, 5);
	frustumHull->addTriangle(1, 5, 7);

	frustumHull->setVisible(true);

	return true;
}

ccCameraSensor::ccCameraSensor()
	: ccSensor("Camera Sensor")
	, m_projectionMatrixIsValid(false)
{
	//graphic representation
	lockVisibility(false);
	setSelectionBehavior(SELECTION_FIT_BBOX);
}

ccCameraSensor::ccCameraSensor(const IntrinsicParameters& iParams)
	: ccSensor("Camera Sensor")
	, m_projectionMatrixIsValid(false)
{
	//graphic representation
	lockVisibility(false);
	setSelectionBehavior(SELECTION_FIT_BBOX);

	// projection
	setIntrinsicParameters(iParams);
}

ccCameraSensor::ccCameraSensor(const ccCameraSensor& sensor)
	: ccSensor(sensor)
	, m_projectionMatrix(sensor.m_projectionMatrix)
	, m_projectionMatrixIsValid(false)
{
	setIntrinsicParameters(sensor.m_intrinsicParams);

	//distortion params
	if (m_distortionParams)
	{
		LensDistortionParameters::Shared clonedDistParams;
		switch (m_distortionParams->getModel())
		{
		case SIMPLE_RADIAL_DISTORTION:
		{
			//simply duplicate the struct
			RadialDistortionParameters* clone = new RadialDistortionParameters;
			*clone = *static_cast<const RadialDistortionParameters*>(sensor.m_distortionParams.data());
			clonedDistParams = LensDistortionParameters::Shared(clone);
		}
		break;

		case EXTENDED_RADIAL_DISTORTION:
		{
			//simply duplicate the struct
			ExtendedRadialDistortionParameters* clone = new ExtendedRadialDistortionParameters;
			*clone = *static_cast<const ExtendedRadialDistortionParameters*>(sensor.m_distortionParams.data());
			clonedDistParams = LensDistortionParameters::Shared(clone);
		}
		break;

		case BROWN_DISTORTION:
		{
			//simply duplicate the struct
			BrownDistortionParameters* clone = new BrownDistortionParameters;
			*clone = *static_cast<const BrownDistortionParameters*>(sensor.m_distortionParams.data());
			clonedDistParams = LensDistortionParameters::Shared(clone);
		}
		break;

		default:
			//unhandled type?!
			assert(false);
			break;
		}
		setDistortionParameters(clonedDistParams);
	}
}

ccBBox ccCameraSensor::getOwnBB(bool withGLFeatures/*=false*/)
{
	if (!withGLFeatures)
	{
		return ccBBox();
	}

	//get current sensor position
	ccIndexedTransformation sensorPos;
	if (!getAbsoluteTransformation(sensorPos, m_activeIndex))
	{
		return ccBBox();
	}

	CCVector3 upperLeftPoint = computeUpperLeftPoint();

	ccPointCloud cloud;
	if (!cloud.reserve(5))
	{
		//not enough memory?!
		return ccBBox();
	}

	cloud.addPoint(CCVector3(0, 0, 0));
	cloud.addPoint(CCVector3( upperLeftPoint.x, upperLeftPoint.y,-upperLeftPoint.z));
	cloud.addPoint(CCVector3(-upperLeftPoint.x, upperLeftPoint.y,-upperLeftPoint.z));
	cloud.addPoint(CCVector3(-upperLeftPoint.x,-upperLeftPoint.y,-upperLeftPoint.z));
	cloud.addPoint(CCVector3( upperLeftPoint.x,-upperLeftPoint.y,-upperLeftPoint.z));

	//add frustum corners if necessary
	if (m_frustumInfos.isComputed
		&& (m_frustumInfos.drawFrustum || m_frustumInfos.drawSidePlanes)
		&& m_frustumInfos.frustumCorners)
	{
		unsigned cornerCount = m_frustumInfos.frustumCorners->size();
		if (cloud.reserve(cloud.size() + cornerCount))
		{
			for (unsigned i = 0; i < cornerCount; ++i)
				cloud.addPoint(*m_frustumInfos.frustumCorners->getPoint(i));
		}
	}

	cloud.applyRigidTransformation(sensorPos);
	return cloud.getOwnBB(false);
}

ccBBox ccCameraSensor::getOwnFitBB(ccGLMatrix& trans)
{
	//get current sensor position
	ccIndexedTransformation sensorPos;
	if (!getAbsoluteTransformation(sensorPos,m_activeIndex))
	{
		return ccBBox();
	}

	trans = sensorPos;

	CCVector3 upperLeftPoint = computeUpperLeftPoint();
	return ccBBox(-upperLeftPoint, CCVector3(upperLeftPoint.x, upperLeftPoint.y, 0));
}

void ccCameraSensor::setVertFocal_pix(float vertFocal_pix)
{
	assert(vertFocal_pix > 0);
	m_intrinsicParams.vertFocal_pix = vertFocal_pix;

	//old frustum is not valid anymore!
	m_frustumInfos.isComputed = false;
	//same thing for the projection matrix
	m_projectionMatrixIsValid = false;
}

void ccCameraSensor::setVerticalFov_rad(float fov_rad)
{
	assert(fov_rad > 0);
	m_intrinsicParams.vFOV_rad = fov_rad;
}

void ccCameraSensor::setIntrinsicParameters(const IntrinsicParameters& params)
{
	m_intrinsicParams = params;
	//old frustum is not valid anymore!
	m_frustumInfos.isComputed = false;
	//same thing for the projection matrix
	m_projectionMatrixIsValid = false;
}

bool ccCameraSensor::applyViewport(ccGenericGLDisplay* win/*=0*/)
{
	if (!win)
	{
		win = getDisplay();
		if (!win)
		{
			ccLog::Warning("[ccCameraSensor::applyViewport] No associated display!");
			return false;
		}
	}

	ccIndexedTransformation trans;
	if (!getActiveAbsoluteTransformation(trans))
	{
		return false;
	}

	if (m_intrinsicParams.arrayHeight <= 0)
	{
		ccLog::Warning("[ccCameraSensor::applyViewport] Sensor height is 0!");
		return false;
	}

	//aspect ratio
	float ar = static_cast<float>(m_intrinsicParams.arrayWidth) / m_intrinsicParams.arrayHeight;
	//fov
	float fov_deg = static_cast<float>(m_intrinsicParams.vFOV_rad * CC_RAD_TO_DEG);
	//camera position/orientation
	ccGLMatrixd transd(trans.data());
	win->setupProjectiveViewport(transd, fov_deg, ar);

	return true;
}

bool ccCameraSensor::getProjectionMatrix(ccGLMatrix& matrix)
{
	if (!m_projectionMatrixIsValid)
		computeProjectionMatrix();

	matrix = m_projectionMatrix;

	return m_projectionMatrixIsValid; //even if we have computed the projection matrix, it may still have failed!
}

void ccCameraSensor::computeProjectionMatrix()
{
	m_projectionMatrix.toZero();
	float* mat = m_projectionMatrix.data();

	//diagonal
	mat[0]  = getHorizFocal_pix();
	mat[5]  = getVertFocal_pix();
	mat[10] = 1.0f;
	mat[15] = 1.0f;

	//skew
	mat[4]  = m_intrinsicParams.skew;

	//translation from image (0,0)
	mat[12] = m_intrinsicParams.principal_point[0];
	mat[13] = m_intrinsicParams.principal_point[1];

	m_projectionMatrixIsValid = true;
}

bool ccCameraSensor::toFile_MeOnly(QFile& out) const
{
	if (!ccSensor::toFile_MeOnly(out))
		return false;

	//projection matrix (35 <= dataVersion < 38)
	//if (!m_projectionMatrix.toFile(out))
	//	return WriteError();

	/** various parameters (dataVersion>=35) **/

	//IntrinsicParameters
	QDataStream outStream(&out);
	outStream << m_intrinsicParams.vertFocal_pix;
	outStream << m_intrinsicParams.arrayWidth;
	outStream << m_intrinsicParams.arrayHeight;
	outStream << m_intrinsicParams.pixelSize_mm[0];
	outStream << m_intrinsicParams.pixelSize_mm[1];
	outStream << m_intrinsicParams.skew;
	outStream << m_intrinsicParams.vFOV_rad;
	outStream << m_intrinsicParams.zNear_mm;
	outStream << m_intrinsicParams.zFar_mm;
	outStream << m_intrinsicParams.principal_point[0];
	outStream << m_intrinsicParams.principal_point[1];

	//distortion parameters (dataVersion>=38)
	DistortionModel distModel = m_distortionParams ? m_distortionParams->getModel() : NO_DISTORTION_MODEL;
	outStream << static_cast<uint32_t>(distModel);

	if (m_distortionParams)
	{
		switch(m_distortionParams->getModel())
		{
		case SIMPLE_RADIAL_DISTORTION:
			{
				RadialDistortionParameters* params = static_cast<RadialDistortionParameters*>(m_distortionParams.data());
				outStream << params->k1;
				outStream << params->k2;
			}
			break;

		case EXTENDED_RADIAL_DISTORTION:
		{
			ExtendedRadialDistortionParameters* params = static_cast<ExtendedRadialDistortionParameters*>(m_distortionParams.data());
			outStream << params->k1;
			outStream << params->k2;
			outStream << params->k3;
		}
		break;

		case BROWN_DISTORTION:
			{
				BrownDistortionParameters* params = static_cast<BrownDistortionParameters*>(m_distortionParams.data());
				outStream << params->K_BrownParams[0];
				outStream << params->K_BrownParams[1];
				outStream << params->K_BrownParams[2];
				outStream << params->P_BrownParams[0];
				outStream << params->P_BrownParams[1];
				outStream << params->principalPointOffset[0];
				outStream << params->principalPointOffset[1];
				outStream << params->linearDisparityParams[0];
				outStream << params->linearDisparityParams[1];
			}
			break;
		default:
			assert(false);
			break;
		}
	}

	//FrustumInformation
	outStream << m_frustumInfos.drawFrustum;
	outStream << m_frustumInfos.drawSidePlanes;
	outStream << m_frustumInfos.center.x;
	outStream << m_frustumInfos.center.y;
	outStream << m_frustumInfos.center.z;

	return true;
}

bool ccCameraSensor::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccSensor::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//serialization wasn't possible before v3.5!
	if (dataVersion < 35)
		return false;

	//projection matrix (35 <= dataVersion < 38)
	if (dataVersion < 38)
	{
		//we don't need to save/load this matrix as it is dynamically computed!
		ccGLMatrix dummyMatrix;
		if (!dummyMatrix.fromFile(in, dataVersion, flags, oldToNewIDMap))
			return ReadError();
	}
	m_projectionMatrixIsValid = false;

	/** various parameters (dataVersion>=35) **/

	//IntrinsicParameters
	QDataStream inStream(&in);
	inStream >> m_intrinsicParams.vertFocal_pix;
	inStream >> m_intrinsicParams.arrayWidth;
	inStream >> m_intrinsicParams.arrayHeight;
	inStream >> m_intrinsicParams.pixelSize_mm[0];
	inStream >> m_intrinsicParams.pixelSize_mm[1];
	inStream >> m_intrinsicParams.skew;
	inStream >> m_intrinsicParams.vFOV_rad;
	inStream >> m_intrinsicParams.zNear_mm;
	inStream >> m_intrinsicParams.zFar_mm;

	if (dataVersion >= 43)
	{
		//since version 43, we added the principal point
		inStream >> m_intrinsicParams.principal_point[0];
		inStream >> m_intrinsicParams.principal_point[1];
	}
	else
	{
		m_intrinsicParams.principal_point[0] = m_intrinsicParams.arrayWidth / 2.0f;
		m_intrinsicParams.principal_point[1] = m_intrinsicParams.arrayHeight / 2.0f;
	}

	//distortion parameters
	DistortionModel distModel = NO_DISTORTION_MODEL;
	if (dataVersion < 38)
	{
		//before v38, only Brown's parameters were used (and always set)
		distModel = BROWN_DISTORTION;
	}
	else
	{
		uint32_t distModeli;
		inStream >> distModeli;
		distModel = static_cast<DistortionModel>(distModeli);
	}

	//load parameters (if any)
	switch (distModel)
	{
	case SIMPLE_RADIAL_DISTORTION:
		{
			RadialDistortionParameters* distParams = new RadialDistortionParameters;
			inStream >> distParams->k1;
			inStream >> distParams->k2;

			setDistortionParameters(LensDistortionParameters::Shared(distParams));
		}
		break;

	case EXTENDED_RADIAL_DISTORTION:
	{
		ExtendedRadialDistortionParameters* distParams = new ExtendedRadialDistortionParameters;
		inStream >> distParams->k1;
		inStream >> distParams->k2;
		inStream >> distParams->k3;

		setDistortionParameters(LensDistortionParameters::Shared(distParams));
	}
	break;

	case BROWN_DISTORTION:
		{
			BrownDistortionParameters* distParams = new BrownDistortionParameters;
			inStream >> distParams->K_BrownParams[0];
			inStream >> distParams->K_BrownParams[1];
			inStream >> distParams->K_BrownParams[2];
			inStream >> distParams->P_BrownParams[0];
			inStream >> distParams->P_BrownParams[1];
			inStream >> distParams->principalPointOffset[0];
			inStream >> distParams->principalPointOffset[1];
			inStream >> distParams->linearDisparityParams[0];
			inStream >> distParams->linearDisparityParams[1];

			setDistortionParameters(LensDistortionParameters::Shared(distParams));
		}
		break;

	default:
		//do nothing
		break;
	}

	//FrustumInformation
	if (dataVersion < 38)
	{
		bool dummyBool; //formerly: m_frustumInfos.isComputed (no need to save/load it!)
		inStream >> dummyBool;
	}
	m_frustumInfos.isComputed = false;
	inStream >> m_frustumInfos.drawFrustum;
	inStream >> m_frustumInfos.drawSidePlanes;
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, m_frustumInfos.center.u, 3);

	if (dataVersion < 38)
	{
		//frustum corners: no need to save/load them!
		for (unsigned i = 0; i < 8; ++i)
		{
			CCVector3 P;
			ccSerializationHelper::CoordsFromDataStream(inStream, flags, P.u, 3);
		}
	}

	return true;
}

bool ccCameraSensor::fromLocalCoordToGlobalCoord(const CCVector3& localCoord, CCVector3& globalCoord) const
{
	ccIndexedTransformation trans;

	if (!getActiveAbsoluteTransformation(trans))
		return false;

	globalCoord = localCoord;
	trans.apply(globalCoord);

	return true;
}

bool ccCameraSensor::fromGlobalCoordToLocalCoord(const CCVector3& globalCoord, CCVector3& localCoord) const
{
	ccIndexedTransformation trans;

	if (!getActiveAbsoluteTransformation(trans))
		return false;

	localCoord = globalCoord;
	trans.inverse().apply(localCoord);

	return true;
}

bool ccCameraSensor::fromLocalCoordToImageCoord(const CCVector3& localCoord, CCVector2& imageCoord, bool withLensError/*=true*/) const
{
#ifdef CHECK_THIS_AFTERWARDS

	// Change in 3D image coordinates system for good projection
	CCVector3 imageCoordSystem(localCoord.x, localCoord.y, -localCoord.z);

	// We test if the point is in front or behind the sensor ? If it is behind (or in the center of the sensor i.e. z=0.0), we can't project!
	if (imageCoordSystem.z < FLT_EPSILON)
		return false;

	// projection
	ccGLMatrix mat;
	if (!getProjectionMatrix(mat))
		return false;
	CCVector3 projCoord = mat * imageCoordSystem; // at this stage, coordinates are homogeneous
	projCoord = projCoord/projCoord.z; // coordinates are now in pixels
	CCVector2 initial(projCoord.x, projCoord.y);
	CCVector2 coord = initial;

	//apply lens correction if necessary
	//if (withLensError)
	//	fromIdealImCoordToRealImCoord(initial, coord);

	//test if the projected point is into the image boundaries (width,height)
	if (	coord.x < 0 || coord.x >= m_intrinsicParams.arrayWidth
		||	coord.y < 0 || coord.y >= m_intrinsicParams.arrayHeight )
	{
		return false;
	}

	// Change in 3D image coordinates system
	imageCoord = coord;

#else

	// We test if the point is in front or behind the sensor ? If it is behind (or in the center of the sensor i.e. depth = 0), we can't project!
	double depth = -static_cast<double>(localCoord.z); //warning: the camera looks backward!
#define BACK_POINTS_CULLING
#ifdef BACK_POINTS_CULLING
	if (depth < FLT_EPSILON)
		return false;
#endif

	//perspective division
	CCVector2d p(localCoord.x / depth, localCoord.y / depth);

	//conversion to pixel coordinates
	double factor = m_intrinsicParams.vertFocal_pix;

	//apply radial distortion (if any)
	if (withLensError && m_distortionParams)
	{
		if (m_distortionParams->getModel() == SIMPLE_RADIAL_DISTORTION)
		{
			const RadialDistortionParameters* params = static_cast<RadialDistortionParameters*>(m_distortionParams.data());
			double norm2 = p.norm2();
			double rp = 1.0 + norm2  * (params->k1 + norm2 * params->k2); //scaling factor to undo the radial distortion
			factor *= rp;
		}
		else if (m_distortionParams->getModel() == EXTENDED_RADIAL_DISTORTION)
		{
			const ExtendedRadialDistortionParameters* params = static_cast<ExtendedRadialDistortionParameters*>(m_distortionParams.data());
			double norm2 = p.norm2();
			double rp = 1.0 + norm2  * (params->k1 + norm2 * (params->k2 + norm2 * params->k3)); //scaling factor to undo the radial distortion
			factor *= rp;
		}
	}
	//*/

	CCVector2d p2 = p * factor;

	p2.x += m_intrinsicParams.principal_point[0];
	p2.y = m_intrinsicParams.principal_point[1] - p2.y;

	imageCoord.x = static_cast<PointCoordinateType>(p2.x);
	imageCoord.y = static_cast<PointCoordinateType>(p2.y);

#endif

	return true;
}

bool ccCameraSensor::fromImageCoordToLocalCoord(const CCVector2& imageCoord, CCVector3& localCoord, PointCoordinateType depth, bool withLensCorrection/*=true*/) const
{
	CCVector3d p2(imageCoord.x, imageCoord.y, 0.0);

	p2.x -= m_intrinsicParams.principal_point[0];
	p2.y = m_intrinsicParams.principal_point[1] - p2.y;

	//apply inverse radial distortion (if any)
	//TODO

	double factor = static_cast<double>(m_intrinsicParams.vertFocal_pix);
	CCVector3d p = p2 / factor;

	//perspective
	localCoord = CCVector3(	static_cast<PointCoordinateType>(p.x * depth),
							static_cast<PointCoordinateType>(p.y * depth),
							-depth);

	return true;
}

bool ccCameraSensor::fromGlobalCoordToImageCoord(const CCVector3& globalCoord, CCVector2& imageCoord, bool withLensError/*=true*/) const
{
	CCVector3 localCoord;
	if (!fromGlobalCoordToLocalCoord(globalCoord,localCoord))
		return false;

	return fromLocalCoordToImageCoord(localCoord, imageCoord, withLensError);
}

bool ccCameraSensor::fromImageCoordToGlobalCoord(const CCVector2& imageCoord, CCVector3& globalCoord, PointCoordinateType z0, bool withLensCorrection/*=true*/) const
{
	ccIndexedTransformation trans;

	if (!getActiveAbsoluteTransformation(trans))
		return false;

	CCVector3 localCoord;
	if (!fromImageCoordToLocalCoord(imageCoord, localCoord, PC_ONE, withLensCorrection))
		return false;

	//update altitude: we must compute the intersection between the plane Z = Z0 (world) and the camera (input pixel) viewing direction
	CCVector3 viewDir = localCoord;
	trans.applyRotation(viewDir);
	viewDir.normalize();

	if (fabs(viewDir.z) < ZERO_TOLERANCE)
	{
		//viewing dir is parallel to the plane Z = Z0!
		return false;
	}

	CCVector3 camC = trans.getTranslationAsVec3D();
	PointCoordinateType dZ = z0 - camC.z;

	PointCoordinateType u = dZ / viewDir.z;
#ifdef BACK_POINTS_CULLING
	if (u < 0)
		return false; //wrong direction!
#endif

	globalCoord = camC + u * viewDir;

	return true;
}

bool ccCameraSensor::fromRealImCoordToIdealImCoord(const CCVector2& real, CCVector2& ideal) const
{
	//no distortion parameters?
	if (!m_distortionParams)
	{
		ideal = real;
		return true;
	}

	switch (m_distortionParams->getModel())
	{
	case SIMPLE_RADIAL_DISTORTION:
	case EXTENDED_RADIAL_DISTORTION:
		{
			//TODO: we need a pre-computed distortion map to do this!
		}
		break;

	case BROWN_DISTORTION:
		{
			const BrownDistortionParameters* distParams = static_cast<BrownDistortionParameters*>(m_distortionParams.data());
			const float& sX = m_intrinsicParams.pixelSize_mm[0];
			const float& sY = m_intrinsicParams.pixelSize_mm[1];

			// 1st correction : principal point correction
			float cx = m_intrinsicParams.principal_point[0] + distParams->principalPointOffset[0] / sX; // in pixels
			float cy = m_intrinsicParams.principal_point[1] + distParams->principalPointOffset[1] / sY; // in pixels

			// 2nd correction : Brown's lens distortion correction
			float dx = (static_cast<float>(real.x) - cx) * m_intrinsicParams.pixelSize_mm[0];	// real distance
			float dy = (static_cast<float>(real.y) - cy) * m_intrinsicParams.pixelSize_mm[1];	// real distance
			float dx2 = dx*dx;
			float dy2 = dy*dy;
			float r = sqrt(dx2 + dy2);
			float r2 = r*r;
			float r4 = r2*r2;
			float r6 = r4*r2;
			const float& K1 = distParams->K_BrownParams[0];
			const float& K2 = distParams->K_BrownParams[1];
			const float& K3 = distParams->K_BrownParams[2];
			const float& P1 = distParams->P_BrownParams[0];
			const float& P2 = distParams->P_BrownParams[1];

			// compute new value
			float correctedX = (dx * (1 + K1*r2 + K2*r4 + K3*r6)  +  P1 * (r2 + 2*dx2)  +  2*P2*dx*dy);
			float correctedY = (dy * (1 + K1*r2 + K2*r4 + K3*r6)  +  P2 * (r2 + 2*dy2)  +  2*P1*dx*dy);
			ideal.x = static_cast<PointCoordinateType>(correctedX / sX);
			ideal.y = static_cast<PointCoordinateType>(correctedY / sY);

			// We test if the new pixel falls inside the image boundaries
			//return (	ideal.x >= 0 && ideal.x < m_intrinsicParams.arrayWidth
			//		&&	ideal.y >= 0 && ideal.y < m_intrinsicParams.arrayHeight );
			//DGM: the ideal pixel can be outside of the original image of course!!!
			return true;
		}

	default:
		//not handled?
		assert(false);
		break;
	}

	return false;
}

//TODO
//bool ccCameraSensor::fromIdealImCoordToRealImCoord(const CCVector2& ideal, CCVector2& real) const
//{
//	return true;
//}

bool ccCameraSensor::computeUncertainty(const CCVector2& pixel, const float depth, Vector3Tpl<ScalarType>& sigma) const
{
	//no distortion parameters?
	if (!m_distortionParams)
	{
		return false;
	}

	switch (m_distortionParams->getModel())
	{
	case SIMPLE_RADIAL_DISTORTION:
	case EXTENDED_RADIAL_DISTORTION:
		{
			//TODO
			return false;
		}

	case BROWN_DISTORTION:
		{
			const BrownDistortionParameters* distParams = static_cast<BrownDistortionParameters*>(m_distortionParams.data());
			//TODO ==> check if the input pixel coordinate must be the real or ideal projection

			const int& width = m_intrinsicParams.arrayWidth;
			const int& height = m_intrinsicParams.arrayHeight;
			const float* c = m_intrinsicParams.principal_point;

			// check validity
			if (	pixel.x < 0 || pixel.x > width
				||	pixel.y < 0 || pixel.y > height
				||	depth < FLT_EPSILON )
				return false;

			// init parameters
			const float& A = distParams->linearDisparityParams[0];
			float z2 = depth*depth;
			float invSigmaD = 8.0f;
			float factor = A * z2 / invSigmaD;

			const float& mu = m_intrinsicParams.pixelSize_mm[0];
			const float verFocal_pix = getVertFocal_pix();
			const float horizFocal_pix = getHorizFocal_pix();

			// computes uncertainty
			sigma.x = static_cast<ScalarType>(std::abs(factor * (pixel.x - c[0]) / horizFocal_pix));
			sigma.y = static_cast<ScalarType>(std::abs(factor * (pixel.y - c[1]) / verFocal_pix));
			sigma.z = static_cast<ScalarType>(std::abs(factor * mu));

			return true;
		}

	default:
		{
			//not handled?
			assert(false);
		}
		break;
	}

	return false;
}

bool ccCameraSensor::computeUncertainty(CCLib::ReferenceCloud* points, std::vector< Vector3Tpl<ScalarType> >& accuracy/*, bool lensCorrection*/)
{
	if (!points || points->size() == 0)
	{
		ccLog::Warning("[ccCameraSensor::computeUncertainty] Internal error: invalid input cloud");
		return false;
	}

	if (!m_distortionParams || m_distortionParams->getModel() != BROWN_DISTORTION)
	{
		ccLog::Warning("[ccCameraSensor::computeUncertainty] Sensor has no associated uncertainty model! (Brown, etc.)");
		return false;
	}

	unsigned count = points->size();
	accuracy.clear();
	try
	{
		accuracy.resize(count);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccCameraSensor::computeUncertainty] Not enough memory!");
		return false;
	}

	for (unsigned i = 0; i < count; i++)
	{
		const CCVector3* coordGlobal = points->getPoint(i);
		CCVector3 coordLocal;
		CCVector2 coordImage;

		if (	fromGlobalCoordToLocalCoord(*coordGlobal,coordLocal)
			&&	fromLocalCoordToImageCoord(coordLocal, coordImage) )
		{
			computeUncertainty(coordImage, std::abs(coordLocal.z), accuracy[i]);
		}
		else
		{
			accuracy[i].x = accuracy[i].y = accuracy[i].z = NAN_VALUE;
		}
	}

	return true;
}

//see http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html
QImage ccCameraSensor::undistort(const QImage& image) const
{
	if (image.isNull())
	{
		ccLog::Warning("[ccCameraSensor::undistort] Invalid input image!");
		return QImage();
	}

	//nothing to do
	//no distortion parameters?
	if (!m_distortionParams)
	{
		ccLog::Warning("[ccCameraSensor::undistort] No distortion model set!");
		return QImage();
	}

	switch (m_distortionParams->getModel())
	{
	case SIMPLE_RADIAL_DISTORTION:
	case EXTENDED_RADIAL_DISTORTION:
		{
			const RadialDistortionParameters* params = static_cast<RadialDistortionParameters*>(m_distortionParams.data());
			float k1 = params->k1;
			float k2 = params->k2;
			if (k1 == 0 && k2 == 0)
			{
				ccLog::Warning("[ccCameraSensor::undistort] Invalid radial distortion coefficients!");
				return QImage();
			}
			float k3 = 0;
			if (m_distortionParams->getModel() == EXTENDED_RADIAL_DISTORTION)
			{
				k3 = static_cast<ExtendedRadialDistortionParameters*>(m_distortionParams.data())->k3;
			}

			int width = image.width();
			int height = image.height();

			float xScale = image.width() / static_cast<float>(m_intrinsicParams.arrayWidth);
			float yScale = image.height() / static_cast<float>(m_intrinsicParams.arrayHeight);
			float rScale = sqrt(xScale * xScale + yScale * yScale);

			//try to reserve memory for new image
			QImage newImage(QSize(width, height), image.format());
			if (newImage.isNull())
			{
				ccLog::Warning("[ccCameraSensor::undistort] Not enough memory!");
				return QImage();
			}
			newImage.fill(0);

			float vertFocal_pix = getVertFocal_pix() * xScale;
			float horizFocal_pix = getHorizFocal_pix() * yScale;
			float vf2 = vertFocal_pix * vertFocal_pix;
			float hf2 = horizFocal_pix * horizFocal_pix;
			float cx = m_intrinsicParams.principal_point[0] * xScale;
			float cy = m_intrinsicParams.principal_point[1] * yScale;
			k1 *= rScale;
			k2 *= rScale;
			k3 *= rScale;

			assert((image.depth() % 8) == 0);
			int depth = image.depth() / 8;
			int bytesPerLine = image.bytesPerLine();
			const uchar* iImageBits = image.bits();
			uchar* oImageBits = newImage.bits();

			//image undistortion
			{
				for (int i = 0; i < width; ++i)
				{
					float x = i - cx;
					float x2 = x*x;
					for (int j = 0; j < height; ++j)
					{
						float y = j - cy;
						float y2 = y*y;

						float p2 = x2 / hf2 + y2 / vf2; //p = pix/f
						float rp = 1.0f + p2 * (k1 + p2 * (k2 + p2 * k3)); //r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4 + k3 * ||p||^6
						float eqx = rp * x + cx;
						float eqy = rp * y + cy;

						int pixx = static_cast<int>(eqx);
						int pixy = static_cast<int>(eqy);
						if (	pixx >= 0
							&&	pixx < width
							&&	pixy >= 0
							&&	pixy < height)
						{
							const uchar* iPixel = iImageBits + j * bytesPerLine + i * depth;
							uchar* oPixel = oImageBits + pixy * bytesPerLine + pixx * depth;
							memcpy(oPixel, iPixel, depth);
							//newImage.setPixel(i, j, image.pixel(pixx, pixy));
						}
					}
				}
			}

			return newImage;
		}

	case BROWN_DISTORTION:
		//TODO
		break;

	default:
		//not handled?
		assert(false);
		break;
	}

	ccLog::Warning("[ccCameraSensor::undistort] Can't undistort the image with the current distortion model!");

	return QImage();
}

ccImage* ccCameraSensor::undistort(ccImage* image, bool inplace/*=true*/) const
{
	if (!image || image->data().isNull())
	{
		ccLog::Warning("[ccCameraSensor::undistort] Invalid/empty input image!");
		return nullptr;
	}

	QImage newImage = undistort(image->data());
	if (newImage.isNull())
	{
		//warning message should have been already issued
		return nullptr;
	}

	//update image parameters
	if (inplace)
	{
		image->setData(newImage);
		return image;
	}

	return new ccImage(newImage, image->getName() + QString(".undistort"));
}

bool ccCameraSensor::isGlobalCoordInFrustum(const CCVector3& globalCoord/*, bool withLensCorrection*/) const
{
	CCVector3 localCoord;

	// Tests if the projection is in the field of view
	if (!fromGlobalCoordToLocalCoord(globalCoord, localCoord/*, withLensCorrection*/))
		return false;

	// Tests if the projected point is between zNear and zFar
	const float& z = localCoord.z;
	const float& n = m_intrinsicParams.zNear_mm;
	const float& f = m_intrinsicParams.zFar_mm;

	return (-z <= f && -z > n && std::abs(f+z) >= FLT_EPSILON && std::abs(n+z) >= FLT_EPSILON);
}

CCVector3 ccCameraSensor::computeUpperLeftPoint() const
{
	if (m_intrinsicParams.arrayHeight == 0)
		return CCVector3(0,0,0);

	float ar = m_intrinsicParams.arrayHeight != 0 ? static_cast<float>(m_intrinsicParams.arrayWidth) / m_intrinsicParams.arrayHeight : 1.0f;
	float halfFov = m_intrinsicParams.vFOV_rad / 2;

	CCVector3 upperLeftPoint;
	upperLeftPoint.z = m_scale * ConvertFocalPixToMM(m_intrinsicParams.vertFocal_pix, m_intrinsicParams.pixelSize_mm[1]);
	upperLeftPoint.y = upperLeftPoint.z * tan(halfFov);
	upperLeftPoint.x = upperLeftPoint.z * tan(halfFov * ar);

	return upperLeftPoint;
}

bool ccCameraSensor::computeFrustumCorners()
{
	if (m_intrinsicParams.arrayHeight == 0)
	{
		ccLog::Warning("[ccCameraSensor::computeFrustumCorners] Sensor height is 0!");
		return false;
	}

	float ar = static_cast<float>(m_intrinsicParams.arrayWidth) / m_intrinsicParams.arrayHeight;
	float halfFov = m_intrinsicParams.vFOV_rad / 2;

	float xIn = std::abs( tan(halfFov * ar) );
	float yIn = std::abs( tan(halfFov     ) );
	const float& zNear = m_intrinsicParams.zNear_mm;
	const float& zFar  = m_intrinsicParams.zFar_mm;

	// compute points of frustum in image coordinate system (warning: in the system, z=-z)
	if (!m_frustumInfos.initFrustumCorners())
	{
		ccLog::Warning("[ccCameraSensor::computeFrustumCorners] Not enough memory!");
		return false;
	}

	// DO NOT MODIFY THE ORDER OF THE CORNERS!! A LOT OF CODE DEPENDS OF THIS ORDER!!
	m_frustumInfos.frustumCorners->addPoint(CCVector3( xIn, yIn, -PC_ONE) * zNear);
	m_frustumInfos.frustumCorners->addPoint(CCVector3( xIn, yIn, -PC_ONE) * zFar);
	m_frustumInfos.frustumCorners->addPoint(CCVector3( xIn,-yIn, -PC_ONE) * zNear);
	m_frustumInfos.frustumCorners->addPoint(CCVector3( xIn,-yIn, -PC_ONE) * zFar);
	m_frustumInfos.frustumCorners->addPoint(CCVector3(-xIn,-yIn, -PC_ONE) * zNear);
	m_frustumInfos.frustumCorners->addPoint(CCVector3(-xIn,-yIn, -PC_ONE) * zFar);
	m_frustumInfos.frustumCorners->addPoint(CCVector3(-xIn, yIn, -PC_ONE) * zNear);
	m_frustumInfos.frustumCorners->addPoint(CCVector3(-xIn, yIn, -PC_ONE) * zFar);

	// compute center of the circumscribed sphere
	const CCVector3* P0 = m_frustumInfos.frustumCorners->getPoint(0);
	const CCVector3* P5 = m_frustumInfos.frustumCorners->getPoint(5);

	float dz = P0->z-P5->z;
	float z = (std::abs(dz) < FLT_EPSILON ? P0->z : (P0->norm2() - P5->norm2()) / (2*dz));

	m_frustumInfos.center = CCVector3(0, 0, z);

	// frustum corners are now computed
	m_frustumInfos.isComputed = true;

	return true;
}

bool ccCameraSensor::computeGlobalPlaneCoefficients(float planeCoefficients[6][4], CCVector3 frustumCorners[8], CCVector3 edges[6], CCVector3& center)
{
	if (!m_frustumInfos.isComputed)
	{
		if (!computeFrustumCorners())
		{
			return false;
		}
	}

	assert(m_frustumInfos.frustumCorners && m_frustumInfos.frustumCorners->size() == 8);

	// compute frustum corners in the global coordinates system
	fromLocalCoordToGlobalCoord(*m_frustumInfos.frustumCorners->getPoint(0), frustumCorners[0]);
	fromLocalCoordToGlobalCoord(*m_frustumInfos.frustumCorners->getPoint(1), frustumCorners[1]);
	fromLocalCoordToGlobalCoord(*m_frustumInfos.frustumCorners->getPoint(2), frustumCorners[2]);
	fromLocalCoordToGlobalCoord(*m_frustumInfos.frustumCorners->getPoint(3), frustumCorners[3]);
	fromLocalCoordToGlobalCoord(*m_frustumInfos.frustumCorners->getPoint(4), frustumCorners[4]);
	fromLocalCoordToGlobalCoord(*m_frustumInfos.frustumCorners->getPoint(5), frustumCorners[5]);
	fromLocalCoordToGlobalCoord(*m_frustumInfos.frustumCorners->getPoint(6), frustumCorners[6]);
	fromLocalCoordToGlobalCoord(*m_frustumInfos.frustumCorners->getPoint(7), frustumCorners[7]);

	/*
	//-- METHOD 1 --//
	// See "Fast Extraction of Viewing Frustum Planes from the World-View-Projection Matrix" of Gil Gribb and Klaus Hartmann
	// Attention !! With this method, plane equations are not normalized ! You should add normalization if you need it :
	// It means that if you have your plane equation in the form (ax + by + cz + d = 0), then --> k = sqrt(a*a + b*b + c*c) and your new coefficients are --> a=a/k, b=b/k, c=c/k, d=d/k
	ccGLMatrix projectionMatrix;
	if (!getProjectionMatrix(projectionMatrix))
		return false;

	ccGLMatrix mat = projectionMatrix * m_orientMatrix;
	float* coeffs = mat.data();
	// right
	planeCoefficients[0][0] = coeffs[3] - coeffs[0];
	planeCoefficients[0][1] = coeffs[7] - coeffs[4];
	planeCoefficients[0][2] = coeffs[11] - coeffs[8];
	planeCoefficients[0][3] = coeffs[15] - coeffs[12];
	// bottom
	planeCoefficients[1][0] = coeffs[3] + coeffs[1];
	planeCoefficients[1][1] = coeffs[7] + coeffs[5];
	planeCoefficients[1][2] = coeffs[11] + coeffs[9];
	planeCoefficients[1][3] = coeffs[15] + coeffs[12];
	// left
	planeCoefficients[2][0] = coeffs[3] + coeffs[0];
	planeCoefficients[2][1] = coeffs[7] + coeffs[4];
	planeCoefficients[2][2] = coeffs[11] + coeffs[8];
	planeCoefficients[2][3] = coeffs[15] + coeffs[12];
	// top
	planeCoefficients[3][0] = coeffs[3] - coeffs[1];
	planeCoefficients[3][1] = coeffs[7] - coeffs[5];
	planeCoefficients[3][2] = coeffs[11] - coeffs[9];
	planeCoefficients[3][3] = coeffs[15] - coeffs[12];
	// near
	planeCoefficients[4][0] = coeffs[3] + coeffs[2];
	planeCoefficients[4][1] = coeffs[7] + coeffs[6];
	planeCoefficients[4][2] = coeffs[11] + coeffs[10];
	planeCoefficients[4][3] = coeffs[15] + coeffs[14];
	// far
	planeCoefficients[5][0] = coeffs[3] - coeffs[2];
	planeCoefficients[5][1] = coeffs[7] - coeffs[6];
	planeCoefficients[5][2] = coeffs[11] - coeffs[10];
	planeCoefficients[5][3] = coeffs[15] - coeffs[14];
	// normalization --> temporary because it is quite long ; could be done before...
	for (int i=0 ; i<6 ; i++)
	{
		float a = planeCoefficients[i][0];
		float b = planeCoefficients[i][1];
		float c = planeCoefficients[i][2];
		float d = planeCoefficients[i][3];
		float k = sqrt(pow(a,2)+pow(b,2)+pow(c,2));
		planeCoefficients[i][0] = a/k;
		planeCoefficients[i][1] = b/k;
		planeCoefficients[i][2] = c/k;
		planeCoefficients[i][3] = d/k;
	}
	*/


	//-- METHOD 2 --//
	// If you do not like method 1, use this standard method!
	// compute equations for side planes
	for (int i = 0; i < 4; i++)
	{
		CCVector3 v1 = frustumCorners[i * 2 + 1] - frustumCorners[i * 2];
		CCVector3 v2 = frustumCorners[((i + 1) * 2) % 8] - frustumCorners[i * 2];
		CCVector3 n = v1.cross(v2); n.normalize();
		planeCoefficients[i][0] = n.x;
		planeCoefficients[i][1] = n.y;
		planeCoefficients[i][2] = n.z;
		planeCoefficients[i][3] = -frustumCorners[i * 2].dot(n);
	}
	// compute equations for near and far planes
	{
		CCVector3 v1 = frustumCorners[0] - frustumCorners[6];
		CCVector3 v2 = frustumCorners[4] - frustumCorners[6];
		CCVector3 n = v1.cross(v2); n.normalize();
		planeCoefficients[4][0] = n.x;
		planeCoefficients[4][1] = n.y;
		planeCoefficients[4][2] = n.z;
		planeCoefficients[4][3] = -frustumCorners[6].dot(n);
		planeCoefficients[5][0] = -n.x;
		planeCoefficients[5][1] = -n.y;
		planeCoefficients[5][2] = -n.z;
		planeCoefficients[5][3] = -frustumCorners[7].dot(-n);
	}

	// compute frustum edges
	{
		edges[0] = frustumCorners[1] - frustumCorners[0];
		edges[1] = frustumCorners[3] - frustumCorners[2];
		edges[2] = frustumCorners[5] - frustumCorners[4];
		edges[3] = frustumCorners[7] - frustumCorners[6];
		edges[4] = frustumCorners[6] - frustumCorners[0];
		edges[5] = frustumCorners[2] - frustumCorners[0];
		for (unsigned i = 0; i < 6; i++)
		{
			edges[i].normalize();
		}
	}

	// compute frustum center in the global coordinates system
	fromLocalCoordToGlobalCoord(m_frustumInfos.center, center);

	return true;
}

void ccCameraSensor::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Draw3D(context))
		return;

	//we draw a little 3d representation of the sensor and some of its attributes

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert( glFunc != nullptr );

	if ( glFunc == nullptr )
		return;

	bool pushName = MACRO_DrawEntityNames(context);

	if (pushName)
	{
		//not particularly fast
		if (MACRO_DrawFastNamesOnly(context))
			return;
		glFunc->glPushName(getUniqueIDForDisplay());
	}

	glFunc->glMatrixMode(GL_MODELVIEW);
	glFunc->glPushMatrix();
	{
		ccIndexedTransformation sensorPos;
		if (!getAbsoluteTransformation(sensorPos,m_activeIndex))
		{
			//no visible position for this index!
			glFunc->glPopMatrix();
			if (pushName)
				glFunc->glPopName();
			return;
		}

		glFunc->glMultMatrixf(sensorPos.data());
	}

	CCVector3 upperLeftPoint = computeUpperLeftPoint();

	//up arrow
	const PointCoordinateType arrowHeight		= 3 * upperLeftPoint.y / 2;
	const PointCoordinateType baseHeight		= 6 * upperLeftPoint.y / 5;
	const PointCoordinateType arrowHalfWidth	= 2 * upperLeftPoint.x / 5;
	const PointCoordinateType baseHalfWidth		= 1 * upperLeftPoint.x / 5;

	glFunc->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	ccGL::Color3v(glFunc, m_color.rgb);

	//near plane
	glFunc->glBegin(GL_LINE_LOOP);
	ccGL::Vertex3(glFunc,  upperLeftPoint.x,  upperLeftPoint.y, -upperLeftPoint.z);
	ccGL::Vertex3(glFunc, -upperLeftPoint.x,  upperLeftPoint.y, -upperLeftPoint.z);
	ccGL::Vertex3(glFunc, -upperLeftPoint.x, -upperLeftPoint.y, -upperLeftPoint.z);
	ccGL::Vertex3(glFunc,  upperLeftPoint.x, -upperLeftPoint.y, -upperLeftPoint.z);
	glFunc->glEnd();

	//force line size
	glFunc->glPushAttrib(GL_LINE_BIT);
	glFunc->glLineWidth(1.0f);

	//side lines
	glFunc->glBegin(GL_LINES);
	glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
	ccGL::Vertex3(glFunc,  upperLeftPoint.x,  upperLeftPoint.y, -upperLeftPoint.z);
	glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
	ccGL::Vertex3(glFunc, -upperLeftPoint.x,  upperLeftPoint.y, -upperLeftPoint.z);
	glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
	ccGL::Vertex3(glFunc, -upperLeftPoint.x, -upperLeftPoint.y, -upperLeftPoint.z);
	glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
	ccGL::Vertex3(glFunc,  upperLeftPoint.x, -upperLeftPoint.y, -upperLeftPoint.z);
	glFunc->glEnd();

	glFunc->glPopAttrib(); //GL_LINE_BIT

	//base
	glFunc->glBegin(GL_QUADS);
	ccGL::Vertex3(glFunc, -baseHalfWidth, upperLeftPoint.y, -upperLeftPoint.z);
	ccGL::Vertex3(glFunc,  baseHalfWidth, upperLeftPoint.y, -upperLeftPoint.z);
	ccGL::Vertex3(glFunc,  baseHalfWidth, baseHeight,       -upperLeftPoint.z);
	ccGL::Vertex3(glFunc, -baseHalfWidth, baseHeight,       -upperLeftPoint.z);
	glFunc->glEnd();

	//arrow
	glFunc->glBegin(GL_TRIANGLES);
	ccGL::Vertex3(glFunc,  0,              arrowHeight, -upperLeftPoint.z);
	ccGL::Vertex3(glFunc, -arrowHalfWidth, baseHeight,  -upperLeftPoint.z);
	ccGL::Vertex3(glFunc,  arrowHalfWidth, baseHeight,  -upperLeftPoint.z);
	glFunc->glEnd();

	//frustum
	if (m_frustumInfos.drawFrustum || m_frustumInfos.drawSidePlanes)
	{
		if (!m_frustumInfos.isComputed)
			computeFrustumCorners();

		if (m_frustumInfos.frustumCorners && m_frustumInfos.frustumCorners->size() >= 8)
		{
			//frustum area (lines)
			if (m_frustumInfos.drawFrustum)
			{
				const CCVector3* P0 = m_frustumInfos.frustumCorners->getPoint(0);
				const CCVector3* P1 = m_frustumInfos.frustumCorners->getPoint(1);
				const CCVector3* P2 = m_frustumInfos.frustumCorners->getPoint(2);
				const CCVector3* P3 = m_frustumInfos.frustumCorners->getPoint(3);
				const CCVector3* P4 = m_frustumInfos.frustumCorners->getPoint(4);
				const CCVector3* P5 = m_frustumInfos.frustumCorners->getPoint(5);
				const CCVector3* P6 = m_frustumInfos.frustumCorners->getPoint(6);
				const CCVector3* P7 = m_frustumInfos.frustumCorners->getPoint(7);

				glFunc->glPushAttrib(GL_LINE_BIT);
				glFunc->glLineWidth(2.0);

				glFunc->glBegin(GL_LINE_LOOP);
				ccGL::Vertex3v(glFunc, P0->u);
				ccGL::Vertex3v(glFunc, P1->u);
				ccGL::Vertex3v(glFunc, P3->u);
				ccGL::Vertex3v(glFunc, P2->u);
				glFunc->glEnd();
				glFunc->glBegin(GL_LINE_LOOP);
				ccGL::Vertex3v(glFunc, P2->u);
				ccGL::Vertex3v(glFunc, P3->u);
				ccGL::Vertex3v(glFunc, P5->u);
				ccGL::Vertex3v(glFunc, P4->u);
				glFunc->glEnd();
				glFunc->glBegin(GL_LINE_LOOP);
				ccGL::Vertex3v(glFunc, P4->u);
				ccGL::Vertex3v(glFunc, P5->u);
				ccGL::Vertex3v(glFunc, P7->u);
				ccGL::Vertex3v(glFunc, P6->u);
				glFunc->glEnd();
				glFunc->glBegin(GL_LINE_LOOP);
				ccGL::Vertex3v(glFunc, P6->u);
				ccGL::Vertex3v(glFunc, P7->u);
				ccGL::Vertex3v(glFunc, P1->u);
				ccGL::Vertex3v(glFunc, P0->u);
				glFunc->glEnd();
				glFunc->glBegin(GL_LINE_LOOP);
				ccGL::Vertex3v(glFunc, P6->u);
				ccGL::Vertex3v(glFunc, P0->u);
				ccGL::Vertex3v(glFunc, P2->u);
				ccGL::Vertex3v(glFunc, P4->u);
				glFunc->glEnd();
				glFunc->glBegin(GL_LINE_LOOP);
				ccGL::Vertex3v(glFunc, P1->u);
				ccGL::Vertex3v(glFunc, P7->u);
				ccGL::Vertex3v(glFunc, P5->u);
				ccGL::Vertex3v(glFunc, P3->u);
				glFunc->glEnd();

				glFunc->glPopAttrib();
			}

			//frustum area (planes)
			if (m_frustumInfos.drawSidePlanes && m_frustumInfos.initFrustumHull())
			{
				//set the rigth display (just to be sure)
				m_frustumInfos.frustumHull->setDisplay(getDisplay());
				m_frustumInfos.frustumHull->setTempColor(m_color);

				//glFunc->glPushAttrib(GL_COLOR_BUFFER_BIT);
				//glFunc->glEnable(GL_BLEND);
				//glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				//glFunc->glColor4ub(m_color.x, m_color.y, m_color.z, 76);

				m_frustumInfos.frustumHull->showWired(false);
				m_frustumInfos.frustumHull->enableStippling(true);
				m_frustumInfos.frustumHull->draw(context);

				//glFunc->glPopAttrib();
			}
		}
	}

	//axis (for test)
	if (!pushName)
	{
		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(2.0f);

		float l = static_cast<float>(fabs(upperLeftPoint.z)/2);

		// right vector
		ccGL::Color4v(glFunc, ccColor::red.rgba);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(l, 0.0f, 0.0f);
		glFunc->glEnd();

		// up vector
		ccGL::Color4v(glFunc, ccColor::green.rgba);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, l, 0.0f);
		glFunc->glEnd();

		// view vector
		ccGL::Color4v(glFunc, ccColor::blue.rgba);
		glFunc->glBegin(GL_LINES);
		glFunc->glVertex3f(0.0f, 0.0f, 0.0f);
		glFunc->glVertex3f(0.0f, 0.0f, -l);
		glFunc->glEnd();

		glFunc->glPopAttrib(); //GL_LINE_BIT
	}

	if (pushName)
		glFunc->glPopName();

	glFunc->glPopMatrix();
}

float ccCameraSensor::ConvertFocalPixToMM(float focal_pix, float ccdPixelSize_mm)
{
	if (ccdPixelSize_mm < FLT_EPSILON)
	{
		ccLog::Warning("[ccCameraSensor::convertFocalPixToMM] Invalid CCD pixel size! (<= 0)");
		return -1.0f;
	}

	return focal_pix * ccdPixelSize_mm;
}

float ccCameraSensor::ConvertFocalMMToPix(float focal_mm, float ccdPixelSize_mm)
{
	if (ccdPixelSize_mm < FLT_EPSILON)
	{
		ccLog::Warning("[ccCameraSensor::convertFocalMMToPix] Invalid CCD pixel size! (<= 0)");
		return -1.0f;
	}

	return focal_mm / ccdPixelSize_mm;
}

float ccCameraSensor::ComputeFovRadFromFocalPix(float focal_pix, int imageSize_pix)
{
	if (imageSize_pix <= 0)
	{
		//invalid image size
		return -1.0f;
	}

	return 2 * atan( imageSize_pix / (2*focal_pix) );
}

float ccCameraSensor::ComputeFovRadFromFocalMm(float focal_mm, float ccdSize_mm)
{
	if (ccdSize_mm < FLT_EPSILON)
	{
		//invalid CDD size
		return -1.0f;
	}

	return 2 * atan( ccdSize_mm / (2 * focal_mm) );
}

bool ccCameraSensor::computeOrthoRectificationParams(	const ccImage* image,
														CCLib::GenericIndexedCloud* keypoints3D,
														std::vector<KeyPoint>& keypointsImage,
														double a_out[3],
														double b_out[3],
														double c_out[3]) const
{
	if (!image || !keypoints3D)
		return false;

	unsigned count = static_cast<unsigned>(keypointsImage.size());
	if (count < 4)
		return false;

	//first guess for X (a0 a1 a2 b0 b1 b2 c1 c2)
	double norm = static_cast<double>(std::max(image->getW(),image->getH()));
	double X0[8] = {	1.0 / sqrt(norm),
						1.0 / norm,
						1.0 / norm,
						1.0 / sqrt(norm),
						1.0 / norm,
						1.0 / norm,
						1.0 / norm,
						1.0 / norm };

	//compute the A matrix and b vector
	unsigned Neq = 2 * count; //number of equations
	double *A = new double[8 * Neq]; // 8 coefficients: a0 a1 a2 b0 b1 b2 c1 c2
	double *b = new double[Neq];
	if (!A || !b)
	{
		//not enough memory
		delete[] A;
		delete[] b;
		return false;
	}

	//for all points
	{
		double* _A = A;
		double* _b = b;
		for (unsigned i = 0; i < count; ++i)
		{
			const KeyPoint& kp = keypointsImage[i];
			double kpx = static_cast<double>(kp.x);
			double kpy = static_cast<double>(kp.y);
			const CCVector3* P = keypoints3D->getPoint(kp.index);

			*_A++ = 1.0;
			*_A++ = kpx;
			*_A++ = kpy;
			*_A++ = 0.0;
			*_A++ = 0.0;
			*_A++ = 0.0;
			*_A++ = -kpx * static_cast<double>(P->x);
			*_A++ = -kpy * static_cast<double>(P->x);
			*_b++ = static_cast<double>(P->x);

			*_A++ = 0.0;
			*_A++ = 0.0;
			*_A++ = 0.0;
			*_A++ = 1.0;
			*_A++ = kpx;
			*_A++ = kpy;
			*_A++ = -kpx * static_cast<double>(P->y);
			*_A++ = -kpy * static_cast<double>(P->y);
			*_b++ = static_cast<double>(P->y);
		}
	}

	//conjugate gradient initialization
	//we solve tA.A.X = tA.b
	CCLib::ConjugateGradient<8, double> cg;
	CCLib::SquareMatrixd& tAA = cg.A();
	double* tAb = cg.b();

	//compute tA.A and tA.b
	{
		for (unsigned i = 0; i < 8; ++i)
		{
			//tA.A part
			for (unsigned j = i; j < 8; ++j)
			{
				double sum_prod = 0;
				const double* _Ai = A + i;
				const double* _Aj = A + j;
				for (unsigned k = 0; k < Neq; ++k)
				{
					//sum_prod += A[(8*2*k)+i]*A[(8*2*k)+j];
					sum_prod += (*_Ai) * (*_Aj);
					_Ai += 8;
					_Aj += 8;
				}
				tAA.m_values[j][i] = tAA.m_values[i][j] = sum_prod;
			}

			//tA.b part
			{
				double sum_prod = 0;
				const double* _Ai = A + i;
				const double* _b = b;
				for (unsigned k = 0; k < Neq; ++k)
				{
					//sum_prod += A[(8*2*k)+i]*b[k];
					sum_prod += (*_Ai) * (*_b++);
					_Ai += 8;
				}
				tAb[i] = sum_prod;
			}
		}
	}

	//init. conjugate gradient
	cg.initConjugateGradient(X0);

	//conjugate gradient iterations
	{
		double convergenceThreshold = 1.0e-8/* * norm*/;  //max. error for convergence
		for (unsigned i = 0; i < 1500; ++i)
		{
			double lastError = cg.iterConjugateGradient(X0);
			if (lastError < convergenceThreshold) //converged
			{
				ccLog::PrintDebug(QString("[computeOrthoRectificationParams] Convergence reached in %1 iteration(s) (error: %2)").arg(i + 1).arg(lastError));
				break;
			}
		}
	}

	delete[] A;
	A = nullptr;
	delete[] b;
	b = nullptr;

	a_out[0] = X0[0];
	a_out[1] = X0[1];
	a_out[2] = X0[2];
	b_out[0] = X0[3];
	b_out[1] = X0[4];
	b_out[2] = X0[5];
	c_out[0] = 1.0;
	c_out[1] = X0[6];
	c_out[2] = X0[7];

	return true;
}

ccImage* ccCameraSensor::orthoRectifyAsImageDirect(	const ccImage* image,
													PointCoordinateType Z0,
													double& pixelSize,
													bool undistortImages/*=true*/,
													double* minCorner/*=0*/,
													double* maxCorner/*=0*/,
													double* realCorners/*=0*/) const
{
	//first, we compute the ortho-rectified image corners
	double corners[8];

	int width = static_cast<int>(image->getW());
	int height = static_cast<int>(image->getH());

	//top-left
	{
		CCVector2 xTopLeft(0, 0);
		CCVector3 P3D;
		if (!fromImageCoordToGlobalCoord(xTopLeft, P3D, Z0))
			return nullptr;
#ifdef QT_DEBUG
		//internal check
		CCVector2 check(0,0);
		fromGlobalCoordToImageCoord(P3D,check,false);
		assert((xTopLeft-check).norm2() < std::max(width,height)*FLT_EPSILON);
#endif
		corners[0] = P3D.x;
		corners[1] = P3D.y;
	}

	//top-right
	{
		CCVector2 xTopRight(static_cast<PointCoordinateType>(width), 0);
		CCVector3 P3D;
		if (!fromImageCoordToGlobalCoord(xTopRight, P3D, Z0))
			return nullptr;
#ifdef QT_DEBUG
		//internal check
		CCVector2 check(0,0);
		fromGlobalCoordToImageCoord(P3D,check,false);
		assert((xTopRight-check).norm2() < std::max(width,height)*FLT_EPSILON);
#endif
		corners[2] = P3D.x;
		corners[3] = P3D.y;
	}

	//bottom-right
	{
		CCVector2 xBottomRight(static_cast<PointCoordinateType>(width), static_cast<PointCoordinateType>(height));
		CCVector3 P3D;
		if (!fromImageCoordToGlobalCoord(xBottomRight, P3D, Z0))
			return nullptr;
#ifdef QT_DEBUG
		//internal check
		CCVector2 check(0,0);
		fromGlobalCoordToImageCoord(P3D,check,false);
		assert((xBottomRight-check).norm2() < std::max(width,height)*FLT_EPSILON);
#endif
		corners[4] = P3D.x;
		corners[5] = P3D.y;
	}

	//bottom-left
	{
		CCVector2 xBottomLeft(0, static_cast<PointCoordinateType>(height));
		CCVector3 P3D;
		if (!fromImageCoordToGlobalCoord(xBottomLeft, P3D, Z0))
			return nullptr;
#ifdef QT_DEBUG
		//internal check
		CCVector2 check(0,0);
		fromGlobalCoordToImageCoord(P3D,check,false);
		assert((xBottomLeft-check).norm2() < std::max(width,height)*FLT_EPSILON);
#endif
		corners[6] = P3D.x;
		corners[7] = P3D.y;
	}

	if (realCorners)
		memcpy(realCorners, corners, 8 * sizeof(double));

	//we look for min and max bounding box
	double minC[2] = {corners[0],corners[1]};
	double maxC[2] = {corners[0],corners[1]};

	for (unsigned k = 1; k < 4; ++k)
	{
		const double* C = corners + 2 * k;
		if (minC[0] > C[0])
			minC[0] = C[0];
		else if (maxC[0] < C[0])
			maxC[0] = C[0];

		if (minC[1] > C[1])
			minC[1] = C[1];
		else if (maxC[1] < C[1])
			maxC[1] = C[1];
	}

	//output 3D boundaries (optional)
	if (minCorner)
	{
		minCorner[0] = minC[0];
		minCorner[1] = minC[1];
	}
	if (maxCorner)
	{
		maxCorner[0] = maxC[0];
		maxCorner[1] = maxC[1];
	}

	double dx = maxC[0] - minC[0];
	double dy = maxC[1] - minC[1];

	double _pixelSize = pixelSize;
	if (_pixelSize <= 0)
	{
		int maxSize = std::max(width,height);
		_pixelSize = std::max(dx,dy)/maxSize;
	}
	unsigned w = static_cast<unsigned>(dx/_pixelSize);
	unsigned h = static_cast<unsigned>(dy/_pixelSize);

	QImage orthoImage(w,h,QImage::Format_ARGB32);
	if (orthoImage.isNull()) //not enough memory!
		return nullptr;

	const QRgb blackValue = qRgb(0, 0, 0);
	const QRgb blackAlphaZero = qRgba(0, 0, 0, 0);

	for (unsigned i = 0; i < w; ++i)
	{
		PointCoordinateType xip = static_cast<PointCoordinateType>(minC[0] + i*_pixelSize);
		for (unsigned j = 0; j < h; ++j)
		{
			PointCoordinateType yip = static_cast<PointCoordinateType>(minC[1] + j*_pixelSize);

			QRgb rgb = blackValue; //output pixel is (transparent) black by default

			CCVector3 P3D(xip,yip,Z0);
			CCVector2 imageCoord;
			if (fromGlobalCoordToImageCoord(P3D,imageCoord,undistortImages))
			{
				int x = static_cast<int>(imageCoord.x);
				int y = static_cast<int>(imageCoord.y);
				if (x >= 0 && x < width && y >= 0 && y < height)
				{
					rgb = image->data().pixel(x,y);
				}
			}

			//pure black pixels are treated as transparent ones!
			orthoImage.setPixel(i, h - 1 - j, rgb != blackValue ? rgb : blackAlphaZero);
		}
	}

	//output pixel size (auto)
	pixelSize = _pixelSize;

	return new ccImage(orthoImage,getName());
}

ccImage* ccCameraSensor::orthoRectifyAsImage(	const ccImage* image,
												CCLib::GenericIndexedCloud* keypoints3D,
												std::vector<KeyPoint>& keypointsImage,
												double& pixelSize,
												double* minCorner/*=0*/,
												double* maxCorner/*=0*/,
												double* realCorners/*=0*/) const
{
	double a[3]{ 0.0, 0.0, 0.0 };
	double b[3]{ 0.0, 0.0, 0.0 };
	double c[3]{ 0.0, 0.0, 0.0 };

	if (!computeOrthoRectificationParams(image, keypoints3D, keypointsImage, a, b, c))
	{
		return nullptr;
	}

	const double& a0 = a[0];
	const double& a1 = a[1];
	const double& a2 = a[2];
	const double& b0 = b[0];
	const double& b1 = b[1];
	const double& b2 = b[2];
	//const double& c0 = c[0];
	const double& c1 = c[1];
	const double& c2 = c[2];

	//first, we compute the ortho-rectified image corners
	double corners[8];
	double xi;
	double yi;
	double qi;

	int width = static_cast<int>(image->getW());
	int height = static_cast<int>(image->getH());
	double halfWidth = width / 2.0;
	double halfHeight = height / 2.0;

	//top-left
	xi = -halfWidth;
	yi = -halfHeight;
	qi = 1.0 + c1*xi + c2*yi;
	corners[0] = (a0 + a1*xi + a2*yi) / qi;
	corners[1] = (b0 + b1*xi + b2*yi) / qi;

	//top-right
	xi =  halfWidth;
	yi = -halfHeight;
	qi = 1.0 + c1*xi + c2*yi;
	corners[2] = (a0 + a1*xi + a2*yi) / qi;
	corners[3] = (b0 + b1*xi + b2*yi) / qi;

	//bottom-right
	xi = halfWidth;
	yi = halfHeight;
	qi = 1.0 + c1*xi + c2*yi;
	corners[4] = (a0 + a1*xi + a2*yi) / qi;
	corners[5] = (b0 + b1*xi + b2*yi) / qi;

	//bottom-left
	xi = -halfWidth;
	yi =  halfHeight;
	qi = 1.0 + c1*xi + c2*yi;
	corners[6] = (a0 + a1*xi + a2*yi) / qi;
	corners[7] = (b0 + b1*xi + b2*yi) / qi;

	if (realCorners)
	{
		memcpy(realCorners, corners, 8 * sizeof(double));
	}

	//we look for min and max bounding box
	double minC[2] = { corners[0], corners[1] };
	double maxC[2] = { corners[0], corners[1] };

	for (unsigned k = 1; k < 4; ++k)
	{
		const double* C = corners + 2 * k;
		if (minC[0] > C[0])
			minC[0] = C[0];
		else if (maxC[0] < C[0])
			maxC[0] = C[0];

		if (minC[1] > C[1])
			minC[1] = C[1];
		else if (maxC[1] < C[1])
			maxC[1] = C[1];
	}

	//output 3D boundaries (optional)
	if (minCorner)
	{
		minCorner[0] = minC[0];
		minCorner[1] = minC[1];
	}
	if (maxCorner)
	{
		maxCorner[0] = maxC[0];
		maxCorner[1] = maxC[1];
	}

	double dx = maxC[0] - minC[0];
	double dy = maxC[1] - minC[1];

	double _pixelSize = pixelSize;
	if (_pixelSize <= 0)
	{
		int maxSize = std::max(width, height);
		_pixelSize = std::max(dx, dy) / maxSize;
	}
	unsigned w = static_cast<unsigned>(dx / _pixelSize);
	unsigned h = static_cast<unsigned>(dy / _pixelSize);

	QImage orthoImage(w, h, QImage::Format_ARGB32);
	if (orthoImage.isNull()) //not enough memory!
		return nullptr;

	const QRgb blackValue = qRgb(0, 0, 0);
	const QRgb blackAlphaZero = qRgba(0, 0, 0, 0);

	for (unsigned i = 0; i < w; ++i)
	{
		double xip = minC[0] + static_cast<double>(i)*_pixelSize;
		for (unsigned j = 0; j < h; ++j)
		{
			QRgb rgb = blackValue; //output pixel is (transparent) black by default

			double yip = minC[1] + static_cast<double>(j)*_pixelSize;
			double q = (c2*xip - a2)*(c1*yip - b1) - (c2*yip - b2)*(c1*xip - a1);
			double p = (a0 - xip)*(c1*yip - b1) - (b0 - yip)*(c1*xip - a1);
			double yi = p / q;
			yi += halfHeight;
			int y = static_cast<int>(yi);

			if (y >= 0 && y < height)
			{
				q = (c1*xip - a1)*(c2*yip - b2) - (c1*yip - b1)*(c2*xip - a2);
				p = (a0 - xip)*(c2*yip - b2) - (b0 - yip)*(c2*xip - a2);
				double  xi = p / q;
				xi += halfWidth;
				int x = static_cast<int>(xi);

				if (x >= 0 && x < width)
				{
					rgb = image->data().pixel(x, y);
				}
			}

			//pure black pixels are treated as transparent ones!
			orthoImage.setPixel(i, h - 1 - j, rgb != blackValue ? rgb : blackAlphaZero);
		}
	}

	//output pixel size (auto)
	pixelSize = _pixelSize;

	return new ccImage(orthoImage,getName());
}

bool ccCameraSensor::OrthoRectifyAsImages(	std::vector<ccImage*> images,
											double a[], double b[], double c[],
											unsigned maxSize,
											QDir* outputDir/*=0*/,
											std::vector<ccImage*>* result/*=0*/,
											std::vector<std::pair<double,double> >* relativePos/*=0*/)
{
	size_t count = images.size();
	if (count == 0)
	{
		ccLog::Warning("[OrthoRectifyAsImages] No image to process?!");
		return false;
	}

	//min & max corners for each images
	std::vector<double> minCorners;
	std::vector<double> maxCorners;
	try
	{
		minCorners.resize(2*count);
		maxCorners.resize(2*count);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		ccLog::Warning("[OrthoRectifyAsImages] Not enough memory!");
		return false;
	}
	//max dimension of all (ortho-rectified) images, horizontally or vertically
	double maxDimAllImages = 0;
	//corners for the global set
	double globalCorners[4] = { 0, 0, 0, 0};

	//compute output corners and max dimension for all images
	for (size_t k=0; k<count; ++k)
	{
		const double& a0 = a[k*3  ];
		const double& a1 = a[k*3+1];
		const double& a2 = a[k*3+2];
		const double& b0 = b[k*3  ];
		const double& b1 = b[k*3+1];
		const double& b2 = b[k*3+2];
		//const double& c0 = c[k*3];
		const double& c1 = c[k*3+1];
		const double& c2 = c[k*3+2];

		//first, we compute the ortho-rectified image corners
		double corners[8];
		double xi;
		double yi;
		double qi;

		unsigned width = images[k]->getW();
		unsigned height = images[k]->getH();

		//top-left
		xi = -0.5*width;
		yi = -0.5*height;
		qi = 1.0+c1*xi+c2*yi;
		corners[0] = (a0+a1*xi+a2*yi)/qi;
		corners[1] = (b0+b1*xi+b2*yi)/qi;

		//top-right
		xi =  0.5*width;
		//yi = -0.5*height;
		qi = 1.0+c1*xi+c2*yi;
		corners[2] = (a0+a1*xi+a2*yi)/qi;
		corners[3] = (b0+b1*xi+b2*yi)/qi;

		//bottom-right
		//xi =  0.5*width;
		yi = 0.5*height;
		qi = 1.0+c1*xi+c2*yi;
		corners[4] = (a0+a1*xi+a2*yi)/qi;
		corners[5] = (b0+b1*xi+b2*yi)/qi;

		//bottom-left
		xi =  -0.5*width;
		//yi = 0.5*height;
		qi = 1.0+c1*xi+c2*yi;
		corners[6] = (a0+a1*xi+a2*yi)/qi;
		corners[7] = (b0+b1*xi+b2*yi)/qi;

		//we look for min and max bounding box
		double* minC = &minCorners[2*k];
		double* maxC = &maxCorners[2*k];
		maxC[0] = minC[0] = corners[0];
		maxC[1] = minC[1] = corners[1];
		for (unsigned k=1; k<4; ++k)
		{
			const double* C = corners+2*k;
			//dimension: X
			if (minC[0] > C[0])
				minC[0] = C[0];
			else if (maxC[0] < C[0])
				maxC[0] = C[0];

			if (globalCorners[0] > minC[0])
				globalCorners[0] = minC[0];
			if (globalCorners[2] < maxC[0])
				globalCorners[2] = maxC[0];

			//dimension: Y
			if (minC[1] > C[1])
				minC[1] = C[1];
			else if (maxC[1] < C[1])
				maxC[1] = C[1];

			if (globalCorners[1] > minC[1])
				globalCorners[1] = minC[1];
			if (globalCorners[3] < maxC[1])
				globalCorners[3] = maxC[1];
		}

		double dx = maxC[0]-minC[0];
		double dy = maxC[1]-minC[1];
		double maxd = std::max(dx,dy);
		if (maxd > maxDimAllImages)
			maxDimAllImages=maxd;
	}

	//deduce pixel size
	double pixelSize = maxDimAllImages/maxSize;

	if (outputDir)
	{
		//write header
		QFile f(outputDir->absoluteFilePath("ortho_rectification_log.txt"));
		if (f.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			QTextStream stream(&f);
			stream.setRealNumberNotation(QTextStream::FixedNotation);
			stream.setRealNumberPrecision(6);
			stream << "PixelSize" << ' ' << pixelSize << endl;
			stream << "Global3DBBox" << ' ' << globalCorners[0] << ' ' << globalCorners[1] << ' ' << globalCorners[2] << ' ' << globalCorners[3] << endl;
			int globalWidth = static_cast<int>(ceil((globalCorners[2]-globalCorners[0])/pixelSize));
			int globalHeight = static_cast<int>(ceil((globalCorners[3]-globalCorners[1])/pixelSize));
			stream << "Global2DBBox" << ' ' << 0 << ' ' << 0 << ' ' << globalWidth-1 << ' ' << globalHeight-1 << endl;
		}
	}

	//projet each image accordingly
	for (size_t k=0; k<count; ++k)
	{
		double* minC = &minCorners[2*k];
		double* maxC = &maxCorners[2*k];
		double dx = maxC[0]-minC[0];
		double dy = maxC[1]-minC[1];

		ccImage* image = images[k];
		unsigned width = images[k]->getW();
		unsigned height = images[k]->getH();
		unsigned w = static_cast<unsigned>(ceil(dx/pixelSize));
		unsigned h = static_cast<unsigned>(ceil(dy/pixelSize));

		QImage orthoImage(w,h,QImage::Format_ARGB32);
		if (orthoImage.isNull()) //not enough memory!
		{
			//clear mem.
			if (result)
			{
				while (!result->empty())
				{
					delete result->back();
					result->pop_back();
				}
			}
			ccLog::Warning("[OrthoRectifyAsImages] Not enough memory!");
			return false;
		}

		//ortho rectification parameters
		const double& a0 = a[k*3  ];
		const double& a1 = a[k*3+1];
		const double& a2 = a[k*3+2];
		const double& b0 = b[k*3  ];
		const double& b1 = b[k*3+1];
		const double& b2 = b[k*3+2];
		//const double& c0 = c[k*3];
		const double& c1 = c[k*3+1];
		const double& c2 = c[k*3+2];

		for (unsigned i=0; i<w; ++i)
		{
			double xip = minC[0] + static_cast<double>(i)*pixelSize;
			for (unsigned j=0;j<h;++j)
			{
				double yip = minC[1] + static_cast<double>(j)*pixelSize;
				double q = (c2*xip-a2)*(c1*yip-b1)-(c2*yip-b2)*(c1*xip-a1);
				double p = (a0-xip)*(c1*yip-b1)-(b0-yip)*(c1*xip-a1);
				double yi = p/q;

				q = (c1*xip-a1)*(c2*yip-b2)-(c1*yip-b1)*(c2*xip-a2);
				p = (a0-xip)*(c2*yip-b2)-(b0-yip)*(c2*xip-a2);
				double  xi = p/q;

				xi += 0.5 * width;
				yi += 0.5 * height;

				int x = static_cast<int>(xi);
				int y = static_cast<int>(yi);
				if (x >= 0 && x < static_cast<int>(width) && y >= 0 && y < static_cast<int>(height))
				{
					QRgb rgb = image->data().pixel(x,y);
					//pure black pixels are treated as transparent ones!
					if (qRed(rgb) + qGreen(rgb) + qBlue(rgb) > 0)
						orthoImage.setPixel(i, h - 1 - j, rgb);
					else
						orthoImage.setPixel(i, h - 1 - j, qRgba(qRed(rgb), qGreen(rgb), qBlue(rgb), 0));
				}
				else
					orthoImage.setPixel(i, h - 1 - j, qRgba(255, 0, 255, 0));
			}
		}

		//eventually compute relative pos
		if (relativePos)
		{
			double xShift = (minC[0]-minCorners[0])/pixelSize;
			double yShift = (minC[1]-minCorners[1])/pixelSize;
			relativePos->emplace_back(xShift, yShift);
		}

		if (outputDir)
		{
			//export image
			QString exportFilename = QString("ortho_rectified_%1.png").arg(image->getName());
			orthoImage.save(outputDir->absoluteFilePath(exportFilename));

			//export meta-data
			QFile f(outputDir->absoluteFilePath("ortho_rectification_log.txt"));
			if (f.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) //always append
			{
				double xShiftGlobal = (minC[0]-globalCorners[0])/pixelSize;
				double yShiftGlobal = (minC[1]-globalCorners[1])/pixelSize;
				QTextStream stream(&f);
				stream.setRealNumberNotation(QTextStream::FixedNotation);
				stream.setRealNumberPrecision(6);
				stream << "Image" << ' ' << exportFilename  << ' ';
				stream << "Local3DBBox" << ' ' << minC[0] << ' ' << minC[1] << ' ' << maxC[0] << ' ' << maxC[1] << ' ';
				stream << "Local2DBBox" << ' ' << xShiftGlobal << ' ' << yShiftGlobal <<  ' ' << xShiftGlobal+static_cast<double>(w-1) << ' ' << yShiftGlobal+static_cast<double>(h-1) << endl;
				f.close();
			}
		}

		if (result)
			result->push_back(new ccImage(orthoImage,image->getName()));
	}

	return true;
}

ccPointCloud* ccCameraSensor::orthoRectifyAsCloud(	const ccImage* image,
													CCLib::GenericIndexedCloud* keypoints3D,
													std::vector<KeyPoint>& keypointsImage) const
{
	double a[3]{ 0.0, 0.0, 0.0 };
	double b[3]{ 0.0, 0.0, 0.0 };
	double c[3]{ 0.0, 0.0, 0.0 };

	if (!computeOrthoRectificationParams(image,keypoints3D,keypointsImage,a,b,c))
		return nullptr;

	const double& a0 = a[0];
	const double& a1 = a[1];
	const double& a2 = a[2];
	const double& b0 = b[0];
	const double& b1 = b[1];
	const double& b2 = b[2];
	//const double& c0 = c[0];
	const double& c1 = c[1];
	const double& c2 = c[2];

	PointCoordinateType defaultZ = 0;

	unsigned width = image->getW();
	unsigned height = image->getH();

	ccPointCloud* proj = new ccPointCloud(getName()+QString(".ortho-rectified"));
	if (!proj->reserve(width*height) || !proj->reserveTheRGBTable())
	{
		ccLog::Warning("[orthoRectifyAsCloud] Not enough memory!");
		delete proj;
		return nullptr;
	}
	proj->showColors(true);

	unsigned realCount = 0;

	//ortho rectification
	{
		for (unsigned pi = 0; pi < width; ++pi)
		{
			double xi = static_cast<double>(pi) - 0.5*width;
			for (unsigned pj = 0; pj < height; ++pj)
			{
				double yi = static_cast<double>(pj) - 0.5*height;
				double qi = 1.0 + c1*xi + c2*yi;
				CCVector3 P(static_cast<PointCoordinateType>((a0 + a1 * xi + a2 * yi) / qi),
							static_cast<PointCoordinateType>((b0 + b1 * xi + b2 * yi) / qi),
							defaultZ);

				//and color?
				QRgb rgb = image->data().pixel(pi, pj);
				int r = qRed(rgb);
				int g = qGreen(rgb);
				int b = qBlue(rgb);
				if (r + g + b > 0)
				{
					//add point
					proj->addPoint(P);
					//and color
					int a = qAlpha(rgb);
					ccColor::Rgba color(static_cast<ColorCompType>(r),
										static_cast<ColorCompType>(g),
										static_cast<ColorCompType>(b),
										static_cast<ColorCompType>(a));
					proj->addColor(color);
					++realCount;
				}
			}
		}
	}

	if (realCount == 0)
	{
		ccLog::Warning(QString("[orthoRectifyAsCloud] Image '%1' was black, nothing to project!").arg(image->getName()));
		delete proj;
		proj = nullptr;
	}
	else
	{
		proj->resize(realCount);
	}

	return proj;
}

/********************************************************************/
/*******************                              *******************/
/*******************  ccOctreeFrustumIntersector  *******************/
/*******************                              *******************/
/********************************************************************/

bool ccOctreeFrustumIntersector::build(CCLib::DgmOctree* octree)
{
	if (!octree)
		return false;

	for (auto &cell : m_cellsBuilt)
	{
		cell.clear();
	}
	
	const CCLib::DgmOctree::cellsContainer& thePointsAndTheirCellCodes = octree->pointsAndTheirCellCodes();
	CCLib::DgmOctree::cellsContainer::const_iterator it = thePointsAndTheirCellCodes.begin();

	try
	{
		for (it=thePointsAndTheirCellCodes.begin(); it!=thePointsAndTheirCellCodes.end(); ++it)
		{
			CCLib::DgmOctree::CellCode completeCode = it->theCode;
			for (unsigned char level=1; level<=CCLib::DgmOctree::MAX_OCTREE_LEVEL; level++)
			{
				unsigned char bitDec = CCLib::DgmOctree::GET_BIT_SHIFT(level);
				m_cellsBuilt[level].insert(completeCode >> bitDec);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[ccCameraSensor::prepareOctree] Not enough memory!");
		for (int i = 0; i <= CCLib::DgmOctree::MAX_OCTREE_LEVEL; i++)
		{
			m_cellsBuilt[i].clear();
		}
		return false;
	}

	m_associatedOctree = octree;

	return true;
}

//// an other method to compute frustum cell intersection (not used)
//
//unsigned char boxIntersectPlane(const CCVector3& minCorner, const CCVector3& maxCorner, const float planeCoefficient[4])
//{
//	CCVector3 n(planeCoefficient[0], planeCoefficient[1], planeCoefficient[2]);
//	float d = planeCoefficient[3];
//
//	CCVector3 c = (maxCorner + minCorner) / 2.0;
//	CCVector3 h = (maxCorner - minCorner) / 2.0;
//
//	float e = h[0]*abs(n[0]) + h[1]*abs(n[1]) + h[2]*abs(n[2]);
//	float s = c.dot(n) + d;
//
//	if ((s-e) > 0.0)
//		return CELL_OUTSIDE_FRUSTUM;
//	if ((s+e) < 0.0)
//		return CELL_INSIDE_FRUSTUM;
//	return CELL_INTERSECT_FRUSTUM;
//}
//
//unsigned char boxIntersectFrustum(const CCVector3& minCorner, const CCVector3& maxCorner, const float planesCoefficients[6][4])
//{
//	bool intersecting = false;
//
//	for (int i=0 ; i<6 ; i++)
//	{
//		float onePlaneCoefficients[4];
//		for (int j=0 ; j<4 ; j++)
//			onePlaneCoefficients[j] = planesCoefficients[i][j];
//
//		int result = boxIntersectPlane(minCorner, maxCorner, onePlaneCoefficients);
//
//		//pay attention to the signification of OUTSIDE and INSIDE there : INSIDE means that the box is in the positive half space delimited by the plane, OUTSIDE means that the box is in the negative half space !!
//		if (result == CELL_OUTSIDE_FRUSTUM)
//			return CELL_OUTSIDE_FRUSTUM;
//		if (result == CELL_INTERSECT_FRUSTUM)
//			intersecting = true;
//	}
//
//	if (intersecting == true)
//		return CELL_INTERSECT_FRUSTUM;
//	return CELL_INSIDE_FRUSTUM;
//}

ccOctreeFrustumIntersector::OctreeCellVisibility
ccOctreeFrustumIntersector::separatingAxisTest(const CCVector3& bbMin,
														const CCVector3& bbMax,
														const float planesCoefficients[6][4],
														const CCVector3 frustumCorners[8],
														const CCVector3 frustumEdges[6],
														const CCVector3& frustumCenter)
{
	// first test : if the box is too far from the frustum, there is no intersection
	CCVector3 boxCenter = (bbMax + bbMin) / 2;
	PointCoordinateType dCenter = (boxCenter - frustumCenter).norm();
	PointCoordinateType boxRadius = (bbMax - bbMin).norm();
	PointCoordinateType frustumRadius = (frustumCorners[0] - frustumCenter).norm();
	if (dCenter > boxRadius + frustumRadius)
		return CELL_OUTSIDE_FRUSTUM;

	// We could add a test : is the cell circumscribed circle in the frustum incircle frustum ?
	// --> if we are lucky, it could save a lot of time !...

	//box corners
	CCVector3 boxCorners[8];
	{
		for (unsigned i=0; i<8; i++)
			boxCorners[i] = CCVector3(	(i & 4) ? bbMin.x : bbMax.x,
										(i & 2) ? bbMin.y : bbMax.y,
										(i & 1) ? bbMin.z : bbMax.z);
	}

	//There are 28 tests to perform:
	//	nbFacesCube = n1 = 3;
	//	nbFacesFrustum = n2 = 5;
	//	nbEdgesCube = n3 = 3;
	//	nbEdgesFrustum = n4 = 6;
	//	nbOtherFrustumCombinations = n5 = 2
	//	nbVecToTest = n1 + n2 + n3*n4 = 3 + 5 + 3*6 + n5 = 28;
	static const unsigned nbVecToTest = 28;
	CCVector3 VecToTest[nbVecToTest];
	// vectors orthogonals to box planes
	VecToTest[0]  = CCVector3(1, 0, 0);
	VecToTest[1]  = CCVector3(0, 1, 0);
	VecToTest[2]  = CCVector3(0, 0, 1);
	// vectors orthogonals to frustum planes
	VecToTest[3]  = CCVector3(planesCoefficients[0][0], planesCoefficients[0][1], planesCoefficients[0][2]);
	VecToTest[4]  = CCVector3(planesCoefficients[1][0], planesCoefficients[1][1], planesCoefficients[1][2]);
	VecToTest[5]  = CCVector3(planesCoefficients[2][0], planesCoefficients[2][1], planesCoefficients[2][2]);
	VecToTest[6]  = CCVector3(planesCoefficients[3][0], planesCoefficients[3][1], planesCoefficients[3][2]);
	VecToTest[7]  = CCVector3(planesCoefficients[4][0], planesCoefficients[4][1], planesCoefficients[4][2]);
	// combinations box and frustum
	VecToTest[8]  = VecToTest[0].cross(frustumEdges[0]);
	VecToTest[9]  = VecToTest[0].cross(frustumEdges[1]);
	VecToTest[10] = VecToTest[0].cross(frustumEdges[2]);
	VecToTest[11] = VecToTest[0].cross(frustumEdges[3]);
	VecToTest[12] = VecToTest[0].cross(frustumEdges[4]);
	VecToTest[13] = VecToTest[0].cross(frustumEdges[5]);
	VecToTest[14] = VecToTest[1].cross(frustumEdges[0]);
	VecToTest[15] = VecToTest[1].cross(frustumEdges[1]);
	VecToTest[16] = VecToTest[1].cross(frustumEdges[2]);
	VecToTest[17] = VecToTest[1].cross(frustumEdges[3]);
	VecToTest[18] = VecToTest[1].cross(frustumEdges[4]);
	VecToTest[19] = VecToTest[1].cross(frustumEdges[5]);
	VecToTest[20] = VecToTest[2].cross(frustumEdges[0]);
	VecToTest[21] = VecToTest[2].cross(frustumEdges[1]);
	VecToTest[22] = VecToTest[2].cross(frustumEdges[2]);
	VecToTest[23] = VecToTest[2].cross(frustumEdges[3]);
	VecToTest[24] = VecToTest[2].cross(frustumEdges[4]);
	VecToTest[25] = VecToTest[2].cross(frustumEdges[5]);
	// combinations frustum
	VecToTest[26] = frustumEdges[0].cross(frustumEdges[2]);
	VecToTest[27] = frustumEdges[1].cross(frustumEdges[3]);

	// normalization
	{
		for (auto &testVec : VecToTest)
		{
			testVec.normalize();
		}
	}

	bool boxInside = true;

	// project volume corners
	{
		for (const auto &testVec : VecToTest)
		{
			//box points
			float dMinBox = testVec.dot(boxCorners[0]);
			float dMaxBox = dMinBox;
			{
				for (unsigned j=1; j<8; j++)
				{
					float d = testVec.dot(boxCorners[j]);
					if (d > dMaxBox)
						dMaxBox = d;
					if (d < dMinBox)
						dMinBox = d;
				}
			}

			//frustum points
			float dMinFru = testVec.dot(frustumCorners[0]);
			float dMaxFru = dMinFru;
			{
				for (unsigned j=1; j<8; j++)
				{
					float d = testVec.dot(frustumCorners[j]);
					if (d > dMaxFru)
						dMaxFru = d;
					if (d < dMinFru)
						dMinFru = d;
				}
			}

			//if this plane is a separating plane, the cell is outside the frustum
			if (dMaxBox < dMinFru || dMaxFru < dMinBox)
				return CELL_OUTSIDE_FRUSTUM;

			// if this plane is NOT a separating plane, the cell is at least intersecting the frustum
			else
			{
				// moreover, the cell can be completely inside the frustum...
				if (dMaxBox>dMaxFru || dMinBox<dMinFru)
					boxInside = false;
			}
		}
	}

	return boxInside ? CELL_INSIDE_FRUSTUM : CELL_INTERSECT_FRUSTUM;
}

void ccOctreeFrustumIntersector::computeFrustumIntersectionByLevel(unsigned char level,
																	CCLib::DgmOctree::CellCode parentTruncatedCode,
																	OctreeCellVisibility parentResult,
																	const float planesCoefficients[6][4],
																	const CCVector3 ptsFrustum[8],
																	const CCVector3 edges[6],
																	const CCVector3& center)
{
	if (parentResult == CELL_OUTSIDE_FRUSTUM)
		return;

	// move code to the left
	CCLib::DgmOctree::CellCode baseTruncatedCode = (parentTruncatedCode << 3);

	// test to do on the 8 child cells
	for (unsigned i=0; i<8; i++)
	{
		// set truncated code of the current cell
		CCLib::DgmOctree::CellCode truncatedCode = baseTruncatedCode + i;

		// if the current cell has not been built (contains no 3D points), we skip it
		std::unordered_set<CCLib::DgmOctree::CellCode>::const_iterator got = m_cellsBuilt[level].find(truncatedCode);
		if (got != m_cellsBuilt[level].end())
		{
			// get extrema of the current cell
			CCVector3 bbMin;
			CCVector3 bbMax;
			m_associatedOctree->computeCellLimits(truncatedCode, level, bbMin, bbMax, true);

			// look if there is a separating plane
			OctreeCellVisibility result = (parentResult == CELL_INSIDE_FRUSTUM ? CELL_INSIDE_FRUSTUM : separatingAxisTest(bbMin, bbMax, planesCoefficients, ptsFrustum, edges, center));

			// if the cell is not outside the frustum, there is a kind of intersection (inside or intesecting)
			if (result != CELL_OUTSIDE_FRUSTUM)
			{
				if (result == CELL_INSIDE_FRUSTUM)
					m_cellsInFrustum[level].insert(truncatedCode);
				else
					m_cellsIntersectFrustum[level].insert(truncatedCode);

				// we do the same for the children (if we have not already reached the end of the tree)
				if (level < CCLib::DgmOctree::MAX_OCTREE_LEVEL)
					computeFrustumIntersectionByLevel(level+1, truncatedCode, result, planesCoefficients, ptsFrustum, edges, center);
			}
		}
	}
}

void ccOctreeFrustumIntersector::computeFrustumIntersectionWithOctree(	std::vector< std::pair<unsigned, CCVector3> >& pointsToTest,
																		std::vector<unsigned>& inCameraFrustum,
																		const float planesCoefficients[6][4],
																		const CCVector3 ptsFrustum[8],
																		const CCVector3 edges[6],
																		const CCVector3& center)
{
	// clear old result
	{
		for (int i=0; i<=CCLib::DgmOctree::MAX_OCTREE_LEVEL; i++)
		{
			m_cellsInFrustum[i].clear();
			m_cellsIntersectFrustum[i].clear();
		}
	}

	// find intersecting cells
	computeFrustumIntersectionByLevel(1, 0, CELL_INTERSECT_FRUSTUM, planesCoefficients, ptsFrustum, edges, center);

	// get points
	unsigned char level = static_cast<unsigned char>(CCLib::DgmOctree::MAX_OCTREE_LEVEL);

	// dealing with cells completely inside the frustum
	std::unordered_set<CCLib::DgmOctree::CellCode>::const_iterator it;
	CCLib::ReferenceCloud pointsInCell(m_associatedOctree->associatedCloud());
	for (it = m_cellsInFrustum[level].begin(); it != m_cellsInFrustum[level].end(); ++it)
	{
		// get all points in cell
		if (m_associatedOctree->getPointsInCell(*it, level, &pointsInCell, true))
		{
			// all points are inside the frustum since the cell itself is completely inside
			for (size_t i=0 ; i<pointsInCell.size() ; i++)
				inCameraFrustum.push_back(pointsInCell.getPointGlobalIndex(static_cast<unsigned>(i)));
		}
	}

	// dealing with cells intersecting the frustum (not completely inside)
	for (it = m_cellsIntersectFrustum[level].begin(); it != m_cellsIntersectFrustum[level].end(); ++it)
	{
		// get all points in cell
		if (m_associatedOctree->getPointsInCell(*it, level, &pointsInCell, true))
		{
			// all points may not be inside the frustum since the cell itself is not completely inside
			size_t pointCount = pointsInCell.size();
			size_t sizeBefore = pointsToTest.size();
			pointsToTest.resize(pointCount + sizeBefore);
			for (size_t i=0; i<pointCount; i++)
			{
				unsigned currentIndice = pointsInCell.getPointGlobalIndex(static_cast<unsigned>(i));
				const CCVector3* vec = pointsInCell.getPoint(static_cast<unsigned>(i));
				pointsToTest[sizeBefore+i] = std::pair<unsigned, CCVector3>(currentIndice, *vec);
			}
		}
	}
}
