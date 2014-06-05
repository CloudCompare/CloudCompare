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

#include "ccCameraSensor.h"

//default parameters
// Parameters extracted from :
// "Accuracy and Resolution of Kinect Depth Data for Indoor Mapping Applications"
// Kourosh Khoshelham and Sander Oude Elberink
static const int	s_width =		640;						// image width 
static const int	s_height =		480;						// image height
static const float	s_f =			static_cast<float>(5.45 * 1.0e-3);				// focal length (real distance in meter)
static const float	s_sX =			static_cast<float>(9.3 * 1.0e-6);				// pixel size in x (real distance in meter)
static const float	s_sY =			static_cast<float>(9.3 * 1.0e-6);				// pixel size in y (real distance in meter)
static const float	s_vFov =		static_cast<float>(43.0 * M_PI / 180.0);		// vertical field of view (in rad)
static const float	s_skew =		static_cast<float>(0.0);						// skew in image
static const float	s_pX_offset =	static_cast<float>(-0.063 * 1.0e-3);			// offset of the principal point in x 
static const float	s_pY_offset =	static_cast<float>(-0.039 * 1.0e-3);			// offset of the principal point in y
static const float	s_zNear =		static_cast<float>(0.5);						// distance to the closest recordable depth
static const float	s_zFar =		static_cast<float>(5.0);						// distance to the furthest recordable depth
static const float	s_A =			static_cast<float>(-2.85 * 1.0e-3);				// coefficient for linearization (in meters^(-1))
static const float	s_B =			static_cast<float>(3.0);						// constant for linearization (in meters^(-1))
static const float	s_K1 =			static_cast<float>(2.42 * 1.0e-3);				// 1st radial distortion coefficient (Brown's model) 
static const float	s_K2 =			static_cast<float>(-1.7 * 1.0e-4);				// 2nd radial distortion coefficient (Brown's model)
static const float	s_K3 =			static_cast<float>(0.0);						// 3rd radial distortion coefficient (Brown's model)
static const float	s_P1 =			static_cast<float>(-3.3 * 1.0e-4);				// 1st tangential distortion coefficient (Brown's model)
static const float	s_P2 =			static_cast<float>(5.25 * 1.0e-4);				// 2nd tangential distortion coefficient (Brown's model)

ccCameraSensor::ccCameraSensor()
	: ccSensor("Camera Sensor")
{
	//graphic representation
	lockVisibility(false);

	// scale
	m_scale = 50.0f;

	// projection
	m_intrinsicParams.focalLength = s_f;
	m_intrinsicParams.pixelSize[0] = s_sX;
	m_intrinsicParams.pixelSize[1] = s_sY;
	m_intrinsicParams.skew = s_skew;
	m_intrinsicParams.vFieldOfView = s_vFov;
	m_intrinsicParams.zBoundary[0] = s_zNear;
	m_intrinsicParams.zBoundary[1] = s_zFar;
	m_intrinsicParams.imageSize[0] = s_width;
	m_intrinsicParams.imageSize[1] = s_height;
	computeProjectionMatrix();

	// uncertainty
	m_uncertaintyParams.principalPointOffset[0] = s_pX_offset;
	m_uncertaintyParams.principalPointOffset[1] = s_pY_offset;
	m_uncertaintyParams.linearDisparityParams[0] = s_A;
	m_uncertaintyParams.linearDisparityParams[1] = s_B;
	m_uncertaintyParams.K_BrownParams[0] = s_K1;
	m_uncertaintyParams.K_BrownParams[1] = s_K2;
	m_uncertaintyParams.K_BrownParams[2] = s_K3;
	m_uncertaintyParams.P_BrownParams[0] = s_P1;
	m_uncertaintyParams.P_BrownParams[1] = s_P2;
}

ccCameraSensor::~ccCameraSensor()
{
}

void ccCameraSensor::computeProjectionMatrix()
{
	m_projecMatrix.toZero();
	float* mat = m_projecMatrix.data();
	mat[0] = m_intrinsicParams.focalLength / m_intrinsicParams.pixelSize[0];
	mat[4] = m_intrinsicParams.skew;
	mat[5] = m_intrinsicParams.focalLength / m_intrinsicParams.pixelSize[1];
	mat[8] = static_cast<float>(m_intrinsicParams.imageSize[0]) / 2;
	mat[9] = static_cast<float>(m_intrinsicParams.imageSize[1]) / 2;
	mat[10] = 1.0f;
}

ccBBox ccCameraSensor::getMyOwnBB()
{
	CCVector3 vec;
	if (getActiveAbsoluteCenter(vec))
		return ccBBox(vec,vec);
	else 
		return ccBBox(); 
}

//ccBBox ccCameraSensor::getDisplayBB()
//{
//	//return getMyOwnBB();
//	ccIndexedTransformation sensorPos;
//	if (!getAbsoluteTransformation(sensorPos,m_activeIndex))
//		return ccBBox();
//
//	CCVector3 center = sensorPos.getTranslationAsVec3D();
//
//	return ccBBox(	center + CCVector3(-1,-1,-1) * m_scale,
//					center + CCVector3( 1, 1, 1) * m_scale);
//}

bool ccCameraSensor::toFile_MeOnly(QFile& out) const
{
	if (!ccSensor::toFile_MeOnly(out))
		return false;

	//projection matrix (dataVersion>=35)
	if (!m_projecMatrix.toFile(out))
		return WriteError();

	/** various parameters (dataVersion>=35) **/

	//IntrinsicParameters
	QDataStream outStream(&out);
	outStream << m_intrinsicParams.focalLength;
	outStream << m_intrinsicParams.imageSize[0];
	outStream << m_intrinsicParams.imageSize[1];
	outStream << m_intrinsicParams.pixelSize[0];
	outStream << m_intrinsicParams.pixelSize[1];
	outStream << m_intrinsicParams.skew;
	outStream << m_intrinsicParams.vFieldOfView;
	outStream << m_intrinsicParams.zBoundary[0];
	outStream << m_intrinsicParams.zBoundary[1];

	//UncertaintyParameters
	outStream << m_uncertaintyParams.K_BrownParams[0];
	outStream << m_uncertaintyParams.K_BrownParams[1];
	outStream << m_uncertaintyParams.K_BrownParams[2];
	outStream << m_uncertaintyParams.P_BrownParams[0];
	outStream << m_uncertaintyParams.P_BrownParams[1];
	outStream << m_uncertaintyParams.principalPointOffset[0];
	outStream << m_uncertaintyParams.principalPointOffset[1];
	outStream << m_uncertaintyParams.linearDisparityParams[0];
	outStream << m_uncertaintyParams.linearDisparityParams[1];
	
	//FrustumInformation
	outStream << m_frustrumInfos.isComputed;
	outStream << m_frustrumInfos.drawFrustum;
	outStream << m_frustrumInfos.drawSidePlanes;
	outStream << m_frustrumInfos.center.x;
	outStream << m_frustrumInfos.center.y;
	outStream << m_frustrumInfos.center.z;
	for (unsigned i=0; i<8; ++i)
	{
		outStream << m_frustrumInfos.frustumCorners[i].x;
		outStream << m_frustrumInfos.frustumCorners[i].y;
		outStream << m_frustrumInfos.frustumCorners[i].z;
	}

	return true;
}

bool ccCameraSensor::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccSensor::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//serialization wasn't possible before v3.5!
	if (dataVersion < 35)
		return false;

	//projection matrix (dataVersion>=35)
	if (!m_projecMatrix.fromFile(in,dataVersion,flags))
		return ReadError();

	/** various parameters (dataVersion>=35) **/

	//IntrinsicParameters
	QDataStream inStream(&in);
	inStream >> m_intrinsicParams.focalLength;
	inStream >> m_intrinsicParams.imageSize[0];
	inStream >> m_intrinsicParams.imageSize[1];
	inStream >> m_intrinsicParams.pixelSize[0];
	inStream >> m_intrinsicParams.pixelSize[1];
	inStream >> m_intrinsicParams.skew;
	inStream >> m_intrinsicParams.vFieldOfView;
	inStream >> m_intrinsicParams.zBoundary[0];
	inStream >> m_intrinsicParams.zBoundary[1];

	//UncertaintyParameters
	inStream >> m_uncertaintyParams.K_BrownParams[0];
	inStream >> m_uncertaintyParams.K_BrownParams[1];
	inStream >> m_uncertaintyParams.K_BrownParams[2];
	inStream >> m_uncertaintyParams.P_BrownParams[0];
	inStream >> m_uncertaintyParams.P_BrownParams[1];
	inStream >> m_uncertaintyParams.principalPointOffset[0];
	inStream >> m_uncertaintyParams.principalPointOffset[1];
	inStream >> m_uncertaintyParams.linearDisparityParams[0];
	inStream >> m_uncertaintyParams.linearDisparityParams[1];
	
	//FrustumInformation
	inStream >> m_frustrumInfos.isComputed;
	inStream >> m_frustrumInfos.drawFrustum;
	inStream >> m_frustrumInfos.drawSidePlanes;
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,m_frustrumInfos.center.u,3);
	for (unsigned i=0; i<8; ++i)
		ccSerializationHelper::CoordsFromDataStream(inStream,flags,m_frustrumInfos.frustumCorners[i].u,3);

	return true;
}

bool ccCameraSensor::fromLocalCoordToGlobalCoord(const CCVector3& localCoord, CCVector3& globalCoord) const
{
	ccGLMatrix rotation;
	CCVector3 center;

	if (!getActiveAbsoluteRotation(rotation) || !getActiveAbsoluteCenter(center))
		return false;

	globalCoord = rotation * localCoord + center;
	return true;
}

bool ccCameraSensor::fromGlobalCoordToLocalCoord(const CCVector3& globalCoord, CCVector3& localCoord) const
{
	ccGLMatrix rotation;
	CCVector3 center;

	if (!getActiveAbsoluteRotation(rotation) || !getActiveAbsoluteCenter(center))
		return false;

	rotation = rotation.inverse();
	localCoord = rotation * (globalCoord - center);
	return true;
}

bool ccCameraSensor::fromLocalCoordToImageCoord(const CCVector3& localCoord, CCVector2i& imageCoord/*, bool withLensError*/) const
{
	// Change in 3D image coordinates system for good projection
	CCVector3 imageCoordSystem(localCoord.x, localCoord.y, -localCoord.z); 
	
	// We test if the point is in front or behind the sensor ? If it is behind (or in the center of the sensor i.e. z=0.0), projection has no sense ! 
	if (imageCoordSystem.z < FLT_EPSILON)
		return false;
	
	// projection
	ccGLMatrix mat = getProjectionMatrix();
	CCVector3 projCoord = mat * imageCoordSystem; // at this stage, coordinates are homogeneous
	projCoord = projCoord/projCoord.z; // coordinates are now in pixels
	CCVector2i initial(static_cast<int>(projCoord.x), static_cast<int>(projCoord.y)); 
	CCVector2i coord = initial;
	
	//apply lens correction if necessary
	//if (withLensError)
	//	fromIdealImCoordToRealImCoord(initial, coord);

	//test if the projected point is into the image boundaries (width,height)
	if (	coord.x < 0 || coord.x >= m_intrinsicParams.imageSize[0]
		||	coord.y < 0 || coord.y >= m_intrinsicParams.imageSize[1] )
	{
		return false;
	}

	// Change in 3D image coordinates system
	imageCoord = coord;

	return true;
}

bool ccCameraSensor::fromImageCoordToLocalCoord(const CCVector2i& imageCoord, CCVector3& localCoord, bool withLensCorrection, float depth/*=0*/) const
{
	CCVector2i coord = imageCoord;

	// applies lens correction if needed
	if (withLensCorrection)
		fromRealImCoordToIdealImCoord(imageCoord, coord);

	// If specified depth is 0.0, it means that we want unprojection to be made in the focal plane
	float focal = m_intrinsicParams.focalLength;
	float newDepth = depth;
	
	if (abs(depth) < FLT_EPSILON)
		newDepth = focal;

	// We test if the pixel is into the image boundaries (width*height) and if input depth is positive
	if (	coord.x < 0 || coord.x >= m_intrinsicParams.imageSize[0]
		||	coord.y < 0 || coord.y >= m_intrinsicParams.imageSize[1]
		||	newDepth < focal)
	{
		return false;
	}

	// Compute local 3D coordinates
	PointCoordinateType x = (coord.x - m_intrinsicParams.imageSize[0]/2) * m_intrinsicParams.pixelSize[0] / focal;
	PointCoordinateType y = (coord.y - m_intrinsicParams.imageSize[1]/2) * m_intrinsicParams.pixelSize[1] / focal;
	localCoord = CCVector3(x, y, -PC_ONE) * newDepth;
	
	return true;
}

bool ccCameraSensor::fromGlobalCoordToImageCoord(const CCVector3& globalCoord, CCVector3& localCoord, CCVector2i& imageCoord/*, bool withLensError*/) const
{
	if (!fromGlobalCoordToLocalCoord(globalCoord,localCoord))
		return false;
	
	return fromLocalCoordToImageCoord(localCoord, imageCoord/*, withLensError*/);
}

bool ccCameraSensor::fromImageCoordToGlobalCoord(const CCVector2i& imageCoord, CCVector3& localCoord, CCVector3& globalCoord, bool withLensCorrection, float depth/*=0*/) const
{	
	if (!fromImageCoordToLocalCoord(imageCoord, localCoord, withLensCorrection, depth))
		return false;
	
	return fromLocalCoordToGlobalCoord(localCoord, globalCoord);
}

bool ccCameraSensor::fromRealImCoordToIdealImCoord(const CCVector2i& real, CCVector2i& ideal) const
{
	const float& sX = m_intrinsicParams.pixelSize[0];
	const float& sY = m_intrinsicParams.pixelSize[1];

	// 1st correction : principal point correction
	float cx = static_cast<float>(m_intrinsicParams.imageSize[0]) / 2 + m_uncertaintyParams.principalPointOffset[0] / sX; // in pixels
	float cy = static_cast<float>(m_intrinsicParams.imageSize[1]) / 2 + m_uncertaintyParams.principalPointOffset[1] / sY; // in pixels

	// 2nd correction : Brown's lens distortion correction
	float dx = (static_cast<float>(real.x)-cx) * m_intrinsicParams.pixelSize[0];	// real distance 
	float dy = (static_cast<float>(real.y)-cy) * m_intrinsicParams.pixelSize[1];	// real distance
	float dx2 = dx*dx;
	float dy2 = dy*dy;
	float r = sqrt(dx2 + dy2);
	float r2 = r*r;
	float r4 = r2*r2;
	float r6 = r4*r2;
	float K1 = m_uncertaintyParams.K_BrownParams[0];
	float K2 = m_uncertaintyParams.K_BrownParams[1];
	float K3 = m_uncertaintyParams.K_BrownParams[2];
	float P1 = m_uncertaintyParams.P_BrownParams[0];
	float P2 = m_uncertaintyParams.P_BrownParams[1];

	// compute new value
	float correctedX = (dx * (1 + K1*r2 + K2*r4 + K3*r6)  +  P1 * (r2 + 2*dx2)  +  2*P2*dx*dy);
	float correctedY = (dy * (1 + K1*r2 + K2*r4 + K3*r6)  +  P2 * (r2 + 2*dy2)  +  2*P1*dx*dy);
	ideal.x = static_cast<int>(correctedX / sX);
	ideal.y = static_cast<int>(correctedY / sY);

	// We test if the new pixel is into the image boundaries
	return (	ideal.x >= 0 && ideal.x < m_intrinsicParams.imageSize[0]
			&&	ideal.y >= 0 && ideal.y < m_intrinsicParams.imageSize[1] );
}

//TODO
//bool ccCameraSensor::fromIdealImCoordToRealImCoord(const CCVector2i& ideal, CCVector2i& real) const
//{
//	return true;
//}

bool ccCameraSensor::computeUncertainty(const CCVector2i& pixel, const float depth, Vector3Tpl<ScalarType>& sigma) const
{
	//TODO ==> check if the input pixel coordinate must be the real or ideal projection

	const int& u = pixel.x;
	const int& v = pixel.y;
	const int& width = m_intrinsicParams.imageSize[0];
	const int& height = m_intrinsicParams.imageSize[1];

	// check validity 
	if (u < 0 || u > width || v < 0 || v > height || depth < FLT_EPSILON)
		return false;

	// init parameters
	const float& mu = m_intrinsicParams.pixelSize[0];
	const float& A = m_uncertaintyParams.linearDisparityParams[0];
	const float& f = m_intrinsicParams.focalLength;
	float sigmaD = mu / 8;
	float z2 = depth*depth;

	// computes uncertainty
	sigma.x = static_cast<ScalarType>(abs(A * (u-width/2) / f * z2 * sigmaD));
	sigma.y = static_cast<ScalarType>(abs(A * (v-height/2) / f * z2 * sigmaD));
	sigma.z = static_cast<ScalarType>(abs(A * z2 * sigmaD));

	return true;
}

bool ccCameraSensor::computeUncertainty(CCLib::ReferenceCloud* points, std::vector< Vector3Tpl<ScalarType> >& accuracy/*, bool lensCorrection*/) const
{
	if (!points)
		return false;

	unsigned count = points->size();
	accuracy.clear();
	try
	{
		accuracy.resize(count);
	}
	catch(std::bad_alloc)
	{
		ccLog::Warning("[ccCameraSensor::computeUncertainty] Not enough memory!");
		return false;
	}

	for (unsigned i=0; i<count; i++)
	{
		const CCVector3* coordGlobal = points->getPoint(i);
		CCVector3 coordLocal;
		CCVector2i coordImage;

		if (fromGlobalCoordToImageCoord(*coordGlobal, coordLocal, coordImage/*, lensCorrection*/))
		{
			computeUncertainty(coordImage, abs(coordLocal.z), accuracy[i]);
		}
		else
		{
			accuracy[i].x = accuracy[i].y = accuracy[i].z = NAN_VALUE;
		}
	}

	return true;
}

bool ccCameraSensor::isGlobalCoordInFrustrum(const CCVector3& globalCoord/*, bool withLensCorrection*/) const
{
	CCVector3 localCoord;
	CCVector2i imageCoord;

	// Tests if the projection is in the field of view
	if (!fromGlobalCoordToImageCoord(globalCoord, localCoord, imageCoord/*, withLensCorrection*/))
		return false;

	// Tests if the projected point is between zNear and zFar
	float z = localCoord.z;
	float n = m_intrinsicParams.zBoundary[0];
	float f = m_intrinsicParams.zBoundary[1];

	return (-z <= f && -z > n && abs(f+z) >= FLT_EPSILON && abs(n+z) >= FLT_EPSILON);
}

void ccCameraSensor::computeFrustumCorners()
{
	const float& focal = m_intrinsicParams.focalLength;
	float aspectRatio = static_cast<float>(m_intrinsicParams.imageSize[1]) / static_cast<float>(m_intrinsicParams.imageSize[0]);
	float xInFocal = abs( tan(m_intrinsicParams.vFieldOfView / aspectRatio / 2) * focal );
	float yInFocal = abs( tan(m_intrinsicParams.vFieldOfView / 2) * focal );
	const float& zNear = m_intrinsicParams.zBoundary[0];
	const float& zFar = m_intrinsicParams.zBoundary[1];

	// compute points of frustum in image coordinate system (attention : in the system, z=-z)
	// DO NOT MODIFY THE ORDER OF THE CORNERS!! A LOT OF CODE DEPENDS OF THIS ORDER!! 
	m_frustrumInfos.frustumCorners[0] = CCVector3( xInFocal/focal, yInFocal/focal,-PC_ONE) * zNear;
	m_frustrumInfos.frustumCorners[1] = CCVector3( xInFocal/focal, yInFocal/focal,-PC_ONE) * zFar;
	m_frustrumInfos.frustumCorners[2] = CCVector3( xInFocal/focal,-yInFocal/focal,-PC_ONE) * zNear;
	m_frustrumInfos.frustumCorners[3] = CCVector3( xInFocal/focal,-yInFocal/focal,-PC_ONE) * zFar;
	m_frustrumInfos.frustumCorners[4] = CCVector3(-xInFocal/focal,-yInFocal/focal,-PC_ONE) * zNear;
	m_frustrumInfos.frustumCorners[5] = CCVector3(-xInFocal/focal,-yInFocal/focal,-PC_ONE) * zFar;
	m_frustrumInfos.frustumCorners[6] = CCVector3(-xInFocal/focal, yInFocal/focal,-PC_ONE) * zNear;
	m_frustrumInfos.frustumCorners[7] = CCVector3(-xInFocal/focal, yInFocal/focal,-PC_ONE) * zFar;

	// compute center of the circumscribed sphere 
	const float& x0 = m_frustrumInfos.frustumCorners[0].x;
	const float& y0 = m_frustrumInfos.frustumCorners[0].y;
	const float& z0 = m_frustrumInfos.frustumCorners[0].z;
	const float& x5 = m_frustrumInfos.frustumCorners[5].x;
	const float& y5 = m_frustrumInfos.frustumCorners[5].y;
	const float& z5 = m_frustrumInfos.frustumCorners[5].z;
	float z = (abs(z0-z5) < FLT_EPSILON ? z0 : (x0*x0 + y0*y0 + z0*z0 - x5*x5 - y5*y5 - z5*z5) / (2*(z0-z5)));
	m_frustrumInfos.center = CCVector3(0, 0, z);

	// frustrum corners are now computed
	m_frustrumInfos.isComputed = true;
}

void ccCameraSensor::computeGlobalPlaneCoefficients(float planeCoefficients[6][4], CCVector3 frustrumCorners[8], CCVector3 edges[6], CCVector3& center)
{
	if (!m_frustrumInfos.isComputed)
		computeFrustumCorners();

	// compute frustrum corners in the global coordinates system
	fromLocalCoordToGlobalCoord(m_frustrumInfos.frustumCorners[0], frustrumCorners[0]);
	fromLocalCoordToGlobalCoord(m_frustrumInfos.frustumCorners[1], frustrumCorners[1]);
	fromLocalCoordToGlobalCoord(m_frustrumInfos.frustumCorners[2], frustrumCorners[2]);
	fromLocalCoordToGlobalCoord(m_frustrumInfos.frustumCorners[3], frustrumCorners[3]);
	fromLocalCoordToGlobalCoord(m_frustrumInfos.frustumCorners[4], frustrumCorners[4]);
	fromLocalCoordToGlobalCoord(m_frustrumInfos.frustumCorners[5], frustrumCorners[5]);
	fromLocalCoordToGlobalCoord(m_frustrumInfos.frustumCorners[6], frustrumCorners[6]);
	fromLocalCoordToGlobalCoord(m_frustrumInfos.frustumCorners[7], frustrumCorners[7]);

	/*
	//-- METHOD 1 --//
	// See "Fast Extraction of Viewing Frustum Planes from the World-View-Projection Matrix" of Gil Gribb and Klaus Hartmann
	// Attention !! With this method, plane equations are not normalized ! You should add normalization if you need it :
	// It means that if you have your plane equation in the form (ax + by + cz + d = 0), then --> k = sqrt(a*a + b*b + c*c) and your new coefficients are --> a=a/k, b=b/k, c=c/k, d=d/k
	ccGLMatrix mat = m_projecMatrix * m_orientMatrix;
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
	for (int i=0 ; i<4 ; i++)
	{
		CCVector3 v1 = frustrumCorners[i*2+1] - frustrumCorners[i*2];
		CCVector3 v2 = frustrumCorners[((i+1)*2)%8] - frustrumCorners[i*2];
		CCVector3 n = v1.cross(v2); n.normalize();
		planeCoefficients[i][0] = n.x;
		planeCoefficients[i][1] = n.y;
		planeCoefficients[i][2] = n.z;
		planeCoefficients[i][3] = -frustrumCorners[i*2].dot(n);
	}
	// compute equations for near and far planes
	{
		CCVector3 v1 = frustrumCorners[0] - frustrumCorners[6];
		CCVector3 v2 = frustrumCorners[4] - frustrumCorners[6];
		CCVector3 n = v1.cross(v2); n.normalize();
		planeCoefficients[4][0] = n.x;
		planeCoefficients[4][1] = n.y;
		planeCoefficients[4][2] = n.z;
		planeCoefficients[4][3] = -frustrumCorners[6].dot(n);
		planeCoefficients[5][0] = -n.x;
		planeCoefficients[5][1] = -n.y;
		planeCoefficients[5][2] = -n.z;
		planeCoefficients[5][3] = -frustrumCorners[7].dot(-n);
	}	

	// compute frustrum edges
	{
		edges[0] = frustrumCorners[1] - frustrumCorners[0];
		edges[1] = frustrumCorners[3] - frustrumCorners[2];
		edges[2] = frustrumCorners[5] - frustrumCorners[4];
		edges[3] = frustrumCorners[7] - frustrumCorners[6];
		edges[4] = frustrumCorners[6] - frustrumCorners[0];
		edges[5] = frustrumCorners[2] - frustrumCorners[0];
		for (int i=0 ; i<6 ; i++)
			edges[i].normalize();
	}

	// compute frustrum center in the global coordinates system
	fromLocalCoordToGlobalCoord(m_frustrumInfos.center, center);
}

void ccCameraSensor::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	//we draw here a little 3d representation of the sensor and some of its attributes
	if (MACRO_Draw3D(context))
	{
		bool pushName = MACRO_DrawEntityNames(context);

		if (pushName)
		{
			//not particulary fast
			if (MACRO_DrawFastNamesOnly(context))
				return;
			glPushName(getUniqueID());
		}

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			ccIndexedTransformation sensorPos;
			if (!getAbsoluteTransformation(sensorPos,m_activeIndex))
			{
				//no visible position for this index!
				glPopMatrix();
				if (pushName)
					glPopName();
				return;
			}
				
			glMultMatrixf(sensorPos.data());
		}

		{
			float aspectRatio = static_cast<float>(m_intrinsicParams.imageSize[1]) / static_cast<float>(m_intrinsicParams.imageSize[0]);
			CCVector3 upperLeftPoint;
			upperLeftPoint.z = m_scale * m_intrinsicParams.focalLength;
			upperLeftPoint.y = upperLeftPoint.z * tan(m_intrinsicParams.vFieldOfView / 2);
			upperLeftPoint.x = upperLeftPoint.z * tan(m_intrinsicParams.vFieldOfView / aspectRatio / 2);

			//up arrow
			const float arrowHeight		= 1.5f * upperLeftPoint.y;
			const float baseHeight		= 1.2f * upperLeftPoint.y;
			const float arrowHalfWidth	= 0.5f * upperLeftPoint.x;
			const float baseHalfWidth	= 0.3f * upperLeftPoint.x;

			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glColor3ubv(m_color.u);
			
			//near plane
			glBegin(GL_LINE_LOOP);
			glVertex3f( upperLeftPoint.x,  upperLeftPoint.y, -upperLeftPoint.z);
			glVertex3f(-upperLeftPoint.x,  upperLeftPoint.y, -upperLeftPoint.z);
			glVertex3f(-upperLeftPoint.x, -upperLeftPoint.y, -upperLeftPoint.z);
			glVertex3f( upperLeftPoint.x, -upperLeftPoint.y, -upperLeftPoint.z);
			glEnd();

			//side lines
			glBegin(GL_LINES);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f( upperLeftPoint.x,  upperLeftPoint.y, -upperLeftPoint.z);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(-upperLeftPoint.x,  upperLeftPoint.y, -upperLeftPoint.z);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(-upperLeftPoint.x, -upperLeftPoint.y, -upperLeftPoint.z);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f( upperLeftPoint.x, -upperLeftPoint.y, -upperLeftPoint.z);
			glEnd();

			//base
			glBegin(GL_QUADS);
			glVertex3f(-baseHalfWidth, upperLeftPoint.y, -upperLeftPoint.z);
			glVertex3f( baseHalfWidth, upperLeftPoint.y, -upperLeftPoint.z);
			glVertex3f( baseHalfWidth, baseHeight,  -upperLeftPoint.z);
			glVertex3f(-baseHalfWidth, baseHeight,  -upperLeftPoint.z);
			glEnd();

			//arrow
			glBegin(GL_TRIANGLES);
			glVertex3f( 0.0f,           arrowHeight, -upperLeftPoint.z);
			glVertex3f(-arrowHalfWidth, baseHeight,  -upperLeftPoint.z);
			glVertex3f( arrowHalfWidth, baseHeight,  -upperLeftPoint.z);
			glEnd();

			//frustrum
			if (m_frustrumInfos.drawFrustum)
			{
				if (!m_frustrumInfos.isComputed)
					computeFrustumCorners();

				CCVector3* pts = m_frustrumInfos.frustumCorners;

				//frustum area (lines)
				glLineWidth(2.0);
				glBegin(GL_LINE_LOOP);
				glVertex3fv(pts[0].u);
				glVertex3fv(pts[1].u);
				glVertex3fv(pts[3].u);
				glVertex3fv(pts[2].u);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3fv(pts[2].u);
				glVertex3fv(pts[3].u);
				glVertex3fv(pts[5].u);
				glVertex3fv(pts[4].u);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3fv(pts[4].u);
				glVertex3fv(pts[5].u);
				glVertex3fv(pts[7].u);
				glVertex3fv(pts[6].u);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3fv(pts[6].u);
				glVertex3fv(pts[7].u);
				glVertex3fv(pts[1].u);
				glVertex3fv(pts[0].u);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3fv(pts[6].u);
				glVertex3fv(pts[0].u);
				glVertex3fv(pts[2].u);
				glVertex3fv(pts[4].u);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3fv(pts[1].u);
				glVertex3fv(pts[7].u);
				glVertex3fv(pts[5].u);
				glVertex3fv(pts[3].u);
				glEnd();
				glLineWidth(1.0f);

				//frustum area (planes)
				if (m_frustrumInfos.drawSidePlanes)
				{
					glEnable(GL_BLEND);
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
					glColor4ub(m_color.x, m_color.y, m_color.z, 76);

					glBegin(GL_QUADS);
					glVertex3fv(pts[0].u);
					glVertex3fv(pts[2].u);
					glVertex3fv(pts[3].u);
					glVertex3fv(pts[1].u);
					glVertex3fv(pts[2].u);
					glVertex3fv(pts[4].u);
					glVertex3fv(pts[5].u);
					glVertex3fv(pts[3].u);
					glVertex3fv(pts[4].u);
					glVertex3fv(pts[6].u);
					glVertex3fv(pts[7].u);
					glVertex3fv(pts[5].u);
					glVertex3fv(pts[6].u);
					glVertex3fv(pts[0].u);
					glVertex3fv(pts[1].u);
					glVertex3fv(pts[7].u);
					glVertex3fv(pts[6].u);
					glVertex3fv(pts[4].u);
					glVertex3fv(pts[2].u);
					glVertex3fv(pts[0].u);
					glVertex3fv(pts[1].u);
					glVertex3fv(pts[3].u);
					glVertex3fv(pts[5].u);
					glVertex3fv(pts[7].u);
					glEnd();
					glDisable(GL_BLEND); 
				}
			}

			//axis (temporary)
			if (true)
			{
				glLineWidth(1.0);

				// right vector
				glColor3ubv(ccColor::white);
				glBegin(GL_LINES);
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(1.0f, 0.0f, 0.0f);
				glEnd();

				// up vector
				glColor3ubv(ccColor::green);
				glBegin(GL_LINES);
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(0.0f, 1.0f, 0.0f);
				glEnd();

				// view vector
				glColor3ubv(ccColor::blue);
				glBegin(GL_LINES);
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(0.0f, 0.0f, 1.0f);
				glEnd();
			}

			if (pushName)
				glPopName();
		}

		glPopMatrix();
	}
}


bool ccOctreeFrustrumIntersector::build(CCLib::DgmOctree* octree)
{
	if (!octree)
		return false;

	for (int i=0; i<CCLib::DgmOctree::MAX_OCTREE_LEVEL+1 ; i++)
		m_cellsBuilt[i].clear();

	const CCLib::DgmOctree::cellsContainer& thePointsAndTheirCellCodes = octree->pointsAndTheirCellCodes();
	CCLib::DgmOctree::cellsContainer::const_iterator it = thePointsAndTheirCellCodes.begin();

	try
	{
		for (it=thePointsAndTheirCellCodes.begin(); it!=thePointsAndTheirCellCodes.end(); it++)
		{
			CCLib::DgmOctree::OctreeCellCodeType completeCode = it->theCode;
			for (unsigned char level=1; level<=CCLib::DgmOctree::MAX_OCTREE_LEVEL; level++)
			{
				uchar bitDec = GET_BIT_SHIFT(level);
				m_cellsBuilt[level].insert(completeCode >> bitDec);
			}
		}
	}
	catch (std::bad_alloc)
	{
		ccLog::Warning("[ccCameraSensor::prepareOctree] Not enough memory!");
		for (int i=0; i<=CCLib::DgmOctree::MAX_OCTREE_LEVEL; i++)
			m_cellsBuilt[i].clear();
		return false;
	}

	m_associatedOctree = octree;

	return true;
}

//// an other method to compute frustrum cell intersection (not used)
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
//		return CELL_OUTSIDE_FRUSTRUM;
//	if ((s+e) < 0.0)
//		return CELL_INSIDE_FRUSTRUM;
//	return CELL_INTERSECT_FRUSTRUM;
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
//		if (result == CELL_OUTSIDE_FRUSTRUM)
//			return CELL_OUTSIDE_FRUSTRUM;
//		if (result == CELL_INTERSECT_FRUSTRUM)
//			intersecting = true;
//	}
//
//	if (intersecting == true)
//		return CELL_INTERSECT_FRUSTRUM;
//	return CELL_INSIDE_FRUSTRUM;
//}

ccOctreeFrustrumIntersector::OctreeCellVisibility
	ccOctreeFrustrumIntersector::separatingAxisTest(	const CCVector3& bbMin,
														const CCVector3& bbMax,
														const float planesCoefficients[6][4],
														const CCVector3 frustrumCorners[8],
														const CCVector3 frustrumEdges[6],
														const CCVector3& frustrumCenter)
{
	// first test : if the box is too far from the frustrum, there is no intersection
	CCVector3 boxCenter = (bbMax + bbMin) / 2;
	PointCoordinateType dCenter = (boxCenter - frustrumCenter).norm();
	PointCoordinateType boxRadius = (bbMax - bbMin).norm();
	PointCoordinateType frustrumRadius = (frustrumCorners[0] - frustrumCenter).norm();
	if (dCenter > boxRadius + frustrumRadius)
		return CELL_OUTSIDE_FRUSTRUM;

	// We could add a test : is the cell circumscribed circle in the frustrum incircle frustrum ?
	// --> if we are lucky, it could save a lot of time !...

	//box corners
	CCVector3 boxCorners[8];
	{
		for (unsigned i=0; i<8; i++)
			boxCorners[i] = CCVector3(	i&4 ? bbMin.x : bbMax.x,
										i&2 ? bbMin.y : bbMax.y,
										i&1 ? bbMin.z : bbMax.z);
	}

	//There are 28 tests to perform:
	//	nbFacesCube = n1 = 3;
	//	nbFacesFrustrum = n2 = 5;
	//	nbEdgesCube = n3 = 3;
	//	nbEdgesFrustrum = n4 = 6;
	//	nbOtherFrustrumCombinations = n5 = 2
	//	nbVecToTest = n1 + n2 + n3*n4 = 3 + 5 + 3*6 + n5 = 28;
	static const unsigned nbVecToTest = 28;
	CCVector3 VecToTest[nbVecToTest];
	// vectors orthogonals to box planes
	VecToTest[0]  = CCVector3(1, 0, 0);
	VecToTest[1]  = CCVector3(0, 1, 0);
	VecToTest[2]  = CCVector3(0, 0, 1);
	// vectors orthogonals to frustrum planes
	VecToTest[3]  = CCVector3(planesCoefficients[0][0], planesCoefficients[0][1], planesCoefficients[0][2]);
	VecToTest[4]  = CCVector3(planesCoefficients[1][0], planesCoefficients[1][1], planesCoefficients[1][2]);
	VecToTest[5]  = CCVector3(planesCoefficients[2][0], planesCoefficients[2][1], planesCoefficients[2][2]);
	VecToTest[6]  = CCVector3(planesCoefficients[3][0], planesCoefficients[3][1], planesCoefficients[3][2]);
	VecToTest[7]  = CCVector3(planesCoefficients[4][0], planesCoefficients[4][1], planesCoefficients[4][2]);	
	// combinations box and frustrum
	VecToTest[8]  = VecToTest[0].cross(frustrumEdges[0]);
	VecToTest[9]  = VecToTest[0].cross(frustrumEdges[1]);
	VecToTest[10] = VecToTest[0].cross(frustrumEdges[2]);
	VecToTest[11] = VecToTest[0].cross(frustrumEdges[3]);
	VecToTest[12] = VecToTest[0].cross(frustrumEdges[4]);
	VecToTest[13] = VecToTest[0].cross(frustrumEdges[5]);
	VecToTest[14] = VecToTest[1].cross(frustrumEdges[0]);
	VecToTest[15] = VecToTest[1].cross(frustrumEdges[1]);
	VecToTest[16] = VecToTest[1].cross(frustrumEdges[2]);
	VecToTest[17] = VecToTest[1].cross(frustrumEdges[3]);
	VecToTest[18] = VecToTest[1].cross(frustrumEdges[4]);
	VecToTest[19] = VecToTest[1].cross(frustrumEdges[5]);
	VecToTest[20] = VecToTest[2].cross(frustrumEdges[0]);
	VecToTest[21] = VecToTest[2].cross(frustrumEdges[1]);
	VecToTest[22] = VecToTest[2].cross(frustrumEdges[2]);
	VecToTest[23] = VecToTest[2].cross(frustrumEdges[3]);
	VecToTest[24] = VecToTest[2].cross(frustrumEdges[4]);
	VecToTest[25] = VecToTest[2].cross(frustrumEdges[5]);
	// combinations frustrum
	VecToTest[26] = frustrumEdges[0].cross(frustrumEdges[2]);
	VecToTest[27] = frustrumEdges[1].cross(frustrumEdges[3]);

	// normalization
	{
		for (unsigned i=0; i<nbVecToTest; i++)
			VecToTest[i].normalize();
	}

	bool boxInside = true;

	// project volume corners
	{
		for (unsigned i=0; i<nbVecToTest; i++)
		{
			CCVector3 testVec = VecToTest[i];
		
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
		
			//frustrum points
			float dMinFru = testVec.dot(frustrumCorners[0]);
			float dMaxFru = dMinFru;
			{
				for (unsigned j=1; j<8; j++)
				{
					float d = testVec.dot(frustrumCorners[j]);
					if (d > dMaxFru)
						dMaxFru = d;
					if (d < dMinFru)
						dMinFru = d;
				}
			}

			//if this plane is a separating plane, the cell is outside the frustrum
			if (dMaxBox < dMinFru || dMaxFru < dMinBox)
				return CELL_OUTSIDE_FRUSTRUM;

			// if this plane is NOT a separating plane, the cell is at least intersecting the frustrum
			else 
			{
				// moreover, the cell can be completely inside the frustrum...
				if (dMaxBox>dMaxFru || dMinBox<dMinFru)
					boxInside = false;
			}
		}
	}

	return boxInside ? CELL_INSIDE_FRUSTRUM : CELL_INTERSECT_FRUSTRUM;
}

void ccOctreeFrustrumIntersector::computeFrustumIntersectionByLevel(unsigned char level,
																	CCLib::DgmOctree::OctreeCellCodeType parentTruncatedCode,
																	OctreeCellVisibility parentResult,
																	const float planesCoefficients[6][4],
																	const CCVector3 ptsFrustrum[8],
																	const CCVector3 edges[6],
																	const CCVector3& center)
{
	if (parentResult == CELL_OUTSIDE_FRUSTRUM)
		return;

	// move code to the left
	CCLib::DgmOctree::OctreeCellCodeType baseTruncatedCode = (parentTruncatedCode << 3);

	// test to do on the 8 child cells
	for (unsigned i=0; i<8; i++)
	{
		// set truncated code of the current cell
		CCLib::DgmOctree::OctreeCellCodeType truncatedCode = baseTruncatedCode + i;

		// if the cell current has not been built (contains no 3D points), we skip
		std::set<CCLib::DgmOctree::OctreeCellCodeType>::const_iterator got = m_cellsBuilt[level].find(truncatedCode);
		if (got != m_cellsBuilt[level].end())
		{
			// get extrema of the current cell
			CCVector3 bbMin, bbMax;
			m_associatedOctree->computeCellLimits(truncatedCode, level, bbMin.u, bbMax.u, true);

			// look if there is a separating plane
			OctreeCellVisibility result = (parentResult == CELL_INSIDE_FRUSTRUM ? CELL_INSIDE_FRUSTRUM : separatingAxisTest(bbMin, bbMax, planesCoefficients, ptsFrustrum, edges, center));

			// if the cell is not outside the frustrum, there is a kind of intersection (inside or intesecting)
			if (result != CELL_OUTSIDE_FRUSTRUM)
			{
				if (result == CELL_INSIDE_FRUSTRUM)
					m_cellsInFrustum[level].insert(truncatedCode);
				else
					m_cellsIntersectFrustum[level].insert(truncatedCode);
				
				// we do the same for the children (if we have not already reached the end of the tree)
				if (level < CCLib::DgmOctree::MAX_OCTREE_LEVEL)
					computeFrustumIntersectionByLevel(level+1, truncatedCode, result, planesCoefficients, ptsFrustrum, edges, center);
			}
		} 
	}
}

void ccOctreeFrustrumIntersector::computeFrustumIntersectionWithOctree(	std::vector< std::pair<unsigned, CCVector3> >& pointsToTest,
																		std::vector<unsigned>& inCameraFrustrum,
																		const float planesCoefficients[6][4],
																		const CCVector3 ptsFrustrum[8],
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
	computeFrustumIntersectionByLevel(1, 0, CELL_INTERSECT_FRUSTRUM, planesCoefficients, ptsFrustrum, edges, center);

	// get points
	unsigned char level = static_cast<unsigned char>(CCLib::DgmOctree::MAX_OCTREE_LEVEL);

	// dealing with cells completely inside the frustrum
	std::set<CCLib::DgmOctree::OctreeCellCodeType>::const_iterator it;
	CCLib::ReferenceCloud pointsInCell(m_associatedOctree->associatedCloud());
	for (it = m_cellsInFrustum[level].begin(); it != m_cellsInFrustum[level].end(); it++)
	{
		// get all points in cell
		if (m_associatedOctree->getPointsInCell(*it, level, &pointsInCell, true))
		{
			// all points are inside the frustrum since the cell itself is completely inside
			for (size_t i=0 ; i<pointsInCell.size() ; i++)
				inCameraFrustrum.push_back(pointsInCell.getPointGlobalIndex(static_cast<unsigned>(i)));			
		}
	}

	// dealing with cells intersecting the frustrum (not completely inside)
	for (it = m_cellsIntersectFrustum[level].begin(); it != m_cellsIntersectFrustum[level].end(); it++)
	{
		// get all points in cell
		if (m_associatedOctree->getPointsInCell(*it, level, &pointsInCell, true))
		{		
			// all points may not be inside the frustrum since the cell itself is not completely inside
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
