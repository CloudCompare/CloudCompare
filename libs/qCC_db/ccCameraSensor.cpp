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


ccCameraSensor::ccCameraSensor()
	: ccSensor("Camera Sensor")
{
    //graphic representation
    lockVisibility(false);

	//default parameters
	// Parameters extracted from :
	// "Accuracy and Resolution of Kinect Depth Data for Indoor Mapping Applications"
	// Kourosh Khoshelham and Sander Oude Elberink
	const float f = 5.45 * pow(10.0,-3);			// focal length (real distance in meter)
	const float sX = 9.3 * pow(10.0,-6);			// pixel size in x (real distance in meter)
	const float sY = 9.3 * pow(10.0,-6);			// pixel size in y (real distance in meter)
	const float vFov = 43.0 * M_PI / 180.0;			// vertical field of view (in rad)
	const int	width = 640;						// image width 
	const int	height = 480;						// image height
	const float skew = 0.0;							// skew in image
	const float pX_offset = -0.063 * pow(10.0,-3);	// offset of the principal point in x 
	const float pY_offset = -0.039 * pow(10.0,-3);	// offset of the principal point in y
	const float zNear = 0.5;						// distance to the closest recordable depth
	const float zFar = 5.0;							// distance to the furthest recordable depth
	const float A = -2.85 * pow(10.0,-3);			// coefficient for linearization (in meters^(-1))
	const float B = 3.0;							// constant for linearization (in meters^(-1))
	const float K1 = 2.42 * pow(10.0,-3);			// 1st radial distortion coefficient (Brown's model) 
	const float K2 = -1.7 * pow(10.0,-4);			// 2nd radial distortion coefficient (Brown's model)
	const float K3 = 0.0;							// 3rd radial distortion coefficient (Brown's model)
	const float P1 = -3.3 * pow(10.0,-4);			// 1st tangential distortion coefficient (Brown's model)
	const float P2 = 5.25 * pow(10.0,-4);			// 2nd tangential distortion coefficient (Brown's model)
	
	// scale
	m_scale = 50.0f;
	
	// projection
	m_intrinsicParams.focalLength = f;
	m_intrinsicParams.pixelSize[0] = sX;
	m_intrinsicParams.pixelSize[1] = sY;
	m_intrinsicParams.skew = skew;
	m_intrinsicParams.vFieldOfView = vFov;
	m_intrinsicParams.zBoundary[0] = zNear;
	m_intrinsicParams.zBoundary[1] = zFar;
	m_intrinsicParams.imageSize[0] = width;
	m_intrinsicParams.imageSize[1] = height;
	computeProjectionMatrix();

	// uncertainty
	m_uncertaintyParams.principalPointOffset[0] = pX_offset;
	m_uncertaintyParams.principalPointOffset[1] = pY_offset;
	m_uncertaintyParams.linearDisparityParams[0] = A;
	m_uncertaintyParams.linearDisparityParams[1] = B;
	m_uncertaintyParams.K_BrownParams[0] = K1;
	m_uncertaintyParams.K_BrownParams[1] = K2;
	m_uncertaintyParams.K_BrownParams[2] = K3;
	m_uncertaintyParams.P_BrownParams[0] = P1;
	m_uncertaintyParams.P_BrownParams[1] = P2;
}

ccCameraSensor::~ccCameraSensor()
{
}

void ccCameraSensor::computeProjectionMatrix()
{
	float mat[16] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	mat[0] = m_intrinsicParams.focalLength / m_intrinsicParams.pixelSize[0];
	mat[4] = m_intrinsicParams.skew;
	mat[5] = m_intrinsicParams.focalLength / m_intrinsicParams.pixelSize[1];
	mat[8] = (float)m_intrinsicParams.imageSize[0] / 2.0;
	mat[9] = (float)m_intrinsicParams.imageSize[1] / 2.0;
	mat[10] = 1.0;
	m_projecMatrix = ccGLMatrix(mat);
}

const ccGLMatrix& ccCameraSensor::getProjectionMatrix()
{
	return m_projecMatrix;
}

ccBBox ccCameraSensor::getMyOwnBB()
{
	return ccBBox();
}

ccBBox ccCameraSensor::getDisplayBB()
{
	//return getMyOwnBB();
	ccIndexedTransformation sensorPos;
	if (!getAbsoluteTransformation(sensorPos,m_activeIndex))
		return ccBBox();

	CCVector3 center = sensorPos.getTranslationAsVec3D();

    return ccBBox(	center + CCVector3(-1,-1,-1) * m_scale,
					center + CCVector3( 1, 1, 1) * m_scale);
}

bool ccCameraSensor::toFile_MeOnly(QFile& out) const
{
	return true;
}

bool ccCameraSensor::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	return true;
}

bool ccCameraSensor::fromLocalCoordToGlobalCoord(const CCVector3& localCoord, CCVector3& globalCoord)
{
	ccGLMatrix rotation;
	CCVector3 center;

	if (!getActiveAbsoluteRotation(rotation) || !getActiveAbsoluteCenter(center))
		return false;

	globalCoord = rotation * localCoord + center;
	return true;
}

bool ccCameraSensor::fromGlobalCoordToLocalCoord(const CCVector3& globalCoord, CCVector3& localCoord)
{
	ccGLMatrix rotation;
	CCVector3 center;

	if (!getActiveAbsoluteRotation(rotation) || !getActiveAbsoluteCenter(center))
		return false;

	rotation = rotation.inverse();
	localCoord = rotation * (globalCoord - center);
	return true;
}

bool ccCameraSensor::fromLocalCoordToImageCoord(const CCVector3& localCoord, CCVector2i& imageCoord, bool withLensError)
{
	// Change in 3D image coordinates system for good projection
	CCVector3 imageCoordSystem = CCVector3(localCoord.x, localCoord.y, -localCoord.z); 
	
	// We test if the point is in front or behind the sensor ? If it is behind (or in the center of the sensor i.e. z=0.0), projection has no sense ! 
	if (imageCoordSystem.z < 0.0 || abs(imageCoordSystem.z) < FLT_EPSILON)
		return false;
	
	// projection
	ccGLMatrix mat = getProjectionMatrix();
	CCVector3 projCoord = mat * imageCoordSystem; // at this stage, coordinates are homogeneous
	projCoord = projCoord/projCoord.z; // coordinates are now in pixels
	CCVector2i initial((int)(projCoord.x), (int)(projCoord.y)); 
	CCVector2i coord = initial;
	
	// applying lens corrections if needed
	if (withLensError)
		fromIdealImCoordToRealImCoord(initial, coord);

	// We test if the projected point is into the image boundaries (width,height) ?
	if (coord.x<0 || coord.x>=m_intrinsicParams.imageSize[0] || coord.y<0 || coord.y>=m_intrinsicParams.imageSize[1])
		return false;

	// Change in 3D image coordinates system
	imageCoord = CCVector2i(coord.x, coord.y);
	return true;
}

bool ccCameraSensor::fromImageCoordToLocalCoord(const CCVector2i& imageCoord, CCVector3& localCoord, const bool withLensCorrection, const float depth /*=0.0*/)
{
	CCVector2i coord = imageCoord;

	// applies lens correction if needed
	if (withLensCorrection)
		fromRealImCoordToIdealImCoord(imageCoord, coord);

	// If specified depth is 0.0, it means that we want unprojection to be made in the focal plane
	float focal = m_intrinsicParams.focalLength;
	float newDepth = depth;
	if (abs(depth)<FLT_EPSILON)
		newDepth = focal;

	// We test if the pixel is into the image boundaries (width*height) and if input depth is positive
	if (coord.x<0 || coord.x>=m_intrinsicParams.imageSize[0] || coord.y<0 || coord.y>=m_intrinsicParams.imageSize[1] || newDepth<focal)
		return false;

	// Compute local 3D coordinates
	float x = (coord.x - m_intrinsicParams.imageSize[0]/2) * m_intrinsicParams.pixelSize[0] / focal;
	float y = (coord.y - m_intrinsicParams.imageSize[1]/2) * m_intrinsicParams.pixelSize[1] / focal;
	localCoord = CCVector3(x, y, -1.0) * newDepth;
	return true;
}

bool ccCameraSensor::fromGlobalCoordToImageCoord(const CCVector3& globalCoord, CCVector3& localCoord, CCVector2i& imageCoord, const bool withLensError)
{
	fromGlobalCoordToLocalCoord(globalCoord,localCoord);
	if (!fromLocalCoordToImageCoord(localCoord, imageCoord, withLensError))
		return false;
	return true;
}

bool ccCameraSensor::fromImageCoordToGlobalCoord(const CCVector2i& imageCoord, CCVector3& localCoord, CCVector3& globalCoord, const bool withLensCorrection, const float depth /*=0.0*/)
{	
	if (!fromImageCoordToLocalCoord(imageCoord, localCoord, withLensCorrection, depth))
		return false;
	fromLocalCoordToGlobalCoord(localCoord, globalCoord);
	return true; 
}

bool ccCameraSensor::fromRealImCoordToIdealImCoord(const CCVector2i& real, CCVector2i& ideal)
{
	float sX = m_intrinsicParams.pixelSize[0];
	float sY = m_intrinsicParams.pixelSize[1];

	// 1st correction : principal point correction
	float cx = (float)m_intrinsicParams.imageSize[0] / 2.0 + m_uncertaintyParams.principalPointOffset[0] / sX; // in pixels
	float cy = (float)m_intrinsicParams.imageSize[1] / 2.0 + m_uncertaintyParams.principalPointOffset[1] / sY; // in pixels

	// 2nd correction : Brown's lens distortion correction
	float dx = ((float)real.x-cx) * m_intrinsicParams.pixelSize[0];	// real distance 
	float dy = ((float)real.y-cy) * m_intrinsicParams.pixelSize[1];	// real distance
	float dx2 = pow(dx,2);
	float dy2 = pow(dy,2);
	float r = sqrt( pow(dx,2) + pow(dy,2) );
	float r2 = pow(r,2);
	float r4 = pow(r,4);
	float r6 = pow(r,6);
	float K1 = m_uncertaintyParams.K_BrownParams[0];
	float K2 = m_uncertaintyParams.K_BrownParams[1];
	float K3 = m_uncertaintyParams.K_BrownParams[2];
	float P1 = m_uncertaintyParams.P_BrownParams[0];
	float P2 = m_uncertaintyParams.P_BrownParams[1];

	// compute new value
	float correctedX = (dx * (1 + K1*r2 + K2*r4 + K3*r6)  +  P1 * (r2 + 2*dx2)  +  2*P2*dx*dy);
	float correctedY = (dy * (1 + K1*r2 + K2*r4 + K3*r6)  +  P2 * (r2 + 2*dy2)  +  2*P1*dx*dy);
	ideal.x = (int) (correctedX / sX);
	ideal.y = (int) (correctedY / sY);

	// We test if the new pixel is into the image boundaries
	if (ideal.x<0 || ideal.x>=m_intrinsicParams.imageSize[0] || ideal.y<0 || ideal.y>=m_intrinsicParams.imageSize[1])
		return false;
	return true;
}

bool ccCameraSensor::fromIdealImCoordToRealImCoord(const CCVector2i& ideal, CCVector2i& real)
{
	// !! TO CHANGE !! //
	real.x = ideal.x;
	real.y = ideal.y;
	return true;
}

bool ccCameraSensor::computeUncertainty(const CCVector2i& pixel, const float depth, float& sigmaX, float& sigmaY, float& sigmaZ)
{
	////TO DO// ==> check if the input pixel coordinate must be the real or ideal projection

	//int u = pixel.x;
	//int v = pixel.y;
	//int width = m_intrinsicParams.imageSize[0];
	//int height = m_intrinsicParams.imageSize[1];

	//// check validity 
	//if (u<0 || u>width || v<0 || v>height || depth<0.0 || abs(depth)<FLT_EPSILON)
	//	return false;

	//// init parameters
	//float epsilon = 1.0/8.0;
	//float mu = m_intrinsicParams.pixelSize[0];
	//float sigmaD = epsilon * mu;
	//float A = m_uncertaintyParams.linearDisparityParams[0];
	//float z2 = pow(depth,2);
	//float f = m_intrinsicParams.focalLength;

	//// computes uncertainty
	//sigmaX = abs(A * (u-width/2) / f * z2 * sigmaD);
	//sigmaY = abs(A * (v-height/2) / f * z2 * sigmaD);
	//sigmaZ = abs(A * z2 * sigmaD);
	return true;
}

void ccCameraSensor::computeUncertainty(std::vector<const CCVector3*> points, std::vector<CCVector3>& accuracy, bool lensCorrection)
{
	float sigmaX, sigmaY, sigmaZ;
	accuracy.clear();
	accuracy.resize(points.size());

	for (size_t i=0 ; i<points.size() ; i++)
	{
		CCVector3 coordGlobal = CCVector3(points[i]->x, points[i]->y, points[i]->z), coordLocal;
		CCVector2i coordImage;

		if (fromGlobalCoordToImageCoord(coordGlobal, coordLocal, coordImage, lensCorrection))
		{
			computeUncertainty(coordImage, abs(coordLocal.z), sigmaX, sigmaY, sigmaZ);
			accuracy[i] = CCVector3(sigmaX,sigmaY,sigmaZ);
		}
		else
			accuracy[i] = CCVector3(NAN_VALUE,NAN_VALUE,NAN_VALUE);
	}
}

bool ccCameraSensor::isGlobalCoordInFrustrum(const CCVector3& globalCoord, const bool withLensCorrection)
{
	CCVector3 localCoord;
	CCVector2i imageCoord;

	// Tests if the projection is in the field of view
	if (!fromGlobalCoordToImageCoord(globalCoord, localCoord, imageCoord, withLensCorrection))
		return false;

	// Tests if the projected point is between zNear and zFar
	float z = localCoord.z;
	float n = m_intrinsicParams.zBoundary[0];
	float f = m_intrinsicParams.zBoundary[1];
	if (-z>f || -z<n || abs(f+z)<FLT_EPSILON || abs(n+z)<FLT_EPSILON)
		return false;

	return true;
}

void ccCameraSensor::computeFrustumCorners()
{
	float focal	= m_intrinsicParams.focalLength;
	float aspectRatio = (float)m_intrinsicParams.imageSize[1] / (float)m_intrinsicParams.imageSize[0];
	float xInFocal = abs( tan(m_intrinsicParams.vFieldOfView / aspectRatio / 2.0) * focal );
	float yInFocal = abs( tan(m_intrinsicParams.vFieldOfView / 2.0) * focal );
	float zNear = m_intrinsicParams.zBoundary[0];
	float zFar = m_intrinsicParams.zBoundary[1];

	// compute points of frustum in image coordinate system (attention : in the system, z=-z)
	// DO NOT MODIFY THE ORDER OF THE CORNERS !! A LOT OF CODE DEPENDS OF THIS ORDER !! 
	m_frustrumInfos.frustumCorners[0] = CCVector3(xInFocal/focal,yInFocal/focal,-1.0) * zNear;
	m_frustrumInfos.frustumCorners[1] = CCVector3(xInFocal/focal,yInFocal/focal,-1.0) * zFar;
	m_frustrumInfos.frustumCorners[2] = CCVector3(xInFocal/focal,-yInFocal/focal,-1.0) * zNear;
	m_frustrumInfos.frustumCorners[3] = CCVector3(xInFocal/focal,-yInFocal/focal,-1.0) * zFar;
	m_frustrumInfos.frustumCorners[4] = CCVector3(-xInFocal/focal,-yInFocal/focal,-1.0) * zNear;
	m_frustrumInfos.frustumCorners[5] = CCVector3(-xInFocal/focal,-yInFocal/focal,-1.0) * zFar;
	m_frustrumInfos.frustumCorners[6] = CCVector3(-xInFocal/focal,yInFocal/focal,-1.0) * zNear;
	m_frustrumInfos.frustumCorners[7] = CCVector3(-xInFocal/focal,yInFocal/focal,-1.0) * zFar;

	// compute center of the circumscribed sphere 
	float x0 = m_frustrumInfos.frustumCorners[0].x;
	float y0 = m_frustrumInfos.frustumCorners[0].y;
	float z0 = m_frustrumInfos.frustumCorners[0].z;
	float x5 = m_frustrumInfos.frustumCorners[5].x;
	float y5 = m_frustrumInfos.frustumCorners[5].y;
	float z5 = m_frustrumInfos.frustumCorners[5].z;
	float z;
	if (abs(z0-z5) < FLT_EPSILON)
		z = z0;
	else
		z = (x0*x0 + y0*y0 + z0*z0 - x5*x5 - y5*y5 - z5*z5) / (2*(z0-z5));
	m_frustrumInfos.center = CCVector3(0.0, 0.0, z);

	// frustrum corners are now computed
	m_frustrumInfos.isComputed = true;
}


void ccCameraSensor::computeGlobalPlaneCoefficients(float planeCoefficients[6][4], CCVector3 frustrumCorners[8], CCVector3 edges[6], CCVector3& center)
{
	if (m_frustrumInfos.isComputed == false)
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
	//If you do not like method 1, use this standard method !
	// compute equations for side planes
	CCVector3 v1, v2, n;
	for (int i=0 ; i<4 ; i++)
	{
		v1 = frustrumCorners[i*2+1] - frustrumCorners[i*2];
		v2 = frustrumCorners[((i+1)*2)%8] - frustrumCorners[i*2];
		n = v1.cross(v2); n.normalize();
		planeCoefficients[i][0] = n.x;
		planeCoefficients[i][1] = n.y;
		planeCoefficients[i][2] = n.z;
		planeCoefficients[i][3] = -frustrumCorners[i*2].dot(n);
	}
	// compute equations for near and far planes
	v1 = frustrumCorners[0] - frustrumCorners[6];
	v2 = frustrumCorners[4] - frustrumCorners[6];
	n = v1.cross(v2); n.normalize();
	planeCoefficients[4][0] = n.x;
	planeCoefficients[4][1] = n.y;
	planeCoefficients[4][2] = n.z;
	planeCoefficients[4][3] = -frustrumCorners[6].dot(n);
	planeCoefficients[5][0] = -n.x;
	planeCoefficients[5][1] = -n.y;
	planeCoefficients[5][2] = -n.z;
	planeCoefficients[5][3] = -frustrumCorners[7].dot(-n);
	

	// compute frustrum edges
	edges[0] = frustrumCorners[1] - frustrumCorners[0];
	edges[1] = frustrumCorners[3] - frustrumCorners[2];
	edges[2] = frustrumCorners[5] - frustrumCorners[4];
	edges[3] = frustrumCorners[7] - frustrumCorners[6];
	edges[4] = frustrumCorners[6] - frustrumCorners[0];
	edges[5] = frustrumCorners[2] - frustrumCorners[0];
	for (int i=0 ; i<6 ; i++)
		edges[i].normalize();

	// compute frustrum center in the global coordinates system
	fromLocalCoordToGlobalCoord(m_frustrumInfos.center, center);
}

void ccCameraSensor::filterOctree(ccOctree* octree, std::vector<unsigned int>& inCameraFrustrum)
{
	//// initialization
	//float globalPlaneCoefficients[6][4];
	//CCVector3 globalCorners[8];
	//CCVector3 globalEdges[6];
	//CCVector3 globalCenter; 
	//computeGlobalPlaneCoefficients(globalPlaneCoefficients, globalCorners, globalEdges, globalCenter);

	//// get points of cells in frustrum
	//std::vector< std::pair<unsigned int,CCVector3> > pointsToTest;
	//octree->computeFrustumIntersectionWithOctree(pointsToTest, inCameraFrustrum, globalPlaneCoefficients, globalCorners, globalEdges, globalCenter);
	//
	//// project points
	//for (size_t i=0 ; i<pointsToTest.size() ; i++)
	//{
	//	if (isGlobalCoordInFrustrum(pointsToTest[i].second, false))
	//		inCameraFrustrum.push_back(pointsToTest[i].first);
	//}
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
			float aspectRatio = (float)m_intrinsicParams.imageSize[1] / (float)m_intrinsicParams.imageSize[0];
			CCVector3 upperLeftPoint;
			upperLeftPoint.z = m_scale * m_intrinsicParams.focalLength;
			upperLeftPoint.y = upperLeftPoint.z * tan(m_intrinsicParams.vFieldOfView / 2.0);
			upperLeftPoint.x = upperLeftPoint.z * tan(m_intrinsicParams.vFieldOfView / aspectRatio / 2.0);

			//up arrow
			const float arrowHeight    = 1.5f * upperLeftPoint.y;
			const float baseHeight     = 1.2f * upperLeftPoint.y;
			const float arrowHalfWidth = 0.5f * upperLeftPoint.x;
			const float baseHalfWidth  = 0.3f * upperLeftPoint.x;

			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glColor3f(m_color.x, m_color.y, m_color.z);
			
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
				glVertex3f(pts[0].x, pts[0].y, pts[0].z);
				glVertex3f(pts[1].x, pts[1].y, pts[1].z);
				glVertex3f(pts[3].x, pts[3].y, pts[3].z);
				glVertex3f(pts[2].x, pts[2].y, pts[2].z);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3f(pts[2].x, pts[2].y, pts[2].z);
				glVertex3f(pts[3].x, pts[3].y, pts[3].z);
				glVertex3f(pts[5].x, pts[5].y, pts[5].z);
				glVertex3f(pts[4].x, pts[4].y, pts[4].z);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3f(pts[4].x, pts[4].y, pts[4].z);
				glVertex3f(pts[5].x, pts[5].y, pts[5].z);
				glVertex3f(pts[7].x, pts[7].y, pts[7].z);
				glVertex3f(pts[6].x, pts[6].y, pts[6].z);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3f(pts[6].x, pts[6].y, pts[6].z);
				glVertex3f(pts[7].x, pts[7].y, pts[7].z);
				glVertex3f(pts[1].x, pts[1].y, pts[1].z);
				glVertex3f(pts[0].x, pts[0].y, pts[0].z);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3f(pts[6].x, pts[6].y, pts[6].z);
				glVertex3f(pts[0].x, pts[0].y, pts[0].z);
				glVertex3f(pts[2].x, pts[2].y, pts[2].z);
				glVertex3f(pts[4].x, pts[4].y, pts[4].z);
				glEnd();
				glBegin(GL_LINE_LOOP);
				glVertex3f(pts[1].x, pts[1].y, pts[1].z);
				glVertex3f(pts[7].x, pts[7].y, pts[7].z);
				glVertex3f(pts[5].x, pts[5].y, pts[5].z);
				glVertex3f(pts[3].x, pts[3].y, pts[3].z);
				glEnd();
				glLineWidth(1.0);

				//frustum area (planes)
				if (m_frustrumInfos.drawSidePlanes)
				{
					glEnable(GL_BLEND);
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
					glColor4f(m_color.x, m_color.y, m_color.z, 0.3);

					glBegin(GL_QUADS);
					glVertex3f(pts[0].x, pts[0].y, pts[0].z);
					glVertex3f(pts[2].x, pts[2].y, pts[2].z);
					glVertex3f(pts[3].x, pts[3].y, pts[3].z);
					glVertex3f(pts[1].x, pts[1].y, pts[1].z);
					glVertex3f(pts[2].x, pts[2].y, pts[2].z);
					glVertex3f(pts[4].x, pts[4].y, pts[4].z);
					glVertex3f(pts[5].x, pts[5].y, pts[5].z);
					glVertex3f(pts[3].x, pts[3].y, pts[3].z);
					glVertex3f(pts[4].x, pts[4].y, pts[4].z);
					glVertex3f(pts[6].x, pts[6].y, pts[6].z);
					glVertex3f(pts[7].x, pts[7].y, pts[7].z);
					glVertex3f(pts[5].x, pts[5].y, pts[5].z);
					glVertex3f(pts[6].x, pts[6].y, pts[6].z);
					glVertex3f(pts[0].x, pts[0].y, pts[0].z);
					glVertex3f(pts[1].x, pts[1].y, pts[1].z);
					glVertex3f(pts[7].x, pts[7].y, pts[7].z);
					glVertex3f(pts[6].x, pts[6].y, pts[6].z);
					glVertex3f(pts[4].x, pts[4].y, pts[4].z);
					glVertex3f(pts[2].x, pts[2].y, pts[2].z);
					glVertex3f(pts[0].x, pts[0].y, pts[0].z);
					glVertex3f(pts[1].x, pts[1].y, pts[1].z);
					glVertex3f(pts[3].x, pts[3].y, pts[3].z);
					glVertex3f(pts[5].x, pts[5].y, pts[5].z);
					glVertex3f(pts[7].x, pts[7].y, pts[7].z);
					glEnd();
					glDisable(GL_BLEND); 
				}
			}

			//axis (temporary)
			if (true)
			{
				glLineWidth(1.0);

				// right vector
				glColor3f(1.0,0.0,0.0);
				glBegin(GL_LINES);
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(1.0f, 0.0f, 0.0f);
				glEnd();

				// up vector
				glColor3f(0.0,1.0,0.0);
				glBegin(GL_LINES);
				glVertex3f(0.0f, 0.0f, 0.0f);
				glVertex3f(0.0f, 1.0f, 0.0f);
				glEnd();

				// view vector
				glColor3f(0.0,0.0,1.0);
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
