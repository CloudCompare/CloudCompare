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

//Always first
#include "ccIncludeGL.h"

#include "ccGBLSensor.h"

//Local
#include "ccPointCloud.h"
#include "ccProgressDialog.h"
#include "ccSphere.h"

//Qt
#include <QCoreApplication>

//maximum depth buffer dimension (width or height)
static const int s_MaxDepthBufferSize = (1 << 14); //16384

enum Errors {	ERROR_BAD_INPUT      = -1,
				ERROR_MEMORY         = -2,
				ERROR_PROC_CANCELLED = -3,
				ERROR_DB_TOO_SMALL   = -4,
};
								
QString ccGBLSensor::GetErrorString(int errorCode)
{
	switch (errorCode)
	{
	case ERROR_BAD_INPUT:
		return "Internal error: bad input";
	case ERROR_MEMORY:
		return "Error: not enough memory";
	case ERROR_PROC_CANCELLED:
		return "Error: process cancelled by user";
	case ERROR_DB_TOO_SMALL:
		return "Error: depth buffer is void (check input cloud and angular steps)";
	default:
		assert(false);
		break;
	}

	return QString("unknown error (code: %i)").arg(errorCode);
}

ccGBLSensor::ccGBLSensor(ROTATION_ORDER rotOrder/*=YAW_THEN_PITCH*/)
	: ccSensor("TLS/GBL")
	, m_phiMin(0)
	, m_phiMax(0)
	, m_deltaPhi(0)
	, m_pitchAnglesAreShifted(false)
	, m_thetaMin(0)
	, m_thetaMax(0)
	, m_deltaTheta(0)
	, m_yawAnglesAreShifted(false)
	, m_rotationOrder(rotOrder)
	, m_sensorRange(0)
	, m_uncertainty(static_cast<PointCoordinateType>(0.005))
{
	//graphic representation
	lockVisibility(false);
	setSelectionBehavior(SELECTION_FIT_BBOX);
}

ccGBLSensor::ccGBLSensor(const ccGBLSensor &sensor)
	: ccSensor(sensor)
	, m_phiMin(sensor.m_phiMin)
	, m_phiMax(sensor.m_phiMax)
	, m_deltaPhi(sensor.m_deltaPhi)
	, m_pitchAnglesAreShifted(sensor.m_pitchAnglesAreShifted)
	, m_thetaMin(sensor.m_thetaMin)
	, m_thetaMax(sensor.m_thetaMax)
	, m_deltaTheta(sensor.m_deltaTheta)
	, m_yawAnglesAreShifted(sensor.m_yawAnglesAreShifted)
	, m_rotationOrder(sensor.m_rotationOrder)
	, m_sensorRange(sensor.m_sensorRange)
	, m_uncertainty(sensor.m_uncertainty)
{
}

void ccGBLSensor::clearDepthBuffer()
{
	m_depthBuffer.clear();
}

void ccGBLSensor::setPitchRange(PointCoordinateType minPhi, PointCoordinateType maxPhi)
{
	m_phiMin = minPhi;
	m_phiMax = maxPhi;

	if (m_phiMax > static_cast<PointCoordinateType>(M_PI))
		m_pitchAnglesAreShifted = true;

	clearDepthBuffer();
}

void ccGBLSensor::setPitchStep(PointCoordinateType dPhi)
{
	if (m_deltaPhi != dPhi)
	{
		clearDepthBuffer();
		m_deltaPhi = dPhi;
	}
}

void ccGBLSensor::setYawRange(PointCoordinateType minTehta, PointCoordinateType maxTheta)
{
	m_thetaMin = minTehta;
	m_thetaMax = maxTheta;

	if (m_thetaMax > static_cast<PointCoordinateType>(M_PI))
		m_yawAnglesAreShifted = true;

	clearDepthBuffer();
}

void ccGBLSensor::setYawStep(PointCoordinateType dTheta)
{
	if (m_deltaTheta != dTheta)
	{
		clearDepthBuffer();
		m_deltaTheta = dTheta;
	}
}

void ccGBLSensor::projectPoint(	const CCVector3& sourcePoint,
								CCVector2& destPoint,
								PointCoordinateType &depth,
								double posIndex/*=0*/) const
{
	//project point in sensor world
	CCVector3 P = sourcePoint;

	//sensor to world global transformation = sensor position * rigid transformation
	ccIndexedTransformation sensorPos; //identity by default
	if (m_posBuffer)
		m_posBuffer->getInterpolatedTransformation(posIndex,sensorPos);
	sensorPos *= m_rigidTransformation;

	//apply (inverse) global transformation (i.e world to sensor)
	sensorPos.inverse().apply(P);

	//convert to 2D sensor field of view + compute its distance
	switch (m_rotationOrder)
	{
	case YAW_THEN_PITCH:
	{
		//yaw = angle around Z, starting from 0 in the '+X' direction
		destPoint.x = atan2(P.y,P.x);
		//pitch = angle around the lateral axis, between -pi (-Z) to pi (+Z) by default
		destPoint.y = atan2(P.z,sqrt(P.x*P.x + P.y*P.y));
		break;
	}
	case PITCH_THEN_YAW:
	{
		//FIXME
		//yaw = angle around Z, starting from 0 in the '+X' direction
		destPoint.x = -atan2(sqrt(P.y*P.y + P.z*P.z),P.x);
		//pitch = angle around the lateral axis, between -pi (-Z) to pi (+Z) by default
		destPoint.y = -atan2(P.y,P.z);
		break;
	}
	default:
		assert(false);
	}
	
	//if the yaw angles are shifted
	if (m_yawAnglesAreShifted && destPoint.x < 0)
		destPoint.x += static_cast<PointCoordinateType>(2.0*M_PI);
	//if the pitch angles are shifted
	if (m_pitchAnglesAreShifted && destPoint.y < 0)
		destPoint.y += static_cast<PointCoordinateType>(2.0*M_PI);

	depth = P.norm();
}

bool ccGBLSensor::convertToDepthMapCoords(PointCoordinateType yaw, PointCoordinateType pitch, unsigned& i, unsigned& j) const
{
	if (m_depthBuffer.zBuff.empty())
	{
		return false;
	}

	assert(m_depthBuffer.deltaTheta != 0 && m_depthBuffer.deltaPhi != 0);

	//yaw
	if (yaw < m_thetaMin || yaw > m_thetaMax + m_depthBuffer.deltaTheta)
		return false;

	i = static_cast<unsigned>(floor((yaw - m_thetaMin) / m_depthBuffer.deltaTheta));
	if (i == m_depthBuffer.width)
		--i;
	//yaw angles are in the wrong way! (because they are expressed relatively to the sensor)
	assert(i < m_depthBuffer.width);
	i = (m_depthBuffer.width - 1) - i;

	//pitch
	if (pitch < m_phiMin || pitch > m_phiMax + m_depthBuffer.deltaPhi)
		return false;
	j = static_cast<unsigned>(floor((pitch - m_phiMin) / m_depthBuffer.deltaPhi));
	if (j == m_depthBuffer.height)
		--j;
	assert(j < m_depthBuffer.height);

	return true;
}

ccGBLSensor::NormalGrid* ccGBLSensor::projectNormals(	CCLib::GenericCloud* cloud,
														const NormalGrid& theNorms,
														double posIndex/*=0*/) const
{
	if (!cloud || theNorms.capacity() == 0)
		return nullptr;

	unsigned size = m_depthBuffer.height*m_depthBuffer.width;
	if (size == 0)
		return nullptr; //depth buffer empty/not initialized!

	NormalGrid* normalGrid = new NormalGrid;
	try
	{
		static const CCVector3 zeroNormal(0, 0, 0);
		normalGrid->resize(size, zeroNormal);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		delete normalGrid;
		return nullptr;
	}

	//sensor to world global transformation = sensor position * rigid transformation
	ccIndexedTransformation sensorPos; //identity by default
	if (m_posBuffer)
		m_posBuffer->getInterpolatedTransformation(posIndex,sensorPos);
	sensorPos *= m_rigidTransformation;

	//poject each point + normal
	{
		cloud->placeIteratorAtBeginning();
		unsigned pointCount = cloud->size();
		for (unsigned i=0; i<pointCount; ++i)
		{
			const CCVector3* P = cloud->getNextPoint();
			const CCVector3& N = theNorms[i];

			//project point
			CCVector2 Q;
			PointCoordinateType depth1;
			projectPoint(*P, Q, depth1, m_activeIndex);

			CCVector3 S;

			CCVector3 U = *P - sensorPos.getTranslationAsVec3D();
			PointCoordinateType distToSensor = U.norm();

			if (distToSensor > ZERO_TOLERANCE)
			{
				//normal component along sensor viewing dir.
				S.z = -N.dot(U) / distToSensor;

				if (S.z > 1.0 - ZERO_TOLERANCE)
				{
					S.x = 0;
					S.y = 0;
				}
				else
				{
					//and point+normal
					CCVector3 P2 = *P + CCVector3(N);
					CCVector2 S2;
					PointCoordinateType depth2;
					projectPoint(P2, S2, depth2, m_activeIndex);

					//deduce other normals components
					PointCoordinateType coef = sqrt((1 - S.z*S.z) / (S.x*S.x + S.y*S.y));
					S.x = coef * (S2.x - Q.x);
					S.y = coef * (S2.y - Q.y);
				}
			}
			else
			{
				S = CCVector3(N);
			}

			//project in Z-buffer
			unsigned x = 0;
			unsigned y = 0;
			if (convertToDepthMapCoords(Q.x, Q.y, x, y))
			{
				//add the transformed normal
				CCVector3& newN = normalGrid->at(y*m_depthBuffer.width + x);
				newN += S;
			}
			else
			{
				//shouldn't happen!
				assert(false);
			}
		}
	}

	//normalize
	{
		for (unsigned i = 0; i < m_depthBuffer.height*m_depthBuffer.width; ++i)
		{
			normalGrid->at(i).normalize();
		}
	}

	return normalGrid;
}

ccGBLSensor::ColorGrid* ccGBLSensor::projectColors(	CCLib::GenericCloud* cloud,
													const ColorGrid& theColors) const
{
	if (!cloud || theColors.capacity() == 0)
		return nullptr;

	unsigned gridSize = m_depthBuffer.height*m_depthBuffer.width;
	if (gridSize == 0)
		return nullptr; //depth buffer empty or not initialized!

	//number of points per cell of the depth map
	std::vector<size_t> pointPerDMCell;
	try
	{
		pointPerDMCell.resize(gridSize, 0);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return nullptr;
	}

	//temp. array for accumulation
	std::vector<ccColor::Rgbf> colorAccumGrid;
	{
		ccColor::Rgbf blackF(0, 0, 0);
		try
		{
			colorAccumGrid.resize(gridSize, blackF);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return nullptr;
		}
	}
	
	//final array
	ColorGrid* colorGrid = new ColorGrid;
	try
	{
		colorGrid->resize(gridSize, ccColor::blackRGB);
	}
	catch (const std::bad_alloc&)
	{
		delete colorGrid;
		return nullptr; //not enough memory
	}

	//project colors
	{
		unsigned pointCount = cloud->size();
		cloud->placeIteratorAtBeginning();
		{
			for (unsigned i = 0; i < pointCount; ++i)
			{
				const CCVector3 *P = cloud->getNextPoint();
				CCVector2 Q;
				PointCoordinateType depth;
				projectPoint(*P, Q, depth, m_activeIndex);

				unsigned x = 0;
				unsigned y = 0;
				if (convertToDepthMapCoords(Q.x, Q.y, x, y))
				{
					unsigned index = y*m_depthBuffer.width + x;

					//accumulate color
					const ccColor::Rgb& srcC = theColors[i];
					ccColor::Rgbf& destC = colorAccumGrid[index];

					destC.r += srcC.r;
					destC.g += srcC.g;
					destC.b += srcC.b;
					++pointPerDMCell[index];
				}
				else
				{
					//shouldn't happen!
					assert(false);
				}
			}
		}
	}

	//normalize
	{
		for (unsigned i = 0; i < gridSize; ++i)
		{
			if (pointPerDMCell[i] != 0)
			{
				const ccColor::Rgbf& srcC = colorAccumGrid[i];
				ccColor::Rgb& destC = colorGrid->at(i);
				destC.r = static_cast<ColorCompType>(srcC.r / pointPerDMCell[i]);
				destC.g = static_cast<ColorCompType>(srcC.g / pointPerDMCell[i]);
				destC.b = static_cast<ColorCompType>(srcC.b / pointPerDMCell[i] );
			}
		}
	}

	return colorGrid;
}

//! Interval structure used for determining the largest empty angular interval in ccGBLSensor::project
struct Interval
{
	//! Default constructor
	Interval() : start(-1), span(0) {}
	//! Interval start index
	int start;
	//! Interval span
	int span;

	//! Finds the biggest contiguous interval
	template<class T> static Interval FindBiggest(const std::vector<T>& values, T intValue, bool allowLoop = true)
	{
		//look for the largest 'empty' part
		Interval firstEmptyPart;
		Interval bestEmptyPart;
		Interval currentEmptyPart;

		for (size_t i = 0; i < values.size(); ++i)
		{
			//no point for the current angle?
			if (values[i] == intValue)
			{
				//new empty part?
				if (currentEmptyPart.span == 0)
				{
					currentEmptyPart.start = static_cast<int>(i);
				}
				currentEmptyPart.span++;
			}
			else
			{
				//current empty part stops (if any)
				if (currentEmptyPart.span != 0)
				{
					//specific case: remember the very first interval (start == 0)
					if (currentEmptyPart.start == 0)
					{
						firstEmptyPart = currentEmptyPart;
					}
					if (bestEmptyPart.span < currentEmptyPart.span)
					{
						bestEmptyPart = currentEmptyPart;
					}
					currentEmptyPart = Interval();
				}
			}
		}

		//specific case: merge the very first and the very last parts
		if (allowLoop && firstEmptyPart.span != 0 && currentEmptyPart.span != 0)
		{
			currentEmptyPart.span += firstEmptyPart.span;
		}

		//last interval
		if (bestEmptyPart.span < currentEmptyPart.span)
		{
			bestEmptyPart = currentEmptyPart;
		}

		return bestEmptyPart;
	}
};

bool ccGBLSensor::computeAutoParameters(CCLib::GenericCloud* theCloud)
{
	assert(theCloud);
	if (!theCloud)
	{
		//invalid input parameter
		return false;
	}

	std::vector<bool> nonEmptyAnglesYaw;
	std::vector<bool> nonEmptyAnglesPitch;
	try
	{
		nonEmptyAnglesYaw.resize(360, false);
		nonEmptyAnglesPitch.resize(360, false);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	//force no shift for auto search (we'll fix this later if necessary)
	m_yawAnglesAreShifted = false;
	m_pitchAnglesAreShifted = false;

	unsigned pointCount = theCloud->size();

	PointCoordinateType minPitch = 0;
	PointCoordinateType maxPitch = 0;
	PointCoordinateType minYaw = 0;
	PointCoordinateType maxYaw = 0;
	PointCoordinateType maxDepth = 0;
	{
		//first project all points to compute the (yaw,ptich) ranges
		theCloud->placeIteratorAtBeginning();
		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3* P = theCloud->getNextPoint();
			CCVector2 Q;
			PointCoordinateType depth;
			//Q.x and Q.y are inside [-pi;pi] by default (result of atan2)
			projectPoint(*P, Q, depth, m_activeIndex);

			//yaw
			int angleYaw = static_cast<int>(Q.x * CC_RAD_TO_DEG);
			assert(angleYaw >= -180 && angleYaw <= 180);
			if (angleYaw == 180) //360 degrees warp
				angleYaw = -180;
			nonEmptyAnglesYaw[180 + angleYaw] = true;
			if (i != 0)
			{
				if (minYaw > Q.x)
					minYaw = Q.x;
				else if (maxYaw < Q.x)
					maxYaw = Q.x;
			}
			else
			{
				minYaw = maxYaw = Q.x;
			}

			//pitch
			int anglePitch = static_cast<int>(Q.y * CC_RAD_TO_DEG);
			assert(anglePitch >= -180 && anglePitch <= 180);
			if (anglePitch == 180)
				anglePitch = -180;
			nonEmptyAnglesPitch[180 + anglePitch] = true;
			if (i != 0)
			{
				if (minPitch > Q.y)
					minPitch = Q.y;
				else if (maxPitch < Q.y)
					maxPitch = Q.y;
			}
			else
			{
				minPitch = maxPitch = Q.y;
			}

			if (depth > maxDepth)
				maxDepth = depth;
		}
	}

	Interval bestEmptyPartYaw = Interval::FindBiggest<bool>(nonEmptyAnglesYaw, false, true);
	Interval bestEmptyPartPitch = Interval::FindBiggest<bool>(nonEmptyAnglesPitch, false, true);

	m_yawAnglesAreShifted = (bestEmptyPartYaw.start != 0 && bestEmptyPartYaw.span > 1 && bestEmptyPartYaw.start + bestEmptyPartYaw.span < 360);
	m_pitchAnglesAreShifted = (bestEmptyPartPitch.start != 0 && bestEmptyPartPitch.span > 1 && bestEmptyPartPitch.start + bestEmptyPartPitch.span < 360);

	if (m_yawAnglesAreShifted || m_pitchAnglesAreShifted)
	{
		//we re-project all the points in order to update the boundaries!
		theCloud->placeIteratorAtBeginning();
		for (unsigned i = 0; i < pointCount; ++i)
		{
			const CCVector3 *P = theCloud->getNextPoint();
			CCVector2 Q;
			PointCoordinateType depth;
			projectPoint(*P, Q, depth, m_activeIndex);

			if (i != 0)
			{
				if (minYaw > Q.x)
					minYaw = Q.x;
				else if (maxYaw < Q.x)
					maxYaw = Q.x;

				if (minPitch > Q.y)
					minPitch = Q.y;
				else if (maxPitch < Q.y)
					maxPitch = Q.y;
			}
			else
			{
				minYaw = maxYaw = Q.x;
				minPitch = maxPitch = Q.y;
			}
		}
	}

	setYawRange(minYaw, maxYaw);
	setPitchRange(minPitch, maxPitch);
	setSensorRange(maxDepth);

	return true;
}

bool ccGBLSensor::computeDepthBuffer(CCLib::GenericCloud* theCloud, int& errorCode, ccPointCloud* projectedCloud/*=0*/)
{
	assert(theCloud);
	if (!theCloud)
	{
		//invlalid input parameter
		errorCode = ERROR_BAD_INPUT;
		return false;
	}

	//clear previous Z-buffer (if any)
	clearDepthBuffer();

	//init new Z-buffer
	{
		PointCoordinateType deltaTheta = m_deltaTheta;
		PointCoordinateType deltaPhi = m_deltaPhi;

		//yaw as X
		int width = static_cast<int>(ceil((m_thetaMax - m_thetaMin) / m_deltaTheta));
		if (width > s_MaxDepthBufferSize)
		{
			deltaTheta = (m_thetaMax - m_thetaMin) / static_cast<PointCoordinateType>(s_MaxDepthBufferSize);
			width = s_MaxDepthBufferSize;
		}
		//pitch as Y
		int height = static_cast<int>(ceil((m_phiMax - m_phiMin) / m_deltaPhi));
		if (height > s_MaxDepthBufferSize)
		{
			deltaPhi = (m_phiMax - m_phiMin) / static_cast<PointCoordinateType>(s_MaxDepthBufferSize);
			height = s_MaxDepthBufferSize;
		}

		if (width <= 0 || height <= 0)
		{
			//depth buffer dimensions are too small?!
			errorCode = ERROR_DB_TOO_SMALL;
			return false;
		}

		unsigned zBuffSize = width*height;
		try
		{
			assert(m_depthBuffer.zBuff.empty());
			m_depthBuffer.zBuff.resize(zBuffSize, 0);
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			errorCode = ERROR_MEMORY;
			return false;
		}

		m_depthBuffer.width = static_cast<unsigned>(width);
		m_depthBuffer.height = static_cast<unsigned>(height);
		m_depthBuffer.deltaTheta = deltaTheta;
		m_depthBuffer.deltaPhi = deltaPhi;
	}

	unsigned pointCount = theCloud->size();

	//project points and accumulate them in Z-buffer
	{
		if (projectedCloud)
		{
			projectedCloud->clear();
			if (!projectedCloud->reserve(pointCount) || !projectedCloud->enableScalarField())
			{
				//not enough memory
				errorCode = ERROR_MEMORY;
				clearDepthBuffer();
				return false;
			}
		}

		theCloud->placeIteratorAtBeginning();
		{
			//progress bar
			ccProgressDialog pdlg(true);
			CCLib::NormalizedProgress nprogress(&pdlg, pointCount);
			pdlg.setMethodTitle(QObject::tr("Depth buffer"));
			pdlg.setInfo(QObject::tr("Points: %L1").arg(pointCount));
			pdlg.start();
			QCoreApplication::processEvents();

			for (unsigned i = 0; i < pointCount; ++i)
			{
				const CCVector3 *P = theCloud->getNextPoint();
				CCVector2 Q;
				PointCoordinateType depth;
				projectPoint(*P, Q, depth, m_activeIndex);

				unsigned x = 0;
				unsigned y = 0;
				if (convertToDepthMapCoords(Q.x, Q.y, x, y))
				{
					PointCoordinateType& zBuf = m_depthBuffer.zBuff[y*m_depthBuffer.width + x];
					zBuf = std::max(zBuf, depth);
					m_sensorRange = std::max(m_sensorRange, depth);
				}

				if (projectedCloud)
				{
					projectedCloud->addPoint(CCVector3(Q.x, Q.y, 0));
					projectedCloud->setPointScalarValue(i, depth);
				}

				if (!nprogress.oneStep())
				{
					//cancelled by user
					errorCode = ERROR_PROC_CANCELLED;
					clearDepthBuffer();
					return false;
				}
			}
		}
	}

	m_depthBuffer.fillHoles();

	errorCode = 0;
	return true;
}

unsigned char ccGBLSensor::checkVisibility(const CCVector3& P) const
{
	if (m_depthBuffer.zBuff.empty()) //no z-buffer?
	{
		return POINT_VISIBLE;
	}

	//project point
	CCVector2 Q;
	PointCoordinateType depth;
	projectPoint(P, Q, depth, m_activeIndex);

	//out of sight
	if (depth > m_sensorRange)
	{
		return POINT_OUT_OF_RANGE;
	}

	unsigned x = 0;
	unsigned y = 0;
	if (!convertToDepthMapCoords(Q.x, Q.y, x, y))
	{
		//out of field of view
		return POINT_OUT_OF_FOV;
	}

	//hidden?
	if (depth > m_depthBuffer.zBuff[y*m_depthBuffer.width + x] * (1.0f + m_uncertainty))
	{
		return POINT_HIDDEN;
	}

	return POINT_VISIBLE;
}

void ccGBLSensor::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	if (!MACRO_Draw3D(context))
		return;
	
	//we draw a little 3d representation of the sensor
	
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

	//DGM FIXME: this display routine is crap!

	//apply rigid transformation
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

	//test: center as sphere
	//{
	//	ccSphere sphere(m_scale / 10, 0, "Center", 12);
	//	sphere.showColors(true);
	//	sphere.setVisible(true);
	//	sphere.setEnabled(true);

	//	CC_DRAW_CONTEXT sphereContext = context;
	//	sphereContext.drawingFlags &= (~CC_DRAW_ENTITY_NAMES); //we must remove the 'push name flag' so that the sphere doesn't push its own!
	//	sphereContext.display = 0;

	//	sphere.setTempColor(ccColor::magenta);
	//	sphere.draw(sphereContext);
	//}

	const PointCoordinateType halfHeadSize = static_cast<PointCoordinateType>(0.3);

	//force line width
	glFunc->glPushAttrib(GL_LINE_BIT);
	glFunc->glLineWidth(2.0f);

	//sensor axes
	{
		PointCoordinateType axisLength = halfHeadSize * m_scale;
		ccGL::Color4v(glFunc, ccColor::red.rgba);
		CCVector3 C(0, 0, 0);
		glFunc->glBegin(GL_LINES);
		ccGL::Vertex3v(glFunc, C.u);
		ccGL::Vertex3(glFunc, C.x + axisLength, C.y, C.z);
		glFunc->glEnd();
		ccGL::Color4v(glFunc, ccColor::green.rgba);
		glFunc->glBegin(GL_LINES);
		ccGL::Vertex3v(glFunc, C.u);
		ccGL::Vertex3(glFunc, C.x, C.y + axisLength, C.z);
		glFunc->glEnd();
		ccGL::Color4v(glFunc, ccColor::blue.rgba);
		glFunc->glBegin(GL_LINES);
		ccGL::Vertex3v(glFunc, C.u);
		ccGL::Vertex3(glFunc, C.x, C.y, C.z + axisLength);
		glFunc->glEnd();
	}

	//sensor head
	{
		CCVector3 minCorner(-halfHeadSize,-halfHeadSize,-halfHeadSize);
		CCVector3 maxCorner( halfHeadSize, halfHeadSize, halfHeadSize);
		minCorner *= m_scale;
		maxCorner *= m_scale;
		ccBBox bbHead(minCorner,maxCorner);
		bbHead.draw(context, m_color);
	}

	//sensor legs
	{
		CCVector3 headConnect = /*headCenter*/ -CCVector3(0, 0, static_cast<PointCoordinateType>(halfHeadSize)*m_scale);
		ccGL::Color3v(glFunc, m_color.rgb);
		glFunc->glBegin(GL_LINES);
		ccGL::Vertex3v(glFunc, headConnect.u);
		ccGL::Vertex3(glFunc, -m_scale, -m_scale, -m_scale);
		ccGL::Vertex3v(glFunc, headConnect.u);
		ccGL::Vertex3(glFunc, -m_scale, m_scale, -m_scale);
		ccGL::Vertex3v(glFunc, headConnect.u);
		ccGL::Vertex3(glFunc, m_scale, 0, -m_scale);
		glFunc->glEnd();
	}

	glFunc->glPopAttrib(); //GL_LINE_BIT

	if (pushName)
		glFunc->glPopName();

	glFunc->glPopMatrix();
}

ccBBox ccGBLSensor::getOwnBB(bool withGLFeatures/*=false*/)
{
	if (!withGLFeatures)
	{
		return ccBBox();
	}

	//get sensor position
	ccIndexedTransformation sensorPos;
	if (!getAbsoluteTransformation(sensorPos,m_activeIndex))
	{
		return ccBBox();
	}

	ccPointCloud cloud;
	if (!cloud.reserve(8))
	{
		//not enough memory?!
		return ccBBox();
	}

	cloud.addPoint(CCVector3(-m_scale,-m_scale,-m_scale));
	cloud.addPoint(CCVector3(-m_scale,-m_scale, m_scale));
	cloud.addPoint(CCVector3(-m_scale, m_scale,-m_scale));
	cloud.addPoint(CCVector3(-m_scale, m_scale, m_scale));
	cloud.addPoint(CCVector3( m_scale,-m_scale,-m_scale));
	cloud.addPoint(CCVector3( m_scale,-m_scale, m_scale));
	cloud.addPoint(CCVector3( m_scale, m_scale,-m_scale));
	cloud.addPoint(CCVector3( m_scale, m_scale, m_scale));

	cloud.applyRigidTransformation(sensorPos);
	return cloud.getOwnBB(false);
}

ccBBox ccGBLSensor::getOwnFitBB(ccGLMatrix& trans)
{
	//get sensor position
	ccIndexedTransformation sensorPos;
	if (!getAbsoluteTransformation(sensorPos, m_activeIndex))
		return ccBBox();

	trans = sensorPos;

	return ccBBox(	CCVector3(-m_scale,-m_scale,-m_scale),
					CCVector3( m_scale, m_scale, m_scale) );
}

bool ccGBLSensor::applyViewport(ccGenericGLDisplay* win/*=0*/)
{
	if (!win)
	{
		win = getDisplay();
		if (!win)
		{
			ccLog::Warning("[ccGBLSensor::applyViewport] No associated display!");
			return false;
		}
	}

	ccIndexedTransformation trans;
	if (!getActiveAbsoluteTransformation(trans))
	{
		return false;
	}
	//scanner main directions
	CCVector3d sensorX(trans.data()[0], trans.data()[1], trans.data()[2]);
	CCVector3d sensorY(trans.data()[4], trans.data()[5], trans.data()[6]);
	CCVector3d sensorZ(trans.data()[8], trans.data()[9], trans.data()[10]);

	switch (getRotationOrder())
	{
	case ccGBLSensor::YAW_THEN_PITCH:
	{
		double theta = (getMinYaw() + getMaxYaw()) / 2;
		ccGLMatrixd rotz; rotz.initFromParameters(theta, sensorZ, CCVector3d(0, 0, 0));
		rotz.applyRotation(sensorX);
		rotz.applyRotation(sensorY);

		double phi = 0; //(getMinPitch() + getMaxPitch())/2;
		ccGLMatrixd roty; roty.initFromParameters(-phi, sensorY, CCVector3d(0, 0, 0)); //theta = 0 corresponds to the upward vertical direction!
		roty.applyRotation(sensorX);
		roty.applyRotation(sensorZ);

		break;
	}
	case ccGBLSensor::PITCH_THEN_YAW:
	{
		double phi = (getMinPitch() + getMaxPitch()) / 2;
		ccGLMatrixd roty; roty.initFromParameters(-phi, sensorY, CCVector3d(0, 0, 0)); //theta = 0 corresponds to the upward vertical direction!
		roty.applyRotation(sensorX);
		roty.applyRotation(sensorZ);

		double theta = (getMinYaw() + getMaxYaw()) / 2;
		ccGLMatrixd rotz; rotz.initFromParameters(theta, sensorZ, CCVector3d(0, 0, 0));
		rotz.applyRotation(sensorX);
		rotz.applyRotation(sensorY);
		break;
	}
	default:
		assert(false);
		break;
	}

	//center camera on sensor
	CCVector3d sensorCenterd = CCVector3d::fromArray(trans.getTranslation());
	ccGLMatrixd viewMat = ccGLMatrixd::FromViewDirAndUpDir(sensorX, sensorZ);
	viewMat.invert();
	viewMat.setTranslation(sensorCenterd);
	//TODO: can we set the right FOV?
	win->setupProjectiveViewport(viewMat, 0, 1.0f, true, true);

	return true;
}

bool ccGBLSensor::toFile_MeOnly(QFile& out) const
{
	if (!ccSensor::toFile_MeOnly(out))
		return false;

	//rotation order (dataVersion>=34)
	uint32_t rotOrder = m_rotationOrder;
	if (out.write((const char*)&rotOrder, 4) < 0)
		return WriteError();

	//other parameters (dataVersion>=34)
	QDataStream outStream(&out);
	outStream << m_phiMin;
	outStream << m_phiMax;
	outStream << m_deltaPhi;
	outStream << m_thetaMin;
	outStream << m_thetaMax;
	outStream << m_deltaTheta;
	outStream << m_sensorRange;
	outStream << m_uncertainty;
	outStream << m_scale;

	//other parameters (dataVersion>=38)
	outStream << m_pitchAnglesAreShifted;
	outStream << m_yawAnglesAreShifted;

	return true;
}

bool ccGBLSensor::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccSensor::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//rotation order (dataVersion>=34)
	uint32_t rotOrder = 0;
	if (in.read((char*)&rotOrder, 4) < 0)
		return ReadError();
	m_rotationOrder = static_cast<ROTATION_ORDER>(rotOrder);

	//parameters (dataVersion>=34)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_phiMin, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_phiMax, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_deltaPhi, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_thetaMin, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_thetaMax, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_deltaTheta, 1);
	if (dataVersion < 38)
	{
		ScalarType sensorRange{};
		ScalarType uncertainty{};
		ccSerializationHelper::ScalarsFromDataStream(inStream, flags, &sensorRange, 1);
		ccSerializationHelper::ScalarsFromDataStream(inStream, flags, &uncertainty, 1);
		m_sensorRange = static_cast<PointCoordinateType>(sensorRange);
		m_uncertainty = static_cast<PointCoordinateType>(uncertainty);
	}
	else
	{
		ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_sensorRange, 1);
		ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_uncertainty, 1);
	}
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_scale, 1);

	//other parameters (dataVersion>=38)
	if (dataVersion >= 38)
	{
		inStream >> m_pitchAnglesAreShifted;
		inStream >> m_yawAnglesAreShifted;
	}

	return true;
}
