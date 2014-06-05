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

//Always first
#include "ccIncludeGL.h"

#include "ccGBLSensor.h"

//system
#include <string.h>
#include <assert.h>

static const int s_MaxDepthBufferSize = 4096;

ccGBLSensor::ccGBLSensor(ROTATION_ORDER rotOrder/*=THETA_PHI*/)
	: ccSensor("Ground Based Laser Scanner")
	, m_phiMin(0)
	, m_phiMax(0)
	, m_deltaPhi(0)
	, m_thetaMin(0)
	, m_thetaMax(0)
	, m_deltaTheta(0)
	, m_rotationOrder(rotOrder)
	, m_sensorRange(0)
	, m_uncertainty((ScalarType)ZERO_TOLERANCE)
{
	//graphic representation
	lockVisibility(false);
}

ccGBLSensor::ccGBLSensor(const ccGBLSensor &sensor): ccSensor(sensor)
{
	this->m_phiMin = sensor.m_phiMin;
	this->m_phiMax = sensor.m_phiMax;
	this->m_deltaPhi = sensor.m_deltaPhi;
	this->m_thetaMin = sensor.m_thetaMin;
	this->m_thetaMax = sensor.m_thetaMax;
	this->m_deltaTheta = sensor.m_deltaTheta;
	this->m_rotationOrder = sensor.m_rotationOrder;

	this->m_sensorRange = sensor.m_sensorRange;
	this->m_uncertainty = sensor.m_uncertainty;

	//! we cannot simply copy the depth buffer
	// we setup a NULL depthbuffer - it must be recomputed using project
	this->m_depthBuffer = DepthBuffer();
	this->m_depthBuffer.zBuff = 0;
	this->m_depthBuffer.width = 0;
	this->m_depthBuffer.height = 0;
}

ccGBLSensor::~ccGBLSensor()
{
}

void ccGBLSensor::projectPoint(	const CCVector3& sourcePoint,
								CCVector2& destPoint,
								ScalarType &depth,
								double posIndex/*=0*/) const
{
	//project point in sensor world
	CCVector3 P = sourcePoint;

	//sensor center pos in world = sensor position + rigid transformation
	ccIndexedTransformation sensorPos; //identity by default
	if (m_posBuffer)
		m_posBuffer->getInterpolatedTransformation(posIndex,sensorPos);
	sensorPos *= m_rigidTransformation;

	//apply (inverse) global transformation
	sensorPos.inverse().apply(P);

	//convert to 2D ref. + compute its distance
	PointCoordinateType norm = 0;
	switch (m_rotationOrder)
	{
	case THETA_PHI:
	{
		destPoint.x = atan2(P.x,P.y);
		norm = P.x*P.x+P.y*P.y;
		destPoint.y = atan2(P.z,sqrt(norm));
		norm += P.z*P.z;
		break;
	}
	case PHI_THETA:
	{
		destPoint.y = -atan2(P.y,P.z);
		norm = P.y*P.y+P.z*P.z;
		destPoint.x = -atan2(sqrt(norm),P.x);
		norm += P.x*P.x;
		break;
	}
	default:
		assert(false);
	}

	depth = static_cast<ScalarType>(sqrt(norm));
}

CCLib::SimpleCloud* ccGBLSensor::project(CCLib::GenericCloud* theCloud, int& errorCode, bool autoParameters/*false*/)
{
	assert(theCloud);

	CCLib::SimpleCloud* newCloud = new CCLib::SimpleCloud();

	unsigned pointCount = theCloud->size();
	if (!newCloud->reserve(pointCount) || !newCloud->enableScalarField()) //not enough memory
	{
		errorCode = -4;
		delete newCloud;
		return 0;
	}

	ScalarType maxDepth = 0;

	theCloud->placeIteratorAtBegining();
	{
		for (unsigned i = 0; i<pointCount; ++i)
		{
			const CCVector3 *P = theCloud->getNextPoint();
			CCVector2 Q;
			ScalarType depth;
			projectPoint(*P,Q,depth,m_activeIndex);

			newCloud->addPoint(CCVector3(Q.x,Q.y,0));
			newCloud->setPointScalarValue(i,depth);

			if (i != 0)
				maxDepth = std::max(maxDepth,depth);
			else
				maxDepth = depth;
		}
	}

	if (autoParameters)
	{
		//dimensions = theta, phi, 0
		PointCoordinateType bbMin[3],bbMax[3];
		newCloud->getBoundingBox(bbMin,bbMax);

		setTheta(bbMin[0],bbMax[0]);
		setPhi(bbMin[1],bbMax[1]);
		setSensorRange(maxDepth);
	}

	//clear previous Z-buffer
	{
		if (m_depthBuffer.zBuff)
			delete[] m_depthBuffer.zBuff;
		m_depthBuffer.zBuff = 0;
		m_depthBuffer.width = 0;
		m_depthBuffer.height = 0;
	}
	
	//init new Z-buffer
	{
		int width = static_cast<int>(ceil((m_thetaMax-m_thetaMin)/m_deltaTheta));
		int height = static_cast<int>(ceil((m_phiMax-m_phiMin)/m_deltaPhi));

		if (width <= 0 || height <= 0 || std::max(width,height) > s_MaxDepthBufferSize) //too small or... too big!
		{
			errorCode = -2;
			delete newCloud;
			return 0;
		}

		unsigned zBuffSize = width*height;
		m_depthBuffer.zBuff = new ScalarType[zBuffSize];
		if (!m_depthBuffer.zBuff) //not enough memory
		{
			errorCode = -4;
			delete newCloud;
			return 0;
		}
		m_depthBuffer.width = static_cast<unsigned>(width);
		m_depthBuffer.height = static_cast<unsigned>(height);
		memset(m_depthBuffer.zBuff,0,zBuffSize*sizeof(ScalarType));
	}

	//project points and accumulate them in Z-buffer
	newCloud->placeIteratorAtBegining();
	for (unsigned i=0; i<newCloud->size(); ++i)
	{
		const CCVector3 *P = newCloud->getNextPoint();
		ScalarType depth = newCloud->getPointScalarValue(i);

		int x = static_cast<int>(floor((P->x-m_thetaMin)/m_deltaTheta));
		int y = static_cast<int>(floor((P->y-m_phiMin)/m_deltaPhi));

		if (x >= 0 && y >= 0 && static_cast<unsigned>(x) < m_depthBuffer.width && static_cast<unsigned>(y) <= m_depthBuffer.height)
		{
			ScalarType& zBuf = m_depthBuffer.zBuff[static_cast<unsigned>(y)*m_depthBuffer.width+static_cast<unsigned>(x)];
			zBuf = std::max(zBuf,depth);
		}
	}

	errorCode = 0;
	return newCloud;
}

ccGBLSensor::DepthBuffer::DepthBuffer()
	: zBuff(0)
	, width(0)
	, height(0)
{
}

ccGBLSensor::DepthBuffer::~DepthBuffer()
{
	if (zBuff)
		delete[] zBuff;
}

int ccGBLSensor::DepthBuffer::fillHoles()
{
	if (!zBuff)
		return -1; //z-buffer not initialized!

	//new temp buffer
	int dx = width+2;
	int dy = height+2;
	unsigned tempZBuffSize = dx*dy;
	ScalarType* zBuffTemp = new ScalarType[tempZBuffSize];
	if (!zBuffTemp)
		return -2; //not enough memory
	memset(zBuffTemp,0,tempZBuffSize*sizeof(ScalarType));

	//copy old zBuffer in temp one (with 1 pixel border)
	{
		ScalarType *_zBuffTemp = zBuffTemp+dx+1; //2nd line, 2nd column
		ScalarType *_zBuff = zBuff; //first line, first column of the true buffer
		for (unsigned y = 0; y<height; ++y)
		{
			memcpy(_zBuffTemp,_zBuff,width*sizeof(ScalarType));
			_zBuffTemp += dx;
			_zBuff += width;
		}
	}

	//fill holes with their neighbor's mean value
	{
		for (unsigned y = 0; y<height; ++y)
		{
			ScalarType* zu = zBuffTemp + y*dx;
			ScalarType* z = zu + dx;
			ScalarType* zd = z + dx;
			for (unsigned x = 0; x<width; ++x,++zu,++z,++zd)
			{
				if (z[1] == 0) //hole
				{
					uchar nsup = 0; //non empty holes
					//upper line
					nsup += (zu[0]>0);
					nsup += (zu[1]>0);
					nsup += (zu[2]>0);
					//current line
					nsup += ( z[0]>0);
					nsup += ( z[2]>0);
					//next line
					nsup += (zd[0]>0);
					nsup += (zd[1]>0);
					nsup += (zd[2]>0);

					if (nsup>3)
					{
						zBuff[x+y*width] = (zu[0]+zu[1]+zu[2]+ z[0]+z[2]+ zd[0]+zd[1]+zd[2])/static_cast<ScalarType>(nsup);
					}
				}
			}
		}
	}

	delete[] zBuffTemp;

	return 0;
}

PointCoordinateType* ccGBLSensor::projectNormals(CCLib::GenericCloud* aCloud, GenericChunkedArray<3,PointCoordinateType>& theNorms) const
{
	if (!aCloud || !theNorms.isAllocated())
		return 0;

	unsigned size = 3*m_depthBuffer.height*m_depthBuffer.width;
	if (size == 0)
		return 0; //depth buffer empty/not initialized!

	PointCoordinateType* theNewNorms = new PointCoordinateType[size];
	if (!theNewNorms)
		return 0; //not enough memory
	memset(theNewNorms,0,size*sizeof(PointCoordinateType));

	//poject each point+normal
	{
		aCloud->placeIteratorAtBegining();
		theNorms.placeIteratorAtBegining();
		unsigned pointCount = aCloud->size();
		for (unsigned i = 0;i<pointCount;++i)
		{
			const CCVector3* P = aCloud->getNextPoint();
			const PointCoordinateType* N = theNorms.getCurrentValue();

			CCVector3 U = *P - m_rigidTransformation.getTranslation();

			CCVector3 S;
			CCVector2 Q;

			PointCoordinateType norm = U.norm();
			if (norm>ZERO_TOLERANCE)
			{
				//normal component along sensor viewing dir.
				S.z = -CCVector3::vdot(N,U.u)/norm;

				if (S.z > 1.0-ZERO_TOLERANCE)
				{
					S.x = 0;
					S.y = 0;
				}
				else
				{
					//project point
					ScalarType depth1;
					projectPoint(*P,Q,depth1,m_activeIndex);

					//and point+normal
					CCVector3 R = *P + CCVector3(N);
					CCVector2 S2;
					ScalarType depth2;
					projectPoint(R,S2,depth2,m_activeIndex);

					//deduce other normals components
					PointCoordinateType coef = sqrt((1 - S.z*S.z)/(S.x*S.x + S.y*S.y));
					S.x = coef * (S2.x - Q.x);
					S.y = coef * (S2.y - Q.y);
				}
			}
			else
			{
				S = CCVector3(N);
			}

			//project in Z-buffer
			unsigned x = static_cast<unsigned>(floor((Q.x-m_thetaMin)/m_deltaTheta));
			unsigned y = static_cast<unsigned>(floor((Q.y-m_phiMin)/m_deltaPhi));

			//add the transformed normal
			PointCoordinateType* newN = theNewNorms+3*(y*m_depthBuffer.width+x);
			CCVector3::vadd(newN,S.u,newN);

			theNorms.forwardIterator();
		}
	}

	//normalize
	{
		PointCoordinateType* newN = theNewNorms;
		for (unsigned i=0; i<m_depthBuffer.height*m_depthBuffer.width; ++i,newN+=3)
		{
			CCVector3::vnormalize(newN);
		}
	}

	return theNewNorms;
}

colorType* ccGBLSensor::projectColors(CCLib::GenericCloud* aCloud, GenericChunkedArray<3,colorType>& theColors) const
{
	if (!aCloud || !theColors.isAllocated())
		return 0;

	unsigned size = m_depthBuffer.height*m_depthBuffer.width;
	if (size == 0)
		return 0; //depth buffer empty/not initialized!

	colorType* theNewColors = new colorType[3*size];
	if (!theNewColors)
		return 0; //not enough memory
	memset(theNewColors,0,3*size*sizeof(colorType));

	size_t* theNewColorsCount = new size_t[size];
	if (!theNewColorsCount)
	{
		delete[] theNewColors;
		return 0; //not enough memory
	}
	memset(theNewColorsCount,0,size*sizeof(colorType));

	//int x,y,index;
	//colorType *C,*_theNewColors;

	//project colors
	{
		unsigned pointCount = aCloud->size();
		aCloud->placeIteratorAtBegining();
		theColors.placeIteratorAtBegining();
		{
			for (unsigned i=0; i<pointCount; ++i)
			{
				const CCVector3 *P = aCloud->getNextPoint();
				CCVector2 Q;
				ScalarType depth;
				projectPoint(*P,Q,depth,m_activeIndex);

				unsigned x = static_cast<unsigned>(floor((Q.x-m_thetaMin)/m_deltaTheta));
				unsigned y = static_cast<unsigned>(floor((Q.y-m_phiMin)/m_deltaPhi));
				unsigned index = y*m_depthBuffer.width+x;
				++theNewColorsCount[index];
				
				//add color
				colorType* C = theColors.getCurrentValue();
				colorType* _theNewColor = theNewColors + 3*index;

				if (_theNewColor[0] != 0 || _theNewColor[1] != 0 || _theNewColor[2] != 0)
				{
					//crappy mobile mean to avoid overflows!
					_theNewColor[0]=static_cast<colorType>((static_cast<int>(_theNewColor[0])+static_cast<int>(C[0]))>>1);
					_theNewColor[1]=static_cast<colorType>((static_cast<int>(_theNewColor[1])+static_cast<int>(C[1]))>>1);
					_theNewColor[2]=static_cast<colorType>((static_cast<int>(_theNewColor[2])+static_cast<int>(C[2]))>>1);
				}
				else
				{
					_theNewColor[0] = C[0];
					_theNewColor[1] = C[1];
					_theNewColor[2] = C[2];
				}

				theColors.forwardIterator();
			}
		}
	}

	//normalize
	{
		colorType* _theNewColor = theNewColors;
		for (unsigned i=0; i<m_depthBuffer.height*m_depthBuffer.width; ++i,_theNewColor+=3)
		{
			if (theNewColorsCount[i] > 1)
			{
				_theNewColor[0] = static_cast<colorType>(static_cast<float>(_theNewColor[0])/static_cast<float>(theNewColorsCount[i]));
				_theNewColor[1] = static_cast<colorType>(static_cast<float>(_theNewColor[1])/static_cast<float>(theNewColorsCount[i]));
				_theNewColor[2] = static_cast<colorType>(static_cast<float>(_theNewColor[2])/static_cast<float>(theNewColorsCount[i]));
			}
		}
	}

	delete[] theNewColorsCount;

	return theNewColors;
}

uchar ccGBLSensor::checkVisibility(const CCVector3& P) const
{
	if (!m_depthBuffer.zBuff) //no z-buffer?
		return POINT_VISIBLE;

	//project point
	CCVector2 Q;
	ScalarType depth;
	projectPoint(P,Q,depth,m_activeIndex);

	//out of sight
	if (depth > m_sensorRange)
		return POINT_OUT_OF_RANGE;

	int x = static_cast<int>(floor((Q.x-m_thetaMin)/m_deltaTheta));
	int y = static_cast<int>(floor((Q.y-m_phiMin)/m_deltaPhi));

	//out of field
	if (	x < 0 || static_cast<unsigned>(x) >= m_depthBuffer.width
		||	y < 0 || static_cast<unsigned>(y) >= m_depthBuffer.height )
	{
		return POINT_OUT_OF_FOV;
	}

	//hidden?
	if (depth > m_depthBuffer.zBuff[x+y*m_depthBuffer.width]*(1.0f+m_uncertainty))
		return POINT_HIDDEN;

	return POINT_VISIBLE;
}

void ccGBLSensor::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	//we draw here a little 3d representation of the sensor
	if (MACRO_Draw3D(context))
	{
		bool pushName = MACRO_DrawEntityNames(context);

		if (pushName)
		{
			//not particulary fast
			if (MACRO_DrawFastNamesOnly(context))
				return;
			glPushName(getUniqueIDForDisplay());
		}

		//DGM FIXME: this display routine is crap!

		//apply rigid transformation
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

		//sensor head
		const PointCoordinateType halfHeadSize = static_cast<PointCoordinateType>(0.3);
		CCVector3 minCorner(-halfHeadSize,-halfHeadSize,-halfHeadSize);
		CCVector3 maxCorner(halfHeadSize,halfHeadSize,halfHeadSize);
		minCorner *= m_scale;
		maxCorner *= m_scale;
		ccBBox bbHead(minCorner,maxCorner);
		CCVector3 headCenter(0,0,(1-halfHeadSize)*m_scale);
		bbHead += headCenter;
		bbHead.draw(m_color.u);

		//sensor legs
		CCVector3 headConnect = headCenter - CCVector3(0,0,static_cast<PointCoordinateType>(halfHeadSize)*m_scale);
		glColor3ubv(m_color.u);
		glBegin(GL_LINES);
		ccGL::Vertex3v(headConnect.u);
		ccGL::Vertex3(-m_scale,-m_scale,-m_scale);
		ccGL::Vertex3v(headConnect.u);
		ccGL::Vertex3(-m_scale,m_scale,-m_scale);
		ccGL::Vertex3v(headConnect.u);
		ccGL::Vertex3(m_scale,0,-m_scale);
		glEnd();

		if (pushName)
			glPopName();

		glPopMatrix();
	}
}

ccBBox ccGBLSensor::getMyOwnBB()
{
	return ccBBox();
	//ccIndexedTransformation sensorPos;
	//if (!getAbsoluteTransformation(sensorPos,m_activeIndex))
	//	return ccBBox();

	//CCVector3 center = sensorPos.getTranslationAsVec3D();

	//return ccBBox(center + CCVector3(-1,-1,-1) * m_scale,
	//				center + CCVector3( 1, 1, 1) * m_scale);
}

ccBBox ccGBLSensor::getDisplayBB()
{
	//return getMyOwnBB();
	ccIndexedTransformation sensorPos;
	if (!getAbsoluteTransformation(sensorPos,m_activeIndex))
		return ccBBox();

	CCVector3 center = sensorPos.getTranslationAsVec3D();

	return ccBBox(	center + CCVector3(-1,-1,-1) * m_scale,
					center + CCVector3( 1, 1, 1) * m_scale);
}

bool ccGBLSensor::toFile_MeOnly(QFile& out) const
{
	if (!ccSensor::toFile_MeOnly(out))
		return false;

	//rotation order (dataVersion>=34)
	uint32_t rotOrder = m_rotationOrder;
	if (out.write((const char*)&rotOrder,4)<0)
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

	return true;
}

bool ccGBLSensor::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccSensor::fromFile_MeOnly(in, dataVersion, flags))
		return false;

	//rotation order (dataVersion>=34)
	uint32_t rotOrder = 0;
	if (in.read((char*)&rotOrder,4)<0)
		return ReadError();
	m_rotationOrder = static_cast<ROTATION_ORDER>(rotOrder);

	//parameters (dataVersion>=34)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_phiMin,1);
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_phiMax,1);
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_deltaPhi,1);
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_thetaMin,1);
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_thetaMax,1);
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_deltaTheta,1);
	ccSerializationHelper::ScalarsFromDataStream(inStream,flags,&m_sensorRange,1);
	ccSerializationHelper::ScalarsFromDataStream(inStream,flags,&m_uncertainty,1);
	ccSerializationHelper::CoordsFromDataStream(inStream,flags,&m_scale,1);

	return true;
}
