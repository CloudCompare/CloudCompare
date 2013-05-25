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

//CCLib
//#include <GenericIndexedCloud.h>

//system
#include <string.h>
#include <assert.h>

ccGBLSensor::ccGBLSensor(ROTATION_ORDER rotOrder/*=THETA_PHI*/)
	: ccSensor()
	, base(0)
	, sensorCenter(0.0f)
	, phiMin(0)
	, phiMax(0)
	, deltaPhi(0)
	, thetaMin(0)
	, thetaMax(0)
	, deltaTheta(0)
	, rotationOrder(rotOrder)
	, sensorRange(0)
	, uncertainty((ScalarType)ZERO_TOLERANCE)
	, scale(1.0f)
{
	//sensor pose
	setSensorCenter(CCVector3(0.0,0.0,0.0));

	//orientation matrix
	m_orientation.toIdentity();

	//object name
	setName("Ground Based Laser Scanner");

    //graphic representation
    lockVisibility(false);
}

ccGBLSensor::~ccGBLSensor()
{
}

inline void ccGBLSensor::projectPoint(const CCVector3& sourcePoint, CCVector2& destPoint, ScalarType &depth) const
{
	//project point in sensor world
	CCVector3 P = sourcePoint - sensorCenter;
	P.x += base;
	m_orientation.applyRotation(P);

	//convert to 2D ref. + compute its distance
	PointCoordinateType norm = 0;
	switch(rotationOrder)
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

	depth = (ScalarType)sqrt(norm);
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

	ScalarType maxDepth=0;

	theCloud->placeIteratorAtBegining();
	{
		for (unsigned i=0; i<pointCount; ++i)
		{
			const CCVector3 *P = theCloud->getNextPoint();
			CCVector2 Q;
			ScalarType depth;
			projectPoint(*P,Q,depth);

			newCloud->addPoint(CCVector3(Q.x,Q.y,0.0));
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
		m_depthBuffer.zBuff=0;
		m_depthBuffer.width=0;
		m_depthBuffer.height=0;
	}
	
	//init new Z-buffer
	{
		int width = (int)ceil((thetaMax-thetaMin)/deltaTheta);
		int height = (int)ceil((phiMax-phiMin)/deltaPhi);

		if (width*height==0 || std::max(width,height)>2048) //too small or... too big!
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
		m_depthBuffer.width = width;
		m_depthBuffer.height = height;
		memset(m_depthBuffer.zBuff,0,zBuffSize*sizeof(ScalarType));
	}

	//project points and accumulate them in Z-buffer
	newCloud->placeIteratorAtBegining();
	for (unsigned i=0;i<newCloud->size();++i)
	{
		const CCVector3 *P = newCloud->getNextPoint();
		ScalarType depth = newCloud->getPointScalarValue(i);

		int x = (int)floor((P->x-thetaMin)/deltaTheta);
		int y = (int)floor((P->y-phiMin)/deltaPhi);

		ScalarType& zBuf = m_depthBuffer.zBuff[y*m_depthBuffer.width+x];
		zBuf = std::max(zBuf,depth);
	}

	errorCode = 0;
	return newCloud;
}

int ccGBLSensor::fillZBufferHoles()
{
	if (!m_depthBuffer.zBuff)
		return -1; //z-buffer not initialized!

	//new temp buffer
	int dx = m_depthBuffer.width+2;
	int dy = m_depthBuffer.height+2;
	unsigned tempZBuffSize = dx*dy;
	ScalarType* zBuffTemp = new ScalarType[tempZBuffSize];
	if (!zBuffTemp)
		return -2; //not enough memory
	memset(zBuffTemp,0,tempZBuffSize*sizeof(ScalarType));

	//copy old zBuffer in temp one (with 1 pixel border)
	{
		ScalarType *_zBuffTemp = zBuffTemp+dx+1; //2nd line, 2nd column
		ScalarType *_zBuff = m_depthBuffer.zBuff; //first line, first column of the true buffer
		for (int y=0; y<m_depthBuffer.height; ++y)
		{
			memcpy(_zBuffTemp,_zBuff,m_depthBuffer.width*sizeof(ScalarType));
			_zBuffTemp += dx;
			_zBuff += m_depthBuffer.width;
		}
	}

	//fill holes with their neighbor's mean value
	{
		for (int y=0;y<m_depthBuffer.height;++y)
		{
			ScalarType* zu = zBuffTemp + y*dx;
			ScalarType* z = zu + dx;
			ScalarType* zd = z + dx;
			for (int x=0; x<m_depthBuffer.width; ++x,++zu,++z,++zd)
			{
				if (z[1] == 0) //hole
				{
					uchar nsup=0; //non empty holes
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
						m_depthBuffer.zBuff[x+y*m_depthBuffer.width] = (zu[0]+zu[1]+zu[2]+ z[0]+z[2]+ zd[0]+zd[1]+zd[2])/(ScalarType)nsup;
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
		for (unsigned i=0;i<pointCount;++i)
		{
			const CCVector3* P = aCloud->getNextPoint();
			const PointCoordinateType* N = theNorms.getCurrentValue();

			CCVector3 U = *P - sensorCenter;
			U.x += base;

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
					projectPoint(*P,Q,depth1);

					//and point+normal
					CCVector3 R = *P + CCVector3(N);
					CCVector2 S2;
					ScalarType depth2;
					projectPoint(R,S2,depth2);

					//deduce other normals components
					float coef = sqrt((1.0f-S.z*S.z)/(S.x*S.x+S.y*S.y));
					S.x = coef * (S2.x - Q.x);
					S.y = coef * (S2.y - Q.y);
				}
			}
			else
			{
				S = CCVector3(N);
			}

			//project in Z-buffer
			int x = (int)floor((Q.x-thetaMin)/deltaTheta);
			int y = (int)floor((Q.y-phiMin)/deltaPhi);

			//on ajoute la normale transformee
			PointCoordinateType* newN = theNewNorms+3*(y*m_depthBuffer.width+x);
			CCVector3::vadd(newN,S.u,newN);

			theNorms.forwardIterator();
		}
	}

	//normalize
	{
		PointCoordinateType* newN = theNewNorms;
		for (int i=0; i<m_depthBuffer.height*m_depthBuffer.width; ++i,newN+=3)
		{
			CCVector3::vnormalize(newN);
		}
	}

	return theNewNorms;
}

uchar* ccGBLSensor::projectColors(CCLib::GenericCloud* aCloud, GenericChunkedArray<3,uchar>& theColors) const
{
	if (!aCloud || !theColors.isAllocated())
		return 0;

	unsigned size = m_depthBuffer.height*m_depthBuffer.width;
	if (size == 0)
		return 0; //depth buffer empty/not initialized!

	uchar* theNewColors = new uchar[3*size];
	if (!theNewColors)
		return 0; //not enough memory
	memset(theNewColors,0,3*size*sizeof(uchar));

	size_t* theNewColorsCount = new size_t[size];
	if (!theNewColorsCount)
	{
		delete[] theNewColors;
		return 0; //not enough memory
	}
	memset(theNewColorsCount,0,size*sizeof(uchar));

	//int x,y,index;
	//uchar *C,*_theNewColors;

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
				projectPoint(*P,Q,depth);

				int x = (int)floor((Q.x-thetaMin)/deltaTheta);
				int y = (int)floor((Q.y-phiMin)/deltaPhi);
				int index = y*m_depthBuffer.width+x;
				++theNewColorsCount[index];
				
				//add color
				uchar* C = theColors.getCurrentValue();
				uchar* _theNewColor = theNewColors + 3*index;

				if (_theNewColor[0] != 0 || _theNewColor[1]!=0 || _theNewColor[2]!=0)
				{
					//crappy mobile mean to avoid overflows!
					_theNewColor[0]=(uchar)((int(_theNewColor[0])+int(C[0]))>>1);
					_theNewColor[1]=(uchar)((int(_theNewColor[1])+int(C[1]))>>1);
					_theNewColor[2]=(uchar)((int(_theNewColor[2])+int(C[2]))>>1);
				}
				else
				{
					_theNewColor[0]=C[0];
					_theNewColor[1]=C[1];
					_theNewColor[2]=C[2];
				}

				theColors.forwardIterator();
			}
		}
	}

	//normalize
	{
		uchar* _theNewColor = theNewColors;
		for (int i=0;i<m_depthBuffer.height*m_depthBuffer.width;++i,_theNewColor+=3)
		{
			if (theNewColorsCount[i]>1)
			{
				float coef = (float)theNewColorsCount[i];
				_theNewColor[0] = (uchar)((float)_theNewColor[0]/coef);
				_theNewColor[1] = (uchar)((float)_theNewColor[1]/coef);
				_theNewColor[2] = (uchar)((float)_theNewColor[2]/coef);
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
	projectPoint(P,Q,depth);

	//out of sight
	if (depth > sensorRange)
		return POINT_OUT_OF_RANGE;

	int x = (int)floor((Q.x-thetaMin)/deltaTheta);
	int y = (int)floor((Q.y-phiMin)/deltaPhi);

	//out of field
	if (x<0 || x>=m_depthBuffer.width || y<0 || y>=m_depthBuffer.height)
		return POINT_OUT_OF_FOV;

	//hidden?
	if (depth > m_depthBuffer.zBuff[x+y*m_depthBuffer.width]*(1.0f+uncertainty))
		return POINT_HIDDEN;

	return POINT_VISIBLE;
}

void ccGBLSensor::setOrientationMatrix(const ccGLMatrix& mat)
{
	m_orientation = mat;
}

const ccGLMatrix& ccGBLSensor::getOrientationMatrix() const
{
	return m_orientation;
}

void ccGBLSensor::setPhi(float minV, float maxV)
{
	phiMin=minV;
	phiMax=maxV;
}

void ccGBLSensor::setDeltaPhi(float dPhi)
{
	deltaPhi=dPhi;
}

float ccGBLSensor::getPhiMin() const
{
	return phiMin;
}

float ccGBLSensor::getPhiMax() const
{
	return phiMax;
}

float ccGBLSensor::getDeltaPhi() const
{
	return deltaPhi;
}

void ccGBLSensor::setTheta(float minV, float maxV)
{
	thetaMin=minV;
	thetaMax=maxV;
}

void ccGBLSensor::setDeltaTheta(float dTheta)
{
	deltaTheta=dTheta;
}

float ccGBLSensor::getThetaMin() const
{
	return thetaMin;
}

float ccGBLSensor::getThetaMax() const
{
	return thetaMax;
}

float ccGBLSensor::getDeltaTheta() const
{
	return deltaTheta;
}

float ccGBLSensor::getSensorBase() const
{
	return base;
}

void ccGBLSensor::setSensorBase(PointCoordinateType _base)
{
	base=_base;
}

ScalarType ccGBLSensor::getSensorRange() const
{
	return sensorRange;
}

void ccGBLSensor::setSensorRange(ScalarType range)
{
	sensorRange = range;
}

CCVector3 ccGBLSensor::getSensorCenter() const
{
	return sensorCenter;
}

void ccGBLSensor::setSensorCenter(const CCVector3& C)
{
	sensorCenter=C;
}

ScalarType ccGBLSensor::getUncertainty() const
{
	return uncertainty;
}

void ccGBLSensor::setUncertainty(ScalarType u)
{
	uncertainty=u;
}

ccGBLSensor::ROTATION_ORDER ccGBLSensor::getRotationOrder() const
{
	return rotationOrder;
}

void ccGBLSensor::setRotationOrder(ROTATION_ORDER rotOrder)
{
	rotationOrder=rotOrder;
}

const ccGBLSensor::DepthBuffer& ccGBLSensor::getDepthBuffer() const
{
	return m_depthBuffer;
}

void ccGBLSensor::setGraphicScale(double _scale)
{
    scale = _scale;
}

double ccGBLSensor::getGraphicScale() const
{
    return scale;
}

void ccGBLSensor::updateGraphicRepresentation()
{
    //rotation matrix
    ccGLMatrix glTrans = m_orientation.inverse();
	glTrans.shiftRotationCenter(sensorCenter); //rotation center = sensor center
    //translation = sensor center
    glTrans += sensorCenter;

	setGLTransformation(glTrans);
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
            glPushName(getUniqueID());
		}

		//DGM FIXME: crap!
        //sensor head
        const double halfHeadSize=0.3;
        CCVector3 minCorner(-halfHeadSize,-halfHeadSize,-halfHeadSize);
        CCVector3 maxCorner(halfHeadSize,halfHeadSize,halfHeadSize);
        minCorner*=scale;
        maxCorner*=scale;
        ccBBox bbHead(minCorner,maxCorner);
        CCVector3 headCenter(0.0,0.0,(1.0-halfHeadSize)*scale);
        bbHead += headCenter;
        bbHead.draw(ccColor::green);

        //sensor legs
        CCVector3 headConnect = headCenter - CCVector3(0.0,0.0,halfHeadSize*scale);
        glBegin(GL_LINES);
        glVertex3fv(headConnect.u);
        glVertex3f(-scale,-scale,-scale);
        glVertex3fv(headConnect.u);
        glVertex3f(-scale,scale,-scale);
        glVertex3fv(headConnect.u);
        glVertex3f(scale,0.0,-scale);
        glEnd();

        if (pushName)
            glPopName();
    }
}

/*ccBBox ccGBLSensor::getMyOwnBB()
{
}
//*/

ccBBox ccGBLSensor::getDisplayBB()
{
    CCVector3 minCorner(-1.0,-1.0,-1.0);
    CCVector3 maxCorner(1.0,1.0,1.0);
    minCorner*=scale;
    maxCorner*=scale;

    return ccBBox(minCorner,maxCorner);
}

