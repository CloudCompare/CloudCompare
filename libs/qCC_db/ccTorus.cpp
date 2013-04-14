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

#include "ccTorus.h"

//Local
#include "ccPointCloud.h"
#include "ccNormalVectors.h"

ccTorus::ccTorus(PointCoordinateType insideRadius,
				 PointCoordinateType outsideRadius,
				 double angle_rad/*=2.0*M_PI*/,
				 bool rectangularSection/*=false*/,
				 PointCoordinateType rectSectionHeight/*=0*/,
				 const ccGLMatrix* transMat/*=0*/,
				 QString name/*=QString("Torus")*/,
				 unsigned precision/*=24*/)
	: ccGenericPrimitive(name,transMat)
	, m_insideRadius(fabs(insideRadius))
	, m_outsideRadius(fabs(outsideRadius))
	, m_rectSection(rectangularSection)
	, m_rectSectionHeight(fabs(rectSectionHeight))
	, m_angle_rad(fabs(angle_rad))
{
	setDrawingPrecision(std::max<unsigned>(precision,4)); //automatically calls buildUp + 	applyTransformationToVertices
}

ccTorus::ccTorus(QString name/*=QString("Torus")*/)
	: ccGenericPrimitive(name)
	, m_insideRadius(0)
	, m_outsideRadius(0)
	, m_rectSection(false)
	, m_rectSectionHeight(0)
   , m_angle_rad(0)
{
}

ccGenericPrimitive* ccTorus::clone() const
{
	return finishCloneJob(new ccTorus(m_insideRadius,m_outsideRadius,m_angle_rad,m_rectSection,m_rectSectionHeight,&m_transformation,getName(),m_drawPrecision));
}

bool ccTorus::buildUp()
{
	if (m_drawPrecision<4)
		return false;

	//invalid parameters?
	if ((m_rectSection && m_rectSectionHeight < ZERO_TOLERANCE) || m_insideRadius >= m_outsideRadius || m_angle_rad < ZERO_TOLERANCE)
		return false;

	//topology
	bool closed = (m_angle_rad >= 2.0*M_PI);

	const unsigned steps = m_drawPrecision;

	unsigned sweepSteps = 4*(closed ? steps : (unsigned)ceil(m_angle_rad*(double)steps/(2.0*M_PI)));
	unsigned sectSteps = (m_rectSection ? 4 : steps);

	//vertices
	unsigned vertCount = (sweepSteps+(closed ? 0 : 1))*sectSteps; //DGM: +1 row for non closed loops
	//faces
	unsigned facesCount = sweepSteps*sectSteps*2;
	//faces normals
	unsigned faceNormCount = (sweepSteps+(closed ? 0 : 1))*sectSteps; //DGM: +1 row for non closed loops
	if (!closed)
		facesCount += (m_rectSection ? 2 : sectSteps)*2;

	if (!init(vertCount+(closed || m_rectSection ? 0 : 2),false,facesCount,faceNormCount+(closed ? 0 : 2)))
	{
		ccLog::Error("[ccTorus::buildUp] Not enough memory");
		return false;
	}

	//2D section
	CCVector3* sectPoints = new CCVector3[sectSteps];
	if (!sectPoints)
	{
		init(0,false,0,0);
		ccLog::Error("[ccTorus::buildUp] Not enough memory");
		return false;
	}

	PointCoordinateType sectionRadius = (m_outsideRadius-m_insideRadius)/2;
	if (m_rectSection)
	{
		//rectangular section
		sectPoints[0].x = (m_outsideRadius-m_insideRadius)/2;
		sectPoints[0].z = m_rectSectionHeight/2;
		sectPoints[1].x = -sectPoints[0].x;
		sectPoints[1].z = sectPoints[0].z;
		sectPoints[2].x = sectPoints[1].x;
		sectPoints[2].z = -sectPoints[1].z;
		sectPoints[3].x = -sectPoints[2].x;
		sectPoints[3].z = sectPoints[2].z;
	}
	else
	{
		//circular section
		for (unsigned i=0;i<sectSteps;++i)
		{
			float sect_angle_rad = (float)i/(float)sectSteps*(float)(2.0*M_PI);
			sectPoints[i].x = cos(sect_angle_rad) * sectionRadius;
			sectPoints[i].z = sin(sect_angle_rad) * sectionRadius;
		}
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	//main sweep
	PointCoordinateType sweepRadius = (m_insideRadius+m_outsideRadius)/(PointCoordinateType)2.0;
	double sweepStep_rad = m_angle_rad/(double)sweepSteps;
	for (unsigned t=0;t<(closed ? sweepSteps : sweepSteps+1);++t)
	{
		//unit director vector
		CCVector3 sweepU(cos((double)t*sweepStep_rad),
						 sin((double)t*sweepStep_rad),
						 0);

		//section points
		for (unsigned i=0;i<sectSteps;++i)
		{
			CCVector3 P(sweepU.x * (sweepRadius + sectPoints[i].x),
						sweepU.y * (sweepRadius + sectPoints[i].x),
						sectPoints[i].z);
			verts->addPoint(P);
		}

		//normals
		if (m_rectSection)
		{
			m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0.0,0.0,1.0).u));
			m_triNormals->addElement(ccNormalVectors::GetNormIndex((-sweepU).u));
			m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0.0,0.0,-1.0).u));
			m_triNormals->addElement(ccNormalVectors::GetNormIndex((-sweepU).u));
		}
		else //circular section
		{
			for (unsigned i=0;i<sectSteps;++i)
			{
				float sectAngle_rad = (float)i/(float)sectSteps*(float)(2.0*M_PI);
				CCVector3 sectU(cos(sectAngle_rad),0.0,sin(sectAngle_rad));
				CCVector3 N(sweepU.x * sectU.x,
					sweepU.y * sectU.x,
					sectU.z);
				m_triNormals->addElement(ccNormalVectors::GetNormIndex(N.u));
			}
		}
	}

	if (!closed && !m_rectSection)
	{
		CCVector3 P(sweepRadius,0.0,0.0);
		verts->addPoint(P);
		CCVector3 P2(cos(m_angle_rad)*sweepRadius,
						 sin(m_angle_rad)*sweepRadius,
						 0);
		verts->addPoint(P2);
	}

	if (!closed)
	{
		//first section (left side)
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0.0,-1.0,0.0).u));
		//last section (right side)
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(-sin(m_angle_rad),cos(m_angle_rad),0.0).u));
	}

	delete[] sectPoints;
	sectPoints=0;

	//mesh faces
	{
		assert(m_triIndexes);

		for (unsigned t=0;t<sweepSteps;++t)
		{
			unsigned sweepStart = t*sectSteps;
			for (unsigned i=0;i<sectSteps;++i)
			{
				unsigned iNext = (i+1)%sectSteps;
				addTriangle(sweepStart+i,(sweepStart+i+sectSteps)%vertCount,(sweepStart+iNext+sectSteps)%vertCount);
				if (m_rectSection)
					addTriangleNormalIndexes(sweepStart+i,(sweepStart+i+sectSteps)%faceNormCount,(sweepStart+i+sectSteps)%faceNormCount);
				else
					addTriangleNormalIndexes(sweepStart+i,(sweepStart+i+sectSteps)%faceNormCount,(sweepStart+iNext+sectSteps)%faceNormCount);
				addTriangle(sweepStart+i,(sweepStart+iNext+sectSteps)%vertCount,sweepStart+iNext);
				if (m_rectSection)
					addTriangleNormalIndexes(sweepStart+i,(sweepStart+i+sectSteps)%faceNormCount,sweepStart+i);
				else
					addTriangleNormalIndexes(sweepStart+i,(sweepStart+iNext+sectSteps)%faceNormCount,sweepStart+iNext);
			}
		}

		if (!closed)
		{
			unsigned lastSectionShift = sweepSteps*sectSteps;
			if (m_rectSection)
			{
				//rectangular left section
				addTriangle(0,1,2);
				addTriangleNormalIndexes(faceNormCount,faceNormCount,faceNormCount);
				addTriangle(0,2,3);
				addTriangleNormalIndexes(faceNormCount,faceNormCount,faceNormCount);
				//rectangular right section
				addTriangle(lastSectionShift,lastSectionShift+2,lastSectionShift+1);
				addTriangleNormalIndexes(faceNormCount+1,faceNormCount+1,faceNormCount+1);
				addTriangle(lastSectionShift,lastSectionShift+3,lastSectionShift+2);
				addTriangleNormalIndexes(faceNormCount+1,faceNormCount+1,faceNormCount+1);
			}
			else
			{
				unsigned lastSectionCenterShift = vertCount;
				//circular 'left' section
				for (unsigned i=0;i<sectSteps;++i)
				{
					unsigned iNext = (i+1)%sectSteps;
					addTriangle(lastSectionCenterShift,i,iNext);
					addTriangleNormalIndexes(faceNormCount,faceNormCount,faceNormCount);
				}
				//circular 'right' section
				for (unsigned i=0;i<sectSteps;++i)
				{
					unsigned iNext = (i+1)%sectSteps;
					addTriangle(lastSectionCenterShift+1,lastSectionShift+iNext,lastSectionShift+i);
					addTriangleNormalIndexes(faceNormCount+1,faceNormCount+1,faceNormCount+1);
				}
			}
		}
	}

	updateModificationTime();
	showTriNorms(true);

	return true;
}

bool ccTorus::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);
	outStream << m_insideRadius;
	outStream << m_outsideRadius;
	outStream << m_rectSection;
	outStream << m_rectSectionHeight;
	outStream << m_angle_rad;

	return true;
}

bool ccTorus::fromFile_MeOnly(QFile& in, short dataVersion)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	inStream >> m_insideRadius;
	inStream >> m_outsideRadius;
	inStream >> m_rectSection;
	inStream >> m_rectSectionHeight;
	inStream >> m_angle_rad;

	return true;
}
