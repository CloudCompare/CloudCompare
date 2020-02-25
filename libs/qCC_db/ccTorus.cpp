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

#include "ccTorus.h"

//Local
#include "ccPointCloud.h"
#include "ccNormalVectors.h"

ccTorus::ccTorus(	PointCoordinateType insideRadius,
					PointCoordinateType outsideRadius,
					double angle_rad/*=2.0*M_PI*/,
					bool rectangularSection/*=false*/,
					PointCoordinateType rectSectionHeight/*=0*/,
					const ccGLMatrix* transMat/*=0*/,
					QString name/*=QString("Torus")*/,
					unsigned precision/*=DEFAULT_DRAWING_PRECISION*/,
					unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccGenericPrimitive(name, transMat, uniqueID)
	, m_insideRadius(fabs(insideRadius))
	, m_outsideRadius(fabs(outsideRadius))
	, m_rectSection(rectangularSection)
	, m_rectSectionHeight(fabs(rectSectionHeight))
	, m_angle_rad(fabs(angle_rad))
{
	setDrawingPrecision(std::max<unsigned>(precision,MIN_DRAWING_PRECISION)); //automatically calls updateRepresentation
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
	if (m_drawPrecision < MIN_DRAWING_PRECISION)
		return false;

	//invalid parameters?
	if ((m_rectSection && m_rectSectionHeight < ZERO_TOLERANCE) || m_insideRadius >= m_outsideRadius || m_angle_rad < ZERO_TOLERANCE)
		return false;

	//topology
	bool closed = (m_angle_rad >= 2.0*M_PI);

	const unsigned steps = m_drawPrecision;

	unsigned sweepSteps = 4 * (closed ? steps : static_cast<unsigned>(ceil((m_angle_rad * steps)/(2.0*M_PI))));
	unsigned sectSteps = (m_rectSection ? 4 : steps);

	//vertices
	unsigned vertCount = (sweepSteps + (closed ? 0 : 1)) * sectSteps; //DGM: +1 row for non closed loops
	//faces
	unsigned facesCount = sweepSteps * sectSteps *2;
	//faces normals
	unsigned faceNormCount = (sweepSteps + (closed ? 0 : 1)) * sectSteps; //DGM: +1 row for non closed loops
	if (!closed)
		facesCount += (m_rectSection ? 2 : sectSteps)*2;

	if (!init(vertCount + (closed || m_rectSection ? 0 : 2), false, facesCount, faceNormCount + (closed ? 0 : 2)))
	{
		ccLog::Error("[ccTorus::buildUp] Not enough memory");
		return false;
	}

	//2D section
	std::vector<CCVector3> sectPoints;
	try
	{
		sectPoints.resize(sectSteps);
	}
	catch (const std::bad_alloc&)
	{
		init(0,false,0,0);
		ccLog::Error("[ccTorus::buildUp] Not enough memory");
		return false;
	}

	double sweepStep_rad = m_angle_rad / sweepSteps;
	double sectStep_rad = (2.0*M_PI) / sectSteps;

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
		for (unsigned i=0; i<sectSteps; ++i)
		{
			double sect_angle_rad = i * sectStep_rad;
			sectPoints[i].x = static_cast<PointCoordinateType>(cos(sect_angle_rad) * sectionRadius);
			sectPoints[i].z = static_cast<PointCoordinateType>(sin(sect_angle_rad) * sectionRadius);
		}
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	//main sweep
	PointCoordinateType sweepRadius = (m_insideRadius + m_outsideRadius)/2;
	for (unsigned t=0; t<(closed ? sweepSteps : sweepSteps+1); ++t)
	{
		//unit director vector
		CCVector3 sweepU(static_cast<PointCoordinateType>(cos(t*sweepStep_rad)),
						 static_cast<PointCoordinateType>(sin(t*sweepStep_rad)),
						 0);

		//section points
		for (unsigned i=0; i<sectSteps; ++i)
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
			m_triNormals->addElement(ccNormalVectors::GetNormIndex(sweepU.u));
		}
		else //circular section
		{
			for (unsigned i=0; i<sectSteps; ++i)
			{
				double sectAngle_rad = i * sectStep_rad;
				CCVector3 sectU = CCVector3::fromArray(CCVector3(cos(sectAngle_rad), 0.0, sin(sectAngle_rad)).u);
				CCVector3 N(sweepU.x * sectU.x,
							sweepU.y * sectU.x,
							sectU.z);
				m_triNormals->addElement(ccNormalVectors::GetNormIndex(N.u));
			}
		}
	}

	if (!closed && !m_rectSection)
	{
		CCVector3 P(sweepRadius,0,0);
		verts->addPoint(P);
		CCVector3 P2(	static_cast<PointCoordinateType>(cos(m_angle_rad))*sweepRadius,
						static_cast<PointCoordinateType>(sin(m_angle_rad))*sweepRadius,
						 0);
		verts->addPoint(P2);
	}

	if (!closed)
	{
		//first section (left side)
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(0,-1,0).u));
		//last section (right side)
		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(	static_cast<PointCoordinateType>(-sin(m_angle_rad)),
																			static_cast<PointCoordinateType>(cos(m_angle_rad)),
																			0).u));
	}

	sectPoints.clear();

	//mesh faces
	{
		assert(m_triVertIndexes);

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

	notifyGeometryUpdate();
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

bool ccTorus::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_insideRadius, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_outsideRadius, 1);
	inStream >> m_rectSection;
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_rectSectionHeight, 1);
	inStream >> m_angle_rad;

	return true;
}
