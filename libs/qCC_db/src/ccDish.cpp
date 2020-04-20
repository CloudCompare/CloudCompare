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

#include "ccDish.h"

//Local
#include "ccPointCloud.h"
#include "ccNormalVectors.h"

ccDish::ccDish(	PointCoordinateType radius,
				PointCoordinateType height,
				PointCoordinateType radius2/*=0*/,
				const ccGLMatrix* transMat/*=0*/,
				QString name/*="Dish"*/,
				unsigned precision/*=DEFAULT_DRAWING_PRECISION*/)
	: ccGenericPrimitive(name,transMat)
	, m_baseRadius(radius)
	, m_secondRadius(radius2)
	, m_height(height)
{
	if (radius2 == 0)
		m_height = std::min(height,radius); //in spherical mode, the height can't be superior to the radius! (dishes are at most hemispheres)
	setDrawingPrecision(std::max<unsigned>(precision,MIN_DRAWING_PRECISION));  //automatically calls buildUp + 	applyTransformationToVertices
}

ccDish::ccDish(QString name/*="Sphere"*/)
	: ccGenericPrimitive(name)
	, m_baseRadius(0)
	, m_secondRadius(0)
	, m_height(0)
{
}

ccGenericPrimitive* ccDish::clone() const
{
	return finishCloneJob(new ccDish(m_baseRadius,m_height,m_secondRadius,&m_transformation,getName(),m_drawPrecision));
}

bool ccDish::buildUp()
{
	if (m_drawPrecision < MIN_DRAWING_PRECISION)
		return false;

	if (m_height <= 0 || m_baseRadius <= 0 || m_secondRadius < 0) //invalid parameters
		return false;

	//section angular span
	double startAngle_rad = 0.0;
	const double endAngle_rad = M_PI/2.0;

	PointCoordinateType realRadius = m_baseRadius;
	if (m_secondRadius == 0 && m_height<m_baseRadius) //partial spherical mode
	{
		realRadius = (m_height*m_height+m_baseRadius*m_baseRadius)/(2*m_height);
		startAngle_rad = acos(m_baseRadius/realRadius);
		assert(startAngle_rad<endAngle_rad);
	}

	const unsigned steps = m_drawPrecision;
	double angleStep_rad = 2.0*M_PI/steps;
	unsigned sectionSteps = static_cast<unsigned>(ceil((endAngle_rad-startAngle_rad)*m_drawPrecision/(2.0*M_PI)));
	double sectionAngleStep_rad = (endAngle_rad-startAngle_rad)/sectionSteps;

	//vertices
	unsigned vertCount = steps*sectionSteps+1; //+1 for north pole
	//faces
	unsigned faceCount = steps*((sectionSteps-1)*2+1);

	if (!init(vertCount,true,faceCount,0))
	{
		ccLog::Error("[ccDish::buildUp] Not enough memory");
		return false;
	}

	//vertices
	ccPointCloud* verts = vertices();
	assert(verts);

	//first point: north pole
	verts->addPoint(CCVector3(0,0,m_height));
	verts->addNorm(CCVector3(0,0,1));

	//then, angular sweep
	{
		for (unsigned j=1; j<=sectionSteps; ++j)
		{
			PointCoordinateType theta = static_cast<PointCoordinateType>(endAngle_rad - j * sectionAngleStep_rad); //we start from north pole!
			PointCoordinateType cos_theta = cos(theta);
			PointCoordinateType sin_theta = sin(theta);

			CCVector3 N0(cos_theta, 0, sin_theta);
		
			for (unsigned i=0; i<steps; ++i) //then we make a full revolution
			{
				PointCoordinateType phi = static_cast<PointCoordinateType>(i * angleStep_rad);
				PointCoordinateType cos_phi = cos(phi);
				PointCoordinateType sin_phi = sin(phi);

				CCVector3 N(N0.x * cos_phi, N0.x * sin_phi, N0.z);
				N.normalize();

				CCVector3 P = N * realRadius;

				if (m_secondRadius > 0) //half-ellipsoid mode
				{
					P.y *= (m_secondRadius / m_baseRadius);
					P.z *= (m_height / m_baseRadius);
				}
				else //spherical section mode
				{
					P.z += m_height-realRadius;
				}

				verts->addPoint(P);
				verts->addNorm(N);
			}
		}
	}

	//faces
	{
		//north pole
		{
			for (unsigned i=0; i<steps; ++i)
			{
				unsigned A = 1+i;
				unsigned B = (i+1<steps ? A+1 : 1);
				addTriangle(A,B,0);
			}
		}

		//slices
		for (unsigned j=1; j<sectionSteps; ++j)
		{
			unsigned shift = 1+(j-1)*steps;		
			for (unsigned i=0; i<steps; ++i)
			{
				unsigned A = shift+i;
				unsigned B = (i+1<steps ? A+1 : shift);
				assert(B < vertCount);
				addTriangle(A,A+steps,B);
				addTriangle(B+steps,B,A+steps);
			}
		}
	}

	notifyGeometryUpdate();
	showNormals(true);

	return true;
}

bool ccDish::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=21)
	QDataStream outStream(&out);
	outStream << m_baseRadius;
	outStream << m_secondRadius;
	outStream << m_height;

	return true;
}

bool ccDish::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//parameters (dataVersion>=21)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_baseRadius);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_secondRadius);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_height);

	return true;
}
