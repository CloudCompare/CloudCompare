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

//Always on top!
#include "ccIncludeGL.h"

#include "ccQuadric.h"

//qCC_db
#include "ccPointCloud.h"
#include "ccNormalVectors.h"
#include "ccMaterialSet.h"

//CCLIB
#include "DistanceComputationTools.h"

//system
#include <string.h>

ccQuadric::ccQuadric(	CCVector2 minCorner,
						CCVector2 maxCorner,
						const PointCoordinateType eq[6],
						const Tuple3ub* dims/*=0*/,
						const ccGLMatrix* transMat/*=0*/,
						QString name/*=QString("Quadric")*/,
						unsigned precision/*=DEFAULT_DRAWING_PRECISION*/)
	: ccGenericPrimitive(name,transMat)
	, m_minCorner(minCorner)
	, m_maxCorner(maxCorner)
	, m_dims(0,1,2)
	, m_minZ(0)
	, m_maxZ(0)
{
	memcpy(m_eq,eq,sizeof(PointCoordinateType)*6);

	if (dims)
	{
		m_dims = *dims;
	}
	
	setDrawingPrecision(std::max<unsigned>(precision,MIN_DRAWING_PRECISION));  //automatically calls updateRepresentation
}

ccQuadric::ccQuadric(QString name /*=QString("Plane")*/)
	: ccGenericPrimitive(name)
	, m_minCorner(0,0)
	, m_maxCorner(0,0)
	, m_dims(0,1,2)
	, m_minZ(0)
	, m_maxZ(0)
{}

bool ccQuadric::buildUp()
{
	if (m_drawPrecision < MIN_DRAWING_PRECISION)
		return false;

	unsigned vertCount = m_drawPrecision*m_drawPrecision;
	unsigned triCount = (m_drawPrecision-1)*(m_drawPrecision-1)*2;
	if (!init(vertCount,true,triCount,0))
	{
		ccLog::Error("[ccQuadric::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(verts->hasNormals());

	CCVector2 areaSize = m_maxCorner - m_minCorner;
	PointCoordinateType stepX = areaSize.x/static_cast<PointCoordinateType>(m_drawPrecision-1);
	PointCoordinateType stepY = areaSize.y/static_cast<PointCoordinateType>(m_drawPrecision-1);

	for (unsigned x=0; x<m_drawPrecision; ++x)
	{
		CCVector3 P(m_minCorner.x + stepX * x, 0, 0);
		for (unsigned y=0; y<m_drawPrecision; ++y)
		{
			P.y = m_minCorner.y + stepY * y;
			
			P.z = m_eq[0] + m_eq[1]*P.x + m_eq[2]*P.y + m_eq[3]*P.x*P.x + m_eq[4]*P.x*P.y + m_eq[5]*P.y*P.y;

			//compute the min and max heights of the quadric!
			if (x != 0 || y != 0)
			{
				if (m_minZ > P.z)
					m_minZ = P.z;
				else if (m_maxZ < P.z)
					m_maxZ = P.z;
			}
			else
			{
				m_minZ = m_maxZ = P.z;
			}

			verts->addPoint(P);

			//normal --> TODO: is there a simple way to deduce it from the equation?
			//CCVector3 N;
			//N.x = m_eq[1] + 2*m_eq[3]*P.x + m_eq[4]*P.y;
			//N.y = m_eq[2] + 2*m_eq[5]*P.y + m_eq[4]*P.x;
			//N.z = -PC_ONE;
			//N.normalize();
			//verts->addNorm(N);

			if (x != 0 && y != 0)
			{
				unsigned iA = (x-1) * m_drawPrecision + y-1;
				unsigned iB = iA + 1;
				unsigned iC = iA + m_drawPrecision;
				unsigned iD = iB + m_drawPrecision;

				addTriangle(iA,iC,iB);
				addTriangle(iB,iC,iD);
			}
		}
	}

	computeNormals(true);

	return true;
}

ccGenericPrimitive* ccQuadric::clone() const
{
	return finishCloneJob(new ccQuadric(m_minCorner,m_maxCorner,m_eq,&m_dims,&m_transformation,getName(),m_drawPrecision));
}

ccQuadric* ccQuadric::Fit(CCLib::GenericIndexedCloudPersist *cloud, double* rms/*=0*/)
{
	//number of points
	unsigned count = cloud->size();
	if (count < CC_LOCAL_MODEL_MIN_SIZE[QUADRIC])
	{
		ccLog::Warning(QString("[ccQuadric::fitTo] Not enough points in input cloud to fit a quadric! (%1 at the very least are required)").arg(CC_LOCAL_MODEL_MIN_SIZE[QUADRIC]));
		return nullptr;
	}

	//project the points on a 2D plane
	CCVector3 G;
	CCVector3 X;
	CCVector3 Y;
	CCVector3 N;
	{
		CCLib::Neighbourhood Yk(cloud);
		
		//plane equation
		const PointCoordinateType* theLSPlane = Yk.getLSPlane();
		if (!theLSPlane)
		{
			ccLog::Warning("[ccQuadric::Fit] Not enough points to fit a quadric!");
			return nullptr;
		}

		assert(Yk.getGravityCenter());
		G = *Yk.getGravityCenter();

		//local base
		N = CCVector3(theLSPlane);
		assert(Yk.getLSPlaneX() && Yk.getLSPlaneY());
		X = *Yk.getLSPlaneX(); //main direction
		Y = *Yk.getLSPlaneY(); //secondary direction
	}

	//project the points in a temporary cloud
	ccPointCloud tempCloud("temporary");
	if (!tempCloud.reserve(count))
	{
		ccLog::Warning("[ccQuadric::Fit] Not enough memory!");
		return nullptr;
	}

	cloud->placeIteratorAtBeginning();
	for (unsigned k=0; k<count; ++k)
	{
		//projection into local 2D plane ref.
		CCVector3 P = *(cloud->getNextPoint()) - G;

		tempCloud.addPoint(CCVector3(P.dot(X),P.dot(Y),P.dot(N)));
	}

	CCLib::Neighbourhood Zk(&tempCloud);
	{
		//set exact values for gravity center and plane equation
		//(just to be sure and to avoid re-computing them)
		Zk.setGravityCenter(CCVector3(0,0,0));
		PointCoordinateType perfectEq[4] = { 0, 0, 1, 0 };
		Zk.setLSPlane(	perfectEq,
						CCVector3(1,0,0),
						CCVector3(0,1,0),
						CCVector3(0,0,1));
	}

	Tuple3ub dims;
	const PointCoordinateType* eq = Zk.getQuadric(&dims);
	if (!eq)
	{
		ccLog::Warning("[ccQuadric::Fit] Failed to fit a quadric!");
		return nullptr;
	}

	//we recenter the quadric object
	ccGLMatrix glMat(X,Y,N,G);

	ccBBox bb = tempCloud.getOwnBB();
	CCVector2 minXY(bb.minCorner().x,bb.minCorner().y);
	CCVector2 maxXY(bb.maxCorner().x,bb.maxCorner().y);

	ccQuadric* quadric = new ccQuadric(minXY, maxXY, eq, &dims, &glMat);

	quadric->setMetaData(QString("Equation"),QVariant(quadric->getEquationString()));

	//compute rms if necessary
	if (rms)
	{
		const unsigned char dX = dims.x;
		const unsigned char dY = dims.y;
		//const unsigned char dZ = dims.z;

		*rms = 0;

		for (unsigned k=0; k<count; ++k)
		{
			//projection into local 2D plane ref.
			const CCVector3* P = tempCloud.getPoint(k);

			PointCoordinateType z = eq[0] + eq[1]*P->u[dX] + eq[2]*P->u[dY] + eq[3]*P->u[dX]*P->u[dX] + eq[4]*P->u[dX]*P->u[dY] + eq[5]*P->u[dY]*P->u[dY];
			*rms += (z-P->z)*(z-P->z);
		}

		if (count)
		{
			*rms = sqrt(*rms / count);
			quadric->setMetaData(QString("RMS"),QVariant(*rms));
		}
	}
	
	return quadric;
}

PointCoordinateType ccQuadric::projectOnQuadric(const CCVector3& P, CCVector3& Q) const
{
	//back project into quadric coordinate system
	Q = P;
	m_transformation.inverse().apply(Q);

	const unsigned char dX = m_dims.x;
	const unsigned char dY = m_dims.y;
	const unsigned char dZ = m_dims.z;

	PointCoordinateType originalZ = Q.u[dZ];
	Q.u[dZ] = m_eq[0] + m_eq[1]*Q.u[dX] + m_eq[2]*Q.u[dY] + m_eq[3]*Q.u[dX]*Q.u[dX] + m_eq[4]*Q.u[dX]*Q.u[dY] + m_eq[5]*Q.u[dY]*Q.u[dY];

	m_transformation.apply(Q);

	return originalZ - Q.u[dZ];
}

QString ccQuadric::getEquationString() const
{
	const unsigned char dX = m_dims.x;
	const unsigned char dY = m_dims.y;
	const unsigned char dZ = m_dims.z;
	static const char dimChars[3] = {'x','y','z'};

	QString equationStr = QString("%1 = %2 + %3 * %4").arg(dimChars[dZ]).arg(m_eq[0]).arg(m_eq[1]).arg(dimChars[dX]);
	equationStr += QString(" + %1 * %2 + %3 * %4^2").arg(m_eq[2]).arg(dimChars[dY]).arg(m_eq[3]).arg(dimChars[dX]);
	equationStr += QString(" + %1 * %2*%3 + %4 * %5^2").arg(m_eq[4]).arg(dimChars[dX]).arg(dimChars[dY]).arg(m_eq[5]).arg(dimChars[dY]);

	return equationStr;
}

bool ccQuadric::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return false;

	//parameters (dataVersion>=35)
	QDataStream outStream(&out);
	outStream << m_minCorner.x;
    outStream << m_minCorner.y;
	outStream << m_maxCorner.x;
    outStream << m_maxCorner.y;

	for (unsigned i=0; i<6; ++i)
		outStream << m_eq[i];

	return true;
}

bool ccQuadric::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	//parameters (dataVersion>=35)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_minCorner.x, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_minCorner.y, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_maxCorner.x, 1);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_maxCorner.y, 1);

	for (unsigned i = 0; i < 6; ++i)
		ccSerializationHelper::CoordsFromDataStream(inStream, flags, m_eq + i, 1);

	return true;
}

ccBBox ccQuadric::getOwnFitBB(ccGLMatrix& trans)
{
	trans = m_transformation;
	return ccBBox(CCVector3(m_minCorner.x, m_minCorner.y, m_minZ), CCVector3(m_maxCorner.x, m_maxCorner.y, m_maxZ));
}
