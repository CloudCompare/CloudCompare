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
#include "Neighbourhood.h"

//system
#include <string.h>

ccQuadric::ccQuadric(	CCVector2 minCorner,
						CCVector2 maxCorner,
						const PointCoordinateType eq[6],
						const CCCoreLib::SquareMatrix* toLocalOrientation/*=nullptr*/,
						const ccGLMatrix* transMat/*=nullptr*/,
						QString name/*=QString("Quadric")*/,
						unsigned precision/*=DEFAULT_DRAWING_PRECISION*/)
	: ccGenericPrimitive(name, transMat)
	, m_minCorner(minCorner)
	, m_maxCorner(maxCorner)
	, m_minZ(0)
	, m_maxZ(0)
{
	memcpy(m_eq, eq, sizeof(PointCoordinateType) * 6);

	if (toLocalOrientation)
	{
		m_toLocalOrientation = *toLocalOrientation;
	}

	setDrawingPrecision(std::max<unsigned>(precision,MIN_DRAWING_PRECISION));  //automatically calls updateRepresentation
}

ccQuadric::ccQuadric(QString name /*=QString("Plane")*/)
	: ccGenericPrimitive(name)
	, m_minCorner(0,0)
	, m_maxCorner(0,0)
	, m_minZ(0)
	, m_maxZ(0)
{}

bool ccQuadric::buildUp()
{
	if (m_drawPrecision < MIN_DRAWING_PRECISION)
		return false;

	unsigned vertCount = m_drawPrecision * m_drawPrecision;
	unsigned triCount = (m_drawPrecision - 1) * (m_drawPrecision - 1) * 2;
	if (!init(vertCount, true, triCount, 0))
	{
		ccLog::Error("[ccQuadric::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(verts->hasNormals());

	CCVector2 areaSize = m_maxCorner - m_minCorner;
	PointCoordinateType stepX = areaSize.x / static_cast<PointCoordinateType>(m_drawPrecision - 1);
	PointCoordinateType stepY = areaSize.y / static_cast<PointCoordinateType>(m_drawPrecision - 1);

	for (unsigned x = 0; x < m_drawPrecision; ++x)
	{
		CCVector3 P(m_minCorner.x + stepX * x, 0, 0);
		for (unsigned y = 0; y < m_drawPrecision; ++y)
		{
			P.y = m_minCorner.y + stepY * y;

			P.z = m_eq[0]
				+ m_eq[1] * P.x
				+ m_eq[2] * P.y
				+ m_eq[3] * P.x * P.x
				+ m_eq[4] * P.x * P.y
				+ m_eq[5] * P.y * P.y;

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
				unsigned iA = (x - 1) * m_drawPrecision + y - 1;
				unsigned iB = iA + 1;
				unsigned iC = iA + m_drawPrecision;
				unsigned iD = iB + m_drawPrecision;

				addTriangle(iA, iC, iB);
				addTriangle(iB, iC, iD);
			}
		}
	}

	computeNormals(true);

	return true;
}

ccGenericPrimitive* ccQuadric::clone() const
{
	return finishCloneJob(new ccQuadric(m_minCorner,
										m_maxCorner,
										m_eq,
										m_toLocalOrientation.size() != 0 ? &m_toLocalOrientation : nullptr,
										&m_transformation,
										getName(),
										m_drawPrecision)
	);
}

ccQuadric* ccQuadric::Fit(CCCoreLib::GenericIndexedCloudPersist *cloud, double* rms/*=nullptr*/)
{
	//number of points
	unsigned count = cloud->size();
	if (count < CCCoreLib::CC_LOCAL_MODEL_MIN_SIZE[CCCoreLib::QUADRIC])
	{
		ccLog::Warning(QString("[ccQuadric::fitTo] Not enough points in input cloud to fit a quadric! (%1 at the very least are required)").arg(CCCoreLib::CC_LOCAL_MODEL_MIN_SIZE[CCCoreLib::QUADRIC]));
		return nullptr;
	}

	CCCoreLib::Neighbourhood Zk(cloud);

	CCCoreLib::SquareMatrix toLocalOrientation;
	const PointCoordinateType* eq = Zk.getQuadric(&toLocalOrientation);
	if (!eq)
	{
		ccLog::Warning("[ccQuadric::Fit] Failed to fit a quadric!");
		return nullptr;
	}

	//we recenter the quadric object
	CCCoreLib::SquareMatrix globalOrientation = toLocalOrientation.transposed(); // transposed is equivalent to inverse for a 3x3 rotation matrix
	CCVector3 X(globalOrientation.getValue(0, 0),
				globalOrientation.getValue(1, 0),
				globalOrientation.getValue(2, 0));
	CCVector3 Y(globalOrientation.getValue(0, 1),
				globalOrientation.getValue(1, 1),
				globalOrientation.getValue(2, 1));
	CCVector3 N(globalOrientation.getValue(0, 2),
				globalOrientation.getValue(1, 2),
				globalOrientation.getValue(2, 2));
	const CCVector3* G = Zk.getGravityCenter();
	ccGLMatrix glMat(X, Y, N, *G);

	if (rms)
	{
		*rms = 0.0;
	}

	ccBBox localBB;
	for (unsigned i = 0; i < cloud->size(); ++i)
	{
		CCVector3 Plocal = toLocalOrientation * (*cloud->getPoint(i) - *G);
		localBB.add(Plocal);

		if (rms)
		{
			//compute rms if necessary
			PointCoordinateType z =   eq[0]
									+ eq[1] * Plocal.x
									+ eq[2] * Plocal.y
									+ eq[3] * Plocal.x * Plocal.x
									+ eq[4] * Plocal.x * Plocal.y
									+ eq[5] * Plocal.y * Plocal.y;

			*rms += pow(static_cast<double>(z - Plocal.z), 2.0);
		}
	}

	if (rms)
	{
		if (count)
		{
			*rms = sqrt(*rms / count);
		}
		else
		{
			*rms = std::numeric_limits<double>::quiet_NaN();
		}
	}

	CCVector2 minXY(localBB.minCorner().x, localBB.minCorner().y);
	CCVector2 maxXY(localBB.maxCorner().x, localBB.maxCorner().y);

	ccQuadric* quadric = new ccQuadric(minXY, maxXY, eq, &toLocalOrientation, &glMat);

	quadric->setMetaData(QString("Equation"), QVariant(quadric->getEquationString()));
	if (rms)
	{
		quadric->setMetaData("RMS", *rms);
	}

	return quadric;
}

PointCoordinateType ccQuadric::projectOnQuadric(const CCVector3& P, CCVector3& Q) const
{
	//back project into quadric coordinate system
	Q = P;
	m_transformation.inverse().apply(Q);

	CCVector3 QLocal = m_toLocalOrientation * Q;

	PointCoordinateType originalZ = QLocal.z;
	QLocal.z =    m_eq[0]
				+ m_eq[1] * QLocal.x
				+ m_eq[2] * QLocal.y
				+ m_eq[3] * QLocal.x*QLocal.x
				+ m_eq[4] * QLocal.x*QLocal.y
				+ m_eq[5] * QLocal.y*QLocal.y;

	Q = m_toLocalOrientation.inv() * QLocal;

	m_transformation.apply(Q);

	return originalZ - QLocal.z;
}

QString ccQuadric::getEquationString() const
{
	QString equationStr = QString("X = %1 + %2 * X").arg(m_eq[0]).arg(m_eq[1]);
	equationStr += QString(" + %1 * Y + %2 * X^2").arg(m_eq[2]).arg(m_eq[3]);
	equationStr += QString(" + %1 * X*Y + %2 * Y^2").arg(m_eq[4]).arg(m_eq[5]);

	return equationStr;
}

bool ccQuadric::toFile_MeOnly(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 35)
	{
		assert(false);
		return false;
	}

	if (!ccGenericPrimitive::toFile_MeOnly(out, dataVersion))
	{
		return false;
	}

	//parameters (dataVersion>=35)
	QDataStream outStream(&out);
	outStream << m_minCorner.x;
    outStream << m_minCorner.y;
	outStream << m_maxCorner.x;
	outStream << m_maxCorner.y;

	for (unsigned i = 0; i < 6; ++i)
	{
		outStream << m_eq[i];
	}

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
	{
		ccSerializationHelper::CoordsFromDataStream(inStream, flags, m_eq + i, 1);
	}

	return true;
}

short ccQuadric::minimumFileVersion_MeOnly() const
{
	return std::max(static_cast<short>(35), ccGenericPrimitive::minimumFileVersion_MeOnly());
}

ccBBox ccQuadric::getOwnFitBB(ccGLMatrix& trans)
{
	trans = m_transformation;
	return ccBBox(CCVector3(m_minCorner.x, m_minCorner.y, m_minZ), CCVector3(m_maxCorner.x, m_maxCorner.y, m_maxZ), true);
}
