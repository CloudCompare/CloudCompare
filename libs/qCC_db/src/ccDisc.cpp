// qCC_db
#include<ccPointCloud.h>

#include "ccDisc.h"

ccDisc::ccDisc(PointCoordinateType radius,
			   PointCoordinateType xOff /*=0*/,
			   PointCoordinateType yOff /*=0*/,
			   const ccGLMatrix* transMat  /*= nullptr*/,
			   QString name /*= QString("Disc")*/,
			   unsigned precision /*= DEFAULT_DRAWING_PRECISION*/,
			   unsigned uniqueID  /*= ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccGenericPrimitive(name, transMat, uniqueID)
	, m_radius(std::abs(radius))
	, m_xOff(xOff)
	, m_yOff(yOff)
{
	setDrawingPrecision(std::max<unsigned>(precision, MIN_DRAWING_PRECISION)); // automatically calls buildUp & applyTransformationToVertices
}

ccDisc::ccDisc(QString name /*="Cylinder"*/)
	: ccGenericPrimitive(name)
	, m_radius(0)
	, m_xOff(0)
	, m_yOff(0)
{
}

ccGenericPrimitive* ccDisc::clone() const
{
	return finishCloneJob(new ccDisc(m_radius, m_xOff, m_yOff, &m_transformation, getName(), m_drawPrecision));
}

bool ccDisc::buildUp()
{
	if (m_drawPrecision < MIN_DRAWING_PRECISION)
		return false;

	// invalid dimensions?
	if (CCCoreLib::LessThanEpsilon(m_radius))
	{
		return false;
	}

	unsigned steps = m_drawPrecision;

	// vertices
	unsigned vertCount = steps + 1; // At least the center

	// normals
	unsigned faceNormCounts = 1;
	// faces
	unsigned facesCount = steps;

	// allocate (& clear) structures
	if (!init(vertCount, false, facesCount, faceNormCounts))
	{
		ccLog::Error("[ccCone::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	// first point: center of the  surface
	CCVector3 center = CCVector3(m_xOff, m_yOff, 0) / 2;
	// add center to the vertices
	verts->addPoint(center);
	CompressedNormType nIndex = ccNormalVectors::GetNormIndex(CCVector3(0, 0, 1).u);
	m_triNormals->addElement(nIndex);

	// then, angular sweep for the surface
	PointCoordinateType angle_rad_step = static_cast<PointCoordinateType>(2.0 * M_PI) / static_cast<PointCoordinateType>(steps);
		// bottom surface
	for (unsigned i = 0; i < steps; ++i)
	{
		CCVector3 P(center.x + cos(angle_rad_step * i) * m_radius,
					center.y + sin(angle_rad_step * i) * m_radius,
					center.z);
		verts->addPoint(P);
	}

	// mesh faces
	assert(m_triVertIndexes);

	// surface
	for (unsigned i = 0; i < steps; ++i)
	{
		unsigned i2 = 1 + i;
		unsigned i3 = 1 + (i + 1) % steps;
		addTriangle(0, i2, i3);
		addTriangleNormalIndexes(0, 0, 0);
	}

	notifyGeometryUpdate();
	showTriNorms(true);

	return true;
}

bool ccDisc::toFile_MeOnly(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 45)
	{
		assert(false);
		return false;
	}

	if (!ccGenericPrimitive::toFile_MeOnly(out, dataVersion))
	{
		return false;
	}

	// parameters (dataVersion>=45)
	QDataStream outStream(&out);
	outStream << m_radius;
	outStream << m_xOff;
	outStream << m_yOff;

	return true;
}

bool ccDisc::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	// parameters (dataVersion>=45)
	QDataStream inStream(&in);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_radius);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_xOff);
	ccSerializationHelper::CoordsFromDataStream(inStream, flags, &m_yOff);

	return true;
}

short ccDisc::minimumFileVersion_MeOnly() const
{
	return std::max(static_cast<short>(45), ccGenericPrimitive::minimumFileVersion_MeOnly());
}
