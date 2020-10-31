#include "../include/ccSpline.h"

#include <ccPointCloud.h>

//System
#include <cassert>
#include <algorithm>

ccSpline::ccSpline(	GenericIndexedCloudPersist* associatedCloud,
					unsigned order/*=3*/,
					unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccPolyline(associatedCloud, uniqueID)
	, m_degree(std::max<unsigned>(order, 2) - 1)
	, m_drawPrecision(10)
{
	if (order < 2)
	{
		assert(false);
		ccLog::Warning("Invalid input spline order (can't be below 2)");
	}
}

ccSpline::ccSpline(	const ccPolyline& poly,
					unsigned order/*=3*/,
					NodeType nodeType/*=OpenUniform*/)
	: ccPolyline(poly)
	, m_degree(std::max<unsigned>(order, 2) - 1)
	, m_drawPrecision(10)
{
	if (order < 2)
	{
		assert(false);
		ccLog::Warning("Invalid input spline order (can't be below 2)");
	}
	if (size() > m_degree)
	{
		initNodes(nodeType);
	}
}

ccSpline::ccSpline(const ccSpline& spline)
	: ccPolyline(spline)
	, m_degree(spline.m_degree)
	, m_nodes(spline.m_nodes)
	, m_drawPrecision(spline.m_drawPrecision)
{
}

unsigned ccSpline::expectedKnotCount() const
{
	return size() + m_degree + 1;
}

bool ccSpline::computePosition(double s, CCVector3& P) const
{
	if (!isValid())
	{
		//invalid spline
		assert(false);
		return false;
	}

	if (s < 0.0 || s > 1.0)
	{
		return false;
	}

	unsigned vertexCount = size();

	//remap the input (s) to the actual range
	//double low = m_nodes[m_degree];
	//double high = m_nodes[vertexCount];
	//s = s * (high - low) + low;

	//domain of interest spans from knot 'm_degree' to knot 'size()' 
	unsigned index = m_degree;
	for (; index < vertexCount; ++index)
	{
		if (s >= m_nodes[index] && s <= m_nodes[index + 1])
		{
			break;
		}
		if (index + 1 == vertexCount)
		{
			//cope with rounding issues
			if (std::abs(s - m_nodes[index + 1]) < 1.0e-6)
			{
				s = m_nodes[index + 1];
				break;
			}
			else
			{
				return false;
			}
		}
	}

	//we use the 'index' point and the 'm_degree' previous one
	unsigned i0 = index - m_degree;
	std::vector<CCVector3> v(m_degree + 1);
	for (unsigned i = i0; i <= index; ++i)
	{
		v[i - i0] = *getPoint(static_cast<unsigned>(i));
	}

	for (unsigned l = 0; l < m_degree; ++l)
	{
		// build level l of the pyramid
		unsigned offset = m_degree - l;
		for (unsigned i = index; i > index - offset; --i)
		{
			double alpha = (s - m_nodes[i]) / (m_nodes[i + offset] - m_nodes[i]);

			// interpolate each component
			v[i - i0] = (1.0 - alpha) * v[i - i0 - 1] + alpha * v[i - i0];
		}
	}
	
	P = v[m_degree];

	return true;
}

bool ccSpline::isValid() const
{
	return (m_degree != 0)
		&& (size() > m_degree)
		&& (m_nodes.size() == expectedKnotCount());
}

bool ccSpline::initNodes(NodeType nodeType)
{
	unsigned vertexCount = size();
	if (vertexCount <= m_degree)
	{
		ccLog::Warning("[Spline::setupNodes] Not enough vertices");
		m_nodes.clear();
		return false;
	}

	if (m_degree == 0 || size() <= m_degree)
	{
		ccLog::Warning("[Spline::setupNodes] Invalid degree or vertex count");
		assert(false);
		return false;
	}

	try
	{
		m_nodes.resize(expectedKnotCount(), 0.0);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("[Spline::setupNodes] Not enough memory");
		//not enough memory
		return false;
	}

	switch (nodeType)
	{
	case OpenUniform:
		setNodeToOpenUniform();
		break;
	case Uniform:
		setNodeToUniform();
		break;
	case Custom:
		//nothing to do
		break;
	default:
		assert(false);
		return false;
	}

#ifdef _DEBUG
	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		ccLog::Print(QString("Node %1 = %2").arg(i + 1).arg(m_nodes[i]));
	}
#endif

	return true;
}

void ccSpline::setNodeToUniform()
{
	assert(m_nodes.size() == expectedKnotCount());
	
	double step = static_cast<double>(size() - m_degree);

	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i] = (static_cast<int>(i) - static_cast<int>(m_degree)) / step;
	}
}

void ccSpline::setNodeToOpenUniform()
{
	assert(m_nodes.size() == expectedKnotCount());

	size_t internalNodeIndex = 0;
	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		if (i <= m_degree)
		{
			m_nodes[i] = 0.0;
		}
		else if (i > size())
		{
			m_nodes[i] = 1.0;
		}
		else
		{
			m_nodes[i] = static_cast<double>(++internalNodeIndex) / (size() - m_degree);
		}
	}
}

bool ccSpline::toFile_MeOnly(QFile& out) const
{
	if (!ccPolyline::toFile_MeOnly(out))
		return false;

	// spline degree
	uint32_t k = static_cast<uint32_t>(m_degree);
	if (out.write((const char*)&k, 4) < 0)
		return WriteError();
						
	// drawing precision
	uint32_t prec = static_cast<uint32_t>(m_drawPrecision);
	if (out.write((const char*)&prec, 4) < 0)
		return WriteError();

	// node values
	if (!ccSerializationHelper::GenericArrayToFile<double, 1, double>(m_nodes, out))
		return false;

	return true;
}

bool ccSpline::fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (!ccPolyline::fromFile_MeOnly(in, dataVersion, flags, oldToNewIDMap))
		return false;

	// spline degree (dataVersion >= 51)
	{
		uint32_t d = 0;
		if (in.read((char*)&d, 4) < 0)
			return ReadError();
		if (d == 0)
		{
			ccLog::Warning("[ccSpline::fromFile] Invalid spline degree value (can't be zero)");
			return false;
		}
		m_degree = d;
	}

	// drawing precision (dataVersion >= 51)
	{
		uint32_t prec = 0;
		if (in.read((char*)&prec, 4) < 0)
			return ReadError();
		if (prec < MIN_DRAWING_PRECISION)
		{
			ccLog::Warning("[ccSpline::fromFile] Invalid drawing precision value (too small)");
			//return false; nothing serious though
		}
		setDrawingPrecision(static_cast<unsigned>(prec));
	}

	// node values (dataVersion >= 51)
	if (!ccSerializationHelper::GenericArrayFromTypedFile<double, 1, double, double>(m_nodes, in, dataVersion))
	{
		return false;
	}

	return true;
}

void ccSpline::drawMeOnly(CC_DRAW_CONTEXT& context)
{
	unsigned vertCount = size();
	if (vertCount < 2)
		return;

	if (!isValid())
		return;

	bool draw = false;

	if (MACRO_Draw3D(context))
	{
		draw = !m_mode2D;
	}
	else if (m_mode2D)
	{
		bool drawFG = MACRO_Foreground(context);
		draw = ((drawFG && m_foreground) || (!drawFG && !m_foreground));
	}

	if (!draw)
		return;

	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1 *glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	//standard case: list names pushing
	bool pushName = MACRO_DrawEntityNames(context);
	if (pushName)
		glFunc->glPushName(getUniqueIDForDisplay());

	if (isColorOverriden())
		ccGL::Color4v(glFunc, getTempColor().rgba);
	else if (colorsShown())
		ccGL::Color3v(glFunc, m_rgbColor.rgb);

	//display polyline
	if (m_width != 0)
	{
		glFunc->glPushAttrib(GL_LINE_BIT);
		glFunc->glLineWidth(static_cast<GLfloat>(m_width));
	}

	//DGM: we do the 'GL_LINE_LOOP' manually as I have a strange bug
	//on one on my graphic card with this mode!
	//glBegin(m_isClosed ? GL_LINE_LOOP : GL_LINE_STRIP);
	glFunc->glBegin(GL_LINE_STRIP);
	CCVector3 firstPoint;
	bool firstPointRecorded = false;
	for (unsigned i = m_degree; i <vertCount; ++i)
	{
		double start = m_nodes[i];
		double stop = m_nodes[i + 1];
		double delta = stop - start;
		if (delta < 1.0e-6)
			continue;
		for (unsigned j = 0; j < m_drawPrecision; ++j)
		{
			CCVector3 P;
			if (!computePosition(start + (j * delta) / (m_drawPrecision - 1), P))
			{
				continue;
			}
			ccGL::Vertex3v(glFunc, P.u);

			if (!firstPointRecorded)
			{
				firstPoint = P;
				firstPointRecorded = true;
			}
		}
	}
	if (m_isClosed && firstPointRecorded)
	{
		ccGL::Vertex3v(glFunc, firstPoint.u);
	}
	glFunc->glEnd();

	if (m_width != 0)
	{
		glFunc->glPopAttrib();
	}

	//display the control points
	if (m_showVertices)
	{
		glFunc->glPushAttrib(GL_POINT_BIT);
		glFunc->glPointSize(static_cast<GLfloat>(m_vertMarkWidth));

		glFunc->glBegin(GL_POINTS);
		for (unsigned i = 0; i < vertCount; ++i)
		{
			ccGL::Vertex3v(glFunc, getPoint(i)->u);
		}
		glFunc->glEnd();

		glFunc->glPopAttrib();
	}

	if (pushName)
		glFunc->glPopName();
}

ccPolyline* ccSpline::toPoly() const
{
	if (!isValid())
	{
		return nullptr;
	}

	ccPointCloud* vertices = new ccPointCloud("vertices");
	{
		unsigned controlPointCount = size();
		unsigned vertCount = (controlPointCount - m_degree) * m_drawPrecision;

		if (!vertices->reserve(vertCount))
		{
			//not enough memory
			delete vertices;
			return nullptr;
		}
		
		CCVector3 firstPoint;
		bool firstPointRecorded = false;
		for (unsigned i = m_degree; i < controlPointCount; ++i)
		{
			double start = m_nodes[i];
			double stop = m_nodes[i + 1];
			double delta = stop - start;
			if (delta < 1.0e-6)
				continue;
			for (unsigned j = 0; j < m_drawPrecision; ++j)
			{
				CCVector3 P;
				if (!computePosition(start + (j * delta) / (m_drawPrecision - 1), P))
				{
					continue;
				}
				vertices->addPoint(P);

				if (!firstPointRecorded)
				{
					firstPoint = P;
					firstPointRecorded = true;
				}
			}
		}
		if (m_isClosed && firstPointRecorded)
		{
			vertices->addPoint(firstPoint);
		}

		vertices->shrinkToFit();
		vertices->setGLTransformationHistory(getGLTransformationHistory());
		vertices->setGlobalShift(getGlobalShift());
		vertices->setGlobalScale(getGlobalScale());
	}
	
	ccPolyline* poly = new ccPolyline(vertices);
	poly->addChild(vertices);
	vertices->setEnabled(false);
	if (!poly->addPointIndex(0, vertices->size()))
	{
		delete poly;
		return nullptr;
	}
	poly->importParametersFrom(*this);

	return poly;
}