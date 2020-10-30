#include "../include/ccSpline.h"

//System
#include <cassert>
#include <algorithm>

ccSpline::ccSpline(	GenericIndexedCloudPersist* associatedCloud,
					size_t order/*=3*/,
					NodeType nodeType/*=OpenUniform*/,
					unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccPolyline(associatedCloud, uniqueID)
	, m_nodeType(nodeType)
	, m_degree(std::max<size_t>(order, 2) - 1)
{
	if (order < 2)
	{
		assert(false);
		ccLog::Warning("Invalid input spline order (can't be below 2)");
	}
	if (size() > m_degree)
	{
		updateInternalState();
	}
}

ccSpline::ccSpline(	const ccPolyline& poly,
					size_t order/*=3*/,
					NodeType nodeType/*=OpenUniform*/)
	: ccPolyline(poly)
	, m_nodeType(nodeType)
	, m_degree(std::max<size_t>(order, 2) - 1)
{
	if (order < 2)
	{
		assert(false);
		ccLog::Warning("Invalid input spline order (can't be below 2)");
	}
	if (size() > m_degree)
	{
		updateInternalState();
	}
}

ccSpline::ccSpline(const ccSpline& spline)
	: ccPolyline(spline)
	, m_nodeType(spline.m_nodeType)
	, m_degree(spline.m_degree)
	, m_nodes(spline.m_nodes)
{
}

bool ccSpline::updateInternalState()
{
	m_nodes.clear();

	unsigned vertexCount = size();
	if (vertexCount <= m_degree)
	{
		ccLog::Warning("[Spline] Not enough vertices");
		return false;
	}

	if (!setNodalVector())
	{
		return false;
	}
	assert(isValid());

	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		ccLog::Print(QString("Node %1 = %2").arg(i+1).arg(m_nodes[i]));
	}

	return true;
}

size_t ccSpline::expectedKnotCount() const
{
	return size() + m_degree + 1;
}

bool ccSpline::setNodeType(NodeType type)
{
	m_nodeType = type;
	if (setNodalVector())
	{
		assert(isValid());
		return true;
	}
	else
	{
		return false;
	}
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

	size_t vertexCount = size();

	//domain of interest spans from knot 'm_degree' to knot 'size()' 
	size_t index = m_degree;
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
	size_t i0 = index - m_degree;
	std::vector<CCVector3> v(m_degree + 1);
	for (size_t i = i0; i <= index; ++i)
	{
		v[i - i0] = *getPoint(static_cast<unsigned>(i));
	}

	for (size_t l = 0; l < m_degree; ++l)
	{
		// build level l of the pyramid
		size_t offset = m_degree - l;
		for (size_t i = index; i > index - offset; --i)
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

bool ccSpline::setNodalVector()
{
	switch (m_nodeType)
	{
	case OpenUniform:
		return setNodeToOpenUniform();
	case Uniform:
		return setNodeToUniform();
	default:
		assert(false);
		return false;
	}
}

bool ccSpline::setNodeToUniform()
{
	if (m_degree == 0 || size() <= m_degree)
	{
		assert(false);
		return false;
	}
	
	try
	{
		m_nodes.resize(expectedKnotCount());
	}
	catch (const std::bad_alloc&)
	{
		assert(false);
		return false;
	}

	double step = static_cast<double>(size() - m_degree);

	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i] = (static_cast<int>(i) - static_cast<int>(m_degree)) / step;
	}

	return true;
}

bool ccSpline::setNodeToOpenUniform()
{
	if (m_degree == 0 || size() <= m_degree)
	{
		assert(false);
		return false;
	}

	try
	{
		m_nodes.resize(expectedKnotCount());
	}
	catch (const std::bad_alloc&)
	{
		assert(false);
		return false;
	}

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

	return true;
}

bool ccSpline::toFile_MeOnly(QFile& out) const
{
	if (!ccPolyline::toFile_MeOnly(out))
		return false;

	// spline degree
	uint32_t k = static_cast<uint32_t>(m_degree);
	if (out.write((const char*)&m_degree, 4) < 0)
		return WriteError();
						
	// node type
	uint32_t nodeType = static_cast<uint32_t>(m_nodeType);
	if (out.write((const char*)&nodeType, 4) < 0)
		return WriteError();

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

	// node type (dataVersion >= 51)
	{
		uint32_t nodeType = 0;
		if (in.read((char*)&nodeType, 4) < 0)
			return ReadError();
		if (nodeType == OpenUniform)
		{
			m_nodeType = OpenUniform;
		}
		else if (nodeType == Uniform)
		{
			m_nodeType = Uniform;
		}
		else
		{
			ccLog::Error(QString("Unknown node type (%1)").arg(nodeType));
			return false;
		}
	}

	// DGM: we can't update the polyline right now as the vertices are not attached yet!
	//if (size() > m_degree && !updateInternalState())
	//{
	//	return false;
	//}

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

	unsigned displayVertCount = vertCount * 10; //FIXME

	//DGM: we do the 'GL_LINE_LOOP' manually as I have a strange bug
	//on one on my graphic card with this mode!
	//glBegin(m_isClosed ? GL_LINE_LOOP : GL_LINE_STRIP);
	glFunc->glBegin(GL_LINE_STRIP);
	double s = 0.0;
	double step = 1.0 / displayVertCount;
	CCVector3 firstPoint;
	for (unsigned i = 0; i <= displayVertCount; ++i)
	{
		CCVector3 P;
		if (!computePosition(s, P))
		{
			continue;
		}
		s += step;
		ccGL::Vertex3v(glFunc, P.u);

		if (i == 0)
			firstPoint = P;
	}
	if (m_isClosed)
	{
		ccGL::Vertex3v(glFunc, firstPoint.u);
	}
	glFunc->glEnd();

	if (m_width != 0)
	{
		glFunc->glPopAttrib();
	}

	//display the real vertices
	if (m_showVertices)
	{
		glFunc->glPushAttrib(GL_POINT_BIT);
		glFunc->glPointSize(static_cast<GLfloat>(m_vertMarkWidth));

		glFunc->glBegin(GL_POINTS);
		//double s = 0.0;
		//double step = 1.0 / displayVertCount;
		//CCVector3 firstPoint;
		//for (unsigned i = 0; i <= displayVertCount; ++i)
		//{
		//	CCVector3 P;
		//	if (!computePosition(s, P))
		//	{
		//		continue;
		//	}
		//	s += step;
		//	ccGL::Vertex3v(glFunc, P.u);
		//}
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
