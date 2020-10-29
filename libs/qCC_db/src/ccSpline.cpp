#include "../include/ccSpline.h"

//System
#include <cassert>
#include <algorithm>

ccSpline::ccSpline(	GenericIndexedCloudPersist* associatedCloud,
					size_t k/*=2*/,
					NodeType nodeType/*=OpenUniform*/,
					unsigned uniqueID/*=ccUniqueIDGenerator::InvalidUniqueID*/)
	: ccPolyline(associatedCloud, uniqueID)
	, m_nodeType(nodeType)
	, m_k(k)
{
	if (size() >= 2)
	{
		updateInternalState();
	}
}

ccSpline::ccSpline(	const ccPolyline& poly,
					size_t k/*=2*/,
					NodeType nodeType/*=OpenUniform*/)
	: ccPolyline(poly)
	, m_nodeType(nodeType)
	, m_k(k)
{
	if (size() >= 2)
	{
		updateInternalState();
	}
}

ccSpline::ccSpline(const ccSpline& spline)
	: ccPolyline(spline)
	, m_nodeType(spline.m_nodeType)
	, m_k(spline.m_k)
	, m_deltas(spline.m_deltas)
	, m_nodes(spline.m_nodes)
{
}


bool ccSpline::updateInternalState()
{
	m_deltas.clear();
	m_nodes.clear();

	unsigned vertexCount = size();
	if (vertexCount < 2)
	{
		ccLog::Warning("[Spline] Not enough vertices");
		return false;
	}

	try
	{
		m_deltas.resize(vertexCount - 1);
		for (unsigned i = 0; i < vertexCount - 1; ++i)
		{
			m_deltas[i] = *getPoint(i + 1) - *getPoint(i);
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}
	
	if (!setNodalVector())
	{
		return false;
	}
	assert(isValid());

	//for (size_t i = 0; i < m_nodes.size(); ++i)
	//{
	//	ccLog::Print(QString("Node %1 = %2").arg(i+1).arg(m_nodes[i]));
	//}

	for (size_t i = 0; i < m_deltas.size(); ++i)
	{
		m_deltas[i] = m_deltas[i] / static_cast<PointCoordinateType>(m_nodes[m_k + i] - m_nodes[i + 1]);
	}

	//for (size_t i = 0; i < m_deltas.size(); ++i)
	//{
	//	ccLog::Print(QString("Delta %1 = %2").arg(i + 1).arg(m_deltas[i].norm()));
	//}

	return true;
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

CCVector3 ccSpline::computePosition(double s) const
{
	s = std::max(std::min(s, 1.0), 0.0); // clamp between [0 1]
	return eval(s, m_k, m_nodes);
}

CCVector3 ccSpline::computeSpeed(double s) const
{
	s = std::max(std::min(s, 1.0), 0.0); // clamp between [0 1]
	return static_cast<PointCoordinateType>(m_k - 1) * eval(s, m_k - 1, m_nodes, &m_deltas, 1);
}

bool ccSpline::isValid() const
{
	return (m_k > 1)
		&& (size() >= m_k)
		&& (m_nodes.size() == m_k + size())
		&& (size() == m_deltas.size() + 1);
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
	if (size() < m_k || m_k < 2)
	{
		assert(false);
		return false;
	}
	
	try
	{
		m_nodes.resize(m_k + size());
	}
	catch (const std::bad_alloc&)
	{
		assert(false);
		return false;
	}

	double step = 1.0 / (size() + 1 - m_k);
	
	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i] = step * i - step * (m_k - 1);
	}

	return true;
}

bool ccSpline::setNodeToOpenUniform()
{
	if (size() < m_k || m_k < 2)
	{
		assert(false);
		return false;
	}

	try
	{
		m_nodes.resize(m_k + size());
	}
	catch (const std::bad_alloc&)
	{
		assert(false);
		return false;
	}

	double acc = 1.0;
	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		if (i < m_k)
		{
			m_nodes[i] = 0.0;
		}
		else if (i >= size() + 1)
		{
			m_nodes[i] = 1.0;
		}
		else
		{
			m_nodes[i] = acc / (size() + 1 - m_k);
			acc += 1.0;
		}
	}

	return true;
}

CCVector3 ccSpline::eval(	double s,
							size_t k,
							const std::vector<double>& nodes,
							const std::vector<CCVector3>* points/*=nullptr*/,
							size_t off/*=0*/) const
{
	assert(k > 1);
	assert(points ? points->size() >= k : size() >= static_cast<unsigned>(k));
	assert(isValid());
	
	// TODO: better search with binary search?
	size_t dec = 0;
	while ((dec + k + off < nodes.size()) && (dec + k < (points ? points->size() : size()) && (s > nodes[dec + k + off])))
	{
		++dec;
	}

	std::vector<CCVector3> points_rec(k);
	for (size_t i = 0; i < k; ++i)
	{
		points_rec[i] = points ? (*points)[i + dec] : *getPoint(static_cast<unsigned>(i + dec));
	}

	size_t nodesCount = 2 * k - 2;
	size_t nodesOff = dec + 1 + off;
	std::vector<double> nodes_rec(nodesCount, 0.0);
	for (size_t i = 0; i < nodesCount; ++i)
	{
		nodes_rec[i] = nodes[i + nodesOff];
	}

	return eval_rec(s, points_rec, k, nodes_rec);
}

CCVector3 ccSpline::eval_rec(	double s,
								const std::vector<CCVector3>& points,
								size_t k,
								const std::vector<double>& nodes) const
{
	if (points.size() == 1)
	{
		return points[0];
	}

	assert(k >= 2);
	std::vector<CCVector3> p_out(k - 1);
	for (size_t i = 0; i < k - 1; ++i)
	{
		const double n0 = nodes[i + k - 1];
		const double n1 = nodes[i];
		const PointCoordinateType f0 = static_cast<PointCoordinateType>((n0 - s) / (n0 - n1));
		const PointCoordinateType f1 = static_cast<PointCoordinateType>((s - n1) / (n0 - n1));

		p_out[i] = points[i] * f0 + points[i + 1] * f1;
	}

	std::vector<double> nodes_out(nodes.size() - 2);
	for (size_t i = 1, j = 0; i < nodes.size() - 1; ++i, ++j)
	{
		nodes_out[j] = nodes[i];
	}

	return eval_rec(s, p_out, k - 1, nodes_out);
}

bool ccSpline::toFile_MeOnly(QFile& out) const
{
	if (!ccPolyline::toFile_MeOnly(out))
		return false;

	// spline order
	uint32_t k = static_cast<uint32_t>(m_k);
	if (out.write((const char*)&m_k, 4) < 0)
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

	// spline order (dataVersion >= 51)
	{
		uint32_t k = 0;
		if (in.read((char*)&k, 4) < 0)
			return ReadError();
		m_k = k;
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
	//if (size() >= 2 && !updateInternalState())
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
		CCVector3 P = computePosition(s);
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
		//	CCVector3 P = computePosition(s);
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
