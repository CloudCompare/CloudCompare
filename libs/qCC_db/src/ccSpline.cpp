#include "../include/ccSpline.h"

//System
#include <cassert>
#include <algorithm>

ccSpline::ccSpline(size_t k, NodeType nodeType)
	: m_nodeType(nodeType)
	, m_k(k)
	, m_points(m_k)
	, m_deltas(m_k > 1 ? m_k - 1 : 0)
	, m_nodes(m_k + m_points.size())
{
	assert(isValid());
}

void ccSpline::setControlPoints(const std::vector<CCVector3>& points)
{
	m_points = points;
	
	m_deltas.resize(m_points.size() - 1);
	for (size_t i = 0; i < m_deltas.size(); ++i)
	{
		m_deltas[i] = m_points[i + 1] - m_points[i];
	}
	
	setNodalVector();
	assert(isValid());

	for (size_t i = 0; i < m_deltas.size(); ++i)
	{
		m_deltas[i] = m_deltas[i] / static_cast<PointCoordinateType>(m_nodes[m_k + i] - m_nodes[i + 1]);
	}
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
	return eval(s, m_points, m_k, m_nodes);
}

CCVector3 ccSpline::computeSpeed(double s) const
{
	s = std::max(std::min(s, 1.0), 0.0); // clamp between [0 1]
	return static_cast<PointCoordinateType>(m_k - 1) * eval(s, m_deltas, m_k - 1, m_nodes, 1);
}

bool ccSpline::isValid() const
{
	return (m_k > 1)
		&& (m_points.size() >= m_k)
		&& (m_nodes.size() == m_k + m_points.size())
		&& (m_points.size() == m_deltas.size() + 1);
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
	if (m_points.size() < m_k || m_k < 2)
	{
		assert(false);
		return false;
	}
	
	try
	{
		m_nodes.resize(m_k + m_points.size());
	}
	catch (const std::bad_alloc&)
	{
		assert(false);
		return false;
	}

	double step = 1.0 / (m_points.size() + 1 - m_k);
	
	for (size_t i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i] = step * i - step * (m_k - 1);
	}

	return true;
}

bool ccSpline::setNodeToOpenUniform()
{
	if (m_points.size() < m_k || m_k < 2)
	{
		assert(false);
		return false;
	}

	try
	{
		m_nodes.resize(m_k + m_points.size());
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
		else if (i >= m_points.size() + 1)
		{
			m_nodes[i] = 1.0;
		}
		else
		{
			m_nodes[i] = acc / (m_points.size() + 1 - m_k);
			acc += 1.0;
		}
	}

	return true;
}

CCVector3 ccSpline::eval(	double s,
							const std::vector<CCVector3>& points,
							size_t k,
							const std::vector<double>& nodes,
							size_t off/*=0*/) const
{
	assert(k > 1);
	assert(points.size() >= k);
	assert(isValid());
	
	// TODO: better search with binary search?
	size_t dec = 0;
	while (dec + k + off < nodes.size() && s > nodes[dec + k + off])
	{
		++dec;
	}

	std::vector<CCVector3> points_rec(k);
	for (size_t i = 0; i < k; ++i)
	{
		points_rec[i] = points[i + dec];
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