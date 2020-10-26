#pragma once

//CCCoreLib
#include <CCGeom.h>

//System
#include <vector>

//! 3D spline curve
/** Inspired from https://raw.githubusercontent.com/brainexcerpts/Spline
**/
class ccSpline
{
public:

	//! Type of the nodal vector
	/** This will define the behavior of the spline with its control points
		as well as its speed according to its parameter.
	**/
	enum NodeType
	{
		Uniform,
		OpenUniform // Connected to the first and last control points
	};

	//! Constructor
	/** \param k order of the spline (minimum is two)
		\param nodeType nodal vector type
	**/
	ccSpline(size_t k = 2, NodeType node_type = OpenUniform);

	//! Returns the spline order
	inline size_t getOrder() const { return m_k; }

	//! Returns the set of control points
	const std::vector<CCVector3>&  getControlPoints() const { return m_points; }

	//! Sets the control points
	void setControlPoints(const std::vector<CCVector3>& points);

	//! Sets the control weights
	void setControlWeights(const std::vector<double>& weights);

	//! Sets the nodal vector type
	bool setNodeType(NodeType type);

	//! Evaluates the position of the spline at a given curvilinear position
	/** \param s curvilinear position (between 0 and 1)
		\return 3D point
	**/
	CCVector3 computePosition(double s) const;

	//! Evaluates the speed of the spline at a given curvilinear position
	/** \param s curvilinear position (between 0 and 1)
		\return 3D speed
	**/
	CCVector3 computeSpeed(double s) const;

protected: //methods

	//! Returns whether the spline is valid or not
	bool isValid() const;

	//! Sets value and size of the nodal vector depending on the current number of control points
	bool setNodalVector();

	//! Sets values of the nodal vector to be uniform
	bool setNodeToUniform();

	//! Sets values of the nodal vector to be open uniform
	bool setNodeToOpenUniform();

	//! Evaluates the equation of a splines using the blossom algorithm
	/** \param s curvilinear position (between nodes[k-1] and nodes[point.size()])
		\param points the control points (point.size() >= k)
		\param k the spline order (degree == k-1)
		\param nodes the nodal vector which defines the speed of the spline (size must be equal to k + point.size())
		\param off offset to apply to the nodal vector 'nodes' before reading from it. this is useful to compute derivatives.
	**/
	CCVector3 eval(	double s,
					const std::vector<CCVector3>& points,
					size_t k,
					const std::vector<double>& nodes,
					size_t off = 0) const;

	CCVector3 eval_rec(	double u,
						const std::vector<CCVector3>& points,
						size_t k,
						const std::vector<double>& nodes) const;

protected: //attributes
	NodeType m_nodeType;				//! Nodal vector type
	size_t m_k;							//! Spline order
	std::vector<CCVector3> m_points;	//! Control points
	std::vector<CCVector3> m_deltas;	//! Control points deltas
	std::vector<double>	m_nodes;		//! Nodal vector
};
