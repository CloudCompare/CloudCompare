#pragma once

//Local
#include "ccPolyline.h"

//CCCoreLib
#include <CCGeom.h>

//System
#include <vector>

//! 3D spline curve
/** Inspired from https://github.com/brainexcerpts/Spline
**/
class QCC_DB_LIB_API ccSpline : public ccPolyline
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

	//! Default constructor
	/** \param associatedCloud the associated point cloud (i.e. the vertices)
		\param k order of the spline (minimum is 2)
		\param nodeType nodal vector type
		\param uniqueID unique ID (handle with care)
	**/
	explicit ccSpline(	GenericIndexedCloudPersist* associatedCloud,
						size_t k = 2,
						NodeType nodeType = OpenUniform,
						unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::SPLINE_LINE; }

	//! Returns the spline order
	inline size_t getOrder() const { return m_k; }

	//! Sets the control weights
	//void setControlWeights(const std::vector<double>& weights);

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

	//! Updates the spline based on the current state of the associated cloud / vertices
	/** Should be called whenever the vertices are changed
	**/
	bool updateInternalState();

protected: //methods

	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//inherited methods (ccHObject)
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

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
		\param k the spline order (degree == k-1)
		\param nodes the nodal vector which defines the speed of the spline (size must be equal to k + point.size())
		\param points if not null, the control points (point.size() >= k) - otherwise the polyline vertices are used by default
		\param off offset to apply to the nodal vector 'nodes' before reading from it. this is useful to compute derivatives.
	**/
	CCVector3 eval(	double s,
					size_t k,
					const std::vector<double>& nodes,
					const std::vector<CCVector3>* points = nullptr,
					size_t off = 0) const;

	CCVector3 eval_rec(	double u,
						const std::vector<CCVector3>& points,
						size_t k,
						const std::vector<double>& nodes) const;

protected: //attributes
	NodeType m_nodeType;				//! Nodal vector type
	size_t m_k;							//! Spline order
	std::vector<CCVector3> m_deltas;	//! Control points deltas
	std::vector<double>	m_nodes;		//! Nodal vector
};
