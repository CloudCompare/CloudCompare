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
		\param k order of the spline (minimum is 2). Order is equal to 'degree + 1'
		\param nodeType nodal vector type
		\param uniqueID unique ID (handle with care)
	**/
	explicit ccSpline(	GenericIndexedCloudPersist* associatedCloud,
						size_t order = 3,
						NodeType nodeType = OpenUniform,
						unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Constructor from a polyline
	/** \param poly polyline to 'clone'
	**/
	ccSpline(	const ccPolyline& poly,
				size_t order = 3,
				NodeType nodeType = OpenUniform	);

	//! Copy constructor
	/** \param spline spline to clone
	**/
	ccSpline(const ccSpline& spline);

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::SPLINE_LINE; }

	//! Returns the spline order
	inline size_t getOrder() const { return m_degree + 1; }

	//! Returns the spline degree
	inline size_t getDegree() const { return m_degree; }

	//! Sets the nodal vector type
	bool setNodeType(NodeType type);

	//! Evaluates the position of the spline at a given curvilinear position
	/** \param s curvilinear position (between 0 and 1)
		\param[out] P the position (if valid)
		\return position validity (otherwise 's' is out of range)
	**/
	bool computePosition(double s, CCVector3& P) const;

	//! Updates the spline based on the current state of the associated cloud / vertices
	/** Should be called whenever the vertices are changed
	**/
	bool updateInternalState();

	//! The splines nodes
	std::vector<double>& nodes() { return m_nodes; }

	//! The splines nodes (const version)
	const std::vector<double>& nodes() const { return m_nodes; }

protected: //methods

	//! Returns the theoretical number of knots
	size_t expectedKnotCount() const;

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

	//! Evaluates the equation of a spline using the blossom algorithm
	/** \param s curvilinear position
		\param k current order
		\param points control points
		\param nodes the nodal vector
	**/
	CCVector3 recursiveEval(double s,
							size_t k,
							const std::vector<CCVector3>& points,
							const std::vector<double>& nodes) const;

protected: //attributes
	NodeType m_nodeType;				//! Nodal vector type
	size_t m_degree;					//! Spline degree
	std::vector<CCVector3> m_deltas;	//! Control points deltas
	std::vector<double>	m_nodes;		//! Nodal vector
};
