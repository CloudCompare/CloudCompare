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
		Uniform,		// Uniform (with mirrored node values on the sides)
		OpenUniform,	// Connected to the first and last control points
		Custom			// Custom values (must be set manually)
	};

	//! Default constructor (empty spline)
	/** After the control points are set, one should call 'initNodes' to initialize the knots.
		\param associatedCloud the associated point cloud (i.e. the vertices)
		\param k order of the spline (minimum is 2). Order is equal to 'degree + 1'
		\param uniqueID unique ID (handle with care)
	**/
	explicit ccSpline(	GenericIndexedCloudPersist* associatedCloud,
						unsigned order = 3,
						unsigned uniqueID = ccUniqueIDGenerator::InvalidUniqueID);

	//! Constructor from a polyline
	/** \param poly polyline to 'clone'
	**/
	ccSpline(	const ccPolyline& poly,
				unsigned order = 3,
				NodeType nodeType = OpenUniform	);

	//! Copy constructor
	/** \param spline spline to clone
	**/
	ccSpline(const ccSpline& spline);

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::SPLINE_LINE; }

	//! Returns the spline order (= degree + 1)
	inline unsigned getOrder() const { return m_degree + 1; }

	//! Returns the spline degree (= order - 1)
	inline unsigned getDegree() const { return m_degree; }

	//! Returns the number of (virtual) control points (= size() or size() + m_degree if the polyline is closed)
	inline unsigned getCPCount() const { return isClosed() ? size() + m_degree : size(); }

	//! Inits the knots
	/** \warning Will reset the node values
	**/
	bool initNodes(NodeType type);

	//! Evaluates the position of the spline at a given curvilinear position
	/** \param s curvilinear position (between 0 and 1)
		\param[out] P the position (if valid)
		\return position validity (otherwise 's' is out of range)
	**/
	bool computePosition(double s, CCVector3& P) const;

	//! Returns the node values
	std::vector<double>& nodes() { return m_nodes; }

	//! Returns the node values (const version)
	const std::vector<double>& nodes() const { return m_nodes; }

	//! Minimum drawing precision
	/** \warning Never pass a 'constant initializer' by reference
	**/
	static const unsigned MIN_DRAWING_PRECISION = 4;

	//! Sets drawing precision
	/** \param precision drawing precision (should be >= MIN_DRAWING_PRECISION)
	**/
	inline void setDrawingPrecision(unsigned precision)
	{
		m_drawPrecision = std::max(MIN_DRAWING_PRECISION, precision);
	}

	//! Returns drawing precision (or 0 if feature is not supported)
	inline unsigned getDrawingPrecision() const { return m_drawPrecision; }

	//! Converts this spline to a polyline (using the current drawing precision)
	ccPolyline* toPoly() const;

	//! Sets whether the polyline/spline is closed or not
	/** \warning Calling this method after the node values have been set will invalidate the spline!
	**/
	void setClosed(bool state) override;

protected: //methods

	//! Returns the theoretical number of knots
	unsigned expectedKnotCount() const;

	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;

	//inherited methods (ccHObject)
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//! Returns whether the spline is valid or not
	bool isValid() const;

	//! Sets values of the nodal vector to be uniform
	void setNodeToUniform();

	//! Sets values of the nodal vector to be open uniform
	void setNodeToOpenUniform();

protected: //attributes
	unsigned m_degree;					//!< Spline degree
	std::vector<double>	m_nodes;		//!< Nodal vector
	unsigned m_drawPrecision;			//!< Drawing precision
};
