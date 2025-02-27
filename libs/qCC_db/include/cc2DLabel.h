#pragma once

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

//Local
#include "ccHObject.h"
#include "ccInteractor.h"
#include "ccGenericGLDisplay.h"

//Qt
#include <QRect>
//System
#include <array>

class ccGenericPointCloud;
class ccGenericMesh;

//! 2D label (typically attached to points)
class QCC_DB_LIB_API cc2DLabel : public ccHObject, public ccInteractor
{
public:

	//! Default constructor
	cc2DLabel(QString name = QString("label"));

	//! Copy constructor
	cc2DLabel(const cc2DLabel& label, bool copyPoints = true);

	//inherited from ccObject
	virtual QString getName() const override;
	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::LABEL_2D; }
	inline virtual bool isSerializable() const override { return true; }

	//! Returns 'raw' name (no replacement of default keywords)
	inline QString getRawName() const { return m_name; }

	//! Gets label content (as it will be displayed)
	/** \param precision displayed numbers precision
		\return label body (one string per line)
	**/
	QStringList getLabelContent(int precision) const;

	//! Returns the (3D) label title
	/** \param precision displayed numbers precision
		\return label title
	**/
	QString getTitle(int precision) const;

	//inherited from ccInteractor
	virtual bool acceptClick(int x, int y, Qt::MouseButton button) override;
	virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;

	//! Relative position (percentage)
	typedef std::array<float, 2> RelativePos;

	//! Sets relative position
	void setPosition(float x, float y);

	//! Returns relative position
	inline const RelativePos& getPosition() const { return m_screenPos; }

	//! Clears label
	void clear(bool ignoreDependencies = false);

	//! Returns current size
	inline unsigned size() const { return static_cast<unsigned>(m_pickedPoints.size()); }

	//! Adds a point to this label
	/** Adding a point to a label will automatically make it 'mutate'.
		1 point  = 'point' label (point position, normal, color, etc.)
		2 points = 'vector' label (vertices position, distance)
		3 points = "triangle/plane' label (vertices position, area, normal)
		\return false if 'full'
	**/
	bool addPickedPoint(ccGenericPointCloud* cloud, unsigned pointIndex, bool entityCenter = false);

	//! Adds a point to this label
	/** Adding a point to a label will automatically make it 'mutate'.
		1 point  = 'point' label (point position, normal, color, etc.)
		2 points = 'vector' label (vertices position, distance)
		3 points = "triangle/plane' label (vertices position, area, normal)
		\return false if 'full'
	**/
	bool addPickedPoint(ccGenericMesh* mesh, unsigned triangleIndex, const CCVector2d& uv, bool entityCenter = false);

	//! Whether to collapse label or not
	inline void setCollapsed(bool state) { m_showFullBody = !state; }

	//! Returns whether the label is collapsed or not
	inline bool isCollapsed() const { return !m_showFullBody; }

	//! Whether to display the point(s) legend (title only)
	inline void displayPointLegend(bool state) { m_dispPointsLegend = state; }

	//! Returns whether the point(s) legend is displayed
	inline bool isPointLegendDisplayed() const { return m_dispPointsLegend; }

	//! Whether to display the label in 2D
	inline void setDisplayedIn2D(bool state) { m_dispIn2D = state; }

	//! Returns whether the label is displayed in 2D
	inline bool isDisplayedIn2D() const { return m_dispIn2D; }

	//! Picked point descriptor
	/** Label 'points' can be shared between multiple entities
	**/
	struct QCC_DB_LIB_API PickedPoint
	{
		//! Cloud
		ccGenericPointCloud* _cloud;
		//! Mesh
		ccGenericMesh* _mesh;
		//! Point/triangle index
		unsigned index;
		//! Last known '2D' position (i.e. in screen space)
		/** This position is updated on each call to drawMeOnly3D
		**/
		CCVector3d pos2D;
		//! Last known marker scale
		float markerScale;
		//! Barycentric coordinates (for triangles)
		CCVector2d uv;
		//! Entity center mode (index will be invalid)
		bool entityCenterPoint;

		//! Returns the point position (3D)
		CCVector3 getPointPosition() const;
		//! Returns the cloud or the mesh vertices
		ccGenericPointCloud* cloudOrVertices() const;
		//! Returns the cloud or the mesh unique ID
		unsigned getUniqueID() const;
		//! Returns the associated entity (cloud or mesh)
		ccHObject* entity() const;
		//! Returns the item 'title' (either its index or 'Center' if it's a center point)
		QString itemTitle() const;
		//! Returns the point prefix ('Point' or 'Point@Tri' or 'IDXX Center')
		QString prefix(const char* pointTag) const;

		//! Default constructor
		PickedPoint()
			: _cloud(nullptr)
			, _mesh(nullptr)
			, index(0)
			, pos2D(0, 0, 0)
			, markerScale(0)
			, uv(0, 0)
			, entityCenterPoint(false)
		{}

		//! Constructor from a point and its index
		PickedPoint(ccGenericPointCloud* _cloud, unsigned pointIndex, bool centerPoint = false)
			: _cloud(_cloud)
			, _mesh(nullptr)
			, index(pointIndex)
			, pos2D(0, 0, 0)
			, markerScale(0)
			, uv(0, 0)
			, entityCenterPoint(centerPoint)
		{}

		//! Constructor from a triangle, its index and barycentric coordinates
		PickedPoint(ccGenericMesh* _mesh, unsigned triIindex, const CCVector2d& _uv, bool centerPoint = false)
			: _cloud(nullptr)
			, _mesh(_mesh)
			, index(triIindex)
			, pos2D(0, 0, 0)
			, markerScale(0)
			, uv(_uv)
			, entityCenterPoint(centerPoint)
		{}
	};

	//! Adds a point to this label (direct - handle with care)
	bool addPickedPoint(const PickedPoint& pp);

	//! Returns a given point (const version)
	inline const PickedPoint& getPickedPoint(unsigned index) const { return m_pickedPoints[index]; }

	//! Returns a given point
	inline PickedPoint& getPickedPoint(unsigned index) { return m_pickedPoints[index]; }

	//! Sets marker (relative) scale
	/** Default value: 1.0
	**/
	inline void setRelativeMarkerScale(float scale) { m_relMarkerScale = scale; }

	//! Point (marker) picking
	bool pointPicking(	const CCVector2d& clickPos,
						const ccGLCameraParameters& camera,
						int& nearestPointIndex,
						double& nearestSquareDist) const;

protected:

	//! One-point label info
	struct LabelInfo1
	{
		bool hasNormal;
		CCVector3 normal;
		bool hasRGB;
		ccColor::Rgba color;
		bool hasSF;
		ScalarType sfValue;
		QString sfName;
		//! Default constructor
		LabelInfo1()
			: hasNormal(false)
			, normal(0, 0, 0)
			, hasRGB(false)
			, color(0, 0, 0, 0)
			, hasSF(false)
			, sfValue(0)
		{}
	};
	
	//! Returns one-point label info
	void getLabelInfo1(LabelInfo1& info) const;
	
	//! Returns the SF value as a string
	/** Handles:
		- NaN values
		- shifted SF
	**/
	static QString GetSFValueAsString(const LabelInfo1& info, int precision);

	//! Two-points label info
	struct LabelInfo2
	{
		CCVector3 diff;
		//! Default constructor
		LabelInfo2()
			: diff(0, 0, 0)
		{}
	};
	//! Gets two-points label info
	void getLabelInfo2(LabelInfo2& info) const;

	//! Three-points label info
	struct LabelInfo3
	{
		CCVector3 normal;
		PointCoordinateType area;
		CCVector3d angles;
		CCVector3d edges;
		//! Default constructor
		LabelInfo3()
			: normal(0, 0, 0)
			, area(0)
			, angles(0, 0, 0)
			, edges(0, 0, 0)
		{}
	};
	//! Gets three-points label info
	void getLabelInfo3(LabelInfo3& info) const;

	//inherited from ccHObject
	bool toFile_MeOnly(QFile& out, short dataVersion) const override;
	bool fromFile_MeOnly(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap) override;
	short minimumFileVersion_MeOnly() const override;
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;
	void onDeletionOf(const ccHObject* obj) override;

	//! Draws the entity only (not its children) - 2D version
	void drawMeOnly2D(CC_DRAW_CONTEXT& context);
	//! Draws the entity only (not its children) - 3D version
	void drawMeOnly3D(CC_DRAW_CONTEXT& context);

	//! Updates the label 'name'
	void updateName();

protected:

	//! Picked points
	std::vector<PickedPoint> m_pickedPoints;

	//! Whether to show full label body or not
	bool m_showFullBody;

	//! label ROI
	/** ROI is displayed relatively to m_screenPos
		(projected in displayed screen space). m_screenPos
		corresponds to its top-left corner. So ROI[1] is
		the distance to the upper border and ROI[3] is the
		distance to the lower border (relatively to m_screenPos).
	**/
	QRect m_labelROI;

	//! Label position (percentage of screen size)
	RelativePos m_screenPos;

	//! Absolute position (pixels)
	typedef std::array<int, 2> AbsolutePos;

	//! Label position at last display (absolute)
	AbsolutePos m_lastScreenPos;

	//! Whether to display the point(s) legend
	bool m_dispPointsLegend;

	//! Whether to display the label in 2D
	bool m_dispIn2D;

	//! Relative marker scale
	float m_relMarkerScale;
};
