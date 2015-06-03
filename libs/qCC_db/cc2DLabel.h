//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_2D_LABEL_HEADER
#define CC_2D_LABEL_HEADER

//Local
#include "qCC_db.h"
#include "ccHObject.h"
#include "ccInteractor.h"

//Qt
#include <QString>
#include <QStringList>
#include <QFontMetrics>

class ccGenericPointCloud;

//! 2D label (typically attached to points)
class QCC_DB_LIB_API cc2DLabel : public ccHObject, public ccInteractor
{
public:

	//! Default constructor
	cc2DLabel(QString name = QString("label"));

	//inherited from ccObject
	virtual QString getName() const;
	//inherited from ccHObject
	inline virtual CC_CLASS_ENUM getClassID() const { return CC_TYPES::LABEL_2D; }
	inline virtual bool isSerializable() const { return true; }

	//! Returns 'raw' name (no replacement of default keywords)
	inline QString getRawName() const { return m_name; }

	//! Gets label content (as it will be displayed)
	/** \param precision displayed numbers precision
		\return label body (one string per line)
	**/
	QStringList getLabelContent(int precision);

	//! Returns the (3D) label title
	/** \param precision displayed numbers precision
		\return label title
	**/
	QString getTitle(int precision) const;

	//inherited from ccInteractor
	virtual bool acceptClick(int x, int y, Qt::MouseButton button);
	virtual bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight);

	//! Sets relative position
	void setPosition(float x, float y);

	//! Returns relative position
	inline const float* getPosition() const { return m_screenPos; }

	//! Clears label
	void clear(bool ignoreDependencies = false);

	//! Returns current size
	inline unsigned size() const { return static_cast<unsigned>(m_points.size()); }

	//! Adds a point to label
	/** Adding a point to a label will automatically make it 'mutate'.
		1 point  = 'point' label (point position, normal, color, etc.)
		2 points = 'vector' label (vertices position, distance)
		3 points = "triangle/plane' label (vertices position, area, normal)
		\return false if 'full'
	**/
	bool addPoint(ccGenericPointCloud* cloud, unsigned pointIndex);

	//! Whether to collapse label or not
	inline void setCollapsed(bool state) { m_showFullBody = !state; }

	//! Returns Whether the label is collapsed or not
	inline bool isCollapsed() const { return !m_showFullBody; }

	//! Whether to display the label in 3D (title only)
	inline void setDisplayedIn3D(bool state) { m_dispIn3D = state; }

	//! Returns whether the label is displayed in 3D (title only)
	inline bool isDisplayedIn3D() const { return m_dispIn3D; }

	//! Whether to display the label in 2D
	inline void setDisplayedIn2D(bool state) { m_dispIn2D = state; }

	//! Returns whether the label is displayed in 2D
	inline bool isDisplayedIn2D() const { return m_dispIn2D; }

	//! Picked point descriptor
	/** Label 'points' can be shared between multiple entities
	**/
	struct PickedPoint
	{
		//! cloud
		ccGenericPointCloud* cloud;
		//! index
		unsigned index;

		//! Default constructor
		PickedPoint()
			: cloud(0)
			, index(0)
		{}

		//! Constructor from a point and its index
		PickedPoint(ccGenericPointCloud* _cloud, unsigned _index)
			: cloud(_cloud)
			, index(_index)
		{}
	};

	//! Returns a given point
	inline const PickedPoint& getPoint(unsigned index) const { return m_points[index]; }

protected:

	//! One-point label info
	struct LabelInfo1
	{
		unsigned pointIndex;
		ccGenericPointCloud* cloud;
		bool hasNormal;
		CCVector3 normal;
		bool hasRGB;
		Vector3Tpl<colorType> rgb;
		bool hasSF;
		ScalarType sfValue;
		QString sfName;
		//! Default constructor
		LabelInfo1()
			: pointIndex(0)
			, cloud(0)
			, hasNormal(false)
			, normal(0,0,0)
			, hasRGB(false)
			, rgb(0,0,0)
			, hasSF(false)
			, sfValue(NAN_VALUE)
		{}
	};
	//! Gets one-point label info
	void getLabelInfo1(LabelInfo1& info) const;

	//! Two-points label info
	struct LabelInfo2
	{
		unsigned point1Index;
		ccGenericPointCloud* cloud1;
		unsigned point2Index;
		ccGenericPointCloud* cloud2;
		CCVector3 diff;
		//! Default constructor
		LabelInfo2()
			: point1Index(0)
			, cloud1(0)
			, point2Index(0)
			, cloud2(0)
			, diff(0,0,0)
		{}
	};
	//! Gets two-points label info
	void getLabelInfo2(LabelInfo2& info) const;

	//! Three-points label info
	struct LabelInfo3
	{
		unsigned point1Index;
		ccGenericPointCloud* cloud1;
		unsigned point2Index;
		ccGenericPointCloud* cloud2;
		unsigned point3Index;
		ccGenericPointCloud* cloud3;
		CCVector3 normal;
		PointCoordinateType area;
		CCVector3d angles;
		CCVector3d edges;
		//! Default constructor
		LabelInfo3()
			: point1Index(0)
			, cloud1(0)
			, point2Index(0)
			, cloud2(0)
			, point3Index(0)
			, cloud3(0)
			, normal(0,0,0)
			, area(0)
			, angles(0,0,0)
			, edges(0,0,0)
		{}
	};
	//! Gets three-points label info
	void getLabelInfo3(LabelInfo3& info) const;

	//inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags);
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context);
	virtual void onDeletionOf(const ccHObject* obj);

	//! Draws the entity only (not its children) - 2D version
	virtual void drawMeOnly2D(CC_DRAW_CONTEXT& context);
	//! Draws the entity only (not its children) - 3D version
	virtual void drawMeOnly3D(CC_DRAW_CONTEXT& context);

	//! Picked points
	std::vector<PickedPoint> m_points;

	//! Updates the label 'name'
	void updateName();

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

	//! close button ROI
	//int m_closeButtonROI[4];

	//! Label position (percentage of screen size)
	float m_screenPos[2];

	//! Label position at last display (absolute)
	int m_lastScreenPos[2];

	//! Whether to display the label in 3D (title only)
	bool m_dispIn3D;

	//! Whether to display the label in 2D
	bool m_dispIn2D;
};

#endif //CC_2D_LABEL_HEADER
