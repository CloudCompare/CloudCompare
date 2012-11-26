#ifndef CC_2D_LABEL_HEADER
#define CC_2D_LABEL_HEADER

//Local
#include "ccHObject.h"

//Qt
#include <QString>
#include <QStringList>
#include <QFontMetrics>

class ccGenericPointCloud;

//! 2D label (typically attached to points)
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
class QCC_DB_DLL_API cc2DLabel : public ccHObject
#else
class cc2DLabel : public ccHObject
#endif
{
public:

	//! Default constructor
	cc2DLabel(QString name = QString("label"));

	//inherited from ccObject
    virtual QString getName() const;
	//inherited from ccHObject
    virtual CC_CLASS_ENUM getClassID() const {return CC_2D_LABEL;};
	virtual bool isSerializable() const { return true; }

	//! Adds a point to label
	/** Adding a point to a label will automatcillay make it 'mute'.
		1 point  = 'point' label (point position, normal, color, etc.)
		2 points = 'vector' label (vertices position, distance)
		3 points = "triangle/plane' label (vertices position, area, normal)
		\return false if 'full'
	**/
	bool addPoint(ccGenericPointCloud* cloud, unsigned pointIndex);

	//! Gets label content (as it will be displayed)
	/** \param precision displayed numbers precision
		\return label body (one string per line)
	**/
	QStringList getLabelContent(int precision);

	//! Process mouse click
	bool acceptClick(int x, int y, Qt::MouseButton button);

	//! Sets relative position
	void setPosition(float x, float y);

	//! Returns relative position
	const float* getPosition() const { return m_screenPos; }

	//! Moves label (percentages)
	void move(float dx, float dy);

	//! Clears label
	void clear();

	//! Returns current size
	unsigned size() const { return m_points.size(); }

	//! Whether to collapse label or not
	void setCollapsed(bool state) { m_showFullBody = !state; }

	//! Returns Whether the label is collapsed or not
	bool isCollapsed() const { return !m_showFullBody; }

	//! Whether to display the label in 3D (title only)
	void setDisplayedIn3D(bool state) { m_dispIn3D = state; }

	//! Returns whether the label is displayed in 3D (title only)
	bool isDisplayedIn3D() const { return m_dispIn3D; }

	//! Whether to display the label in 2D
	void setDisplayedIn2D(bool state) { m_dispIn2D = state; }

	//! Returns whether the label is displayed in 2D
	bool isDisplayedIn2D() const { return m_dispIn2D; }

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
	const PickedPoint& getPoint(unsigned index) const { return m_points[index]; }

protected:

    //inherited from ccHObject
	virtual bool toFile_MeOnly(QFile& out) const;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion);

    //! Draws the entity only (not its children)
    virtual void drawMeOnly(CC_DRAW_CONTEXT& context);
    //! Draws the entity only (not its children) - 2D version
    virtual void drawMeOnly2D(CC_DRAW_CONTEXT& context);
    //! Draws the entity only (not its children) - 3D version
    virtual void drawMeOnly3D(CC_DRAW_CONTEXT& context);

	//! Picked points
	std::vector<PickedPoint> m_points;

	//! Whether to show full label body or not
	bool m_showFullBody;

	//! label ROI
	/** ROI is displayed relatively to m_screenPos
		(projected in displayed screen space). m_screenPos
		corresponds to its top-left corner. So ROI[1] is
		the distance to the upper border and ROI[3] is the
		distance to the lower border (relatively to m_screenPos).
	**/
	int m_labelROI[4];

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

#endif
