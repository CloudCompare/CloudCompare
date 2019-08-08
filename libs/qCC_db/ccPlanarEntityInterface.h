#ifndef PLANAR_ENTITY_INTERFACE_HEADER
#define PLANAR_ENTITY_INTERFACE_HEADER

//CCLib
#include <CCGeom.h>
#include "ccInteractor.h"

//qCC_gl
#include <ccGLDrawContext.h>
#include <QObject>

class ccHObject;

//! Interface for a planar entity
class QCC_DB_LIB_API ccPlanarEntityInterface : public QObject, public ccInteractor
{
	Q_OBJECT
public:
	
	//! Default constructor
	ccPlanarEntityInterface();

	//! Show normal vector
	inline void showNormalVector(bool state) { m_showNormalVector = state; }
	//! Whether normal vector is shown or not
	inline bool normalVectorIsShown() const { return m_showNormalVector; }

	virtual CCVector3 getCenter() const = 0;

	//! Returns the entity normal
	virtual CCVector3 getNormal() const = 0;

	virtual ccHObject* getPlane() = 0;

	//////////////////////////////////////////////////////////////////////////
	bool getNormalEditState() { return m_editable; }
	void normalEditState(bool edit) { m_editable = edit; }

	//inherited from ccInteractor
 	bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;
 	bool move3D(const CCVector3d& u) override;

	//! Sets last clicked point (on screen)
	void setClickedPoint(int x, int y, int screenWidth, int screenHeight, const ccGLMatrixd& viewMatrix);

	//! Components
	enum Components {
		NONE = 0,
		NORMAL_ARROW = 1,	// normal vector, for rotation
		NORMAL_TORUS = 2,	// rotate locked by normal direction
		CENTER_SPHERE = 3,	// upside the normal vector // for move
		CROSS = 4,			// center location
	};

	//! Sets currently active component
	/** \param id component ID (see Components)
	**/
	void setActiveComponent(int id);

	virtual void notifyPlanarEntityChanged(ccGLMatrix mat, bool trans) = 0;

	CCVector3 projectTo3DGlobal(CCVector3 pt_3d);
	CCVector2 projectTo2DLocal(CCVector3 pt_3d);
	CCVector3 backprojectTo3DGlobal(CCVector2 pt_2d);
	std::vector<CCVector3> projectTo3DGlobal(std::vector<CCVector3> pt_3d);
	std::vector<CCVector2> projectTo2DLocal(std::vector<CCVector3> pt_3d);
	std::vector<CCVector3> backprojectTo3DGlobal(std::vector<CCVector2> pt_2d);

signals:
	void planarEntityChanged();

protected: //members

	//! Draws a normal vector (OpenGL)
	void glDrawNormal(CC_DRAW_CONTEXT& context, const CCVector3& pos, float scale, const ccColor::Rgb* color = 0);

	void glDrawNormal(CC_DRAW_CONTEXT & context, unsigned int id,  const CCVector3 & pos, float scale, const ccColor::Rgb * color);

	//! Whether the facet normal vector should be displayed or not
	bool m_showNormalVector;

	bool m_editable;

	//! Active component
	Components m_activeComponent;

	//! Last "orientation" vector (corresponding to last clicked point)
	CCVector3d m_lastOrientation;

	//! View matrix
	ccGLMatrixd m_viewMatrix;
};

#endif //PLANAR_ENTITY_INTERFACE_HEADER
