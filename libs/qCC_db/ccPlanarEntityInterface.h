#ifndef PLANAR_ENTITY_INTERFACE_HEADER
#define PLANAR_ENTITY_INTERFACE_HEADER

//CCLib
#include <CCGeom.h>
#include "ccInteractor.h"

//qCC_gl
#include <ccGLDrawContext.h>

//! Interface for a planar entity
class QCC_DB_LIB_API ccPlanarEntityInterface : public ccInteractor
{
public:
	
	//! Default constructor
	ccPlanarEntityInterface();

	//! Show normal vector
	inline void showNormalVector(bool state) { m_showNormalVector = state; }
	//! Whether normal vector is shown or not
	inline bool normalVectorIsShown() const { return m_showNormalVector; }

	//! Returns the entity normal
	virtual CCVector3 getNormal() const = 0;

	//////////////////////////////////////////////////////////////////////////

	void normalEditState(bool edit) { m_editable = edit; }

	//inherited from ccInteractor
 	bool move2D(int x, int y, int dx, int dy, int screenWidth, int screenHeight) override;
 	bool move3D(const CCVector3d& u) override;

	//! Sets last clicked point (on screen)
	void setClickedPoint(int x, int y, int screenWidth, int screenHeight, const ccGLMatrixd& viewMatrix);

	//! Components
	enum Components {
		NONE = 0,
		NORMAL_ARROW = 1,	// normal vector, for display
		NORMAL_TORUS = 2,	// rotate locked by normal direction
		CENTER_SPHERE = 3,			// upside the normal vector // for rotation
		CROSS = 4,			// center location
	};

	//! Sets currently active component
	/** \param id component ID (see Components)
	**/
	void setActiveComponent(int id);

	void notifyPlanarEntityChanged();

protected: //members

	//! Draws a normal vector (OpenGL)
	void glDrawNormal(CC_DRAW_CONTEXT& context, const CCVector3& pos, float scale, const ccColor::Rgb* color = 0);

	void glDrawNormalEditable(CC_DRAW_CONTEXT & context, unsigned int id,  const CCVector3 & pos, float scale, const ccColor::Rgb * color);

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
