#ifndef PLANAR_ENTITY_INTERFACE_HEADER
#define PLANAR_ENTITY_INTERFACE_HEADER

//CCLib
#include <CCGeom.h>

//qCC_gl
#include <ccGLDrawContext.h>

//! Interface for a planar entity
class ccPlanarEntityInterface
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

protected: //members

	//! Draws a normal vector (OpenGL)
	void glDrawNormal(CC_DRAW_CONTEXT& context, const CCVector3& pos, float scale, const ccColor::Rgb* color = 0);

	//! Whether the facet normal vector should be displayed or not
	bool m_showNormalVector;

};

#endif //PLANAR_ENTITY_INTERFACE_HEADER
