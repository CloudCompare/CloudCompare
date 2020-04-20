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

#ifndef CC_DRAWABLE_OBJECT_HEADER
#define CC_DRAWABLE_OBJECT_HEADER

//Local
#include "ccGLDrawContext.h"

//CCLib
#include <CCGeom.h>


class ccGenericGLDisplay;

//! Simple (clipping) plane equation
struct ccClipPlane
{
	Tuple4Tpl<double> equation;
};
using ccClipPlaneSet = std::vector<ccClipPlane>;

//! Generic interface for (3D) drawable entities
class QCC_DB_LIB_API ccDrawableObject
{
public:

	//! Default constructor
	ccDrawableObject();
	//! Copy constructor
	ccDrawableObject(const ccDrawableObject& object);
	
	virtual ~ccDrawableObject() = default;

public:  //drawing and drawing options

	//! Draws entity and its children
	virtual void draw(CC_DRAW_CONTEXT& context) = 0;

	//! Returns whether entity is visible or not
	inline virtual bool isVisible() const { return m_visible; }
	//! Sets entity visibility
	inline virtual void setVisible(bool state) { m_visible = state; }
	//! Toggles visibility
	inline virtual void toggleVisibility() { setVisible(!isVisible()); }

	//! Returns whether visibilty is locked or not
	inline virtual bool isVisiblityLocked() const { return m_lockedVisibility; }
	//! Locks/unlocks visibilty
	/** If visibility is locked, the user won't be able to modify it
		(via the properties tree for instance).
	**/
	inline virtual void lockVisibility(bool state) { m_lockedVisibility = state; }

	//! Returns whether entity is selected or not
	inline virtual bool isSelected() const { return m_selected; }
	//! Selects/unselects entity
	inline virtual void setSelected(bool state) { m_selected = state; }

	//! Returns main OpenGL parameters for this entity
	/** These parameters are deduced from the visiblity states
		of its different features (points, normals, etc.).
		\param params a glDrawParams structure
	**/
	virtual void getDrawingParameters(glDrawParams& params) const;

	//! Returns whether colors are enabled or not
	inline virtual bool hasColors() const  { return false; }
	//! Returns whether colors are shown or not
	inline virtual bool colorsShown() const { return m_colorsDisplayed; }
	//! Sets colors visibility
	inline virtual void showColors(bool state) { m_colorsDisplayed = state; }
	//! Toggles colors display state
	inline virtual void toggleColors() { showColors(!colorsShown()); }

	//! Returns whether normals are enabled or not
	inline virtual bool hasNormals() const  { return false; }
	//! Returns whether normals are shown or not
	inline virtual bool normalsShown() const { return m_normalsDisplayed; }
	//! Sets normals visibility
	inline virtual void showNormals(bool state) { m_normalsDisplayed = state; }
	//! Toggles normals display state
	inline virtual void toggleNormals() { showNormals(!normalsShown()); }

public: //scalar fields

	//! Returns whether an active scalar field is available or not
	inline virtual bool hasDisplayedScalarField() const { return false; }

	//! Returns whether one or more scalar fields are instantiated
	/** WARNING: doesn't mean a scalar field is currently displayed
		(see ccDrawableObject::hasDisplayedScalarField).
	**/
	inline virtual bool hasScalarFields() const  { return false; }

	//! Sets active scalarfield visibility
	inline virtual void showSF(bool state) { m_sfDisplayed = state; }

	//! Toggles SF display state
	inline virtual void toggleSF() { showSF(!sfShown()); }

	//! Returns whether active scalar field is visible
	inline virtual bool sfShown() const { return m_sfDisplayed; }

public: //(Mesh) materials

	//! Toggles material display state
	virtual void toggleMaterials() {} //does nothing by default!

public: //Name display in 3D

	//! Sets whether name should be displayed in 3D
	inline virtual void showNameIn3D(bool state) { m_showNameIn3D = state; }

	//! Returns whether name is displayed in 3D or not
	inline virtual bool nameShownIn3D() const { return m_showNameIn3D; }

	//! Toggles name in 3D display state
	inline virtual void toggleShowName() { showNameIn3D(!nameShownIn3D()); }

public: //Temporary color

	//! Returns whether colors are currently overridden by a temporary (unique) color
	/** See ccDrawableObject::setTempColor.
	**/
	inline virtual bool isColorOverriden() const { return m_colorIsOverriden; }

	//! Returns current temporary (unique) color
	inline virtual const ccColor::Rgba& getTempColor() const { return m_tempColor; }

	//! Sets current temporary (unique)
	/** \param col rgba color
		\param autoActivate auto activates temporary color
	**/
	virtual void setTempColor(const ccColor::Rgba& col, bool autoActivate = true);

	//! Sets current temporary (unique)
	/** \param col rgb color
		\param autoActivate auto activates temporary color
	**/
	virtual void setTempColor(const ccColor::Rgb& col, bool autoActivate = true);

	//! Set temporary color activation state
	inline virtual void enableTempColor(bool state) { m_colorIsOverriden = state; }

public: //associated display management

	//! Unlinks entity from a GL display (only if it belongs to it of course)
	virtual void removeFromDisplay(const ccGenericGLDisplay* win);

	//! Sets associated GL display
	virtual void setDisplay(ccGenericGLDisplay* win);

	//! Returns associated GL display
	inline virtual ccGenericGLDisplay* getDisplay() const { return m_currentDisplay; }

	//! Redraws associated GL display
	virtual void redrawDisplay();

	//! Sets associated GL display 'refreshable' before global refresh
	/** Only tagged displays will be refreshed when ccGenericGLDisplay::refresh
		is called (see also MainWindow::RefreshAllGLWindow,
		MainWindow::refreshAll and ccDrawableObject::refreshDisplay).
	**/
	virtual void prepareDisplayForRefresh();

	//! Refreshes associated GL display
	/** See ccGenericGLDisplay::refresh. The display will only be updated
		if it has been 'prepared for refresh' (see prepareDisplayForRefresh).
	**/
	virtual void refreshDisplay(bool only2D = false);

public: //Transformation matrix management (for display only)

	//! Associates entity with a GL transformation (rotation + translation)
	/** \warning FOR DISPLAY PURPOSE ONLY (i.e. should only be temporary)
		If the associated GL transformation is enabled (see
		ccDrawableObject::enableGLTransformation), it will
		be applied before displaying this entity.
		However it will not be taken into account by any CCLib algorithm
		(distance computation, etc.) for instance.
		Note: GL transformation is automatically enabled.
	**/
	virtual void setGLTransformation(const ccGLMatrix& trans);

	//! Enables/disables associated GL transformation
	/** See ccDrawableObject::setGLTransformation.
	**/
	virtual void enableGLTransformation(bool state);

	//! Returns whether a GL transformation is enabled or not
	inline virtual bool isGLTransEnabled() const { return m_glTransEnabled; }

	//! Returns associated GL transformation
	/** See ccDrawableObject::setGLTransformation.
	**/
	inline virtual const ccGLMatrix& getGLTransformation() const { return m_glTrans; }

	//! Resets associated GL transformation
	/** GL transformation is reset to identity.
		Note: GL transformation is automatically disabled.
		See ccDrawableObject::setGLTransformation.
	**/
	virtual void resetGLTransformation();

	//! Mutliplies (left) current GL transformation by a rotation matrix
	/** 'GLtrans = M * GLtrans'
		Note: GL transformation is automatically enabled.
		See ccDrawableObject::setGLTransformation.
	**/
	virtual void rotateGL(const ccGLMatrix& rotMat);

	//! Translates current GL transformation by a rotation matrix
	/** 'GLtrans = GLtrans + T'
		Note: GL transformation is automatically enabled.
		See ccDrawableObject::setGLTransformation.
	**/
	virtual void translateGL(const CCVector3& trans);

public: //clipping planes

	//! Removes all clipping planes (if any)
	virtual void removeAllClipPlanes() { m_clipPlanes.resize(0); }

	//! Registers a new clipping plane
	/** \return false if the planes couldn't be added (not enough memory)
	**/
	virtual bool addClipPlanes(const ccClipPlane& plane);

	//! Enables or disables clipping planes (OpenGL)
	/** \warning If enabling the clipping planes, be sure to call this method AFTER the modelview matrix has been set.
	**/
	virtual void toggleClipPlanes(CC_DRAW_CONTEXT& context, bool enable);

protected: //members

	//! Specifies whether the object is visible or not
	/** Note: this does not influence the children visibility
	**/
	bool m_visible;

	//! Specifies whether the object is selected or not
	bool m_selected;

	//! Specifies whether the visibility can be changed by user or not
	bool m_lockedVisibility;

	//! Specifies whether colors should be displayed
	bool m_colorsDisplayed;
	//! Specifies whether normals should be displayed
	bool m_normalsDisplayed;
	//! Specifies whether scalar field should be displayed
	bool m_sfDisplayed;

	//! Temporary (unique) color
	ccColor::Rgba m_tempColor;
	//! Temporary (unique) color activation state
	bool m_colorIsOverriden;

	//! Current GL transformation
	/** See ccDrawableObject::setGLTransformation.
	**/
	ccGLMatrix m_glTrans;
	//! Current GL transformation activation state
	/** See ccDrawableObject::setGLTransformation.
	**/
	bool m_glTransEnabled;

	//! Whether name is displayed in 3D or not
	bool m_showNameIn3D;
	//! Last 2D position of the '3D' name
	CCVector3d m_nameIn3DPos;

	//! Currently associated GL display
	ccGenericGLDisplay* m_currentDisplay;

	//! Active clipping planes (used for display only)
	ccClipPlaneSet m_clipPlanes;
};

#endif //CC_DRAWABLE_OBJECT_HEADER
