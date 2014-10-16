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

#ifndef CC_DRAWABLE_OBJECT_HEADER
#define CC_DRAWABLE_OBJECT_HEADER

#include <ccIncludeGL.h>

//Local
#include "qCC_db.h"
#include "ccGLMatrix.h"
#include "ccBBox.h"
#include "ccMaterial.h"

class ccGenericGLDisplay;
class ccScalarField;
class ccColorRampShader;
class ccShader;

//! Display parameters of a 3D entity
struct glDrawParams
{
	//! Display scalar field (prioritary on colors)
	bool showSF;
	//! Display colors
	bool showColors;
	//! Display normals
	bool showNorms;
};

//! Display context
struct glDrawContext
{
	//! Drawing options (see below)
	uint16_t flags;
	//! GL screen width
	int glW;
	//! GL screen height
	int glH;
	//! Corresponding GL window
	ccGenericGLDisplay* _win;
	//! Current zoom (screen to file rendering mode)
	float renderZoom;

	//! Default material
	ccMaterial defaultMat;
	//! Default color for mesh (front side)
	float defaultMeshFrontDiff[4];
	//! Default color for mesh (back side)
	float defaultMeshBackDiff[4];
	//! Default point color
	unsigned char pointsDefaultCol[3];
	//! Default text color
	unsigned char textDefaultCol[3];
	//! Default label color
	unsigned char labelDefaultCol[3];
	//! Default bounding-box color
	unsigned char bbDefaultCol[3];

	//! Whether to decimate big clouds when rotating the camera
	bool decimateCloudOnMove;
	//! Whether to decimate big meshes when rotating the camera
	bool decimateMeshOnMove;

	//! Currently displayed color scale (the corresponding scalar field in fact)
	ccScalarField* sfColorScaleToDisplay;
	
	//! Shader for fast dynamic color ramp lookup
	ccColorRampShader* colorRampShader;
	//! Custom rendering shader (OpenGL 3.3+)
	ccShader* customRenderingShader;
	//! Use VBOs for faster display
	bool useVBOs;

	//! Picked points radius
	float pickedPointsRadius;
	//! Picked points shift for label display
	float pickedPointsTextShift;

	//! Numerical precision (for displaying text)
	unsigned dispNumberPrecision;

	//! Label background transparency
	unsigned labelsTransparency;

	//! Blending strategy (source)
	GLenum sourceBlend;
	//! Blending strategy (destination)
	GLenum destBlend;

	//Default constructor
	glDrawContext()
		: flags(0)
		, glW(0)
		, glH(0)
		, _win(0)
		, renderZoom(1.0f)
		, decimateCloudOnMove(true)
		, decimateMeshOnMove(true)
		, sfColorScaleToDisplay(0)
		, colorRampShader(0)
		, customRenderingShader(0)
		, useVBOs(true)
		, pickedPointsRadius(4)
		, pickedPointsTextShift(0.0)
		, dispNumberPrecision(6)
		, labelsTransparency(100)
		, sourceBlend(GL_SRC_ALPHA)
		, destBlend(GL_ONE_MINUS_SRC_ALPHA)
	{}
};
typedef glDrawContext CC_DRAW_CONTEXT;

// Drawing flags (type: short)
#define CC_DRAW_2D								0x0001
#define CC_DRAW_3D								0x0002
#define CC_DRAW_FOREGROUND						0x0004
#define CC_LIGHT_ENABLED						0x0008
#define CC_SKIP_UNSELECTED						0x0010
#define CC_SKIP_SELECTED						0x0020
#define CC_SKIP_ALL								0x0030		// = CC_SKIP_UNSELECTED | CC_SKIP_SELECTED
#define CC_DRAW_ENTITY_NAMES					0x0040
#define CC_DRAW_POINT_NAMES						0x0080
#define CC_DRAW_TRI_NAMES						0x0100
#define CC_DRAW_FAST_NAMES_ONLY					0x0200
#define CC_DRAW_ANY_NAMES						0x03C0		// = CC_DRAW_ENTITY_NAMES | CC_DRAW_POINT_NAMES | CC_DRAW_TRI_NAMES
#define CC_LOD_ACTIVATED						0x0400
#define CC_VIRTUAL_TRANS_ENABLED				0x0800

// Drawing flags testing macros (see ccDrawableObject)
#define MACRO_Draw2D(context) (context.flags & CC_DRAW_2D)
#define MACRO_Draw3D(context) (context.flags & CC_DRAW_3D)
#define MACRO_DrawPointNames(context) (context.flags & CC_DRAW_POINT_NAMES)
#define MACRO_DrawTriangleNames(context) (context.flags & CC_DRAW_TRI_NAMES)
#define MACRO_DrawEntityNames(context) (context.flags & CC_DRAW_ENTITY_NAMES)
#define MACRO_DrawNames(context) (context.flags & CC_DRAW_ANY_NAMES)
#define MACRO_DrawFastNamesOnly(context) (context.flags & CC_DRAW_FAST_NAMES_ONLY)
#define MACRO_SkipUnselected(context) (context.flags & CC_SKIP_UNSELECTED)
#define MACRO_SkipSelected(context) (context.flags & CC_SKIP_SELECTED)
#define MACRO_LightIsEnabled(context) (context.flags & CC_LIGHT_ENABLED)
#define MACRO_Foreground(context) (context.flags & CC_DRAW_FOREGROUND)
#define MACRO_LODActivated(context) (context.flags & CC_LOD_ACTIVATED)
#define MACRO_VirtualTransEnabled(context) (context.flags & CC_VIRTUAL_TRANS_ENABLED)

//! Generic interface for (3D) drawable entities
class QCC_DB_LIB_API ccDrawableObject
{
public:

	//! Default constructor
	ccDrawableObject();
	//! Copy constructor
	ccDrawableObject(const ccDrawableObject& object);

	//! Draws entity and its children
	virtual void draw(CC_DRAW_CONTEXT& context) = 0;

	//! Returns whether entity is visible or not
	virtual bool isVisible() const;
	//! Sets entity visibility
	virtual void setVisible(bool state);
	//! Toggles visibility
	virtual void toggleVisibility();

	//! Returns whether visibilty is locked or not
	virtual bool isVisiblityLocked() const;
	//! Locks/unlocks visibilty
	/** If visibility is locked, the user won't be able to modify it
		(via the properties tree for instance).
	**/
	virtual void lockVisibility(bool state);

	//! Returns whether entity is selected or not
	virtual bool isSelected() const;
	//! Selects/unselects entity
	virtual void setSelected(bool state);

	//! Returns bounding-box
	/** If bbox is not relative, any active GL transformation
		(see setGLTransformation) will be applied to it.
		Moreover, one can compute a full bounding box, taking
		into acount every children, or only the ones displayed
		in a given GL window. Eventualy, one can also choose to
		compute bbox only with geometrical entities, or also with
		full GL features.
		\param relative specifies whether bbox is relative or not
		\param withGLfeatures include GL features (example: octree grid display) inside BB or not
		\param window display to compute bbox only with entities displayed in a given GL window
		\return bounding-box
	**/
	virtual ccBBox getBB(bool relative=true, bool withGLfeatures=false, const ccGenericGLDisplay* window = 0) = 0;

	//! Returns best-fit bounding-box (if available)
	/** WARNING: This method is not supported by all entities!
		Should be re-implemented whenever possible
		(returns the axis-aligned bounding-box by default).
		\param[out] trans associated transformation (so that the bounding-box can be displayed in the right position!)
		\return fit bounding-box
	**/
	virtual ccBBox getFitBB(ccGLMatrix& trans);

	//! Draws absolute (axis aligned) bounding-box
	virtual void drawBB(const colorType col[]);

	//! Returns main OpenGL paramters for this entity
	/** These parameters are deduced from the visiblity states
		of its different features (points, normals, etc.).
		\param params a glDrawParams structure
	**/
	virtual void getDrawingParameters(glDrawParams& params) const;

	//! Returns whether colors are enabled or not
	virtual bool hasColors() const;
	//! Returns whether colors are shown or not
	virtual bool colorsShown() const;
	//! Sets colors visibility
	virtual void showColors(bool state);
	//! Toggles colors display state
	virtual void toggleColors();

	//! Returns whether normals are enabled or not
	virtual bool hasNormals() const;
	//! Returns whether normals are shown or not
	virtual bool normalsShown() const;
	//! Sets normals visibility
	virtual void showNormals(bool state);
	//! Toggles normals display state
	virtual void toggleNormals();

	/*** scalar fields ***/

	//! Returns whether an active scalar field is available or not
	virtual bool hasDisplayedScalarField() const;

	//! Returns whether one or more scalar fields are instantiated
	/** WARNING: doesn't mean a scalar field is currently displayed
		(see ccDrawableObject::hasDisplayedScalarField).
	**/
	virtual bool hasScalarFields() const;

	//! Sets active scalarfield visibility
	virtual void showSF(bool state);

	//! Toggles SF display state
	virtual void toggleSF();

	//! Returns whether active scalar field is visible
	virtual bool sfShown() const;

	/*** Mesh materials ***/

	//! Toggles material display state
	virtual void toggleMaterials() {}; //does nothing by default!

	/*** Name display in 3D ***/

	//! Sets whether name should be displayed in 3D
	virtual void showNameIn3D(bool state);

	//! Returns whether name is displayed in 3D or not
	virtual bool nameShownIn3D() const;

	//! Toggles name in 3D display state
	virtual void toggleShowName();

	/*** temporary color ***/

	//! Returns whether colors are currently overriden by a temporary (unique) color
	/** See ccDrawableObject::setTempColor.
	**/
	virtual bool isColorOverriden() const;

	//! Returns current temporary (unique) color
	virtual const colorType* getTempColor() const;

	//! Sets current temporary (unique)
	/** \param col rgb color
		\param autoActivate auto activates temporary color
	**/
	virtual void setTempColor(const colorType* col, bool autoActivate = true);

	//! Set temporary color activation state
	virtual void enableTempColor(bool state);

	/*** associated display management ***/

	//! Unlinks entity from a GL display (only if it belongs to it of course)
	virtual void removeFromDisplay(const ccGenericGLDisplay* win);

	//! Sets associated GL display
	virtual void setDisplay(ccGenericGLDisplay* win);

	//! Returns associated GL display
	virtual ccGenericGLDisplay* getDisplay() const;

	//! Redraws associated GL display
	virtual void redrawDisplay();

	//! Sets associated GL display 'refreshable' before global refresh
	/** Only tagged displays will be refreshed when ccGenericGLDisplay::refresh
		is called (see also MainWindow::RefreshAllGLWindow,
		MainWindow::refreshAll and ccDrawableObject::refreshDisplay).
	**/
	virtual void prepareDisplayForRefresh();

	//! Refreshes associated GL display
	/** See ccGenericGLDisplay::refresh.
	**/
	virtual void refreshDisplay();

	/*** Transformation matrix management (for display only) ***/

	//! Associates entity with a GL transformation (rotation + translation)
	/** WARNING: FOR DISPLAY PURPOSE ONLY (i.e. should only be temporary)
		If the associated GL transformation is enabled (see
		ccDrawableObject::enableGLTransformation), it will
		be applied before displaying this entity. It will also be
		taken into account during computation of a non-relative
		bounding-box (see ccDrawableObject::getBB). However it
		will not be taken into account by any CCLib algorithm (distance
		computation, etc.) for instance.
		Note: GL transformation is automatically enabled.
	**/
	virtual void setGLTransformation(const ccGLMatrix& trans);

	//! Enables/disables associated GL transformation
	/** See ccDrawableObject::setGLTransformation.
	**/
	virtual void enableGLTransformation(bool state);

	//! Returns whether a GL transformation is enabled or not
	virtual bool isGLTransEnabled() const;

	//! Retuns associated GL transformation
	/** See ccDrawableObject::setGLTransformation.
	**/
	virtual const ccGLMatrix& getGLTransformation() const;

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

protected:

	//! Specifies whether the object is visible or not
	/** Note: this does not influence the children visibility
	**/
	bool m_visible;

	//! Specifies whether the object is selected or not
	bool m_selected;

	//! Specifies whether the visibility can be changed by user or not
	bool m_lockedVisibility;

	/*** OpenGL display parameters ***/

	//! Specifies whether colors should be displayed
	bool m_colorsDisplayed;
	//! Specifies whether normals should be displayed
	bool m_normalsDisplayed;
	//! Specifies whether scalar field should be displayed
	bool m_sfDisplayed;

	//! Temporary (unique) color
	colorType m_tempColor[3];
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

	//! Currently associated GL display
	ccGenericGLDisplay* m_currentDisplay;
};

#endif //CC_DRAWABLE_OBJECT_HEADER
