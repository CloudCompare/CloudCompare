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

#include "ccIncludeGL.h"

//Local
#include "qCC_db.h"
#include "ccGLMatrix.h"
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
	ccMaterial::Shared defaultMat;
	//! Default color for mesh (front side)
	ccColor::Rgbaf defaultMeshFrontDiff;
	//! Default color for mesh (back side)
	ccColor::Rgbaf defaultMeshBackDiff;
	//! Default point color
	ccColor::Rgbub pointsDefaultCol;
	//! Default text color
	ccColor::Rgbub textDefaultCol;
	//! Default label background color
	ccColor::Rgbub labelDefaultBkgCol;
	//! Default label marker color
	ccColor::Rgbub labelDefaultMarkerCol;
	//! Default bounding-box color
	ccColor::Rgbub bbDefaultCol;

	//! Whether to decimate big clouds when updating the 3D view
	bool decimateCloudOnMove;
	//! Minimum level for LOD display
	unsigned char minLODLevel;
	//! Minimum number of points for activating LOD display
	unsigned minLODPointCount;
	//! Current level for LOD display
	unsigned char currentLODLevel;
	//! Start index for current LOD level
	unsigned currentLODStartIndex;
	//! Wheter more points are available or not at the current level
	bool moreLODPointsAvailable;
	//! Wheter higher levels are available or not
	bool higherLODLevelsAvailable;

	//! Whether to decimate big meshes when rotating the camera
	bool decimateMeshOnMove;
	//! Minimum number of triangles for activating LOD display
	unsigned minLODTriangleCount;

	//! Currently displayed color scale (the corresponding scalar field in fact)
	ccScalarField* sfColorScaleToDisplay;
	
	//! Shader for fast dynamic color ramp lookup
	ccColorRampShader* colorRampShader;
	//! Custom rendering shader (OpenGL 3.3+)
	ccShader* customRenderingShader;
	//! Use VBOs for faster display
	bool useVBOs;

	//! Label marker size (radius)
	float labelMarkerSize;
	//! Shift for 3D label marker display (around the marker, in pixels)
	float labelMarkerTextShift_pix;

	//! Numerical precision (for displaying text)
	unsigned dispNumberPrecision;

	//! Label background opacity
	unsigned labelOpacity;

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
		, defaultMat(new ccMaterial("default"))
		, defaultMeshFrontDiff(ccColor::defaultMeshFrontDiff)
		, defaultMeshBackDiff(ccColor::defaultMeshBackDiff)
		, pointsDefaultCol(ccColor::defaultColor)
		, textDefaultCol(ccColor::defaultColor)
		, labelDefaultBkgCol(ccColor::defaultLabelBkgColor)
		, labelDefaultMarkerCol(ccColor::defaultLabelMarkerColor)
		, bbDefaultCol(ccColor::yellow)
		, decimateCloudOnMove(true)
		, minLODLevel(11)
		, minLODPointCount(10000000)
		, currentLODLevel(0)
		, currentLODStartIndex(0)
		, moreLODPointsAvailable(false)
		, higherLODLevelsAvailable(false)
		, decimateMeshOnMove(true)
		, minLODTriangleCount(2500000)
		, sfColorScaleToDisplay(0)
		, colorRampShader(0)
		, customRenderingShader(0)
		, useVBOs(true)
		, labelMarkerSize(5)
		, labelMarkerTextShift_pix(5)
		, dispNumberPrecision(6)
		, labelOpacity(100)
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

	//! Returns main OpenGL paramters for this entity
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

	/*** scalar fields ***/

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

	/*** Mesh materials ***/

	//! Toggles material display state
	virtual void toggleMaterials() {}; //does nothing by default!

	/*** Name display in 3D ***/

	//! Sets whether name should be displayed in 3D
	inline virtual void showNameIn3D(bool state) { m_showNameIn3D = state; }

	//! Returns whether name is displayed in 3D or not
	inline virtual bool nameShownIn3D() const { return m_showNameIn3D; }

	//! Toggles name in 3D display state
	inline virtual void toggleShowName() { showNameIn3D(!nameShownIn3D()); }

	/*** temporary color ***/

	//! Returns whether colors are currently overriden by a temporary (unique) color
	/** See ccDrawableObject::setTempColor.
	**/
	inline virtual bool isColorOverriden() const { return m_colorIsOverriden; }

	//! Returns current temporary (unique) color
	inline virtual const ccColor::Rgb& getTempColor() const { return m_tempColor; }

	//! Sets current temporary (unique)
	/** \param col rgb color
		\param autoActivate auto activates temporary color
	**/
	virtual void setTempColor(const ccColor::Rgb& col, bool autoActivate = true);

	//! Set temporary color activation state
	inline virtual void enableTempColor(bool state) { m_colorIsOverriden = state; }

	/*** associated display management ***/

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
	/** See ccGenericGLDisplay::refresh.
	**/
	virtual void refreshDisplay();

	/*** Transformation matrix management (for display only) ***/

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
	inline virtual void enableGLTransformation(bool state) { m_glTransEnabled = state; }

	//! Returns whether a GL transformation is enabled or not
	inline virtual bool isGLTransEnabled() const { return m_glTransEnabled; }

	//! Retuns associated GL transformation
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
	ccColor::Rgb m_tempColor;
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
