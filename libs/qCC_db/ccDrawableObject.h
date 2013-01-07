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
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2224                                                              $
//$LastChangedDate:: 2012-07-25 19:13:23 +0200 (mer., 25 juil. 2012)       $
//**************************************************************************
//

#ifndef CC_DRAWABLE_OBECJT_HEADER
#define CC_DRAWABLE_OBECJT_HEADER

#include <ccIncludeGL.h>

#include "ccGLMatrix.h"
#include "ccGenericGLDisplay.h"
#include "ccColorTablesManager.h"
#include "ccBBox.h"
#include "ccScalarField.h"
#include "ccMaterial.h"

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

//! Vertex buffer object
#ifdef QCC_DB_USE_AS_DLL
#include "qCC_db_dll.h"
struct QCC_DB_DLL_API vboStruct
#else
struct vboStruct
#endif
{
	bool enabled;
	unsigned idVert;
	unsigned idInd;
	unsigned char* buffer;

	inline unsigned maxSize() const {return MAX_NUMBER_OF_ELEMENTS_PER_CHUNK;}
	inline unsigned xyzShift() const {return 0;}
	inline unsigned normShift() const {return 12;}
	inline unsigned rgbShift() const {return 24;}
	inline unsigned elemSize() const {return 32;}	//NVidia indicates that a multiple of 32 bytes is better
                                                    //and we must put there: 3 floats (coordinates) + 3 floats (normals) + 3 bytes (RGB) = 12+12+3=27 bytes;
	vboStruct();
	void clear();
	void init();
};

//! Display context
struct glDrawContext
{
    unsigned short flags;       //drawing options (see below)
    int glW;                    //GL screen width
    int glH;                    //GL screen height
    ccGenericGLDisplay* _win;   //GL window ref.

	//default materials
	ccMaterial defaultMat; //default material
	float defaultMeshFrontDiff[4];
	float defaultMeshBackDiff[4];
	unsigned char pointsDefaultCol[3];
	unsigned char textDefaultCol[3];
	unsigned char bbDefaultCol[3];

	//decimation options
	bool decimateCloudOnMove;
	bool decimateMeshOnMove;

	//information on displayed color scale
	ccScalarField* sfColorScaleToDisplay;
    bool greyForNanScalarValues;
    char colorRampTitle[256];

	//picked points
	unsigned pickedPointsSize;
	unsigned pickedPointsStartIndex;
	float pickedPointsTextShift;

	//for displaying text
	unsigned dispNumberPrecision;

	//for displaying labels
	unsigned labelsTransparency;

	//VBO
	vboStruct vbo;

	//transparency
	GLenum sourceBlend;
	GLenum destBlend;

    //Default constructor
    glDrawContext()
    : flags(0)
    , glW(0)
    , glH(0)
    , _win(0)
    , decimateCloudOnMove(true)
    , decimateMeshOnMove(true)
    , sfColorScaleToDisplay(0)
    , greyForNanScalarValues(true)
	, pickedPointsSize(4)
	, pickedPointsStartIndex(0)
	, pickedPointsTextShift(0.0)
	, dispNumberPrecision(6)
	, labelsTransparency(100)
	, sourceBlend(GL_SRC_ALPHA)
	, destBlend(GL_ONE_MINUS_SRC_ALPHA)
    {
        colorRampTitle[0]=0;
    }
};
typedef glDrawContext CC_DRAW_CONTEXT;

// Drawing flags
#define CC_DRAW_2D                              0x00000001
#define CC_DRAW_3D                              0x00000002
#define CC_DRAW_FOREGROUND                      0x00000004
#define CC_LIGHT_ENABLED                        0x00000008
#define CC_SKIP_UNSELECTED                      0x00000010
#define CC_SKIP_SELECTED                        0x00000020
#define CC_SKIP_ALL                             0x00000030
#define CC_DRAW_NAMES                           0x00000040
#define CC_DRAW_POINT_NAMES                     0x00000080
#define CC_DRAW_TRI_NAMES						0x00000100
#define CC_LOD_ACTIVATED                        0x00000200
#define CC_VIRTUAL_TRANS_ENABLED                0x00000400

// Drawing flags testing macros (see ccDrawableObject)
#define MACRO_Draw2D(context) (context.flags & CC_DRAW_2D)
#define MACRO_Draw3D(context) (context.flags & CC_DRAW_3D)
#define MACRO_DrawPointNames(context) (context.flags & CC_DRAW_POINT_NAMES)
#define MACRO_DrawTriangleNames(context) (context.flags & CC_DRAW_TRI_NAMES)
#define MACRO_DrawNames(context) (context.flags & CC_DRAW_NAMES)
#define MACRO_SkipUnselected(context) (context.flags & CC_SKIP_UNSELECTED)
#define MACRO_SkipSelected(context) (context.flags & CC_SKIP_SELECTED)
#define MACRO_LightIsEnabled(context) (context.flags & CC_LIGHT_ENABLED)
#define MACRO_Foreground(context) (context.flags & CC_DRAW_FOREGROUND)
#define MACRO_LODActivated(context) (context.flags & CC_LOD_ACTIVATED)
#define MACRO_VirtualTransEnabled(context) (context.flags & CC_VIRTUAL_TRANS_ENABLED)

//! Generic interface for (3D) drawable entities
#ifdef QCC_DB_USE_AS_DLL
class QCC_DB_DLL_API ccDrawableObject
#else
class ccDrawableObject
#endif
{
public:

    //! Default constructor
    ccDrawableObject();

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
        \return bounding box
    **/
    virtual ccBBox getBB(bool relative=true, bool withGLfeatures=false, const ccGenericGLDisplay* window=0) = 0;

    //! Draws absolute bounding-box
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
	//! Toggles colors
	virtual void toggleColors();

    //! Returns whether normals are enabled or not
    virtual bool hasNormals() const;
    //! Returns whether normals are shown or not
    virtual bool normalsShown() const;
    //! Sets normals visibility
    virtual void showNormals(bool state);
	//! Toggles normals
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

	//! Toggles SF
	virtual void toggleSF();

	//! Returns whether active scalar field is visible
    virtual bool sfShown() const;

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
	virtual bool isGLTransEnabled() const { return glTransEnabled; }

    //! Retuns associated GL transformation
    /** See ccDrawableObject::setGLTransformation.
    **/
    virtual const ccGLMatrix& getGLTransformation() const;

    //! Resets associated GL transformation
    /** GL transformation is reset to identity.
        Note: GL transformation is automatically disabled.
        See ccDrawableObject::setGLTransformation.
    **/
    virtual void razGLTransformation();

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
    bool visible;

    //! Specifies whether the object is selected or not
    bool selected;

    //! Specifies whether the visibility can be changed by user or not
    bool lockedVisibility;

    /*** OpenGL display parameters ***/

    //! Specifies whether colors should be displayed
    bool colorsDisplayed;
    //! Specifies whether normals should be displayed
    bool normalsDisplayed;
    //! Specifies whether scalar field should be displayed
    bool sfDisplayed;

    //! Temporary (unique) color
    colorType tempColor[3];
    //! Temporary (unique) color activation state
	bool colorIsOverriden;

    //! Current GL transformation
    /** See ccDrawableObject::setGLTransformation.
    **/
    ccGLMatrix glTrans;
    //! Current GL transformation activation state
    /** See ccDrawableObject::setGLTransformation.
    **/
    bool glTransEnabled;

    //! Currently associated GL display
    ccGenericGLDisplay* currentDisplay;
};

#endif
