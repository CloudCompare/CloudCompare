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

#ifndef CC_GL_DRAW_CONTEXT_HEADER
#define CC_GL_DRAW_CONTEXT_HEADER

#include "ccIncludeGL.h"

//Local
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

// Drawing flags (type: short)
enum CC_DRAWING_FLAGS
{
	CC_DRAW_2D								= 0x0001,
	CC_DRAW_3D								= 0x0002,
	CC_DRAW_FOREGROUND						= 0x0004,
	CC_LIGHT_ENABLED						= 0x0008,
	CC_SKIP_UNSELECTED						= 0x0010,
	CC_SKIP_SELECTED						= 0x0020,
	CC_SKIP_ALL								= 0x0030,		// = CC_SKIP_UNSELECTED | CC_SKIP_SELECTED
	CC_DRAW_ENTITY_NAMES					= 0x0040,
	//CC_FREE_FLAG							= 0x0080,		// UNUSED (formerly CC_DRAW_POINT_NAMES)
	//CC_FREE_FLAG							= 0x0100,		// UNUSED (formerly CC_DRAW_TRI_NAMES)
	CC_DRAW_FAST_NAMES_ONLY					= 0x0200,
	//CC_FREE_FLAG							= 0x03C0,		// UNUSED (formerly CC_DRAW_ANY_NAMES = CC_DRAW_ENTITY_NAMES | CC_DRAW_POINT_NAMES | CC_DRAW_TRI_NAMES)
	CC_LOD_ACTIVATED						= 0x0400,
	CC_VIRTUAL_TRANS_ENABLED				= 0x0800
};

// Drawing flags testing macros (see ccDrawableObject)
#define MACRO_Draw2D(context)              (context.drawingFlags & CC_DRAW_2D)
#define MACRO_Draw3D(context)              (context.drawingFlags & CC_DRAW_3D)
#define MACRO_DrawEntityNames(context)     (context.drawingFlags & CC_DRAW_ENTITY_NAMES)
#define MACRO_DrawFastNamesOnly(context)   (context.drawingFlags & CC_DRAW_FAST_NAMES_ONLY)
#define MACRO_SkipUnselected(context)      (context.drawingFlags & CC_SKIP_UNSELECTED)
#define MACRO_SkipSelected(context)        (context.drawingFlags & CC_SKIP_SELECTED)
#define MACRO_LightIsEnabled(context)      (context.drawingFlags & CC_LIGHT_ENABLED)
#define MACRO_Foreground(context)          (context.drawingFlags & CC_DRAW_FOREGROUND)
#define MACRO_LODActivated(context)        (context.drawingFlags & CC_LOD_ACTIVATED)
#define MACRO_VirtualTransEnabled(context) (context.drawingFlags & CC_VIRTUAL_TRANS_ENABLED)

//! Display context
struct ccGLDrawContext
{
	//! Drawing options (see below)
	int drawingFlags;
	
	//! GL screen width
	int glW;
	//! GL screen height
	int glH;
	//! Device pixel ratio (general 1, 2 on HD displays)
	float devicePixelRatio;
	//! Corresponding GL window
	ccGenericGLDisplay* display;

	//! OpenGL context used to access functions for particular profiles \see glFunctions()
	QOpenGLContext *qGLContext;

	//! Current zoom (screen to file rendering mode)
	float renderZoom;

	//! Default material
	ccMaterial::Shared defaultMat;
	//! Default color for mesh (front side)
	ccColor::Rgbaf defaultMeshFrontDiff;
	//! Default color for mesh (back side)
	ccColor::Rgbaf defaultMeshBackDiff;
	//! Default point color
	ccColor::Rgba pointsDefaultCol;
	//! Default text color
	ccColor::Rgba textDefaultCol;
	//! Default label background color
	ccColor::Rgba labelDefaultBkgCol;
	//! Default label marker color
	ccColor::Rgba labelDefaultMarkerCol;
	//! Default bounding-box color
	ccColor::Rgba bbDefaultCol;

	//! Whether to decimate big clouds when updating the 3D view
	bool decimateCloudOnMove;
	//! Minimum number of points for activating LOD display
	unsigned minLODPointCount;
	//! Current level for LOD display
	unsigned char currentLODLevel;
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

	//! Stereo pass index
	unsigned stereoPassIndex;

	//! Whether to draw rounded points (instead of sqaures)
	bool drawRoundedPoints;

	//Default constructor
	ccGLDrawContext()
		: drawingFlags(0)
		, glW(0)
		, glH(0)
		, devicePixelRatio(1.0f)
		, display(nullptr)
		, qGLContext(nullptr)
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
		, minLODPointCount(10000000)
		, currentLODLevel(0)
		, moreLODPointsAvailable(false)
		, higherLODLevelsAvailable(false)
		, decimateMeshOnMove(true)
		, minLODTriangleCount(2500000)
		, sfColorScaleToDisplay(nullptr)
		, colorRampShader(nullptr)
		, customRenderingShader(nullptr)
		, useVBOs(true)
		, labelMarkerSize(5)
		, labelMarkerTextShift_pix(5)
		, dispNumberPrecision(6)
		, labelOpacity(100)
		, sourceBlend(GL_SRC_ALPHA)
		, destBlend(GL_ONE_MINUS_SRC_ALPHA)
		, stereoPassIndex(0)
		, drawRoundedPoints(false)
	{}
   
	template<class TYPE>
	TYPE *glFunctions() const
	{				
		return qGLContext ? qGLContext->versionFunctions<TYPE>() : 0;
	}   
};

using CC_DRAW_CONTEXT = ccGLDrawContext;

#endif //CC_GL_DRAW_CONTEXT_HEADER
