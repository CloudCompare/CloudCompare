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

#include "ccDrawableObject.h"

//Local
#include "ccGenericGLDisplay.h"

ccDrawableObject::ccDrawableObject()
	: m_currentDisplay(nullptr)
{
	setVisible(true);
	setSelected(false);
	lockVisibility(false);
	showColors(false);
	showNormals(false);
	showSF(false);
	enableTempColor(false);
	setTempColor(ccColor::white, false);
	resetGLTransformation();
	showNameIn3D(false);
}

ccDrawableObject::ccDrawableObject(const ccDrawableObject& object)
	: m_visible(object.m_visible)
	, m_selected(object.m_selected)
	, m_lockedVisibility(object.m_lockedVisibility)
	, m_colorsDisplayed(object.m_colorsDisplayed)
	, m_normalsDisplayed(object.m_normalsDisplayed)
	, m_sfDisplayed(object.m_sfDisplayed)
	, m_tempColor(object.m_tempColor)
	, m_colorIsOverriden(object.m_colorIsOverriden)
	, m_glTrans(object.m_glTrans)
	, m_glTransEnabled(object.m_glTransEnabled)
	, m_showNameIn3D(object.m_showNameIn3D)
	, m_currentDisplay(object.m_currentDisplay)
{
}

void ccDrawableObject::redrawDisplay()
{
	if (m_currentDisplay)
		m_currentDisplay->redraw();
}

void ccDrawableObject::refreshDisplay(bool only2D/*=false*/)
{
	if (m_currentDisplay)
		m_currentDisplay->refresh(only2D);
}

void ccDrawableObject::prepareDisplayForRefresh()
{
	if (m_currentDisplay)
		m_currentDisplay->toBeRefreshed();
}

void ccDrawableObject::setDisplay(ccGenericGLDisplay* win)
{
	if (win && m_currentDisplay != win)
	{
		win->invalidateViewport();
		win->deprecate3DLayer();
	}

	m_currentDisplay = win;
	if (m_currentDisplay)
	{
		m_currentDisplay->deprecate3DLayer();
	}
}

void ccDrawableObject::removeFromDisplay(const ccGenericGLDisplay* win)
{
	if (m_currentDisplay == win)
	{
		if (m_currentDisplay)
		{
			m_currentDisplay->deprecate3DLayer();
		}
		setDisplay(nullptr);
	}
}

void ccDrawableObject::enableGLTransformation(bool state)
{
	m_glTransEnabled = state;
	if (m_currentDisplay)
	{
		m_currentDisplay->deprecate3DLayer();
	}
}

void ccDrawableObject::setGLTransformation(const ccGLMatrix& trans)
{
	m_glTrans = trans;
	enableGLTransformation(true);
}

void ccDrawableObject::rotateGL(const ccGLMatrix& rotMat)
{
	m_glTrans = rotMat * m_glTrans;
	enableGLTransformation(true);
}

void ccDrawableObject::translateGL(const CCVector3& trans)
{
	m_glTrans += trans;
	enableGLTransformation(true);
}

void ccDrawableObject::resetGLTransformation()
{
	enableGLTransformation(false);
	m_glTrans.toIdentity();
}

void ccDrawableObject::setTempColor(const ccColor::Rgba& col, bool autoActivate/*=true*/)
{
	m_tempColor = col;

	if (autoActivate)
		enableTempColor(true);
}

void ccDrawableObject::setTempColor(const ccColor::Rgb& col, bool autoActivate/*=true*/)
{
	m_tempColor = ccColor::Rgba(col, ccColor::MAX);

	if (autoActivate)
		enableTempColor(true);
}

void ccDrawableObject::getDrawingParameters(glDrawParams& params) const
{
	//color override
	if (isColorOverriden())
	{
		params.showColors = true;
		params.showNorms = hasNormals() && normalsShown()/*false*/;
		params.showSF = false;
	}
	else
	{
		params.showNorms = hasNormals() && normalsShown();
		params.showSF = hasDisplayedScalarField() && sfShown();
		//colors are not displayed if scalar field is displayed
		params.showColors = !params.showSF && hasColors() && colorsShown();
	}
}

bool ccDrawableObject::addClipPlanes(const ccClipPlane& plane)
{
	try
	{
		m_clipPlanes.push_back(plane);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	return true;
}

void ccDrawableObject::toggleClipPlanes(CC_DRAW_CONTEXT& context, bool enable)
{
	if (m_clipPlanes.empty())
	{
		return;
	}
	
	//get the set of OpenGL functions (version 2.1)
	QOpenGLFunctions_2_1* glFunc = context.glFunctions<QOpenGLFunctions_2_1>();
	assert(glFunc != nullptr);

	if (glFunc == nullptr)
		return;

	GLint maxPlaneCount = 0;
	glFunc->glGetIntegerv(GL_MAX_CLIP_PLANES, &maxPlaneCount);

	GLint planeCount = static_cast<GLint>(m_clipPlanes.size());
	if (planeCount > maxPlaneCount)
	{
		if (enable)
		{
			ccLog::Warning("[ccDrawableObject::enableClipPlanes] Clipping planes count exceeds the maximum supported number");
		}
		planeCount = maxPlaneCount;
	}
	for (GLint i=0; i<planeCount; ++i)
	{
		GLenum planeIndex = GL_CLIP_PLANE0 + i;
		if (enable)
		{
			glFunc->glClipPlane(planeIndex, m_clipPlanes[i].equation.u);
			glFunc->glEnable(planeIndex);
		}
		else
		{
			glFunc->glDisable(planeIndex);
		}
	}
}
