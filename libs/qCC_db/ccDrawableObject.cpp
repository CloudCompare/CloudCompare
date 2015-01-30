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

#include "ccDrawableObject.h"

//Local
#include "ccGenericGLDisplay.h"

/******************************/
/*      ccDrawableObject      */
/******************************/

ccDrawableObject::ccDrawableObject()
	: m_currentDisplay(0)
{
	setVisible(true);
	setSelected(false);
	lockVisibility(false);
	showColors(false);
	showNormals(false);
	showSF(false);
	enableTempColor(false);
	setTempColor(ccColor::white,false);
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

void ccDrawableObject::refreshDisplay()
{
	if (m_currentDisplay)
		m_currentDisplay->refresh();
}

void ccDrawableObject::prepareDisplayForRefresh()
{
	if (m_currentDisplay)
		m_currentDisplay->toBeRefreshed();
}

void ccDrawableObject::setDisplay(ccGenericGLDisplay* win)
{
	if (win && m_currentDisplay != win)
		win->invalidateViewport();

	m_currentDisplay = win;
}

void ccDrawableObject::removeFromDisplay(const ccGenericGLDisplay* win)
{
	if (m_currentDisplay == win)
		setDisplay(0);
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

void ccDrawableObject::setTempColor(const ccColor::Rgb& col, bool autoActivate/*=true*/)
{
	m_tempColor = col;

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
