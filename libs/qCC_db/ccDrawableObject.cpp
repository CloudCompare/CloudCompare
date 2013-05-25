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

//system
#include <string.h>

/******************************/
/*      ccDrawableObject      */
/******************************/

ccDrawableObject::ccDrawableObject()
{
    setVisible(true);
    setSelected(false);
    showColors(false);
    showNormals(false);
    showSF(false);
	lockVisibility(false);
	showNameIn3D(false);
    m_currentDisplay=0;

    enableTempColor(false);
    setTempColor(ccColor::white,false);
    razGLTransformation();
}

bool ccDrawableObject::isVisible() const
{
    return m_visible;
}

void ccDrawableObject::setVisible(bool state)
{
    m_visible = state;
}

void ccDrawableObject::toggleVisibility()
{
	setVisible(!isVisible());
}

bool ccDrawableObject::isVisiblityLocked() const
{
    return m_lockedVisibility;
}

void ccDrawableObject::lockVisibility(bool state)
{
    m_lockedVisibility = state;
}

bool ccDrawableObject::isSelected() const
{
    return m_selected;
}

void ccDrawableObject::setSelected(bool state)
{
    m_selected = state;
}

void ccDrawableObject::drawBB(const colorType col[])
{
    getBB(true,false,m_currentDisplay).draw(col);
}

ccBBox ccDrawableObject::getFitBB(ccGLMatrix& trans)
{
	//Default behavior: returns axis aligned bounding box!
	trans.toIdentity();
	return getBB(true,false,m_currentDisplay);

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
    if (win && m_currentDisplay!=win)
        win->invalidateViewport();

    m_currentDisplay=win;
}

void ccDrawableObject::removeFromDisplay(const ccGenericGLDisplay* win)
{
    if (m_currentDisplay == win)
        setDisplay(0);
}

ccGenericGLDisplay* ccDrawableObject::getDisplay() const
{
    return m_currentDisplay;
}

const ccGLMatrix& ccDrawableObject::getGLTransformation() const
{
    return m_glTrans;
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

void ccDrawableObject::razGLTransformation()
{
    enableGLTransformation(false);
    m_glTrans.toIdentity();
}

void ccDrawableObject::enableGLTransformation(bool state)
{
    m_glTransEnabled = state;
}

bool ccDrawableObject::isGLTransEnabled() const
{
	return m_glTransEnabled;
}

void ccDrawableObject::showColors(bool state)
{
    m_colorsDisplayed = state;
}

void ccDrawableObject::toggleColors()
{
	showColors(!colorsShown());
}

bool ccDrawableObject::colorsShown() const
{
    return m_colorsDisplayed;
}

bool ccDrawableObject::hasColors() const
{
    return false;
}

void ccDrawableObject::showNormals(bool state)
{
    m_normalsDisplayed = state;
}

void ccDrawableObject::toggleNormals()
{
	showNormals(!normalsShown());
}

bool ccDrawableObject::normalsShown() const
{
    return m_normalsDisplayed;
}

bool ccDrawableObject::hasNormals() const
{
    return false;
}

bool ccDrawableObject::hasScalarFields() const
{
    return false;
}

bool ccDrawableObject::hasDisplayedScalarField() const
{
    return false;
}

void ccDrawableObject::showSF(bool state)
{
    m_sfDisplayed = state;
}

void ccDrawableObject::toggleSF()
{
	showSF(!sfShown());
}

bool ccDrawableObject::sfShown() const
{
    return m_sfDisplayed;
}

void ccDrawableObject::setTempColor(const colorType* col, bool autoActivate /*= true*/)
{
    memcpy(m_tempColor,col,3*sizeof(colorType));

    if (autoActivate)
        enableTempColor(true);
}

void ccDrawableObject::enableTempColor(bool state)
{
    m_colorIsOverriden=state;
}

bool ccDrawableObject::isColorOverriden() const
{
    return m_colorIsOverriden;
}

const colorType* ccDrawableObject::getTempColor() const
{
    return m_tempColor;
}

void ccDrawableObject::getDrawingParameters(glDrawParams& params) const
{
 	//color override
	if (isColorOverriden())
	{
		params.showColors=true;
		params.showNorms=hasNormals() && normalsShown()/*false*/;
		params.showSF=false;
	}
	else
	{
        params.showNorms = hasNormals() && normalsShown();
        params.showSF = hasDisplayedScalarField() && sfShown();
        //colors are not displayed if scalar field is displayed
        params.showColors = !params.showSF && hasColors() && colorsShown();
	}
}

void ccDrawableObject::showNameIn3D(bool state)
{
	m_showNameIn3D = state;
}

bool ccDrawableObject::nameShownIn3D() const
{
	return m_showNameIn3D;
}

void ccDrawableObject::toggleShowName()
{
	showNameIn3D(!nameShownIn3D());
}
