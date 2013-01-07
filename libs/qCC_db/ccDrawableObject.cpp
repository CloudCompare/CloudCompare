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
//$Rev:: 1992                                                              $
//$LastChangedDate:: 2012-01-18 12:17:49 +0100 (mer., 18 janv. 2012)       $
//**************************************************************************
//

#include "ccDrawableObject.h"

/***********************/
/*      vboStruct      */
/***********************/

vboStruct::vboStruct()
	: enabled(false)
	, idVert(0)
	, idInd(0)
	, buffer(0)
{
}

void vboStruct::clear()
{
	if (buffer)
		delete[] buffer;
}

void vboStruct::init()
{
	if (buffer)
		delete[] buffer;
	//vertices & features table
	buffer = new unsigned char[3*maxSize()*elemSize()];
}

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
    currentDisplay=0;

    enableTempColor(false);
    setTempColor(ccColor::white,false);
    razGLTransformation();
}

bool ccDrawableObject::isVisible() const
{
    return visible;
}

void ccDrawableObject::setVisible(bool state)
{
    visible = state;
}

void ccDrawableObject::toggleVisibility()
{
	setVisible(!isVisible());
}

bool ccDrawableObject::isVisiblityLocked() const
{
    return lockedVisibility;
}

void ccDrawableObject::lockVisibility(bool state)
{
    lockedVisibility = state;
}

bool ccDrawableObject::isSelected() const
{
    return selected;
}

void ccDrawableObject::setSelected(bool state)
{
    selected = state;
}

void ccDrawableObject::drawBB(const colorType col[])
{
    getBB(true,false,currentDisplay).draw(col);
}

void ccDrawableObject::redrawDisplay()
{
    if (currentDisplay)
        currentDisplay->redraw();
}

void ccDrawableObject::refreshDisplay()
{
    if (currentDisplay)
        currentDisplay->refresh();
}

void ccDrawableObject::prepareDisplayForRefresh()
{
    if (currentDisplay)
        currentDisplay->toBeRefreshed();
}

void ccDrawableObject::setDisplay(ccGenericGLDisplay* win)
{
    if (win && currentDisplay!=win)
        win->invalidateViewport();

    currentDisplay=win;
}

void ccDrawableObject::removeFromDisplay(const ccGenericGLDisplay* win)
{
    if (currentDisplay == win)
        setDisplay(0);
}

ccGenericGLDisplay* ccDrawableObject::getDisplay() const
{
    return currentDisplay;
}

const ccGLMatrix& ccDrawableObject::getGLTransformation() const
{
    return glTrans;
}

void ccDrawableObject::setGLTransformation(const ccGLMatrix& trans)
{
    glTrans = trans;
    enableGLTransformation(true);
}

void ccDrawableObject::rotateGL(const ccGLMatrix& rotMat)
{
    glTrans = rotMat * glTrans;
    enableGLTransformation(true);
}

void ccDrawableObject::translateGL(const CCVector3& trans)
{
    glTrans += trans;
    enableGLTransformation(true);
}

void ccDrawableObject::razGLTransformation()
{
    enableGLTransformation(false);
    glTrans.toIdentity();
}

void ccDrawableObject::enableGLTransformation(bool state)
{
    glTransEnabled = state;
}

void ccDrawableObject::showColors(bool state)
{
    colorsDisplayed = state;
}

void ccDrawableObject::toggleColors()
{
	showColors(!colorsShown());
}

bool ccDrawableObject::colorsShown() const
{
    return colorsDisplayed;
}

bool ccDrawableObject::hasColors() const
{
    return false;
}

void ccDrawableObject::showNormals(bool state)
{
    normalsDisplayed = state;
}

void ccDrawableObject::toggleNormals()
{
	showNormals(!normalsShown());
}

bool ccDrawableObject::normalsShown() const
{
    return normalsDisplayed;
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
    sfDisplayed = state;
}

void ccDrawableObject::toggleSF()
{
	showSF(!sfShown());
}

bool ccDrawableObject::sfShown() const
{
    return sfDisplayed;
}

void ccDrawableObject::setTempColor(const colorType* col, bool autoActivate /*= true*/)
{
    memcpy(tempColor,col,3*sizeof(colorType));

    if (autoActivate)
        enableTempColor(true);
}

void ccDrawableObject::enableTempColor(bool state)
{
    colorIsOverriden=state;
}

bool ccDrawableObject::isColorOverriden() const
{
    return colorIsOverriden;
}

const colorType* ccDrawableObject::getTempColor() const
{
    return tempColor;
}

void ccDrawableObject::getDrawingParameters(glDrawParams& params) const
{
 	//color override
	if (isColorOverriden())
	{
		params.showColors=true;
		params.showNorms=false;
		params.showSF=false;
	}
	else
	{
        params.showNorms = hasNormals() &&  normalsShown();
        //colors are not displayed if scalar field is displayed
        params.showSF = hasDisplayedScalarField() && sfShown();
        params.showColors = !params.showSF && hasColors() && colorsShown();
	}
}
