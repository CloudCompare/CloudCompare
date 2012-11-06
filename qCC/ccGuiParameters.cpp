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
//$Rev:: 2172                                                              $
//$LastChangedDate:: 2012-06-24 18:33:24 +0200 (dim., 24 juin 2012)        $
//**************************************************************************
//

//Qt
#include <QSettings>

//qCC_db
#include <ccBasicTypes.h>

#include "ccGuiParameters.h"

//! Static unique instance of ccGui
static ccGui* s_gui = 0;

const int c_fColorArraySize = 4*sizeof(float);
const int c_ubColorArraySize = 3*sizeof(unsigned char);

const ccGui::ParamStruct& ccGui::Parameters()
{
    if (!s_gui)
    {
        s_gui = new ccGui();
        s_gui->params.fromPersistentSettings();
    }

    return s_gui->params;
}

void ccGui::ReleaseInstance()
{
    if (s_gui)
        delete s_gui;
    s_gui=0;
}

void ccGui::Set(const ParamStruct& params)
{
    if (!s_gui)
        s_gui = new ccGui();

    s_gui->params = params;
}

ccGui::ParamStruct::ParamStruct()
{
    reset();
}

void ccGui::ParamStruct::reset()
{
    memcpy(lightAmbientColor,   ccColor::night,				    c_fColorArraySize);
    memcpy(lightSpecularColor,  ccColor::middle,				c_fColorArraySize);
    memcpy(lightDiffuseColor,   ccColor::lighter,				c_fColorArraySize);
    memcpy(meshFrontDiff,       ccColor::defaultMeshFrontDiff,	c_fColorArraySize);
    memcpy(meshBackDiff,        ccColor::defaultMeshBackDiff,	c_fColorArraySize);
    memcpy(meshSpecular,        ccColor::middle,				c_fColorArraySize);
    memcpy(pointsDefaultCol,    ccColor::defaultColor,			c_ubColorArraySize);
    memcpy(textDefaultCol,      ccColor::defaultColor,			c_ubColorArraySize);
    memcpy(backgroundCol,       ccColor::defaultBkgColor,		c_ubColorArraySize);
    memcpy(bbDefaultCol,        ccColor::yellow,				c_ubColorArraySize);

    drawBackgroundGradient      = true;
    decimateMeshOnMove          = true;
    decimateCloudOnMove         = true;
    displayCross                = true;

	pickedPointsSize = 4;
	pickedPointsStartIndex = 1;

	colorScaleAlwaysSymmetrical	= true;
	colorScaleAlwaysShowZero	= true;
	colorScaleSquareSize		= 20;
	
	defaultFontSize				= 10;
	displayedNumPrecision		= 6;
	labelsTransparency			= 50;
}

ccGui::ParamStruct& ccGui::ParamStruct::operator =(const ccGui::ParamStruct& params)
{
    memcpy(lightDiffuseColor,   params.lightDiffuseColor,   c_fColorArraySize);
    memcpy(lightAmbientColor,   params.lightAmbientColor,   c_fColorArraySize);
    memcpy(lightSpecularColor,  params.lightSpecularColor,  c_fColorArraySize);
    memcpy(meshFrontDiff,       params.meshFrontDiff,       c_fColorArraySize);
    memcpy(meshBackDiff,        params.meshBackDiff,        c_fColorArraySize);
	memcpy(meshSpecular,		params.meshSpecular,        c_fColorArraySize);
    memcpy(pointsDefaultCol,    params.pointsDefaultCol,    c_ubColorArraySize);
    memcpy(textDefaultCol,      params.textDefaultCol,      c_ubColorArraySize);
    memcpy(backgroundCol,       params.backgroundCol,       c_ubColorArraySize);
    memcpy(bbDefaultCol,        params.bbDefaultCol,        c_ubColorArraySize);

    drawBackgroundGradient      = params.drawBackgroundGradient;
    decimateMeshOnMove          = params.decimateMeshOnMove;
    decimateCloudOnMove         = params.decimateCloudOnMove;
    displayCross                = params.displayCross;
	pickedPointsSize			= params.pickedPointsSize;
	pickedPointsStartIndex		= params.pickedPointsStartIndex;
	colorScaleAlwaysSymmetrical	= params.colorScaleAlwaysSymmetrical;
	colorScaleAlwaysShowZero	= params.colorScaleAlwaysShowZero;
	colorScaleSquareSize		= params.colorScaleSquareSize;
	defaultFontSize				= params.defaultFontSize;
	displayedNumPrecision		= params.displayedNumPrecision;
	labelsTransparency			= params.labelsTransparency;

    return *this;

}

void ccGui::ParamStruct::fromPersistentSettings()
{
    QSettings settings;
    settings.beginGroup("OpenGL");

    memcpy(lightAmbientColor,   settings.value("lightAmbientColor",     QByteArray((const char*)ccColor::night,                 c_fColorArraySize)).toByteArray().data(), c_fColorArraySize);
    memcpy(lightSpecularColor,  settings.value("lightSpecularColor",    QByteArray((const char*)ccColor::middle,                c_fColorArraySize)).toByteArray().data(), c_fColorArraySize);
    memcpy(lightDiffuseColor,   settings.value("lightDiffuseColor",     QByteArray((const char*)ccColor::lighter,               c_fColorArraySize)).toByteArray().data(), c_fColorArraySize);
    memcpy(meshFrontDiff,       settings.value("meshFrontDiff",         QByteArray((const char*)ccColor::defaultMeshFrontDiff,  c_fColorArraySize)).toByteArray().data(), c_fColorArraySize);
    memcpy(meshBackDiff,        settings.value("meshBackDiff",          QByteArray((const char*)ccColor::defaultMeshBackDiff,   c_fColorArraySize)).toByteArray().data(), c_fColorArraySize);
    memcpy(meshSpecular,        settings.value("meshSpecular",          QByteArray((const char*)ccColor::middle,				c_fColorArraySize)).toByteArray().data(), c_fColorArraySize);
    memcpy(pointsDefaultCol,    settings.value("pointsDefaultColor",    QByteArray((const char*)ccColor::defaultColor,          c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);
    memcpy(textDefaultCol,      settings.value("textDefaultColor",      QByteArray((const char*)ccColor::defaultColor,          c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);
    memcpy(backgroundCol,       settings.value("backgroundColor",       QByteArray((const char*)ccColor::defaultBkgColor,       c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);
    memcpy(bbDefaultCol,        settings.value("bbDefaultColor",        QByteArray((const char*)ccColor::yellow,                c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);

    drawBackgroundGradient  = settings.value("backgroundGradient", true).toBool();
    decimateMeshOnMove      = settings.value("meshDecimation", true).toBool();
    decimateCloudOnMove     = settings.value("cloudDecimation", true).toBool();
    displayCross            = settings.value("crossDisplayed", true).toBool();

	pickedPointsSize		= (unsigned)settings.value("pickedPointsSize", 4).toInt();
	pickedPointsStartIndex	= (unsigned)settings.value("pickedPointsStartIndex", 1).toInt();

    colorScaleAlwaysSymmetrical	= settings.value("colorScaleAlwaysSymmetrical", true).toBool();
    colorScaleAlwaysShowZero	= settings.value("colorScaleAlwaysShowZero", true).toBool();
	colorScaleSquareSize		= (unsigned)settings.value("colorScaleSquareSize", 20).toInt();
	
	defaultFontSize				= (unsigned)settings.value("defaultFontSize", 10).toInt();
	displayedNumPrecision		= (unsigned)settings.value("displayedNumPrecision", 6).toInt();
	labelsTransparency			= (unsigned)settings.value("labelsTransparency", 50).toInt();

    settings.endGroup();
}

void ccGui::ParamStruct::toPersistentSettings()
{
    QSettings settings;
    settings.beginGroup("OpenGL");

    settings.setValue("lightDiffuseColor",QByteArray((const char*)lightDiffuseColor,c_fColorArraySize));
    settings.setValue("lightAmbientColor",QByteArray((const char*)lightAmbientColor,c_fColorArraySize));
    settings.setValue("lightSpecularColor",QByteArray((const char*)lightSpecularColor,c_fColorArraySize));
    settings.setValue("meshFrontDiff",QByteArray((const char*)meshFrontDiff,c_fColorArraySize));
    settings.setValue("meshBackDiff",QByteArray((const char*)meshBackDiff,c_fColorArraySize));
    settings.setValue("meshSpecular",QByteArray((const char*)meshSpecular,c_fColorArraySize));
    settings.setValue("pointsDefaultColor",QByteArray((const char*)pointsDefaultCol,c_ubColorArraySize));
    settings.setValue("textDefaultColor",QByteArray((const char*)textDefaultCol,c_ubColorArraySize));
    settings.setValue("backgroundColor",QByteArray((const char*)backgroundCol,c_ubColorArraySize));
    settings.setValue("bbDefaultColor",QByteArray((const char*)bbDefaultCol,c_ubColorArraySize));
    settings.setValue("backgroundGradient",drawBackgroundGradient);
    settings.setValue("meshDecimation",decimateMeshOnMove);
    settings.setValue("cloudDecimation",decimateCloudOnMove);
    settings.setValue("crossDisplayed",displayCross);
	settings.setValue("pickedPointsSize", pickedPointsSize);
	settings.setValue("pickedPointsStartIndex", pickedPointsStartIndex);
	settings.setValue("colorScaleAlwaysSymmetrical", colorScaleAlwaysSymmetrical);
    settings.setValue("colorScaleAlwaysShowZero", colorScaleAlwaysShowZero);
	settings.setValue("colorScaleSquareSize", colorScaleSquareSize);
	settings.setValue("defaultFontSize", defaultFontSize);
	settings.setValue("displayedNumPrecision", displayedNumPrecision);
	settings.setValue("labelsTransparency", labelsTransparency);


    settings.endGroup();
}
