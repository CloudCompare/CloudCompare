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

#include "ccGuiParameters.h"

//Qt
#include <QSettings>

//qCC_db
#include <ccBasicTypes.h>
#include <ccSingleton.h>

//System
#include <string.h>

//! Static unique instance of ccGui
static ccSingleton<ccGui> s_gui;

const int c_fColorArraySize = 4*sizeof(float);
const int c_ubColorArraySize = 3*sizeof(unsigned char);

const ccGui::ParamStruct& ccGui::Parameters()
{
	if (!s_gui.instance)
	{
		s_gui.instance = new ccGui();
		s_gui.instance->params.fromPersistentSettings();
	}

	return s_gui.instance->params;
}

void ccGui::ReleaseInstance()
{
	s_gui.release();
}

void ccGui::Set(const ParamStruct& params)
{
	if (!s_gui.instance)
	{
		s_gui.instance = new ccGui();
		s_gui.instance->params.fromPersistentSettings();
	}

	s_gui.instance->params = params;
}

ccGui::ParamStruct::ParamStruct()
{
	reset();
}

void ccGui::ParamStruct::reset()
{
	memcpy(lightAmbientColor,	ccColor::darkest,				c_fColorArraySize);
	memcpy(lightSpecularColor,	ccColor::darker,				c_fColorArraySize);
	memcpy(lightDiffuseColor,	ccColor::bright,				c_fColorArraySize);
	memcpy(meshFrontDiff,		ccColor::defaultMeshFrontDiff,	c_fColorArraySize);
	memcpy(meshBackDiff,		ccColor::defaultMeshBackDiff,	c_fColorArraySize);
	memcpy(meshSpecular,		ccColor::middle,				c_fColorArraySize);
	memcpy(pointsDefaultCol,	ccColor::defaultColor,			c_ubColorArraySize);
	memcpy(textDefaultCol,		ccColor::defaultColor,			c_ubColorArraySize);
	memcpy(backgroundCol,		ccColor::defaultBkgColor,		c_ubColorArraySize);
	memcpy(histBackgroundCol,	ccColor::defaultHistBkgColor,	c_ubColorArraySize);
	memcpy(labelCol,			ccColor::defaultLabelColor,		c_ubColorArraySize);
	memcpy(bbDefaultCol,		ccColor::yellow,				c_ubColorArraySize);

	drawBackgroundGradient		= true;
	decimateMeshOnMove			= true;
	decimateCloudOnMove			= true;
	useVBOs						= true;
	displayCross				= true;

	pickedPointsSize			= 4;

	colorScaleShowHistogram		= true;
	colorScaleUseShader			= false;
	colorScaleShaderSupported	= false;
	colorScaleRampWidth			= 50;

	defaultFontSize				= 10;
	displayedNumPrecision		= 6;
	labelsTransparency			= 50;
}

ccGui::ParamStruct& ccGui::ParamStruct::operator =(const ccGui::ParamStruct& params)
{
	memcpy(lightDiffuseColor,	params.lightDiffuseColor,	c_fColorArraySize);
	memcpy(lightAmbientColor,	params.lightAmbientColor,	c_fColorArraySize);
	memcpy(lightSpecularColor,	params.lightSpecularColor,	c_fColorArraySize);
	memcpy(meshFrontDiff,		params.meshFrontDiff,		c_fColorArraySize);
	memcpy(meshBackDiff,		params.meshBackDiff,		c_fColorArraySize);
	memcpy(meshSpecular,		params.meshSpecular,		c_fColorArraySize);
	memcpy(pointsDefaultCol,	params.pointsDefaultCol,	c_ubColorArraySize);
	memcpy(textDefaultCol,		params.textDefaultCol,		c_ubColorArraySize);
	memcpy(backgroundCol,		params.backgroundCol,		c_ubColorArraySize);
	memcpy(histBackgroundCol,	params.histBackgroundCol,	c_ubColorArraySize);
	memcpy(labelCol,			params.labelCol,			c_ubColorArraySize);
	memcpy(bbDefaultCol,		params.bbDefaultCol,		c_ubColorArraySize);

	drawBackgroundGradient		= params.drawBackgroundGradient;
	decimateMeshOnMove			= params.decimateMeshOnMove;
	decimateCloudOnMove			= params.decimateCloudOnMove;
	useVBOs						= params.useVBOs;
	displayCross				= params.displayCross;
	pickedPointsSize			= params.pickedPointsSize;
	colorScaleShowHistogram		= params.colorScaleShowHistogram;
	colorScaleUseShader			= params.colorScaleUseShader;
	colorScaleShaderSupported	= params.colorScaleShaderSupported;
	colorScaleRampWidth			= params.colorScaleRampWidth;
	defaultFontSize				= params.defaultFontSize;
	displayedNumPrecision		= params.displayedNumPrecision;
	labelsTransparency			= params.labelsTransparency;

	return *this;

}

void ccGui::ParamStruct::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("OpenGL");

	memcpy(lightAmbientColor,	settings.value("lightAmbientColor",		QByteArray((const char*)ccColor::darkest,				c_fColorArraySize )).toByteArray().data(), c_fColorArraySize);
	memcpy(lightSpecularColor,	settings.value("lightSpecularColor",	QByteArray((const char*)ccColor::darker,				c_fColorArraySize )).toByteArray().data(), c_fColorArraySize);
	memcpy(lightDiffuseColor,	settings.value("lightDiffuseColor",		QByteArray((const char*)ccColor::bright,				c_fColorArraySize )).toByteArray().data(), c_fColorArraySize);
	memcpy(meshFrontDiff,		settings.value("meshFrontDiff",			QByteArray((const char*)ccColor::defaultMeshFrontDiff,	c_fColorArraySize )).toByteArray().data(), c_fColorArraySize);
	memcpy(meshBackDiff,		settings.value("meshBackDiff",			QByteArray((const char*)ccColor::defaultMeshBackDiff,	c_fColorArraySize )).toByteArray().data(), c_fColorArraySize);
	memcpy(meshSpecular,		settings.value("meshSpecular",			QByteArray((const char*)ccColor::middle,				c_fColorArraySize )).toByteArray().data(), c_fColorArraySize);
	memcpy(pointsDefaultCol,	settings.value("pointsDefaultColor",	QByteArray((const char*)ccColor::defaultColor,			c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);
	memcpy(textDefaultCol,		settings.value("textDefaultColor",		QByteArray((const char*)ccColor::defaultColor,			c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);
	memcpy(backgroundCol,		settings.value("backgroundColor",		QByteArray((const char*)ccColor::defaultBkgColor,		c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);
	memcpy(histBackgroundCol,	settings.value("histBackgroundColor",	QByteArray((const char*)ccColor::defaultHistBkgColor,	c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);
	memcpy(labelCol,			settings.value("labelColor",			QByteArray((const char*)ccColor::defaultLabelColor,		c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);
	memcpy(bbDefaultCol,		settings.value("bbDefaultColor",		QByteArray((const char*)ccColor::yellow,				c_ubColorArraySize)).toByteArray().data(), c_ubColorArraySize);

	drawBackgroundGradient	= settings.value("backgroundGradient", true).toBool();
	decimateMeshOnMove		= settings.value("meshDecimation", true).toBool();
	decimateCloudOnMove		= settings.value("cloudDecimation", true).toBool();
	useVBOs					= settings.value("useVBOs", true).toBool();
	displayCross			= settings.value("crossDisplayed", true).toBool();

	pickedPointsSize		= (unsigned)settings.value("pickedPointsSize", 4).toInt();

	colorScaleShowHistogram		= settings.value("colorScaleShowHistogram", true).toBool();
	colorScaleUseShader			= settings.value("colorScaleUseShader", false).toBool();
	//colorScaleShaderSupported	= not saved
	colorScaleRampWidth			= (unsigned)settings.value("colorScaleRampWidth", 50).toInt();

	defaultFontSize				= (unsigned)settings.value("defaultFontSize", 10).toInt();
	displayedNumPrecision		= (unsigned)settings.value("displayedNumPrecision", 6).toInt();
	labelsTransparency			= (unsigned)settings.value("labelsTransparency", 50).toInt();

	settings.endGroup();
}

void ccGui::ParamStruct::toPersistentSettings() const
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
	settings.setValue("histBackgroundColor",QByteArray((const char*)histBackgroundCol,c_ubColorArraySize));
	settings.setValue("labelColor",QByteArray((const char*)labelCol,c_ubColorArraySize));
	settings.setValue("bbDefaultColor",QByteArray((const char*)bbDefaultCol,c_ubColorArraySize));
	settings.setValue("backgroundGradient",drawBackgroundGradient);
	settings.setValue("meshDecimation",decimateMeshOnMove);
	settings.setValue("cloudDecimation",decimateCloudOnMove);
	settings.setValue("useVBOs", useVBOs);
	settings.setValue("crossDisplayed",displayCross);
	settings.setValue("pickedPointsSize", pickedPointsSize);
	settings.setValue("colorScaleShowHistogram", colorScaleShowHistogram);
	settings.setValue("colorScaleUseShader", colorScaleUseShader);
	//settings.setValue("colorScaleShaderSupported", not saved);
	settings.setValue("colorScaleRampWidth", colorScaleRampWidth);
	settings.setValue("defaultFontSize", defaultFontSize);
	settings.setValue("displayedNumPrecision", displayedNumPrecision);
	settings.setValue("labelsTransparency", labelsTransparency);

	settings.endGroup();
}

bool ccGui::ParamStruct::isInPersistentSettings(QString paramName) const
{
	QSettings settings;
	settings.beginGroup("OpenGL");
	return settings.contains(paramName);
}
