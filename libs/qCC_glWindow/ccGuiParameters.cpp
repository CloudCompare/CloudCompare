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

//! Unique instance of ccGui
static ccSingleton<ccGui> s_gui;

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
	lightAmbientColor	= ccColor::darkest;
	lightSpecularColor	= ccColor::darker;
	lightDiffuseColor	= ccColor::bright;
	meshFrontDiff		= ccColor::defaultMeshFrontDiff;
	meshBackDiff		= ccColor::defaultMeshBackDiff;
	meshSpecular		= ccColor::middle;
	pointsDefaultCol	= ccColor::defaultColor;
	textDefaultCol		= ccColor::defaultColor;
	backgroundCol		= ccColor::defaultBkgColor;
	labelBackgroundCol	= ccColor::defaultLabelBkgColor;
	labelMarkerCol		= ccColor::defaultLabelMarkerColor;
	bbDefaultCol		= ccColor::yellow;

	drawBackgroundGradient		= true;
	decimateMeshOnMove			= true;
	minLoDMeshSize				= 2500000;
	decimateCloudOnMove			= true;
	minLoDCloudSize				= 10000000;
	useVBOs						= true;
	useOpenGLPointPicking		= false;
	displayCross				= true;

	labelMarkerSize				= 5;

	colorScaleShowHistogram		= true;
	colorScaleUseShader			= false;
	colorScaleShaderSupported	= false;
	colorScaleRampWidth			= 50;

	defaultFontSize				= 10;
	labelFontSize				= 8;
	displayedNumPrecision		= 6;
	labelOpacity				= 75;

	zoomSpeed					= 1.0;
}

static int c_fColorArraySize  = sizeof(float) * 4;
static int c_ubColorArraySize = sizeof(unsigned char) * 3;

void ccGui::ParamStruct::fromPersistentSettings()
{
	QSettings settings;
	settings.beginGroup("OpenGL");
	lightAmbientColor	= ccColor::Rgbaf (reinterpret_cast<float*>        (settings.value("lightAmbientColor",		QByteArray((const char*)ccColor::darkest.rgba,					c_fColorArraySize )).toByteArray().data()));
	lightSpecularColor	= ccColor::Rgbaf (reinterpret_cast<float*>        (settings.value("lightSpecularColor",		QByteArray((const char*)ccColor::darker.rgba,					c_fColorArraySize )).toByteArray().data()));
	lightDiffuseColor	= ccColor::Rgbaf (reinterpret_cast<float*>        (settings.value("lightDiffuseColor",		QByteArray((const char*)ccColor::bright.rgba,					c_fColorArraySize )).toByteArray().data()));
	meshFrontDiff		= ccColor::Rgbaf (reinterpret_cast<float*>        (settings.value("meshFrontDiff",			QByteArray((const char*)ccColor::defaultMeshFrontDiff.rgba,		c_fColorArraySize )).toByteArray().data()));
	meshBackDiff		= ccColor::Rgbaf (reinterpret_cast<float*>        (settings.value("meshBackDiff",			QByteArray((const char*)ccColor::defaultMeshBackDiff.rgba,		c_fColorArraySize )).toByteArray().data()));
	meshSpecular		= ccColor::Rgbaf (reinterpret_cast<float*>        (settings.value("meshSpecular",			QByteArray((const char*)ccColor::middle.rgba,					c_fColorArraySize )).toByteArray().data()));
	pointsDefaultCol	= ccColor::Rgbaub(reinterpret_cast<unsigned char*>(settings.value("pointsDefaultColor",		QByteArray((const char*)ccColor::defaultColor.rgb,				c_ubColorArraySize)).toByteArray().data()));
	textDefaultCol		= ccColor::Rgbaub(reinterpret_cast<unsigned char*>(settings.value("textDefaultColor",		QByteArray((const char*)ccColor::defaultColor.rgb,				c_ubColorArraySize)).toByteArray().data()));
	backgroundCol		= ccColor::Rgbaub(reinterpret_cast<unsigned char*>(settings.value("backgroundColor",		QByteArray((const char*)ccColor::defaultBkgColor.rgb,			c_ubColorArraySize)).toByteArray().data()));
	labelBackgroundCol	= ccColor::Rgbaub(reinterpret_cast<unsigned char*>(settings.value("labelBackgroundColor",	QByteArray((const char*)ccColor::defaultLabelBkgColor.rgb,		c_ubColorArraySize)).toByteArray().data()));
	labelMarkerCol		= ccColor::Rgbaub(reinterpret_cast<unsigned char*>(settings.value("labelMarkerColor",		QByteArray((const char*)ccColor::defaultLabelMarkerColor.rgb,	c_ubColorArraySize)).toByteArray().data()));
	bbDefaultCol		= ccColor::Rgbaub(reinterpret_cast<unsigned char*>(settings.value("bbDefaultColor",			QByteArray((const char*)ccColor::yellow.rgba,					c_ubColorArraySize)).toByteArray().data()));

	drawBackgroundGradient		=                                  settings.value("backgroundGradient",      true ).toBool();
	decimateMeshOnMove			=                                  settings.value("meshDecimation",          true ).toBool();
	minLoDMeshSize				=                                  settings.value("minLoDMeshSize",       2500000 ).toUInt();
	decimateCloudOnMove			=                                  settings.value("cloudDecimation",         true ).toBool();
	minLoDCloudSize				=                                  settings.value("minLoDCloudSize",     10000000 ).toUInt();
	useVBOs						=                                  settings.value("useVBOs",                 true ).toBool();
	useOpenGLPointPicking		=                                  settings.value("useOpenGLPointPicking",   false).toBool();
	displayCross				=                                  settings.value("crossDisplayed",          true ).toBool();
	labelMarkerSize				= static_cast<unsigned>(std::max(0,settings.value("labelMarkerSize",         5    ).toInt()));
	colorScaleShowHistogram		=                                  settings.value("colorScaleShowHistogram", true ).toBool();
	colorScaleUseShader			=                                  settings.value("colorScaleUseShader",     false).toBool();
	//colorScaleShaderSupported	= not saved
	colorScaleRampWidth			= static_cast<unsigned>(std::max(0,settings.value("colorScaleRampWidth",      50  ).toInt()));
	defaultFontSize				= static_cast<unsigned>(std::max(0,settings.value("defaultFontSize",          10  ).toInt()));
	labelFontSize				= static_cast<unsigned>(std::max(0,settings.value("labelFontSize",            8   ).toInt()));
	displayedNumPrecision		= static_cast<unsigned>(std::max(0,settings.value("displayedNumPrecision",    6   ).toInt()));
	labelOpacity				= static_cast<unsigned>(std::max(0,settings.value("labelOpacity",             75  ).toInt()));
	zoomSpeed					= settings.value("zoomSpeed", 1.0).toDouble();

	settings.endGroup();
}

void ccGui::ParamStruct::toPersistentSettings() const
{
	QSettings settings;
	settings.beginGroup("OpenGL");

	settings.setValue("lightDiffuseColor",        QByteArray((const char*)lightDiffuseColor.rgba,  c_fColorArraySize ));
	settings.setValue("lightAmbientColor",        QByteArray((const char*)lightAmbientColor.rgba,  c_fColorArraySize ));
	settings.setValue("lightSpecularColor",       QByteArray((const char*)lightSpecularColor.rgba, c_fColorArraySize ));
	settings.setValue("meshFrontDiff",            QByteArray((const char*)meshFrontDiff.rgba,      c_fColorArraySize ));
	settings.setValue("meshBackDiff",             QByteArray((const char*)meshBackDiff.rgba,       c_fColorArraySize ));
	settings.setValue("meshSpecular",             QByteArray((const char*)meshSpecular.rgba,       c_fColorArraySize ));
	settings.setValue("pointsDefaultColor",       QByteArray((const char*)pointsDefaultCol.rgb,    c_ubColorArraySize));
	settings.setValue("textDefaultColor",         QByteArray((const char*)textDefaultCol.rgb,      c_ubColorArraySize));
	settings.setValue("backgroundColor",          QByteArray((const char*)backgroundCol.rgb,       c_ubColorArraySize));
	settings.setValue("labelBackgroundColor",     QByteArray((const char*)labelBackgroundCol.rgb,  c_ubColorArraySize));
	settings.setValue("labelMarkerColor",         QByteArray((const char*)labelMarkerCol.rgb,      c_ubColorArraySize));
	settings.setValue("bbDefaultColor",           QByteArray((const char*)bbDefaultCol.rgb,        c_ubColorArraySize));
	settings.setValue("backgroundGradient",       drawBackgroundGradient);
	settings.setValue("meshDecimation",           decimateMeshOnMove);
	settings.setValue("minLoDMeshSize",	          minLoDMeshSize);
	settings.setValue("cloudDecimation",          decimateCloudOnMove);
	settings.setValue("minLoDCloudSize",	      minLoDCloudSize);
	settings.setValue("useVBOs",                  useVBOs);
	settings.setValue("useOpenGLPointPicking",    useOpenGLPointPicking);
	settings.setValue("crossDisplayed",           displayCross);
	settings.setValue("labelMarkerSize",          labelMarkerSize);
	settings.setValue("colorScaleShowHistogram",  colorScaleShowHistogram);
	settings.setValue("colorScaleUseShader",      colorScaleUseShader);
	//settings.setValue("colorScaleShaderSupported", not saved);
	settings.setValue("colorScaleRampWidth",      colorScaleRampWidth);
	settings.setValue("defaultFontSize",          defaultFontSize);
	settings.setValue("labelFontSize",            labelFontSize);
	settings.setValue("displayedNumPrecision",    displayedNumPrecision);
	settings.setValue("labelOpacity",             labelOpacity);
	settings.setValue("zoomSpeed",                zoomSpeed);

	settings.endGroup();
}

bool ccGui::ParamStruct::isInPersistentSettings(QString paramName) const
{
	QSettings settings;
	settings.beginGroup("OpenGL");
	return settings.contains(paramName);
}
