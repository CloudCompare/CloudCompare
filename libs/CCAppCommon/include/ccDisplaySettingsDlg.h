#pragma once
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

#include "CCAppCommon.h"

//Local
#include "ccOptions.h"

//qCC_gl
#include <ccGuiParameters.h>

//Qt
#include <QDialog>

namespace Ui
{
	class DisplaySettingsDlg;
}

//! Dialog to setup display settings
class CCAPPCOMMON_LIB_API ccDisplaySettingsDlg : public QDialog
{
	Q_OBJECT

public:
	explicit ccDisplaySettingsDlg(QWidget* parent);
	~ccDisplaySettingsDlg() override;

Q_SIGNALS:
	void aspectHasChanged();

protected:
	void changeLightDiffuseColor();
	void changeLightAmbientColor();
	void changeLightSpecularColor();
	void changeMeshFrontDiffuseColor();
	void changeMeshBackDiffuseColor();
	void changeMeshSpecularColor();
	void changePointsColor();
	void changeBBColor();
	void changeTextColor();
	void changeBackgroundColor();
	void changeLabelBackgroundColor();
	void changeLabelMarkerColor();
	void changeMaxMeshSize(double);
	void changeMaxCloudSize(double);
	void changeVBOUsage();
	void changeColorScaleRampWidth(int);
	void changePickingCursor(int);
	void changeLogVerbosity(int);

	void changeDefaultFontSize(int);
	void changeLabelFontSize(int);
	void changeNumberPrecision(int);
	void changeLabelOpacity(int);
	void changeLabelMarkerSize(int);

	void changeZoomSpeed(double);

	void changeAutoComputeOctreeOption(int);

	void doAccept();
	void doReject();
	void apply();
	void reset();

protected:

	//! Refreshes dialog to reflect new parameters values
	void refresh();

	// Colors
	QColor m_lightDiffuseColor;
	QColor m_lightAmbientColor;
	QColor m_lightSpecularColor;
	QColor m_meshFrontDiff;
	QColor m_meshBackDiff;
	QColor m_meshSpecularColor;
	QColor m_pointsDefaultCol;
	QColor m_textDefaultCol;
	QColor m_backgroundCol;
	QColor m_labelBackgroundCol;
	QColor m_labelMarkerCol;
	QColor m_bbDefaultCol;

	//! Current GUI parameters
	ccGui::ParamStruct m_parameters;
	//! Current options
	ccOptions m_options;

	//! Old parameters (for restore)
	ccGui::ParamStruct m_oldParameters;
	//! Old options (for restore)
	ccOptions m_oldOptions;

	//! Default style (active when the dialog was first shown)
	int m_defaultAppStyleIndex;
	
private:
	Ui::DisplaySettingsDlg* m_ui;
};
