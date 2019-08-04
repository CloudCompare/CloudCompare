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

#ifndef CC_DISPLAY_OPTIONS_DIALOG_HEADER
#define CC_DISPLAY_OPTIONS_DIALOG_HEADER

//Local
#include "ccOptions.h"

//qCC_gl
#include <ccGuiParameters.h>

//Qt
#include <QDialog>

namespace Ui
{
	class DisplayOptionsDlg;
}

//! Dialog to setup display settings
class ccDisplayOptionsDlg : public QDialog
{
	Q_OBJECT

public:
	explicit ccDisplayOptionsDlg(QWidget* parent);
	~ccDisplayOptionsDlg() override;

signals:
	void aspectHasChanged();

protected slots:
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

	QColor lightDiffuseColor;
	QColor lightAmbientColor;
	QColor lightSpecularColor;
	QColor meshFrontDiff;
	QColor meshBackDiff;
	QColor meshSpecularColor;
	QColor pointsDefaultCol;
	QColor textDefaultCol;
	QColor backgroundCol;
	QColor labelBackgroundCol;
	QColor labelMarkerCol;
	QColor bbDefaultCol;

	//! Current GUI parameters
	ccGui::ParamStruct parameters;
	//! Current options
	ccOptions options;

	//! Old parameters (for restore)
	ccGui::ParamStruct oldParameters;
	//! Old options (for restore)
	ccOptions oldOptions;
	
private:
	Ui::DisplayOptionsDlg* m_ui;
};

#endif
