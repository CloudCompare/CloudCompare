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
//$Rev:: 1802                                                              $
//$LastChangedDate:: 2011-02-15 23:32:56 +0100 (mar., 15 févr. 2011)      $
//**************************************************************************
//

#ifndef CC_DISPLAY_OPTIONS_DIALOG_HEADER
#define CC_DISPLAY_OPTIONS_DIALOG_HEADER

#include <QColor>

#include <ui_displayOptionsDlg.h>

#include "ccGuiParameters.h"

//! Dialog to setup display settings
class ccDisplayOptionsDlg : public QDialog, public Ui::DisplayOptionsDlg
{
    Q_OBJECT

public:

    //! Default constructor
    ccDisplayOptionsDlg(QWidget* parent);

    //! Sets a button background color
    /** Support Windows XP style particularity.
    **/
    static void SetButtonColor(QAbstractButton* button, QColor &col);

    //! Sets a button text color
    /** Support Windows XP style particularity.
    **/
    static void SetButtonTextColor(QAbstractButton* button, QColor &col);

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
    void changeBackgroundGradient();
    void changeMeshDecimation();
    void changeCloudDecimation();
    void changeCrossDisplayed();
	void changeColorScaleAlwaysZero();
	void changeColorScaleAlwaysSym();
	void changeColorScaleSquareSize(int);
	
	void changeDefaultFontSize(int);
	void changeNumberPrecision(int);
	void changeLabelsTransparency(int);

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
    QColor bbDefaultCol;

    //! Current parameters
    ccGui::ParamStruct parameters;

    //! Old parameters (for restore)
    ccGui::ParamStruct oldParameters;
};

#endif
