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

#ifndef CC_TWO_COLORS_CHOICE_DLG_HEADER
#define CC_TWO_COLORS_CHOICE_DLG_HEADER

//Qt
#include <QColor>

#include <ui_twoColorsDlg.h>

//! Dialog to define a simple color ramp (2 colors) and a dimension
class ccTwoColorsDlg : public QDialog, public Ui::TwoColorChoiceDialog
{
    Q_OBJECT

public:

	//! Default constructor
    ccTwoColorsDlg(QWidget* parent);

    static QColor s_firstColor;
    static QColor s_secondColor;

protected slots:

    void changeFirstColor();
    void changeSecondColor();
};

#endif
