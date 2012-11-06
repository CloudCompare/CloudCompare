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
//$Rev:: 1691                                                              $
//$LastChangedDate:: 2010-10-22 16:52:55 +0200 (ven., 22 oct. 2010)        $
//**************************************************************************
//

#ifndef CC_TWO_COLORS_CHOICE_DLG_HEADER
#define CC_TWO_COLORS_CHOICE_DLG_HEADER

#include <QColor>

#include <ui_twoColorsDlg.h>

class ccTwoColorsDlg : public QDialog, public Ui::TwoColorChoiceDialog
{
    Q_OBJECT

public:
    ccTwoColorsDlg(QWidget* parent);

    static QColor s_firstColor;
    static QColor s_secondColor;

protected slots:
    void changeFirstColor();
    void changeSecondColor();
};

#endif
