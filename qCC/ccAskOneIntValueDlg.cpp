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
//$Rev:: 1595                                                              $
//$LastChangedDate:: 2010-07-02 18:04:17 +0200 (ven., 02 juil. 2010)       $
//**************************************************************************
//

#include "ccAskOneIntValueDlg.h"

ccAskOneIntValueDlg::ccAskOneIntValueDlg(const char* valueName,
                                            int minVal,
                                            int maxVal,
                                            int defaultVal,
                                            const char* windowTitle/*=0*/,
                                            QWidget* parent/*=0*/)
    : QDialog(parent), Ui::AskOneIntValueDialog()
{
    setupUi(this);

    setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

    valueLabel->setText(valueName);
    valueSpinBox->setRange(minVal,maxVal);
    valueSpinBox->setValue(defaultVal);

    if (windowTitle)
        setWindowTitle(windowTitle);
}

int ccAskOneIntValueDlg::getValue()
{
    return valueSpinBox->value();
}
