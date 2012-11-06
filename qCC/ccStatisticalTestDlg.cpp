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

#include "ccStatisticalTestDlg.h"

ccStatisticalTestDlg::ccStatisticalTestDlg(const char* p1Label,
                                           const char* p2Label,
                                           const char* p3Label/*=0*/,
                                           const char* windowTitle/*=0*/,
                                           QWidget* parent/*=0*/)
        : QDialog(parent), Ui::StatisticalTestDialog()
{
    setupUi(this);

    setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

    param1Label->setText(p1Label);
    param2Label->setText(p2Label);
    if (p3Label)
        param3Label->setText(p3Label);
    else
    {
        param3Label->setVisible(false);
        param3SpinBox->setVisible(false);
    }

    neighborsSpinBox->setValue(16);

    if (windowTitle)
        setWindowTitle(windowTitle);
}

double ccStatisticalTestDlg::getParam1()
{
    return param1SpinBox->value();
}

double ccStatisticalTestDlg::getParam2()
{
    return param2SpinBox->value();
}

double ccStatisticalTestDlg::getParam3()
{
    return param3SpinBox->value();
}

int ccStatisticalTestDlg::getNeighborsNumber()
{
    return neighborsSpinBox->value();
}

double ccStatisticalTestDlg::getProba()
{
    return probaSpinBox->value();
}
