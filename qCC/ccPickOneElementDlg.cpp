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
//$Rev:: 1631                                                              $
//$LastChangedDate:: 2010-08-25 07:21:40 +0200 (mer., 25 août 2010)       $
//**************************************************************************
//

#include "ccPickOneElementDlg.h"

ccPickOneElementDlg::ccPickOneElementDlg(const char* label,
                                            const char* windowTitle/*=0*/,
                                                QWidget* parent/*=0*/)
        : QDialog(parent), Ui::PickOneElementDialog()
{
    setupUi(this);

    setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

    comboLabel->setText(label);

    if (windowTitle)
        setWindowTitle(windowTitle);
}

void ccPickOneElementDlg::addElement(const char* elementName)
{
    if (!elementName)
        return;

    comboBox->addItem(elementName);
}

void ccPickOneElementDlg::setDefaultIndex(int index)
{
    comboBox->setCurrentIndex(index);
}

int ccPickOneElementDlg::getSelectedIndex()
{
    return comboBox->currentIndex();
}
