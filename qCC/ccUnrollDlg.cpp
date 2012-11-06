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

#include "ccUnrollDlg.h"

ccUnrollDlg::ccUnrollDlg(QWidget* parent/*=0*/)
        : QDialog(parent), Ui::UnrollDialog()
{
    setupUi(this);

    connect(comboBoxUnrollShapeType, SIGNAL(currentIndexChanged(int)), this, SLOT(shapeTypeChanged(int)));
    connect(checkBoxAuto, SIGNAL(stateChanged(int)), this, SLOT(axisAutoStateChanged(int)));
    connect(comboBoxAxisDimension, SIGNAL(currentIndexChanged(int)), this, SLOT(axisDimensionChanged(int)));

    checkBoxAuto->setChecked(true);

    shapeTypeChanged(comboBoxUnrollShapeType->currentIndex());
    axisDimensionChanged(comboBoxAxisDimension->currentIndex());
}

int ccUnrollDlg::getType()
{
    return comboBoxUnrollShapeType->currentIndex();
}

int ccUnrollDlg::getAxisDimension()
{
    return comboBoxAxisDimension->currentIndex();
}

bool ccUnrollDlg::isAxisPositionAuto()
{
    return (checkBoxAuto->checkState()==Qt::Checked);
}

CCVector3 ccUnrollDlg::getAxisPosition()
{
    return CCVector3(doubleSpinBoxAxisX->value(),
                     doubleSpinBoxAxisY->value(),
                     doubleSpinBoxAxisZ->value());
}

double ccUnrollDlg::getRadius()
{
    return doubleSpinBoxRadius->value();
}

double ccUnrollDlg::getAngle()
{
    return doubleSpinBoxAngle->value();
}

void ccUnrollDlg::shapeTypeChanged(int index)
{
    if (index==0)
    {
        labelAngle->setHidden(true);
        doubleSpinBoxAngle->setHidden(true);
        groupBoxAxisPosition->setTitle("Axis position");
        checkBoxAuto->setHidden(false);
        axisAutoStateChanged(checkBoxAuto->checkState());
    }
    else if (index==1)
    {
        labelAngle->setHidden(false);
        doubleSpinBoxAngle->setHidden(false);
        groupBoxAxisPosition->setTitle("Cone apex");
        checkBoxAuto->setHidden(true);
        axisAutoStateChanged(Qt::Unchecked);
    }
}

void ccUnrollDlg::axisAutoStateChanged(int checkState)
{
    bool state = (checkState == Qt::Unchecked);
    doubleSpinBoxAxisX->setEnabled(state);
    doubleSpinBoxAxisY->setEnabled(state);
    doubleSpinBoxAxisZ->setEnabled(state);

    if (state)
        axisDimensionChanged(comboBoxAxisDimension->currentIndex());
}

void ccUnrollDlg::axisDimensionChanged(int index)
{
    //if axis is in auto mode, we
    if (comboBoxUnrollShapeType->currentIndex()==1 || checkBoxAuto->checkState()==Qt::Checked)
        return;

    //in 'cylinder' mode, we hide the axis coordinate that is not needed
    switch (index)
    {
        case 0:
            doubleSpinBoxAxisX->setEnabled(false);
            doubleSpinBoxAxisY->setEnabled(true);
            doubleSpinBoxAxisZ->setEnabled(true);
            break;
        case 1:
            doubleSpinBoxAxisX->setEnabled(true);
            doubleSpinBoxAxisY->setEnabled(false);
            doubleSpinBoxAxisZ->setEnabled(true);
            break;
        case 2:
            doubleSpinBoxAxisX->setEnabled(true);
            doubleSpinBoxAxisY->setEnabled(true);
            doubleSpinBoxAxisZ->setEnabled(false);
            break;
    }
}
