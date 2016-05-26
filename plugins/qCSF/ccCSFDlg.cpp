//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCSF                          #
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
//#               COPYRIGHT: Qi jianbo ; Wan peng                          #
//#                                                                        #
//##########################################################################

#include "ccCSFDlg.h"

#include <ccOctree.h>

static int MaxIteration = 500;
static double cloth_resolution = 1.5;
static double class_threshold = 0.5;
ccCSFDlg::ccCSFDlg(QWidget* parent) : QDialog(parent), Ui::CSFDialog()
{
	setupUi(this);

	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	MaxIterationSpinBox->setValue(MaxIteration);
	cloth_resolutionSpinBox->setValue(cloth_resolution);
	class_thresholdSpinBox->setValue(class_threshold);

}

void ccCSFDlg::saveSettings()
{
   	MaxIteration=MaxIterationSpinBox->value();
   	cloth_resolution=cloth_resolutionSpinBox->value();
    class_threshold=class_thresholdSpinBox->value();
}