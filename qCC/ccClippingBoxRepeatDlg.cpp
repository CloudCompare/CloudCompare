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

#include "ccClippingBoxRepeatDlg.h"

//Qt
#include <QPushButton>

ccClippingBoxRepeatDlg::ccClippingBoxRepeatDlg(QWidget* parent)
	: QDialog(parent)
{
	setupUi(this);

	//setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);
	connect( xRepeatCheckBox, SIGNAL(toggled(bool)), this, SLOT(onDimChecked(bool)));
	connect( yRepeatCheckBox, SIGNAL(toggled(bool)), this, SLOT(onDimChecked(bool)));
	connect( zRepeatCheckBox, SIGNAL(toggled(bool)), this, SLOT(onDimChecked(bool)));
}

void ccClippingBoxRepeatDlg::onDimChecked(bool)
{
	//if only one dimension is checked, then the user can choose to project
	//the points along this dimension
	int sum =	static_cast<int>(xRepeatCheckBox->isChecked())
			+	static_cast<int>(yRepeatCheckBox->isChecked())
			+	static_cast<int>(zRepeatCheckBox->isChecked());

	if (sum == 1)
	{
		if (!projectOnBestFitCheckBox->isEnabled())
			projectOnBestFitCheckBox->setChecked(false);
		projectOnBestFitCheckBox->setEnabled(true);
	}
	else
	{
		projectOnBestFitCheckBox->setEnabled(false);
		projectOnBestFitCheckBox->setChecked(true);
	}

	buttonBox->button(QDialogButtonBox::Ok)->setEnabled(sum != 0);
}
