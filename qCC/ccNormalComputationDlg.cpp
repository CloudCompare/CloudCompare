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

#include "ccNormalComputationDlg.h"

//Qt
#include <QComboBox>

//system
#include <assert.h>

ccNormalComputationDlg::ccNormalComputationDlg(QWidget* parent) : QDialog(parent), Ui::NormalComputationDlg()
{
	setupUi(this);

	setWindowFlags(Qt::Tool/*Qt::Dialog | Qt::WindowStaysOnTopHint*/);

	connect(localModelComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(localModelChanged(int)));
}

CC_LOCAL_MODEL_TYPES ccNormalComputationDlg::getLocalModel() const
{
	switch (localModelComboBox->currentIndex())
	{
	case 0:
		return LS;
	case 1:
		return HF;
	case 2:
		return TRI;
	}

	assert(false);
	return LS;
}

void ccNormalComputationDlg::localModelChanged(int index)
{
	radiusDoubleSpinBox->setEnabled(index != 2);
}

void ccNormalComputationDlg::setRadius(PointCoordinateType radius)
{
	radiusDoubleSpinBox->setValue(static_cast<double>(radius));
}

PointCoordinateType ccNormalComputationDlg::getRadius() const
{
	return static_cast<PointCoordinateType>(radiusDoubleSpinBox->value());
}

int ccNormalComputationDlg::getPreferedOrientation() const
{
	if (!preferedOrientationCheckBox->isChecked())
		return -1;

	return preferedOrientationComboBox->currentIndex();
}
