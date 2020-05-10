//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccClippingBoxRepeatDlg.h"

//Qt
#include <QPushButton>

//system
#include <assert.h>

ccClippingBoxRepeatDlg::ccClippingBoxRepeatDlg(bool singleSliceMode/*=false*/, QWidget* parent/*=0*/)
	: QDialog(parent)
{
	setupUi(this);

	if (!singleSliceMode)
	{
		connect(xRepeatCheckBox, &QAbstractButton::toggled, this, &ccClippingBoxRepeatDlg::onDimChecked);
		connect(yRepeatCheckBox, &QAbstractButton::toggled, this, &ccClippingBoxRepeatDlg::onDimChecked);
		connect(zRepeatCheckBox, &QAbstractButton::toggled, this, &ccClippingBoxRepeatDlg::onDimChecked);
	}
	else
	{
		//single slice extraction mode!
		repeatDimGroupBox->setTitle("Flat dimension");

		connect(xRepeatCheckBox, &QAbstractButton::toggled, this, &ccClippingBoxRepeatDlg::onDimXChecked);
		connect(yRepeatCheckBox, &QAbstractButton::toggled, this, &ccClippingBoxRepeatDlg::onDimYChecked);
		connect(zRepeatCheckBox, &QAbstractButton::toggled, this, &ccClippingBoxRepeatDlg::onDimZChecked);
		
		setFlatDim(0);

		randomColorCheckBox->setChecked(false);
		otherOptionsGroupBox->setVisible(false);
	}
}

void ccClippingBoxRepeatDlg::setRepeatDim(unsigned char dim)
{
	assert(dim < 3);
	QCheckBox* boxes[3] = { xRepeatCheckBox,
							yRepeatCheckBox,
							zRepeatCheckBox };

	for (unsigned char d = 0; d < 3; ++d)
	{
		boxes[d]->setChecked(d == dim);
	}
}

void ccClippingBoxRepeatDlg::onDimXChecked(bool state)
{
	assert(state);
	setFlatDim(0);
}

void ccClippingBoxRepeatDlg::onDimYChecked(bool state)
{
	assert(state);
	setFlatDim(1);
}

void ccClippingBoxRepeatDlg::onDimZChecked(bool state)
{
	assert(state);
	setFlatDim(2);
}

void ccClippingBoxRepeatDlg::setFlatDim(unsigned char dim)
{
	assert(dim < 3);
	QCheckBox* boxes[3] = { xRepeatCheckBox,
							yRepeatCheckBox,
							zRepeatCheckBox };

	for (unsigned char d = 0; d < 3; ++d)
	{
		boxes[d]->blockSignals(true);
		//disable the current dimension
		//and uncheck the other dimensions
		boxes[d]->setChecked(d == dim);
		boxes[d]->setEnabled(d != dim);
		boxes[d]->blockSignals(false);
	}
	
	extractLevelSetGroupBox->setEnabled(true);
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
		if (!envProjectPointsOnBestFitCheckBox->isVisible())
		{
			envProjectPointsOnBestFitCheckBox->setChecked(false);
		}
		envProjectPointsOnBestFitCheckBox->setVisible(true);
		envelopeTypeComboBox->setEnabled(true);
		extractLevelSetGroupBox->setEnabled(true);
	}
	else
	{
		envProjectPointsOnBestFitCheckBox->setVisible(false);
		envProjectPointsOnBestFitCheckBox->setChecked(true);
		envelopeTypeComboBox->setCurrentIndex(2);
		envelopeTypeComboBox->setEnabled(false);
		extractLevelSetGroupBox->setEnabled(false);
		extractLevelSetGroupBox->setChecked(false);
	}

	buttonBox->button(QDialogButtonBox::Ok)->setEnabled(sum != 0);
}
