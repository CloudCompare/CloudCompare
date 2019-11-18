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
//#                    COPYRIGHT: Daniel Girardeau-Montaut                 #
//#                                                                        #
//##########################################################################

#include "ccInterpolationDlg.h"

//System
#include <assert.h>

ccInterpolationDlg::ccInterpolationDlg(QWidget* parent/*=0*/)
	: QDialog(parent, Qt::Tool)
	, Ui::InterpolationDlg()
{
	setupUi(this);

	connect(radiusDoubleSpinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ccInterpolationDlg::onRadiusUpdated);
}

ccPointCloudInterpolator::Parameters::Method ccInterpolationDlg::getInterpolationMethod() const
{
	if (nnRadioButton->isChecked())
		return ccPointCloudInterpolator::Parameters::NEAREST_NEIGHBOR;
	else if (radiusRadioButton->isChecked())
		return ccPointCloudInterpolator::Parameters::RADIUS;
	else if (knnRadioButton->isChecked())
		return ccPointCloudInterpolator::Parameters::K_NEAREST_NEIGHBORS;

	assert(false);
	return ccPointCloudInterpolator::Parameters::NEAREST_NEIGHBOR;
}

void ccInterpolationDlg::setInterpolationMethod(ccPointCloudInterpolator::Parameters::Method method)
{
	switch (method)
	{
	case ccPointCloudInterpolator::Parameters::NEAREST_NEIGHBOR:
		nnRadioButton->setChecked(true);
		break;
	case ccPointCloudInterpolator::Parameters::RADIUS:
		radiusRadioButton->setChecked(true);
		break;
	case ccPointCloudInterpolator::Parameters::K_NEAREST_NEIGHBORS:
		knnRadioButton->setChecked(true);
		break;
	default:
		assert(false);
	}
}

ccPointCloudInterpolator::Parameters::Algo ccInterpolationDlg::getInterpolationAlgorithm() const
{
	if (averageRadioButton->isChecked())
		return ccPointCloudInterpolator::Parameters::AVERAGE;
	else if (medianRadioButton->isChecked())
		return ccPointCloudInterpolator::Parameters::MEDIAN;
	else if (normalDistribRadioButton->isChecked())
		return ccPointCloudInterpolator::Parameters::NORMAL_DIST;

	assert(false);
	return ccPointCloudInterpolator::Parameters::AVERAGE;
}

void ccInterpolationDlg::setInterpolationAlgorithm(ccPointCloudInterpolator::Parameters::Algo algo)
{
	switch (algo)
	{
	case ccPointCloudInterpolator::Parameters::AVERAGE:
		averageRadioButton->setChecked(true);
		break;
	case ccPointCloudInterpolator::Parameters::MEDIAN:
		medianRadioButton->setChecked(true);
		break;
	case ccPointCloudInterpolator::Parameters::NORMAL_DIST:
		normalDistribRadioButton->setChecked(true);
		break;
	default:
		assert(false);
	}
}

void ccInterpolationDlg::onRadiusUpdated(double radius)
{
	kernelDoubleSpinBox->setValue(radius / 2.5);
}
