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

void ccInterpolationDlg::setInterpolationMethd(ccPointCloudInterpolator::Parameters::Method method)
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
