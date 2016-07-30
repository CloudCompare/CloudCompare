//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: qRANSAC_SD                     #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "ccRansacSDDlg.h"

#include <ccOctree.h>

static int    s_minSupport       = 500;		// this is the minimal numer of points required for a primitive
static double s_maxNormalDev_deg = 25.0;	// maximal normal deviation from ideal shape (in degrees)
static double s_probability      = 0.01;	// probability that no better candidate was overlooked during sampling

ccRansacSDDlg::ccRansacSDDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::RansacSDDialog()
{
	setupUi(this);

	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));

	supportPointsSpinBox->setValue(s_minSupport);
	maxNormDevAngleSpinBox->setValue(s_maxNormalDev_deg);
	probaDoubleSpinBox->setValue(s_probability);
}

void ccRansacSDDlg::saveSettings()
{
	s_minSupport = supportPointsSpinBox->value();
	s_maxNormalDev_deg = maxNormDevAngleSpinBox->value();
	s_probability = probaDoubleSpinBox->value();
}