//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHPR                        #
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

#ifndef QHOUGH_NORMALS_HEADER
#define QHOUGH_NORMALS_HEADER

#include "ui_qHoughNormalsDlg.h"

//Qt
#include <QDialog>

//System
#include <cmath>

//! Dialog for Hough Normals dialog
class qHoughNormalsDialog : public QDialog, public Ui::HoughNormalsDialog
{
public:

	//! Default constructor
	explicit qHoughNormalsDialog(QWidget* parent = 0)
		: QDialog(parent)
	{
		setupUi(this);
	}

	//Settings
	struct Parameters
	{
		int K = 100;
		int T = 1000;
		int n_phi = 15;
		int n_rot = 5;
		bool use_density = false;
		float tol_angle_rad = 0.79f;
		int k_density = 5;
	};
	
	void setParameters(const Parameters& params)
	{
		kSpinBox->setValue(params.K);
		tSpinBox->setValue(params.T);
		nPhiSpinBox->setValue(params.n_phi);
		nRotSpinBox->setValue(params.n_rot);
		tolAngleSpinBox->setValue(params.tol_angle_rad * 180.0 / M_PI);
		kDensitySpinBox->setValue(params.k_density);
		useDensityCheckBox->setChecked(params.use_density);
	}

	void getParameters(Parameters& params)
	{
		params.K = kSpinBox->value();
		params.T = tSpinBox->value();
		params.n_phi = nPhiSpinBox->value();
		params.n_rot = nRotSpinBox->value();
		params.tol_angle_rad = tolAngleSpinBox->value() * M_PI / 180.0;
		params.k_density = kDensitySpinBox->value();
		params.use_density = useDensityCheckBox->isChecked();
	}
};

#endif //QHOUGH_NORMALS_HEADER
