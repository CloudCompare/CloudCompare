//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHoughNormals               #
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

#include "qHoughNormalsDialog.h"
#include "ui_qHoughNormalsDlg.h"

namespace {
	#ifndef M_PI
		constexpr double M_PI = 3.14159265358979323846264338327950288;
	#endif	
	
	constexpr double RadiansToDegrees = 180.0 / M_PI;
	constexpr double DegreesToRadians = M_PI / 180.0;
}

qHoughNormalsDialog::qHoughNormalsDialog(QWidget *parent)
	: QDialog(parent)
	, m_ui( new Ui::HoughNormalsDialog )
{
	m_ui->setupUi(this);
}

qHoughNormalsDialog::~qHoughNormalsDialog()
{
	delete m_ui;
}

void qHoughNormalsDialog::setParameters( const Parameters &params )
{
	m_ui->kSpinBox->setValue(params.K);
	m_ui->tSpinBox->setValue(params.T);
	m_ui->nPhiSpinBox->setValue(params.n_phi);
	m_ui->nRotSpinBox->setValue(params.n_rot);
	m_ui->tolAngleSpinBox->setValue(params.tol_angle_rad * RadiansToDegrees);
	m_ui->kDensitySpinBox->setValue(params.k_density);
	m_ui->useDensityCheckBox->setChecked(params.use_density);
}

void qHoughNormalsDialog::getParameters( Parameters &params )
{
	params.K = m_ui->kSpinBox->value();
	params.T = m_ui->tSpinBox->value();
	params.n_phi = m_ui->nPhiSpinBox->value();
	params.n_rot = m_ui->nRotSpinBox->value();
	params.tol_angle_rad = m_ui->tolAngleSpinBox->value() * DegreesToRadians;
	params.k_density = m_ui->kDensitySpinBox->value();
	params.use_density = m_ui->useDensityCheckBox->isChecked();
}
