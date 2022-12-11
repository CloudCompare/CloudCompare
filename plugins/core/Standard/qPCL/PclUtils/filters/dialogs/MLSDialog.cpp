//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
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
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#include "MLSDialog.h"
#include "../MLSSmoothingUpsampling.h"

//Qt
#include <QVariant>

MLSDialog::MLSDialog(QWidget* parent)
	: QDialog(parent)
	, Ui::MLSDialog()
{
	setupUi(this);

	updateCombo();

	connect (upsampling_method, qOverload<int>(&QComboBox::currentIndexChanged),  this, &MLSDialog::activateMenu);
	connect (search_radius,     qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MLSDialog::updateSquaredGaussian);
}

void MLSDialog::updateCombo()
{
	upsampling_method->clear();
	upsampling_method->addItem(tr("None"), QVariant(MLSParameters::NONE));
	upsampling_method->addItem(tr("Sample Local Plane"), QVariant(MLSParameters::SAMPLE_LOCAL_PLANE));
	upsampling_method->addItem(tr("Random Uniform Density"), QVariant(MLSParameters::RANDOM_UNIFORM_DENSITY));
	upsampling_method->addItem(tr("Voxel Grid Dilation"), QVariant(MLSParameters::VOXEL_GRID_DILATION));
	upsampling_method->setCurrentIndex(0);
	activateMenu(0);
}

void MLSDialog::activateMenu(int index)
{
	sample_local_plane_method->setEnabled(index == 1);
	random_uniform_density_method->setEnabled(index == 2);
	voxel_grid_dilation_method->setEnabled(index == 3);
}

void MLSDialog::updateSquaredGaussian(double radius)
{
	squared_gaussian_parameter->setValue(radius * radius);
}
