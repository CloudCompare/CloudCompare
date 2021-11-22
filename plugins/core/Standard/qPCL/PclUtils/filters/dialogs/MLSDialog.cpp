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

	connect (upsampling_method, SIGNAL(currentIndexChanged(QString)), this, SLOT(activateMenu(QString)) );
	connect (search_radius,     SIGNAL(valueChanged(double)),         this, SLOT(updateSquaredGaussian(double)) );

	deactivateAllMethods();
}

void MLSDialog::updateCombo()
{
	upsampling_method->clear();
	upsampling_method->addItem(tr("None"), QVariant(MLSParameters::NONE));
	upsampling_method->addItem(tr("Sample Local Plane"), QVariant(MLSParameters::SAMPLE_LOCAL_PLANE));
	upsampling_method->addItem(tr("Random Uniform Density"), QVariant(MLSParameters::RANDOM_UNIFORM_DENSITY));
	upsampling_method->addItem(tr("Voxel Grid Dilation"), QVariant(MLSParameters::VOXEL_GRID_DILATION));
}

void MLSDialog::activateMenu(QString name)
{
	deactivateAllMethods();

	if (name == "Sample Local Plane")
	{
		sample_local_plane_method->setEnabled(true);
	}
	else if (name == "Random Uniform Density")
	{
		random_uniform_density_method->setEnabled(true);
	}
	else if (name == "Voxel Grid Dilation")
	{
		voxel_grid_dilation_method->setEnabled(true);
	}
	else
	{
		deactivateAllMethods();
	}
}

void MLSDialog::deactivateAllMethods()
{
	sample_local_plane_method->setEnabled(false);
	random_uniform_density_method->setEnabled(false);
	voxel_grid_dilation_method->setEnabled(false);
}

void MLSDialog::toggleMethods(bool status)
{
	if (!status)
	{
		deactivateAllMethods();
	}
}

void MLSDialog::updateSquaredGaussian(double radius)
{
	squared_gaussian_parameter->setValue(radius * radius);
}
