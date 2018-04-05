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

//PCL
//#include <pcl/surface/mls.h>

//Qt
#include <QVariant>

MLSDialog::MLSDialog(QWidget *parent)
	: QDialog(parent)
	, Ui::MLSDialog()
{
	setupUi(this);

	updateCombo();

	connect (this->upsampling_method, SIGNAL(currentIndexChanged(QString)), this, SLOT(activateMenu(QString)) );
	connect (this->search_radius,     SIGNAL(valueChanged(double)),         this, SLOT(updateSquaredGaussian(double)) );

	deactivateAllMethods();
}

void MLSDialog::updateCombo()
{
	this->upsampling_method->clear();
	this->upsampling_method->addItem(QString("None"), QVariant(MLSParameters::NONE));
	this->upsampling_method->addItem(QString("Sample Local Plane"), QVariant(MLSParameters::SAMPLE_LOCAL_PLANE));
	this->upsampling_method->addItem(QString("Random Uniform Density"), QVariant(MLSParameters::RANDOM_UNIFORM_DENSITY));
	this->upsampling_method->addItem(QString("Voxel Grid Dilation"), QVariant(MLSParameters::VOXEL_GRID_DILATION));
}

void MLSDialog::activateMenu(QString name)
{
	deactivateAllMethods();

	if (name == "Sample Local Plane")
	{
		this->sample_local_plane_method->setEnabled(true);
	}
	else if (name == "Random Uniform Density")
	{
		this->random_uniform_density_method->setEnabled(true);
	}
	else if (name == "Voxel Grid Dilation")
	{
		this->voxel_grid_dilation_method->setEnabled(true);
	}
	else
	{
		deactivateAllMethods();
	}
}

void MLSDialog::deactivateAllMethods()
{
	this->sample_local_plane_method->setEnabled(false);
	this->random_uniform_density_method->setEnabled(false);
	this->voxel_grid_dilation_method->setEnabled(false);
}

void MLSDialog::toggleMethods(bool status)
{
	if (!status)
		deactivateAllMethods();
}

void MLSDialog::updateSquaredGaussian(double radius)
{
	this->squared_gaussian_parameter->setValue(radius * radius);
}
