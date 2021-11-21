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

#include "FastGlobalRegistrationDlg.h"

//common
#include <ccQtHelpers.h>

//qCC_db
#include <ccOctree.h>
#include <ccPointCloud.h>

//system
#include <assert.h>

static double s_featureRadius = 0;

FastGlobalRegistrationDialog::FastGlobalRegistrationDialog(	ccPointCloud* aligned,
															ccPointCloud* reference,
															QWidget* parent/*=nullptr*/)
	: QDialog(parent, Qt::Tool)
	, Ui::FastGlobalRegistrationDialog()
{
	assert(aligned && reference);
	alignedCloud = aligned;
	referenceCloud = reference;

	setupUi(this);

	updateGUI();

	ccQtHelpers::SetButtonColor(dataColorButton, Qt::red);
	ccQtHelpers::SetButtonColor(modelColorButton, Qt::yellow);

	//restore semi-persistent settings
	{
		//semi-persistent options
		double previousRadius = s_featureRadius;
		if (previousRadius == 0)
		{
			double alignedRadius = ccOctree::GuessNaiveRadius(alignedCloud);
			double referenceRadius = ccOctree::GuessNaiveRadius(referenceCloud);
			previousRadius = std::max(alignedRadius, referenceRadius);
		}

		featureRadiusDoubleSpinBox->setValue(previousRadius);
	}

	connect(autoRadiusToolButton, &QToolButton::clicked, this, &FastGlobalRegistrationDialog::autoEstimateRadius);
	connect(swapButton, &QAbstractButton::clicked, this, &FastGlobalRegistrationDialog::swapModelAndData);
}

FastGlobalRegistrationDialog::~FastGlobalRegistrationDialog()
{
	if (referenceCloud)
	{
		referenceCloud->enableTempColor(false);
		referenceCloud->prepareDisplayForRefresh_recursive();
	}
	if (alignedCloud)
	{
		alignedCloud->enableTempColor(false);
		alignedCloud->prepareDisplayForRefresh_recursive();
	}

	if (referenceCloud)
	{
		referenceCloud->refreshDisplay();
	}
	if (alignedCloud)
	{
		alignedCloud->refreshDisplay();
	}
}

void FastGlobalRegistrationDialog::saveParameters() const
{
	s_featureRadius = getFeatureRadius();
}

ccPointCloud* FastGlobalRegistrationDialog::getReferenceCloud()
{
	return referenceCloud;
}

ccPointCloud* FastGlobalRegistrationDialog::getAlignedCloud()
{
	return alignedCloud;
}

double FastGlobalRegistrationDialog::getFeatureRadius() const
{
	return featureRadiusDoubleSpinBox->value();
}

void FastGlobalRegistrationDialog::updateGUI()
{
	if (!referenceCloud || !alignedCloud)
		return;

	referenceLineEdit->setText(referenceCloud->getName());
	referenceCloud->setVisible(true);
	referenceCloud->setTempColor(ccColor::yellow);
	referenceCloud->prepareDisplayForRefresh_recursive();

	alignedLineEdit->setText(alignedCloud->getName());
	alignedCloud->setVisible(true);
	alignedCloud->setTempColor(ccColor::red);
	alignedCloud->prepareDisplayForRefresh_recursive();

	referenceCloud->refreshDisplay();
	alignedCloud->refreshDisplay();
}

void FastGlobalRegistrationDialog::swapModelAndData()
{
	std::swap(alignedCloud, referenceCloud);

	updateGUI();
}

void FastGlobalRegistrationDialog::autoEstimateRadius()
{
	ccOctree::BestRadiusParams params;
	{
		params.aimedPopulationPerCell = 64;
		params.aimedPopulationRange = 16;
		params.minCellPopulation = 48;
		params.minAboveMinRatio = 0.97;
	}

	PointCoordinateType alignedRadius = ccOctree::GuessBestRadiusAutoComputeOctree(alignedCloud, params, this);
	if (alignedRadius < 0)
	{
		ccLog::Error("Failed to estimate the radius");
		return;
	}
	PointCoordinateType referenceRadius = ccOctree::GuessBestRadiusAutoComputeOctree(referenceCloud, params, this);
	if (referenceRadius < 0)
	{
		ccLog::Error("Failed to estimate the radius");
		return;
	}

	PointCoordinateType largestRadius = std::max(alignedRadius, referenceRadius);
	featureRadiusDoubleSpinBox->setValue(largestRadius);
}
